/*!\file HalDAC.cpp
 *    \author iot@ektos.net\vda@ektos.net (EKTOS A/S)
 *    \date created on Nov 1, 2020
 *    \version 1.1
 *    \brief description goes here
 *
 *    Belongs to Blocks project
 */

#include "HalDAC.hpp"

//------------------------------------------------------------------------------
HalDAC::Channel::Channel(HalDAC &halDac, HalTimerManager::TimerID id, DacMode mode, Channels num, bool buffDisabled):
timerDescriptor(HalTimerManager::getTimerDescriptor(id)),
dac(halDac),
dma(num?dac.discriptor.dma1:dac.discriptor.dma),
buffDac(0),
idCh(num),
playFinish(false),
playFinishCounter(0),
playPending(false)
{
  Assert(num < ChannelTotal);
  Assert((id == HalTimerManager::TIM_6) || (id == HalTimerManager::TIM_7));
  Assert(mode == DacModeUsual);

  uint32_t cr;
  cr = (mode << DAC_CR_WAVE1_Pos) | DAC_CR_TEN1_Msk | DAC_CR_DMAEN1_Msk | DAC_CR_DMAUDRIE1_Msk;

  if(id == HalTimerManager::TIM_6) {cr |= Timer6TRGOevent << DAC_CR_TSEL1_Pos;}
  else                             {cr |= Timer7TRGOevent << DAC_CR_TSEL1_Pos;}

  if(buffDisabled) cr |= DAC_CR_BOFF1_Msk;

  dac.discriptor.base->CR = cr << (num * 16);

  HalPower::moduleEnable(timerDescriptor.power);
  HalPower::moduleReset(timerDescriptor.power);
  dma.setMode(false, true);
  cr = num?(uint32_t)&dac.discriptor.base->DHR12R2:(uint32_t)&dac.discriptor.base->DHR12R1;
  dma.setPeripheral(cr, HalDma::size_16bit);
  IRQManager::addHandler(dac.discriptor.irq, *this);
  IRQPort::setPriority(dac.discriptor.irq);

  HalDebug::stopInDebugEnable(HalDebug::ID_TIM6);

  Scheduler::add(*this);
}

//------------------------------------------------------------------------------
void HalDAC::Channel::start(){
  timerDescriptor.base.CR1 |= TIM_CR1_CEN;
  dac.discriptor.base->CR  |= (DAC_CR_EN1_Msk | DAC_CR_DMAEN1_Msk)<< (idCh * 16) ;
}

//------------------------------------------------------------------------------
void HalDAC::Channel::stop(){
  playFinish = true;//Delayed stop until the remaining buffer is fully played
}

//------------------------------------------------------------------------------
void HalDAC::Channel::run() {
  if(playFinish){
    mutex.lock();
    size_t size1Out, size2Out;
  	uint8_t *data1Out, *data2Out;
  	InternalLock((void *&) data1Out, size1Out, (void *&) data2Out, size2Out);

  	if(size1Out > 0)                            // If we have some free (already played) frame in DAC buffer.
  	{
  	  playFinishCounter += size1Out;            // Save already cleared frames size

  	  memset(data1Out, 0, size1Out);            // Clean current frame part of DAC buffer.
  	  unlock(size1Out);                         // Part clean is processed and change current "buffDac->count" value.

      if(playFinishCounter >= buffDac->getSize())
      {
    	buffDac->setCount(0);
    	InternalStop();
    	playFinishCounter = 0;
    	playFinish        = false;

        if(playPending)
        {
    	  Assert(buffDac);
    	  buffDac->clear();
    	  dma.stop();
    	  dma.start(buffDac->getData(), buffDac->getSize());
    	  start();
    	  IRQPort::enableIRQ(dac.discriptor.irq);
    	  playPending = false;
        }
      }
  	}
    mutex.unlock();
  }
}
//------------------------------------------------------------------------------

void HalDAC::Channel::InternalStop(){
  uint32_t cr = dac.discriptor.base->CR;
  cr &= ~((DAC_CR_EN1_Msk << (idCh * 16)) | (DAC_CR_DMAEN1_Msk << (idCh * 16)));
  dac.discriptor.base->CR = cr;
  timerDescriptor.base.CR1 &= ~TIM_CR1_CEN;
}

//------------------------------------------------------------------------------
void HalDAC::Channel::play(){
  playPending = true; //Delayed start/restart until the remaining buffer is fully played
}

//------------------------------------------------------------------------------
void HalDAC::Channel::setTimerParameters(uint32_t sampleFrequency){
  Assert(sampleFrequency); // frequency for PWM can't be zero
  TIM_TypeDef &base = timerDescriptor.base;

  uint32_t baseFreq = HalClock::getModuleClockFrequency(timerDescriptor.clock);
  uint32_t divider = baseFreq / sampleFrequency;
  Assert(divider); // requested frequency is more then timer clock
  if(timerDescriptor.is32bit || (divider <= maxValue<uint16_t>())){
    base.ARR = divider;
    base.PSC = 0;
  }else{
    uint16_t psc = (divider / maxValue<uint16_t>());
    base.ARR = divider / (psc+1);
    base.PSC = psc;
  }

  base.CR1  = TIM_CR1_URS | TIM_CR1_ARPE;
  base.CNT  = 0;
  base.CR2  = 2 << TIM_CR2_MMS_Pos;
  base.DIER = 0;
  base.EGR  = TIM_EGR_UG;
}

//------------------------------------------------------------------------------
void HalDAC::Channel::prepare(HalI2SBus::AudioFormat *format){
  setTimerParameters(format->audioFreq);
}

//-----------------------------------------------------------------------------<< IS IT FUCTION EVER USED ? >>
void HalDAC::Channel::processIRQ(){
  stop();
  dac.discriptor.base->SR = (idCh)? DAC_SR_DMAUDR2_Msk : DAC_SR_DMAUDR1_Msk;
  play();
}

//-----------------------------------------------------------------------------
uint32_t HalDAC::Channel::getFillState(){
  Assert(&dma);
  Assert(buffDac);
  uint64_t fill = ringdelta(buffDac->getCount(), dma.getTransferredCount(), buffDac->getSize());
  fill <<= 32;
  return fill/buffDac->getSize();
}


//-----------------------------------------------------------------------------
void HalDAC::Channel::lock(void *&data1, size_t &size1, void *&data2, size_t &size2){
  if(playFinish){
	size1 = size2 = 0;
    data1,  data2 = 0;
  }else{
    InternalLock(data1,size1,data2,size2);
  }
}
//-----------------------------------------------------------------------------
void HalDAC::Channel::InternalLock(void *&data1, size_t &size1, void *&data2, size_t &size2){

	//<< FOR DEBUG ONLY : >>
    /*volatile static uint32_t compare_NDTR = 1;

    volatile uint32_t *NDTR = (uint32_t*)(0x4002608C);

    if(compare_NDTR >= *NDTR)
    {
    	__asm volatile ("NOP");
    	__asm volatile ("NOP");
    }*/
	//<< FOR DEBUG ONLY / >>

  Assert(&dma);
  Assert(buffDac);
  volatile size_t count    = buffDac->getCount();
  volatile size_t dmaCount = dma.getTransferredCount();

  if(count < dmaCount){
    //size2 = 0;
    size1 = dmaCount - count;
    //data1 = (uint8_t *)buffDac->getData() + count;
  }
  else if(count >= dmaCount){
    //data2 = buffDac->getData();
    //size2 = dmaCount;
    //data1 = (uint8_t *)buffDac->getData() + count;
    size1 = buffDac->getSize() - count;
  }
  else{
	//size2 = 0;
    //size1 = buffDac->getSize() - count;
    //data1 = (uint8_t *)buffDac->getData() + count;
  }
  data1 = (uint8_t *)buffDac->getData() + count;
}

//-----------------------------------------------------------------------------
void HalDAC::Channel::unlock(uint32_t count){
  Assert(&dma);
  Assert(buffDac);

  buffDac->setCount(ringsum(buffDac->getCount(), (size_t)count, buffDac->getSize()));
}


