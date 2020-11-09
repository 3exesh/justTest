/*!\file HalDAC.hpp
 *    \author iot@ektos.net (EKTOS A/S)
 *    \date created on Dec 7, 2018
 *    \version 1.0
 *    \brief description goes here
 *
 *    Belongs to Blocks project
 */
#ifndef SOURCE_HAL_STM32F4XX_STREAM_AUDIO_HALDAC_HPP_
#define SOURCE_HAL_STM32F4XX_STREAM_AUDIO_HALDAC_HPP_

/* Once the DAC channelx is enabled, the corresponding GPIO pin (PA4 or PA5) is
 * automatically connected to the analog converter output (DAC_OUTx).
 * In order to avoid parasitic consumption,
 * the PA4 or PA5 pin should first be configured to analog (AIN)*/

#include "HalDACManager.hpp"
#include "Stream/Audio/AudioStreams.hpp"
#include "HalTimerManager.hpp"
#include "HalI2SBus.hpp"

class HalDAC
{
public:
  enum DacMode{
    DacModeUsual       = 0,
    NoiseGeneration    = 1,
    TriangleGeneration = 2,
  };

  enum Channels{
    Channel1,
    Channel2,
    ChannelTotal
  };

  enum Trigger{
    Timer6TRGOevent = 0,
    Timer8TRGOevent = 1,
    Timer7TRGOevent = 2,
    Timer5TRGOevent = 3,
    Timer2TRGOevent = 4,
    Timer4TRGOevent = 5,
    ExternalLine9   = 6,
    SoftwareTrigger = 7
  };

//------------------------------------------------------------------------------
  class Channel: public AudioStreamOut, public IRQManager::Handler, public Runnable{
  public:
    Channel(HalDAC &halDac, HalTimerManager::TimerID id, DacMode mode, Channels num, bool buffDisabled);

  void             start();
  virtual void     stop();
  virtual void     play();
  void             InternalStop();
  void             setTimerParameters(uint32_t sampleFrequency);
  inline void      setBuff(Buffer &buff){buffDac = &buff;}
  void             prepare(HalI2SBus::AudioFormat *format);
  virtual uint32_t getFillState();
  void             InternalLock(void *&data1, size_t &size1, void *&data2, size_t &size2);
  virtual void     lock(void *&data1, size_t &size1, void *&data2, size_t &size2);
  virtual void     unlock(uint32_t count);


  virtual void run();


  private:
  virtual void processIRQ();
  const HalTimerManager::TimerDescriptor &timerDescriptor;
  HalDAC                                 &dac;
  HalDma                                  dma;
  Buffer                                 *buffDac;
  Channels                                idCh;
  bool                                    playFinish;
  uint32_t                                playFinishCounter;
  bool                                    playPending;
  Mutex                                   mutex;
  };

//------------------------------------------------------------------------------
  HalDAC(HalDACManager::DACID id): discriptor(HalDACManager::getDescriptor(id)){
    HalPower::moduleEnable(discriptor.power);
    HalPower::moduleReset(discriptor.power);
  }

private:
  friend class Channel;
  HalDACManager::Descriptor const &discriptor;
};



#endif /* SOURCE_HAL_STM32F4XX_STREAM_AUDIO_HALDAC_HPP_ */
