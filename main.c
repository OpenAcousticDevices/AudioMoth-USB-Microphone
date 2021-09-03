/****************************************************************************
 * main.c
 * openacousticdevices.info
 * August 2021
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "em_usb.h"

#include "audioMoth.h"
#include "microphone.h"
#include "digitalFilter.h"

/* Useful time constants */

#define MILLISECONDS_IN_SECOND                  1000

/* Useful type constant */

#define UINT32_SIZE_IN_BYTES                    4

/* Sleep and LED constants */

#define SHORT_LED_FLASH_DURATION                100
#define DEFAULT_WAIT_INTERVAL                   200

/* Main loop delay constants */

#define MAIN_LOOP_WAIT_INTERVAL                 10
#define DEFAULT_DELAY_INTERVAL                  100
#define MICROPHONE_CHANGE_INTERVAL              800

/* USB EM2 wake constant */

#define USB_EM2_RTC_WAKEUP_INTERVAL             10

/* Buffer constants */

#define NUMBER_OF_BUFFERS                       8
#define NUMBER_OF_BYTES_IN_SAMPLE               2
#define MAXIMUM_SAMPLES_IN_DMA_TRANSFER         (MAXIMUM_SAMPLE_RATE / 1000)
#define MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER       (MAXIMUM_SAMPLES_IN_DMA_TRANSFER * NUMBER_OF_BYTES_IN_SAMPLE)

/* DMA LED constant */

#define DMA_LED_ON_TIME                         40
#define AM_SD_CARD_BUFFER_SIZE                  32768
#define DMA_SAMPLES_PER_LED_PERIOD              (AM_SD_CARD_BUFFER_SIZE / NUMBER_OF_BYTES_IN_SAMPLE)

/* Digital filter constant */

#define FILTER_FREQ_MULTIPLIER                  100

/* DC filter constants */

#define LOW_DC_BLOCKING_FREQ                    8
#define DEFAULT_DC_BLOCKING_FREQ                48

/* USB configuration constant */

#define USB_CONFIG_TIME_CORRECTION              26

/* Energy saver mode constant */

#define ENERGY_SAVER_SAMPLE_RATE_THRESHOLD      48000

/* Default gain setting constant */

#define DEFAULT_GAIN_SETTING                    2

/* Switch debounce constant */

#define USB_DELAY_COUNT                         100
#define SWITCH_DEBOUNCE_COUNT                   10

/* Useful macros */

#define MAX(a, b)                               ((a) > (b) ? (a) : (b))

#define ROUND_UP_TO_MULTIPLE(a, b)              (((a) + (b) - 1) & ~((b)-1))

/* USB configuration data structure */

#pragma pack(push, 1)

typedef struct {
    uint32_t time;
    uint8_t gain;
    uint8_t clockDivider;
    uint8_t acquisitionCycles;
    uint8_t oversampleRate;
    uint32_t sampleRate;
    uint8_t sampleRateDivider;
    uint16_t lowerFilterFreq;
    uint16_t higherFilterFreq;
    uint8_t enableEnergySaverMode : 1; 
    uint8_t disable48HzDCBlockingFilter : 1;
} configSettings_t;

#pragma pack(pop)

static configSettings_t defaultConfigSettings = {
    .time = 0,
    .gain = 2,
    .clockDivider = 4,
    .acquisitionCycles = 16,
    .oversampleRate = 1,
    .sampleRate = 384000,
    .sampleRateDivider = 8,
    .lowerFilterFreq = 0,
    .higherFilterFreq = 0,
    .enableEnergySaverMode = 0,
    .disable48HzDCBlockingFilter = 0,
};

configSettings_t *configSettings = &defaultConfigSettings;

/* Persistent configuration data structure */

#pragma pack(push, 1)

typedef struct {
    uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH];
    uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH];
    configSettings_t configSettings;
} persistentConfigSettings_t;

#pragma pack(pop)

/* DMA transfer variable */

static uint32_t transferCount;

static uint32_t transferPeriod;

static uint32_t energySaverFactor;

static uint32_t numberOfSamplesInBuffer;

static uint32_t numberOfSamplesInDMATransfer;

/* DMA buffers */

static int16_t primaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

static int16_t secondaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

/* USB microphone variables */

static uint32_t tempBuffer;

static bool microphoneMuted;

static bool microphoneRestarting;

static uint16_t microphoneAlternative;

static volatile bool microphoneChanged;

/* Sample buffer variables */

static uint32_t writeBuffer;

static uint32_t writeBufferIndex;

static int16_t* buffers[NUMBER_OF_BUFFERS];

static uint8_t silentBuffer[MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER] __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

static uint8_t sampleBuffer[MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER * NUMBER_OF_BUFFERS] __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 0, 0};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-USB-Microphone";

/* Function prototypes */

int dataSentCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

int muteSettingReceivedCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

/* Set up buffers and microphone */

static void setUpMicrophone(bool useDefaultSetting) {

    /* Calculate effective sample rate */

    uint32_t effectiveSampleRate = configSettings->sampleRate / configSettings->sampleRateDivider;

    /* Calculate transfer period */

    transferPeriod = DMA_SAMPLES_PER_LED_PERIOD * MILLISECONDS_IN_SECOND / effectiveSampleRate;

    /* Calculate energy saver factor */

    bool energySaverMode = configSettings->enableEnergySaverMode && effectiveSampleRate <= ENERGY_SAVER_SAMPLE_RATE_THRESHOLD;

    energySaverFactor = !useDefaultSetting && energySaverMode ? 2 : 1;

    /* Calculate the number of samples in each buffer and DMA transfer */

    numberOfSamplesInBuffer = effectiveSampleRate / MILLISECONDS_IN_SECOND;

    numberOfSamplesInDMATransfer = configSettings->sampleRate / energySaverFactor / MILLISECONDS_IN_SECOND;

    /* Update USB configuration sample rate and buffer size */

    configDesc[SAMPLE_RATE_OFFSET] = (uint8_t)effectiveSampleRate;
    configDesc[SAMPLE_RATE_OFFSET + 1] = (uint8_t)(effectiveSampleRate >> 8);
    configDesc[SAMPLE_RATE_OFFSET + 2] = (uint8_t)(effectiveSampleRate >> 16);

    uint32_t usbBufferSize = numberOfSamplesInBuffer * NUMBER_OF_BYTES_IN_SAMPLE;

    configDesc[BUFFER_SIZE_OFFSET] = (uint8_t)usbBufferSize;
    configDesc[BUFFER_SIZE_OFFSET + 1] = (uint8_t)(usbBufferSize >> 8);

    /* Set up the digital filter */

    DigitalFilter_reset();

    uint32_t blockingFilterFrequency = !useDefaultSetting && configSettings->disable48HzDCBlockingFilter ? LOW_DC_BLOCKING_FREQ : DEFAULT_DC_BLOCKING_FREQ;

    if (useDefaultSetting || (configSettings->lowerFilterFreq == 0 && configSettings->higherFilterFreq == 0)) {

        DigitalFilter_designHighPassFilter(effectiveSampleRate, blockingFilterFrequency);

    } else if (configSettings->lowerFilterFreq == UINT16_MAX) {

        DigitalFilter_designBandPassFilter(effectiveSampleRate, blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    } else if (configSettings->higherFilterFreq == UINT16_MAX) {

        DigitalFilter_designHighPassFilter(effectiveSampleRate, MAX(blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq));

    } else {

        DigitalFilter_designBandPassFilter(effectiveSampleRate, MAX(blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq), FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    }

    /* Calculate the sample multiplier */

    float sampleMultiplier = 16.0f / (float)(configSettings->oversampleRate * configSettings->sampleRateDivider / energySaverFactor);

    DigitalFilter_applyAdditionalGain(sampleMultiplier);

}

void startMicrophoneSamples(bool useDefaultSetting) {

    AudioMoth_enableMicrophone(useDefaultSetting ? DEFAULT_GAIN_SETTING : configSettings->gain, configSettings->clockDivider, configSettings->acquisitionCycles, configSettings->oversampleRate);

    AudioMoth_initialiseDirectMemoryAccess(primaryBuffer, secondaryBuffer, numberOfSamplesInDMATransfer);

    AudioMoth_startMicrophoneSamples(configSettings->sampleRate / energySaverFactor);

}

/* AudioMoth interrupt handlers */

inline void AudioMoth_handleSwitchInterrupt() { }

inline void AudioMoth_handleMicrophoneInterrupt(int16_t sample) { }

inline void AudioMoth_handleMicrophoneChangeInterrupt() { 
    
    microphoneChanged = true; 
    
}

inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool isPrimaryBuffer, int16_t **nextBuffer) { 

    /* Update LED */

    transferCount += 1;

    AudioMoth_setRedLED(!microphoneRestarting && transferCount < DMA_LED_ON_TIME);

    if (transferCount > transferPeriod) transferCount = 0;

    /* Filter the microphone samples */

    int16_t *source = isPrimaryBuffer ? primaryBuffer : secondaryBuffer;

    DigitalFilter_filter(source, buffers[writeBuffer] + writeBufferIndex, configSettings->sampleRateDivider / energySaverFactor, numberOfSamplesInDMATransfer);

    /* Update the current buffer index and write buffer */

    writeBufferIndex += numberOfSamplesInDMATransfer / configSettings->sampleRateDivider * energySaverFactor;

    if (writeBufferIndex == numberOfSamplesInBuffer) {

        writeBufferIndex = 0;

        writeBuffer = (writeBuffer + 1) % NUMBER_OF_BUFFERS;

    }

    AudioMoth_setRedLED(false);

}

/* Provide data for the USB microphone */

static void stateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState) {

    if (newState == USBD_STATE_CONFIGURED) {

        USBD_Write(MICROPHONE_EP_IN, silentBuffer, numberOfSamplesInBuffer * NUMBER_OF_BYTES_IN_SAMPLE, dataSentCallback);

    } else if (oldState == USBD_STATE_CONFIGURED) {
    
        USBD_AbortTransfer(MICROPHONE_EP_IN);

    }

}

static int setupCmd(const USB_Setup_TypeDef *setup) {

    int retVal = USB_STATUS_REQ_UNHANDLED;

    uint8_t *buffer = (uint8_t*)&tempBuffer;

    if (setup->Type == USB_SETUP_TYPE_CLASS) {

        switch (setup->bRequest) {
      
            case USB_AUDIO_GET_CUR:
        
                if (setup->Direction == USB_SETUP_DIR_IN && setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE && setup->wLength == 1) {

                    if (setup->wIndex == 0x0200 && setup->wValue == 0x0100) {
            
                        *buffer = microphoneMuted;

                        retVal = USBD_Write(0, buffer, setup->wLength, NULL);
          
                    }
        
                }
        
                break;

            case USB_AUDIO_SET_CUR:
        
                if (setup->Direction == USB_SETUP_DIR_OUT && setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE && setup->wLength == 1) {
          
                    if (setup->wIndex == 0x0200 && setup->wValue == 0x0100) {
            
                        retVal = USBD_Read(0, buffer, setup->wLength, muteSettingReceivedCallback);
          
                    }
        
                }
        
                break;
    
        }
  
    } else if (setup->Type == USB_SETUP_TYPE_STANDARD) {

        if (setup->bRequest == SET_INTERFACE && setup->wIndex == 0x01 && setup->wLength == 0 && setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE) {
      
            if (setup->wValue == 0 || setup->wValue == 1) {
            
                microphoneAlternative = setup->wValue;
    
                retVal = USB_STATUS_OK;

            }

        } else if (setup->bRequest == GET_INTERFACE && setup->wValue == 0 && setup->wIndex == 0x01 && setup->wLength == 1 && setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE) {
      
            *buffer = (uint8_t)microphoneAlternative;
      
            retVal = USBD_Write(0, buffer, 1, NULL);

        }
        
    }

    return retVal;

}

/*  Callback on completion of USB microphone data send.  Used to send next data. */

int dataSentCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    if (status == USB_STATUS_OK) {

        uint32_t index = (writeBuffer + NUMBER_OF_BUFFERS - 2) % NUMBER_OF_BUFFERS;

        uint8_t *data = microphoneMuted || microphoneRestarting ? silentBuffer : (uint8_t*)buffers[index];

        USBD_Write(MICROPHONE_EP_IN, data, numberOfSamplesInBuffer * NUMBER_OF_BYTES_IN_SAMPLE, dataSentCallback);

    }

    return USB_STATUS_OK;

}

/* Callback on receipt of USB microphone mute message */

int muteSettingReceivedCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    microphoneMuted = (bool)(tempBuffer & 0xFF);

    return USB_STATUS_OK;

}

/* AudioMoth USB message handlers */

inline void AudioMoth_usbFirmwareVersionRequested(uint8_t **firmwareVersionPtr) {

    *firmwareVersionPtr = firmwareVersion;

}

inline void AudioMoth_usbFirmwareDescriptionRequested(uint8_t **firmwareDescriptionPtr) {

    *firmwareDescriptionPtr = firmwareDescription;

}

inline void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size) { 

    memcpy(transmitBuffer + 1, configSettings, sizeof(configSettings_t));

}

inline void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t* receiveBuffer, uint8_t *transmitBuffer, uint32_t size) {

    /* Make persistent configuration settings data structure */

    static persistentConfigSettings_t persistentConfigSettings __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

    memcpy(&persistentConfigSettings.firmwareVersion, &firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

    memcpy(&persistentConfigSettings.firmwareDescription, &firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

    memcpy(&persistentConfigSettings.configSettings, receiveBuffer + 1,  sizeof(configSettings_t));

    /* Copy persistent configuration settings to flash */

    uint32_t numberOfBytes = ROUND_UP_TO_MULTIPLE(sizeof(persistentConfigSettings_t), UINT32_SIZE_IN_BYTES);

    bool success = AudioMoth_writeToFlashUserDataPage((uint8_t*)&persistentConfigSettings, numberOfBytes);

    if (success) {

        /* Copy the USB packet contents to current configuration settings */

        memcpy(configSettings,  &persistentConfigSettings.configSettings, sizeof(configSettings_t));

        /* Copy the current configuration settings to the USB packet */

        memcpy(transmitBuffer + 1, configSettings, sizeof(configSettings_t)); 

        /* Set the time */

        AudioMoth_setTime(configSettings->time, USB_CONFIG_TIME_CORRECTION);

    } else {

        /* Return blank configuration as error indicator */

        memset(transmitBuffer + 1, 0, sizeof(configSettings_t));

    }

}

/* Main function */

int main(void) {

    /* Initialise device */

    AudioMoth_initialise();

    /* Check the persistent configuration */

    persistentConfigSettings_t *persistentConfigSettings = (persistentConfigSettings_t*)AM_FLASH_USER_DATA_ADDRESS;

    if (memcmp(persistentConfigSettings->firmwareVersion, firmwareVersion, AM_FIRMWARE_VERSION_LENGTH) == 0 && memcmp(persistentConfigSettings->firmwareDescription, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH) == 0) {

        memcpy(configSettings, &persistentConfigSettings->configSettings, sizeof(configSettings_t));

    }

    /* Read the switch state */

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (switchPosition == AM_SWITCH_USB) {

        /* Use conventional USB routine */

        AudioMoth_handleUSB();

    } else {

        /* Set up audio buffer */

        writeBuffer = 0;

        writeBufferIndex = 0;

        buffers[0] = (int16_t*)sampleBuffer;

        for (uint32_t i = 1; i < NUMBER_OF_BUFFERS; i += 1) {
            buffers[i] = buffers[i - 1] + MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER / NUMBER_OF_BYTES_IN_SAMPLE;
        }

        /* Enable the USB interface */

        AudioMoth_delay(DEFAULT_DELAY_INTERVAL);

        setUpMicrophone(switchPosition == AM_SWITCH_DEFAULT);

        startMicrophoneSamples(switchPosition == AM_SWITCH_DEFAULT);

        /* Configure data input pin */

        USBD_Init(&initstruct);

        /* Initial the real time clock and enter loop */

        bool cancel = false;

        uint32_t usbCounter = 0;

        uint32_t switchChangeCounter = 0;

        AudioMoth_startRealTimeClock(USB_EM2_RTC_WAKEUP_INTERVAL);

        while (!cancel) { 

            /* Check for switch change */

            AM_switchPosition_t currentSwitchPosition = AudioMoth_getSwitchPosition();

            /* Check for USB switch change */

            usbCounter = currentSwitchPosition == AM_SWITCH_USB ? usbCounter + 1 : 0;
 
            if (usbCounter > USB_DELAY_COUNT) {

                cancel = true;

            }

            /* Check if microphone settings change required */

            switchChangeCounter = ((switchPosition == AM_SWITCH_DEFAULT && currentSwitchPosition == AM_SWITCH_CUSTOM) \
                                    || (switchPosition == AM_SWITCH_CUSTOM && currentSwitchPosition == AM_SWITCH_DEFAULT)) ? switchChangeCounter + 1 : 0;

            if (switchChangeCounter > SWITCH_DEBOUNCE_COUNT) {

                /* Update switch position */

                switchPosition = currentSwitchPosition;

                /* Restart with current microphone settings */

                microphoneRestarting = true;

                AudioMoth_delay(DEFAULT_DELAY_INTERVAL);

                AudioMoth_disableMicrophone();

                AudioMoth_delay(DEFAULT_DELAY_INTERVAL);

                setUpMicrophone(switchPosition == AM_SWITCH_DEFAULT);

                startMicrophoneSamples(switchPosition == AM_SWITCH_DEFAULT);

                AudioMoth_delay(DEFAULT_DELAY_INTERVAL);

                microphoneRestarting = false;

                /* Reset counter */

                switchChangeCounter = 0;

            }

            /* Handle microphone change */

            if (microphoneChanged) {

                microphoneRestarting = true;

                AudioMoth_delay(DEFAULT_DELAY_INTERVAL);

                AudioMoth_disableMicrophone();

                AudioMoth_delay(MICROPHONE_CHANGE_INTERVAL);

                startMicrophoneSamples(switchPosition == AM_SWITCH_DEFAULT);

                AudioMoth_delay(DEFAULT_DELAY_INTERVAL);

                microphoneRestarting = false;

                microphoneChanged = false;

            }

            /* Check and handle time overflow */

            AudioMoth_checkAndHandleTimeOverflow();

            /* Enter low power standby if USB is unplugged */

            if (USBD_SafeToEnterEM2()) {
                
                /* Disable LED and microphone for deep sleep */

                AudioMoth_setRedLED(false);

                AudioMoth_disableMicrophone();

                /* Enter deep sleep (EM2) */

                AudioMoth_deepSleep();

                /* Restart microphone samples */

                startMicrophoneSamples(switchPosition == AM_SWITCH_DEFAULT);

            }

            /* Sleep until next interrupt */

            AudioMoth_delay(MAIN_LOOP_WAIT_INTERVAL);

        }

        /* Disable real time clock */

        AudioMoth_stopRealTimeClock();

        /* Disable USB */

        USBD_AbortAllTransfers();

        USBD_Stop();

        USBD_Disconnect();

        /* Disable the microphone */

        AudioMoth_disableMicrophone();

    }

    /* Turn off LED */

    AudioMoth_setBothLED(false);

    /* Power down */

    AudioMoth_powerDownAndWakeMilliseconds(DEFAULT_WAIT_INTERVAL);

}