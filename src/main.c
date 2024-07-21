/****************************************************************************
 * main.c
 * openacousticdevices.info
 * August 2021
 *****************************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "em_usb.h"
#include "em_gpio.h"
#include "em_wdog.h"

#include "audiomoth.h"
#include "microphone.h"
#include "digitalfilter.h"

/* Useful time constants */

#define MILLISECONDS_IN_SECOND                  1000

/* Useful frequency constants */

#define HERTZ_IN_KILOHERTZ                      1000

/* Useful type constants */

#define BITS_PER_BYTE                           8
#define UINT32_SIZE_IN_BITS                     32
#define UINT32_SIZE_IN_BYTES                    4
#define CHAR16_SIZE_IN_BYTES                    2

/* Sleep and LED constants */

#define SHORT_LED_FLASH_DURATION                100
#define DEFAULT_WAIT_INTERVAL                   200

#define USB_CONFIGURATION_BLINK                 400

/* Main loop delay constants */

#define MAIN_LOOP_WAIT_INTERVAL                 10
#define DEFAULT_DELAY_INTERVAL                  100
#define MICROPHONE_CHANGE_INTERVAL              400

/* USB EM2 wake constant */

#define USB_EM2_RTC_WAKEUP_INTERVAL             10

/* Buffer constants */

#define NUMBER_OF_BUFFERS                       8
#define MINIMUM_BUFFER_SEPARATION               2
#define NUMBER_OF_BYTES_IN_SAMPLE               2
#define MAXIMUM_SAMPLES_IN_DMA_TRANSFER         (MAXIMUM_SAMPLE_RATE / 1000)
#define MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER       (MAXIMUM_SAMPLES_IN_DMA_TRANSFER * NUMBER_OF_BYTES_IN_SAMPLE)

/* DMA LED constant */

#define DMA_LED_ON_TIME                         20
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

/* Switch debounce constant */

#define USB_DELAY_COUNT                         100
#define SWITCH_DEBOUNCE_COUNT                   20

/* Debugging constants */

#define NUMBER_OF_SINE_BUFFERS                  (3 * NUMBER_OF_BUFFERS / 2)
#define GENERATE_SINE_WAVE                      false

/* USB descriptor constants */

#define USB_PRODUCT_STRING_DESCRIPTOR_INDEX     2
#define USB_SERIAL_STRING_DESCRIPTOR_INDEX      3

#define USB_STRING_DESCRIPTOR_OFFSET            2
#define USB_STRING_DESCRIPTOR_SIZE              128

#define USB_SERIAL_NUMBER_LENGTH                4
#define BITS_PER_BCD_DIGIT                      4

/* HID configuration constants */

#define HID_CONFIGURATION_MESSAGE               0x01
#define HID_UPDATE_GAIN_MESSAGE                 0x02
#define HID_SET_LED_MESSAGE                     0x03
#define HID_RESTORE_MESSAGE                     0x04
#define HID_READ_MESSAGE                        0x05
#define HID_PERSIST_MESSAGE                     0x06
#define HID_FIRMARE_MESSAGE                     0x07
#define HID_BOOTLOADER_MESSAGE                  0x08

/* Serial number constants */

#define SERIAL_NUMBER                           "%08X%08X"

#define FORMAT_SERIAL_NUMBER(src)               (unsigned int)*((uint32_t*)src + 1),  (unsigned int)*((uint32_t*)src)

/* Maths constants */

#ifndef M_PI
#define M_PI                                    3.14159265358979323846f
#endif

#ifndef M_TWOPI
#define M_TWOPI                                 (2.0f * M_PI)
#endif

/* Useful macros */

#define MAX(a, b)                               ((a) > (b) ? (a) : (b))

#define ROUND_UP_TO_MULTIPLE(a, b)              (((a) + (b) - 1) & ~((b)-1))

/* Configuration and LED enums */

typedef enum {LED_OFF, LED_FLASH, LED_SOLID} ledStatus_t;

typedef enum {OVERRIDE_NONE, OVERRIDE_GAIN, OVERRIDE_ALL} overrideOption_t;

typedef enum {ENUMERATE_NONE, ENUMERATE_BACKUP, ENUMERATE_PERSISTENT} enumerationRequired_t;

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
    uint8_t enableLowGainRange : 1;
    uint8_t disableLED : 1;
} configSettings_t;

#pragma pack(pop)

static configSettings_t defaultConfigSettings = {
    .time = 0,
    .gain = 2,
    .clockDivider = 4,
    .acquisitionCycles = 16,
    .oversampleRate = 1,
    .sampleRate = 384000,
    .sampleRateDivider = 1,
    .lowerFilterFreq = 0,
    .higherFilterFreq = 0,
    .enableEnergySaverMode = 0,
    .disable48HzDCBlockingFilter = 0,
    .enableLowGainRange = 0,
    .disableLED = 0
};

/* Main and back-up domain configuration and enumeration flag */

static uint32_t *enumerationRequired = (uint32_t*)AM_BACKUP_DOMAIN_START_ADDRESS;

static configSettings_t *configSettings = (configSettings_t*)&defaultConfigSettings; 

static configSettings_t *backupConfigSettings = (configSettings_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);

/* Persistent configuration data structure */

#pragma pack(push, 1)

typedef struct {
    uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH];
    uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH];
    configSettings_t configSettings;
} persistentConfigSettings_t;

#pragma pack(pop)

/* Persistent configuration */

static persistentConfigSettings_t persistentConfigSettings __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

/* USB descriptors */

static uint8_t productDescriptor[USB_STRING_DESCRIPTOR_SIZE] __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

static uint8_t serialDescriptor[USB_STRING_DESCRIPTOR_SIZE] __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

/* LED status */

ledStatus_t redLED = LED_OFF;

ledStatus_t greenLED = LED_OFF;

/* DMA transfer variable */

static uint32_t transferCount;

static uint32_t transferPeriod;

static uint32_t energySaverFactor;

static uint32_t numberOfSamplesInBuffer;

static uint32_t numberOfRawSamplesInDMATransfer;

/* DMA buffers */

static int16_t primaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

static int16_t secondaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

/* USB microphone variables */

static uint32_t tempBuffer;

static bool microphoneMuted;

static bool microphoneRestarting;

static uint16_t microphoneAlternative;

static volatile bool microphoneChanged;

static volatile bool microphoneHIDResponse;

static volatile bool enterSerialBootloader;

/* Configuration variables */

static volatile overrideOption_t overrideOption;

static volatile bool microphoneConfigurationChanged;

static volatile bool writePersistentConfiguration;

/* Sample buffer variables */

static uint32_t readBuffer;

static uint32_t writeBuffer;

static int16_t* buffers[NUMBER_OF_BUFFERS];

static uint8_t silentBuffer[MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER] __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

static uint8_t interpolationBuffer[MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER] __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

static uint8_t sampleBuffer[MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER * NUMBER_OF_BUFFERS] __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

/* Debugging sine buffer */

static uint32_t sineBufferCounter;

static int16_t* sineBuffers[NUMBER_OF_SINE_BUFFERS];

static uint8_t sineBuffer[MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER * NUMBER_OF_SINE_BUFFERS] __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 3, 1};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-USB-Microphone";

/* Microphone USB HID buffers */

SL_ALIGN(4) static uint8_t microphoneHIDReceiveBuffer[2 * MICROPHONE_HID_BUFFER_SIZE];

SL_ALIGN(4) static uint8_t microphoneHIDTransmitBuffer[2 * MICROPHONE_HID_BUFFER_SIZE];

/* Function prototypes */

int dataSentCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

int muteSettingReceivedCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

int dataSentMicrophoneHIDCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

int dataReceivedMicrophoneHIDCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

/* Functions of copy to and from the backup domain */

static void copyFromBackupDomain(uint8_t *dst, uint32_t *src, uint32_t length) {

    for (uint32_t i = 0; i < length; i += 1) {
        *(dst + i) = *((uint8_t*)src + i);
    }

}

static void copyToBackupDomain(uint32_t *dst, uint8_t *src, uint32_t length) {

    uint32_t value = 0;

    for (uint32_t i = 0; i < length / UINT32_SIZE_IN_BYTES; i += 1) {
        *(dst + i) = *((uint32_t*)src + i);
    }

    for (uint32_t i = 0; i < length % UINT32_SIZE_IN_BYTES; i += 1) {
        value = (value << BITS_PER_BYTE) + *(src + length - 1 - i);
    }

    if (length % UINT32_SIZE_IN_BYTES) *(dst + length / UINT32_SIZE_IN_BYTES) = value;

}

/* Set up buffers and microphone and start the microphone samples */

static void setupAndStartMicrophoneSamples(bool useDefaultSetting, bool useDefaultGainRange) {

    /* Calculate effective sample rate */

    uint32_t effectiveSampleRate = configSettings->sampleRate / configSettings->sampleRateDivider;

    /* Calculate transfer period */

    transferPeriod = DMA_SAMPLES_PER_LED_PERIOD * MILLISECONDS_IN_SECOND / effectiveSampleRate;

    /* Calculate energy saver factor */

    bool energySaverMode = configSettings->enableEnergySaverMode && effectiveSampleRate <= ENERGY_SAVER_SAMPLE_RATE_THRESHOLD;

    energySaverFactor = useDefaultSetting == false && energySaverMode ? 2 : 1;

    /* Calculate the number of samples in each buffer and DMA transfer */

    numberOfSamplesInBuffer = effectiveSampleRate / MILLISECONDS_IN_SECOND;

    numberOfRawSamplesInDMATransfer = configSettings->sampleRate / energySaverFactor / MILLISECONDS_IN_SECOND;

    /* Fill the sine buffer */

    if (GENERATE_SINE_WAVE) {

        for (uint32_t i = 0; i < NUMBER_OF_SINE_BUFFERS; i += 1) {

            for (uint32_t j = 0; j < numberOfSamplesInBuffer; j += 1) {

                sineBuffers[i][j] = (int16_t)roundf((float)INT16_MAX / 2.0f * sinf(M_TWOPI * (float)(i * numberOfSamplesInBuffer + j) / (float)NUMBER_OF_SINE_BUFFERS / (float)numberOfSamplesInBuffer));

            }

        }

    }

    /* Update USB device release for sample rate */

    deviceDesc.bcdDevice = 0;

    uint32_t sampleRate = effectiveSampleRate / HERTZ_IN_KILOHERTZ;

    for (uint32_t i = 0; i < USB_SERIAL_NUMBER_LENGTH; i += 1) {

        uint32_t digit = sampleRate % 10;

        deviceDesc.bcdDevice |= digit << (BITS_PER_BCD_DIGIT * i);

        sampleRate = (sampleRate - digit) / 10;

    }

    /* Update USB serial number for sample rate and device unique ID */

    for (uint32_t i = 0; i < USB_STRING_DESCRIPTOR_SIZE; i += 1) serialDescriptor[i] = 0;

    uint32_t length = sprintf((char*)serialDescriptor + USB_STRING_DESCRIPTOR_OFFSET, "%04lu_" SERIAL_NUMBER, effectiveSampleRate / HERTZ_IN_KILOHERTZ, FORMAT_SERIAL_NUMBER(AM_UNIQUE_ID_START_ADDRESS));

    serialDescriptor[0] = CHAR16_SIZE_IN_BYTES * length + CHAR16_SIZE_IN_BYTES;

    serialDescriptor[1] = USB_STRING_DESCRIPTOR;

    char *src = (char*)serialDescriptor + USB_STRING_DESCRIPTOR_OFFSET;

    char16_t *dst = (char16_t*)src;

    for (uint32_t i = 0; i < length; i += 1) dst[length - 1 - i] = src[length - 1 - i];

    strings[USB_SERIAL_STRING_DESCRIPTOR_INDEX] = serialDescriptor;

    /* Update USB string descriptor for sample rate */

    for (uint32_t i = 0; i < USB_STRING_DESCRIPTOR_SIZE; i += 1) productDescriptor[i] = 0;

    length = sprintf((char*)productDescriptor + USB_STRING_DESCRIPTOR_OFFSET, "%lukHz AudioMoth USB Microphone", effectiveSampleRate / HERTZ_IN_KILOHERTZ);

    productDescriptor[0] = CHAR16_SIZE_IN_BYTES * length + CHAR16_SIZE_IN_BYTES;

    productDescriptor[1] = USB_STRING_DESCRIPTOR;

    src = (char*)productDescriptor + USB_STRING_DESCRIPTOR_OFFSET;

    dst = (char16_t*)src;

    for (uint32_t i = 0; i < length; i += 1) dst[length - 1 - i] = src[length - 1 - i];

    strings[USB_PRODUCT_STRING_DESCRIPTOR_INDEX] = productDescriptor;

    /* Update USB configuration sample rate and buffer size */

    uint32_t usbBufferSize = numberOfSamplesInBuffer * NUMBER_OF_BYTES_IN_SAMPLE;

    configDesc[SAMPLE_RATE_OFFSET] = (uint8_t)effectiveSampleRate;
    configDesc[SAMPLE_RATE_OFFSET + 1] = (uint8_t)(effectiveSampleRate >> 8);
    configDesc[SAMPLE_RATE_OFFSET + 2] = (uint8_t)(effectiveSampleRate >> 16);

    configDesc[BUFFER_SIZE_OFFSET] = (uint8_t)usbBufferSize;
    configDesc[BUFFER_SIZE_OFFSET + 1] = (uint8_t)(usbBufferSize >> 8);

    /* Set up the digital filter */

    DigitalFilter_reset();

    uint32_t blockingFilterFrequency = useDefaultSetting == false && configSettings->disable48HzDCBlockingFilter ? LOW_DC_BLOCKING_FREQ : DEFAULT_DC_BLOCKING_FREQ;

    if (useDefaultSetting || (configSettings->lowerFilterFreq == 0 && configSettings->higherFilterFreq == 0)) {

        DigitalFilter_designHighPassFilter(effectiveSampleRate, blockingFilterFrequency);

    } else if (configSettings->lowerFilterFreq == UINT16_MAX) {

        DigitalFilter_designBandPassFilter(effectiveSampleRate, blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    } else if (configSettings->higherFilterFreq == UINT16_MAX) {

        DigitalFilter_designHighPassFilter(effectiveSampleRate, MAX(blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq));

    } else {

        DigitalFilter_designBandPassFilter(effectiveSampleRate, MAX(blockingFilterFrequency, FILTER_FREQ_MULTIPLIER * configSettings->lowerFilterFreq), FILTER_FREQ_MULTIPLIER * configSettings->higherFilterFreq);

    }

    /* Initialise the counters */
    
    readBuffer = 0;

    writeBuffer = NUMBER_OF_BUFFERS / 2;

    if (GENERATE_SINE_WAVE) sineBufferCounter = 0;

    /* Clear the buffers */

    memset(sampleBuffer, 0, sizeof(sampleBuffer));

    /* Enable the microphone and initialise direct memory access */

    AM_gainRange_t gainRange = useDefaultGainRange ? AM_NORMAL_GAIN_RANGE : configSettings->enableLowGainRange ? AM_LOW_GAIN_RANGE : AM_NORMAL_GAIN_RANGE;

    bool externalMicrophone = AudioMoth_enableMicrophone(gainRange, configSettings->gain, configSettings->clockDivider, configSettings->acquisitionCycles, configSettings->oversampleRate);

    AudioMoth_initialiseDirectMemoryAccess(primaryBuffer, secondaryBuffer, numberOfRawSamplesInDMATransfer);

    /* Calculate the sample multiplier */

    float sampleMultiplier = 16.0f / (float)(configSettings->oversampleRate * configSettings->sampleRateDivider / energySaverFactor);

    if (AudioMoth_hasInvertedOutput()) sampleMultiplier = -sampleMultiplier;

    if (externalMicrophone) sampleMultiplier = -sampleMultiplier;

    DigitalFilter_applyAdditionalGain(sampleMultiplier);

    /* Start the samples */

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

    bool redLEDState = microphoneRestarting == false && (redLED == LED_SOLID || (redLED == LED_FLASH && transferCount < DMA_LED_ON_TIME));

    bool greenLEDState = microphoneRestarting == false && (greenLED == LED_SOLID || (greenLED == LED_FLASH && transferCount < DMA_LED_ON_TIME));

    AudioMoth_setRedLED(redLEDState);

    AudioMoth_setGreenLED(greenLEDState);

    if (transferCount > transferPeriod) transferCount = 0;

    /* Filter the microphone samples */

    int16_t *source = isPrimaryBuffer ? primaryBuffer : secondaryBuffer;

    DigitalFilter_filter(source, buffers[writeBuffer], configSettings->sampleRateDivider / energySaverFactor, numberOfRawSamplesInDMATransfer);

    /* Update the current buffer index and write buffer */

    if (GENERATE_SINE_WAVE) {

        memcpy(buffers[writeBuffer], sineBuffers[sineBufferCounter], MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER);

        sineBufferCounter = (sineBufferCounter + 1) % NUMBER_OF_SINE_BUFFERS;

    }

    writeBuffer = (writeBuffer + 1) % NUMBER_OF_BUFFERS;

}

/* Callback on completion of USB state change */

static void stateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState) {

    if (newState == USBD_STATE_CONFIGURED) {

        USBD_Write(MICROPHONE_EP_IN, silentBuffer, numberOfSamplesInBuffer * NUMBER_OF_BYTES_IN_SAMPLE, dataSentCallback);
        
        USBD_Read(HID_EP_OUT, microphoneHIDReceiveBuffer, MICROPHONE_HID_BUFFER_SIZE, dataReceivedMicrophoneHIDCallback);

    } else if (oldState == USBD_STATE_CONFIGURED) {

        USBD_AbortTransfer(MICROPHONE_EP_IN);

        USBD_AbortTransfer(HID_EP_OUT);

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

        } else if (setup->bRequest == GET_DESCRIPTOR) {
        
            switch (setup->wValue >> 8) {

                case USB_HID_REPORT_DESCRIPTOR:

                    USBD_Write(0x00, (void*)HID_ReportDescriptor, SL_MIN(sizeof(HID_ReportDescriptor), setup->wLength), NULL);

                    retVal = USB_STATUS_OK;

                    break;

                case USB_HID_DESCRIPTOR:

                    USBD_Write(0x00, (void*)HID_Descriptor, SL_MIN(sizeof(HID_Descriptor), setup->wLength), NULL);

                    retVal = USB_STATUS_OK;

                    break;

            }
        
        }
    
    }

    return retVal;

}

/*  Callback on completion of USB microphone data send. Used to send next data. */

int dataSentCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    if (status == USB_STATUS_OK) {

        uint8_t *data;

        uint32_t bufferLag = (writeBuffer + NUMBER_OF_BUFFERS - readBuffer) % NUMBER_OF_BUFFERS;

        if (bufferLag < MINIMUM_BUFFER_SEPARATION || bufferLag >= NUMBER_OF_BUFFERS - MINIMUM_BUFFER_SEPARATION) {

            /* Get the previous sample sent */

            uint32_t previousIndex = (readBuffer + NUMBER_OF_BUFFERS - 1) % NUMBER_OF_BUFFERS;

            int16_t previousSample = buffers[previousIndex][numberOfSamplesInBuffer - 1];

            int16_t *interpolationArray = (int16_t*)interpolationBuffer;

            data = (uint8_t*)interpolationArray;

            if (bufferLag < MINIMUM_BUFFER_SEPARATION) {

                /* Generate additional spacer buffer by repeating the last sample sent */

                for (uint32_t i = 0; i < numberOfSamplesInBuffer; i += 1) {
                    
                    interpolationArray[i] = previousSample;

                }

            } else {

                /* Skip buffer and interpolate across the gap */

                uint32_t lastIndex = (readBuffer + 1) % NUMBER_OF_BUFFERS;

                int16_t lastSample = buffers[lastIndex][numberOfSamplesInBuffer - 1];

                float step = ((float)lastSample - (float)previousSample) / (float)numberOfSamplesInBuffer;

                float interpolatedValue = (float)previousSample + step;

                for (uint32_t i = 0; i < numberOfSamplesInBuffer; i += 1) {
                    
                    interpolationArray[i] = (int16_t)interpolatedValue;

                    interpolatedValue += step;

                }

                readBuffer = (readBuffer + 2) % NUMBER_OF_BUFFERS;

            }

        } else {

            /* Select the appropriate source */

            data = (uint8_t*)buffers[readBuffer];

            readBuffer = (readBuffer + 1) % NUMBER_OF_BUFFERS;


        }

        /* Send the data to USB */

        data = microphoneMuted || microphoneRestarting ? silentBuffer : data;

        USBD_Write(MICROPHONE_EP_IN, data, numberOfSamplesInBuffer * NUMBER_OF_BYTES_IN_SAMPLE, dataSentCallback);

    }

    return USB_STATUS_OK;

}

/* Callback on receipt of USB microphone mute message */

int muteSettingReceivedCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    microphoneMuted = (bool)(tempBuffer & 0xFF);

    return USB_STATUS_OK;

}

/* Callbacks on completion of microphone HID data send and receive */

int dataSentMicrophoneHIDCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    USBD_Read(HID_EP_OUT, microphoneHIDReceiveBuffer, MICROPHONE_HID_BUFFER_SIZE, dataReceivedMicrophoneHIDCallback);

    return USB_STATUS_OK;

}

int dataReceivedMicrophoneHIDCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining) {

    /* Clear transmit buffer and copy message type */

    memset(microphoneHIDTransmitBuffer, 0, MICROPHONE_HID_BUFFER_SIZE);

    /* Get a pointer to the received configuration */

    configSettings_t *receivedConfiguration = (configSettings_t*)(microphoneHIDReceiveBuffer + 1);

    /* Configure microphone */

    if (microphoneHIDReceiveBuffer[0] == HID_CONFIGURATION_MESSAGE) {

        /* Confirm message received */

        microphoneHIDTransmitBuffer[0] = microphoneHIDReceiveBuffer[0];

        /* Copy received configuration to transmit buffer */

        memcpy(microphoneHIDTransmitBuffer + 1, receivedConfiguration, sizeof(configSettings_t));

        /* Reset enable LED option */

        receivedConfiguration->disableLED = configSettings->disableLED;

        /* Copy configuration and set flags */

        bool enumerate = receivedConfiguration->sampleRate != configSettings->sampleRate || receivedConfiguration->sampleRateDivider != configSettings->sampleRateDivider;

        if (enumerate) {

            copyToBackupDomain((uint32_t*)backupConfigSettings, (uint8_t*)receivedConfiguration, sizeof(configSettings_t));

            *enumerationRequired = ENUMERATE_BACKUP;

        } else {

            memcpy(configSettings, receivedConfiguration, sizeof(configSettings_t));

        }

        microphoneConfigurationChanged = true;

        overrideOption = OVERRIDE_ALL;

    } else if (microphoneHIDReceiveBuffer[0] == HID_UPDATE_GAIN_MESSAGE) {

        /* Confirm message received */

        microphoneHIDTransmitBuffer[0] = microphoneHIDReceiveBuffer[0];

        /* Copy received configuration to transmit buffer */

        memcpy(microphoneHIDTransmitBuffer + 1, receivedConfiguration, sizeof(configSettings_t));

        /* Copy configuration and set flags */

        configSettings->gain = receivedConfiguration->gain;

        configSettings->enableLowGainRange = receivedConfiguration->enableLowGainRange;

        microphoneConfigurationChanged = true;

        if (overrideOption == OVERRIDE_NONE) overrideOption = OVERRIDE_GAIN;

    } else if (microphoneHIDReceiveBuffer[0] == HID_SET_LED_MESSAGE) {

        /* Confirm message received */

        microphoneHIDTransmitBuffer[0] = microphoneHIDReceiveBuffer[0];

        /* Copy received configuration to transmit buffer */

        memcpy(microphoneHIDTransmitBuffer + 1, receivedConfiguration, sizeof(configSettings_t));

        /* Copy led state and set flags */

        configSettings->disableLED = receivedConfiguration->disableLED;

        microphoneConfigurationChanged = true;

    } else if (microphoneHIDReceiveBuffer[0] == HID_RESTORE_MESSAGE) {

        /* Confirm message received */

        microphoneHIDTransmitBuffer[0] = microphoneHIDReceiveBuffer[0];

        /* Copy configuration from persistent storage and set flags */

        persistentConfigSettings_t *persistentConfigSettings = (persistentConfigSettings_t*)AM_FLASH_USER_DATA_ADDRESS;

        bool enumerate = persistentConfigSettings->configSettings.sampleRate != configSettings->sampleRate || persistentConfigSettings->configSettings.sampleRateDivider != configSettings->sampleRateDivider;

        if (enumerate) {
            
            *enumerationRequired = ENUMERATE_PERSISTENT;

        } else {

            memcpy(configSettings, &persistentConfigSettings->configSettings, sizeof(configSettings_t));

        }

        microphoneConfigurationChanged = true;

        overrideOption = OVERRIDE_NONE;

    } else if (microphoneHIDReceiveBuffer[0] == HID_READ_MESSAGE) {

        /* Confirm message received */

        microphoneHIDTransmitBuffer[0] = microphoneHIDReceiveBuffer[0];

        /* Copy current configuration to transmit buffer */

        memcpy(microphoneHIDTransmitBuffer + 1, configSettings, sizeof(configSettings_t));

    } else if (microphoneHIDReceiveBuffer[0] == HID_PERSIST_MESSAGE) {

        /* Confirm message received */

        microphoneHIDTransmitBuffer[0] = microphoneHIDReceiveBuffer[0];

        /* Set flags */

        writePersistentConfiguration = true;

        microphoneConfigurationChanged = true;

        overrideOption = OVERRIDE_NONE;

    } else if (microphoneHIDReceiveBuffer[0] == HID_FIRMARE_MESSAGE) {

        /* Confirm message received */

        microphoneHIDTransmitBuffer[0] = microphoneHIDReceiveBuffer[0];

        /* Copy firmware version and description to transmit buffer */

        memcpy(microphoneHIDTransmitBuffer + 1, firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

        memcpy(microphoneHIDTransmitBuffer + 1 + AM_FIRMWARE_VERSION_LENGTH, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

    } else if (microphoneHIDReceiveBuffer[0] == HID_BOOTLOADER_MESSAGE) {

        /* Confirm message received */

        microphoneHIDTransmitBuffer[0] = microphoneHIDReceiveBuffer[0];

        /* Set flag */

        enterSerialBootloader = true;

    }

    /* Send the response */

    USBD_Write(HID_EP_IN, microphoneHIDTransmitBuffer, MICROPHONE_HID_BUFFER_SIZE, dataSentMicrophoneHIDCallback);

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

    /* Get the received configuration */

    configSettings_t *receivedConfiguration = (configSettings_t*)(receiveBuffer + 1);

    /* Make persistent configuration settings data structure */

    memcpy(&persistentConfigSettings.firmwareVersion, &firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

    memcpy(&persistentConfigSettings.firmwareDescription, &firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

    memcpy(&persistentConfigSettings.configSettings, receivedConfiguration, sizeof(configSettings_t));

    /* Copy received configuration settings to flash */

    uint32_t numberOfBytes = ROUND_UP_TO_MULTIPLE(sizeof(persistentConfigSettings_t), UINT32_SIZE_IN_BYTES);

    bool success = AudioMoth_writeToFlashUserDataPage((uint8_t*)&persistentConfigSettings, numberOfBytes);

    if (success) {

        /* Copy received configuration to the current configuration */

        memcpy(configSettings, &persistentConfigSettings.configSettings, sizeof(configSettings_t));

        /* Copy the current configuration settings to the USB packet */

        memcpy(transmitBuffer + 1, &persistentConfigSettings.configSettings, sizeof(configSettings_t));

        /* Set the time */

        AudioMoth_setTime(configSettings->time, USB_CONFIG_TIME_CORRECTION);

        /* Blink the green LED */

        AudioMoth_blinkDuringUSB(USB_CONFIGURATION_BLINK);

    } else {

        /* Return blank configuration as error indicator */

        memset(transmitBuffer + 1, 0, sizeof(configSettings_t));

    }

}

/* Function to determine LED colours */

void setLED(AM_switchPosition_t switchPosition) {

    if (configSettings->disableLED) {

        redLED = LED_OFF;
        
        greenLED = LED_OFF;

    } else if (overrideOption == OVERRIDE_NONE) {

        redLED = switchPosition == AM_SWITCH_CUSTOM ? LED_FLASH : LED_OFF;

        greenLED = switchPosition == AM_SWITCH_DEFAULT ? LED_FLASH : LED_OFF;

    } else if (overrideOption == OVERRIDE_GAIN) {

        persistentConfigSettings_t *persistentConfigSettings = (persistentConfigSettings_t*)AM_FLASH_USER_DATA_ADDRESS;

        bool configurationChange = configSettings->gain != persistentConfigSettings->configSettings.gain || configSettings->enableLowGainRange != persistentConfigSettings->configSettings.enableLowGainRange;

        if (switchPosition == AM_SWITCH_CUSTOM) {

            redLED = LED_FLASH;

            greenLED = configurationChange ? LED_SOLID : LED_OFF;

        } else {

            redLED = configurationChange ? LED_SOLID : LED_OFF;

            greenLED = LED_FLASH;

        }

    } else {

        redLED = LED_FLASH;

        greenLED = LED_FLASH;

    }

}

/* Function to restart microphone samples */

void restartMicrophoneSamples(AM_switchPosition_t switchPosition, uint32_t delay) {

    /* Stop the LED */

    redLED = LED_OFF;

    greenLED = LED_OFF;

    AudioMoth_setBothLED(false);

    /* Restart with current microphone settings */

    microphoneRestarting = true;

    AudioMoth_disableMicrophone();

    if (writePersistentConfiguration) {

        /* Make persistent configuration settings data structure */

        memcpy(&persistentConfigSettings.firmwareVersion, &firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

        memcpy(&persistentConfigSettings.firmwareDescription, &firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

        memcpy(&persistentConfigSettings.configSettings, configSettings, sizeof(configSettings_t));

        /* Copy current settings to flash */

        uint32_t numberOfBytes = ROUND_UP_TO_MULTIPLE(sizeof(persistentConfigSettings_t), UINT32_SIZE_IN_BYTES);

        AudioMoth_writeToFlashUserDataPage((uint8_t*)&persistentConfigSettings, numberOfBytes);

        /* Reset flag */

        writePersistentConfiguration = false;
   
    }

    AudioMoth_delay(delay);

    bool useDefaultSetting = (overrideOption == OVERRIDE_NONE || overrideOption == OVERRIDE_GAIN) && switchPosition == AM_SWITCH_DEFAULT;

    bool useDefaultGainRange = overrideOption == OVERRIDE_NONE && switchPosition == AM_SWITCH_DEFAULT;

    setupAndStartMicrophoneSamples(useDefaultSetting, useDefaultGainRange);

    AudioMoth_delay(DEFAULT_DELAY_INTERVAL);

    microphoneRestarting = false;

    /* Restart the LED */

    setLED(switchPosition);

}

/* Main function */

int main(void) {

    /* Initialise device */

    AudioMoth_initialise();

    /* Reset enumeration flag if this is an initial power up */

    if (AudioMoth_isInitialPowerUp()) *enumerationRequired = ENUMERATE_NONE;

    /* Restore the backup or persistent configuration */

    if (*enumerationRequired == ENUMERATE_BACKUP) {

        copyFromBackupDomain((uint8_t*)configSettings, (uint32_t*)backupConfigSettings, sizeof(configSettings_t));

        overrideOption = OVERRIDE_ALL;

    } else {

        persistentConfigSettings_t *persistentConfigSettings = (persistentConfigSettings_t*)AM_FLASH_USER_DATA_ADDRESS;

        if (memcmp(persistentConfigSettings->firmwareVersion, firmwareVersion, AM_FIRMWARE_VERSION_LENGTH) == 0 && memcmp(persistentConfigSettings->firmwareDescription, firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH) == 0) {

            /* Copy persistent configuration to current configuration */

            memcpy(configSettings, &persistentConfigSettings->configSettings, sizeof(configSettings_t));

        } else {

            /* Copy default configuration settings to persistent configuration */

            static persistentConfigSettings_t persistentConfigSettings __attribute__ ((aligned(UINT32_SIZE_IN_BYTES)));

            memcpy(&persistentConfigSettings.firmwareVersion, &firmwareVersion, AM_FIRMWARE_VERSION_LENGTH);

            memcpy(&persistentConfigSettings.firmwareDescription, &firmwareDescription, AM_FIRMWARE_DESCRIPTION_LENGTH);

            memcpy(&persistentConfigSettings.configSettings, configSettings, sizeof(configSettings_t));

            uint32_t numberOfBytes = ROUND_UP_TO_MULTIPLE(sizeof(persistentConfigSettings_t), UINT32_SIZE_IN_BYTES);

            AudioMoth_writeToFlashUserDataPage((uint8_t*)&persistentConfigSettings, numberOfBytes);

        }

        overrideOption = OVERRIDE_NONE;

    }

    /* Reset enumeration flag after possibly restoring configuration */

    *enumerationRequired = ENUMERATE_NONE;

    /* Respond to switch state */

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (switchPosition == AM_SWITCH_USB) {

        /* Use conventional USB routine */

        AudioMoth_handleUSB();

        /* Power down */

        AudioMoth_powerDownAndWakeMilliseconds(DEFAULT_WAIT_INTERVAL);

    }

    /* Set up audio buffers */

    buffers[0] = (int16_t*)sampleBuffer;

    for (uint32_t i = 1; i < NUMBER_OF_BUFFERS; i += 1) {
        buffers[i] = buffers[i - 1] + MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER / NUMBER_OF_BYTES_IN_SAMPLE;
    }

    /* Set up debug sine buffers */

    if (GENERATE_SINE_WAVE) {

        sineBuffers[0] = (int16_t*)sineBuffer;

        for (uint32_t i = 1; i < NUMBER_OF_SINE_BUFFERS; i += 1) {
            sineBuffers[i] = sineBuffers[i - 1] + MAXIMUM_NUMBER_OF_BYTES_IN_BUFFER / NUMBER_OF_BYTES_IN_SAMPLE;
        }

    }

    /* Set LED colour */

    setLED(switchPosition);

    /* Start the microphone samples */

    AudioMoth_delay(DEFAULT_DELAY_INTERVAL);

    bool useDefaultSetting = (overrideOption == OVERRIDE_NONE || overrideOption == OVERRIDE_GAIN) && switchPosition == AM_SWITCH_DEFAULT;

    bool useDefaultGainRange = overrideOption == OVERRIDE_NONE && switchPosition == AM_SWITCH_DEFAULT;

    setupAndStartMicrophoneSamples(useDefaultSetting, useDefaultGainRange);

    /* Enable the USB interface */

    USBD_Init(&initstruct);

    /* Initialise the real time clock and enter loop */

    bool cancel = false;

    uint32_t usbCounter = 0;

    uint32_t switchChangeCounter = 0;

    enterSerialBootloader = false;

    microphoneHIDResponse = false;

    AudioMoth_startRealTimeClock(USB_EM2_RTC_WAKEUP_INTERVAL);

    while (cancel == false && enterSerialBootloader == false) { 

        /* Check for switch change */

        AM_switchPosition_t currentSwitchPosition = AudioMoth_getSwitchPosition();

        /* Check for switch change to USB/OFF or configuration change requiring re-enumeration  and then exit the loop */

        usbCounter = currentSwitchPosition == AM_SWITCH_USB || (microphoneConfigurationChanged && *enumerationRequired != ENUMERATE_NONE) ? usbCounter + 1 : 0;

        if (usbCounter > USB_DELAY_COUNT) cancel = true;

        /* Check for switch change between DEFAULT and CUSTOM */

        switchChangeCounter = ((switchPosition == AM_SWITCH_DEFAULT && currentSwitchPosition == AM_SWITCH_CUSTOM) \
                                || (switchPosition == AM_SWITCH_CUSTOM && currentSwitchPosition == AM_SWITCH_DEFAULT)) ? switchChangeCounter + 1 : 0;

        if (switchChangeCounter > SWITCH_DEBOUNCE_COUNT) {

            /* Update switch position */

            switchPosition = currentSwitchPosition;

            /* Restart the microphone samples */

            restartMicrophoneSamples(switchPosition, DEFAULT_DELAY_INTERVAL);

            /* Reset counter */

            switchChangeCounter = 0;

        }

        /* Handle microphone change */

        if (microphoneChanged) {

            /* Restart the microphone samples */

            restartMicrophoneSamples(switchPosition, MICROPHONE_CHANGE_INTERVAL);

            /* Reset flag */

            microphoneChanged = false;

        }

        /* Handle configuration change that does not require re-enumeration */

        if (microphoneConfigurationChanged && *enumerationRequired == ENUMERATE_NONE) {

            /* Restart the microphone samples */

            restartMicrophoneSamples(switchPosition, DEFAULT_DELAY_INTERVAL);

            /* Reset flag */

            microphoneConfigurationChanged = false;

        }

        /* Handle persistent write */

        if (writePersistentConfiguration) cancel = true;

        /* Check and handle time overflow */

        AudioMoth_checkAndHandleTimeOverflow();

        /* Enter low power standby if USB is unplugged */

        if (USBD_SafeToEnterEM2()) {
            
            /* Disable LED and microphone for deep sleep */

            AudioMoth_disableMicrophone();

            AudioMoth_setBothLED(false);

            /* Enter deep sleep (EM2) */

            AudioMoth_deepSleep();

            /* Restart microphone samples */

            bool useDefaultSetting = (overrideOption == OVERRIDE_NONE || overrideOption == OVERRIDE_GAIN) && switchPosition == AM_SWITCH_DEFAULT;

            bool useDefaultGainRange = overrideOption == OVERRIDE_NONE && switchPosition == AM_SWITCH_DEFAULT;

            setupAndStartMicrophoneSamples(useDefaultSetting, useDefaultGainRange);

        }

        /* Sleep until next interrupt */

        AudioMoth_delay(MAIN_LOOP_WAIT_INTERVAL);

    }

    /* Disable the microphone */

    AudioMoth_disableMicrophone();

    /* Ensure last USB message has been sent */

    if (enterSerialBootloader) AudioMoth_delay(DEFAULT_DELAY_INTERVAL);

    /* Disconnect USB */

    USBD_Disconnect();

    /* Disable real time clock */

    AudioMoth_stopRealTimeClock();

    /* Turn off LED */

    AudioMoth_setBothLED(false);

    /* Enter bootloader */

    if (enterSerialBootloader) {

        /* Disable watch dog timer */

        WDOG_Enable(false);

        /* Pull bootloader pin high */

        GPIO->ROUTE &= ~GPIO_ROUTE_SWCLKPEN;

        GPIO_PinModeSet(gpioPortF, 0, gpioModePushPull, 1);

        /* Jump to bootloader */

        __asm (

            /* Define the bootloader and vector table addresses */

            ".equ BOOTLOADER_ADDRESS, 0x00000000\n\t"

            ".equ SCB_VTOR, (0xE000E000 + 0x0D00 + 0x008)\n\t"

            /* Load the bootloader address */

            "ldr r0, =BOOTLOADER_ADDRESS\n\t"

            /* Set the vector table */

            "ldr r1, =SCB_VTOR\n\t"
            "str r0, [r1]\n\t"

            /* Set the stack pointer */

            "ldr r1, [r0]\n\t"
            "msr msp, r1\n\t"
            "msr psp, r1\n\t"

            /* Jump into the bootloader */

            "ldr r1, [r0, #4]\n\t"
            "mov pc, r1\n\t"

        );

    }

    /* Power down */

    AudioMoth_powerDownAndWakeMilliseconds(DEFAULT_WAIT_INTERVAL);

}