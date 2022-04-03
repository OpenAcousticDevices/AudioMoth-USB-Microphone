/****************************************************************************
 * microphone.h
 * openacousticdevices.info
 * August 2021
 *****************************************************************************/

#ifndef __MICROPHONE_H
#define __MICROPHONE_H

/* USB microphone constants */

#define USB_EP0_SIZE            64

#define MICROPHONE_EP_IN        0x81

/* Configuration descriptor constants */

#define USB_AC_INT_HEADER_DESCSIZE              9
#define USB_FEATURE_UNIT_DESCSIZE               9
#define USB_MONO_FORMAT_DESCSIZE                11

#define USB_AC_ITF_DESCSIZE                     (USB_AC_INT_HEADER_DESCSIZE + USB_CA_INPUT_TERMINAL_DESCSIZE + USB_FEATURE_UNIT_DESCSIZE + USB_CA_OUTPUT_TERMINAL_DESCSIZE)

#define USB_MICROPHONE_DESCSIZE                 (USB_CONFIG_DESCSIZE + USB_INTERFACE_DESCSIZE + USB_AC_ITF_DESCSIZE + (2 * USB_INTERFACE_DESCSIZE) + USB_CA_AS_GENERAL_DESCSIZE \
                                                  + USB_MONO_FORMAT_DESCSIZE + USB_CA_STD_AS_ENDPOINT_DESCSZIE + USB_CA_EP_GENERAL_DESCSIZE)

/* USB configuration constants */

#define MAXIMUM_SAMPLE_RATE     384000

#define MAXIMUM_BUFFER_SIZE     (MAXIMUM_SAMPLE_RATE * 2 / 1000)

#define SAMPLE_RATE_OFFSET      (USB_MICROPHONE_DESCSIZE - USB_CA_EP_GENERAL_DESCSIZE - USB_CA_STD_AS_ENDPOINT_DESCSZIE - 3)

#define BUFFER_SIZE_OFFSET      (USB_MICROPHONE_DESCSIZE - USB_CA_EP_GENERAL_DESCSIZE - 5)

/* Device descriptor */

SL_ALIGN(4)
static const USB_DeviceDescriptor_TypeDef deviceDesc __attribute__ ((aligned(4)))= {
    .bLength            = USB_DEVICE_DESCSIZE,            /* Size of the Descriptor in Bytes          */
    .bDescriptorType    = USB_DEVICE_DESCRIPTOR,          /* Device Descriptor type                   */
    .bcdUSB             = 0x0200,                         /* USB 2.0 compliant                        */
    .bDeviceClass       = 0x00,                           /* Vendor unique device                     */
    .bDeviceSubClass    = 0x00,                           /* Ignored for vendor unique device         */
    .bDeviceProtocol    = 0x00,                           /* Ignored for vendor unique device         */
    .bMaxPacketSize0    = USB_EP0_SIZE,                   /* Max packet size for EP0                  */
    .idVendor           = 0x10C4,                         /* VID                                      */
    .idProduct          = 0x0002,                         /* PID                                      */
    .bcdDevice          = 0x0000,                         /* Device Release number                    */
    .iManufacturer      = 0x01,                           /* Index of Manufacturer String Descriptor  */
    .iProduct           = 0x02,                           /* Index of Product String Descriptor       */
    .iSerialNumber      = 0x03,                           /* Index of Serial Number String Descriptor */
    .bNumConfigurations = 0x01                            /* Number of Possible Configurations        */

};

/* Configuration descriptor */

SL_ALIGN(4)
static uint8_t configDesc[] __attribute__ ((aligned(4)))= {

    /* Configuration descriptor */

    USB_CONFIG_DESCSIZE,                        /* bLength                             */
    USB_CONFIG_DESCRIPTOR,                      /* bDescriptorType                     */
    (uint8_t)USB_MICROPHONE_DESCSIZE,           /* wTotalLength (LSB)                  */
    (uint8_t)(USB_MICROPHONE_DESCSIZE >> 8),    /* wTotalLength (MSB)                  */
    2,                                          /* bNumInterfaces                      */
    0x01,                                       /* bConfigurationValue                 */
    0x00,                                       /* iConfiguration                      */
    CONFIG_DESC_BM_RESERVED_D7 |                /* bmAttrib                            */
    CONFIG_DESC_BM_SELFPOWERED |
    CONFIG_DESC_BM_REMOTEWAKEUP,
    CONFIG_DESC_MAXPOWER_mA(100),               /* bMaxPower: 100 mA                   */

    /* Interface descriptor */

    USB_INTERFACE_DESCSIZE,                     /* bLength                             */
    USB_INTERFACE_DESCRIPTOR,                   /* bDescriptorType                     */
    0,                                          /* bInterfaceNumber                    */
    0,                                          /* bAlternateSetting                   */
    0,                                          /* bNumEndpoints                       */
    USB_CLASS_AUDIO,                            /* bInterfaceClass                     */
    USB_CLASS_AUDIO_CONTROL,                    /* bInterfaceSubClass                  */
    0,                                          /* bInterfaceProtocol                  */
    0,                                          /* iInterface                          */

    /* Class-specific AC Interface Header descriptor */
  
    USB_AC_INT_HEADER_DESCSIZE,                 /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_HEADER_DESCRIPTOR,                   /* bDescriptorSubtype                  */
    0x00, 0x01,                                 /* bcdADC (1.0)                        */
    USB_AC_ITF_DESCSIZE,                        /* wTotalLength (LSB)                  */
    USB_AC_ITF_DESCSIZE >> 8,                   /* wTotalLength (MSB)                  */
    1,                                          /* bInCollection                       */
    1,                                          /* baInterfaceNr                       */

    /** Input Terminal ID1 descriptor */
  
    USB_CA_INPUT_TERMINAL_DESCSIZE,             /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_INPUT_TERMINAL_DESCRIPTOR,           /* bDescriptorSubtype                  */
    1,                                          /* bTerminalID                         */
    0x01, 0x02,                                 /* wTerminalType (Microphone)          */
    0,                                          /* bAssocTerminal                      */
    1,                                          /* bNrChannels                         */
    0x00, 0x00,                                 /* wChannelConfig (Mono)               */
    0,                                          /* iChannelNames                       */
    0,                                          /* iTerminal                           */

    /* Feature Unit ID2 descriptor */
  
    USB_FEATURE_UNIT_DESCSIZE,                  /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_FEATURE_UNIT_DESCRIPTOR,             /* bDescriptorSubtype                  */
    2,                                          /* bUnitID                             */
    1,                                          /* bSourceID                           */
    2,                                          /* bControlSize                        */
    0x01, 0x00,                                 /* bmaControls(0) (Mute)               */
    0,                                          /* iFeature                            */

    /* Output Terminal ID3 descriptor */
  
    USB_CA_OUTPUT_TERMINAL_DESCSIZE,            /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_OUTPUT_TERMINAL_DESCRIPTOR,          /* bDescriptorSubtype                  */
    3,                                          /* bTerminalID                         */
    0x01, 0x01,                                 /* wTerminalType (USB Streaming)       */
    0,                                          /* bAssocTerminal                      */
    2,                                          /* bSourceID                           */
    0,                                          /* iTerminal                           */

    /* Interface descriptor */

    USB_INTERFACE_DESCSIZE,                     /* bLength                             */
    USB_INTERFACE_DESCRIPTOR,                   /* bDescriptorType                     */
    1,                                          /* bInterfaceNumber                    */
    0,                                          /* bAlternateSetting                   */
    0,                                          /* bNumEndpoints                       */
    USB_CLASS_AUDIO,                            /* bInterfaceClass                     */
    USB_CLASS_AUDIO_STREAMING,                  /* bInterfaceSubClass                  */
    0,                                          /* bInterfaceProtocol                  */
    0,                                          /* iInterface                          */

    /* Alternate setting 1 (operational stream) */
  
    USB_INTERFACE_DESCSIZE,                     /* bLength                             */
    USB_INTERFACE_DESCRIPTOR,                   /* bDescriptorType                     */
    1,                                          /* bInterfaceNumber                    */
    1,                                          /* bAlternateSetting                   */
    1,                                          /* bNumEndpoints                       */
    USB_CLASS_AUDIO,                            /* bInterfaceClass                     */
    USB_CLASS_AUDIO_STREAMING,                  /* bInterfaceSubClass                  */
    0,                                          /* bInterfaceProtocol                  */
    0,                                          /* iInterface                          */

    /* Class-specific AS General Interface descriptor */
  
    USB_CA_AS_GENERAL_DESCSIZE,                 /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_AS_GENERAL_DESCRIPTOR,               /* bDescriptorSubtype                  */
    3,                                          /* bTerminalLink                       */
    0,                                          /* bDelay                              */
    0x01, 0x00,                                 /* wFormatTag (PCM)                    */

    /* Mono Type I Format interface descriptor */
  
    USB_MONO_FORMAT_DESCSIZE,                   /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_FORMAT_TYPE_DESCRIPTOR,              /* bDescriptorSubtype                  */
    1,                                          /* bFormatType                         */
    1,                                          /* bNrChannels                         */
    2,                                          /* bSubFrameSize                       */
    16,                                         /* bBitResolution                      */
    1,                                          /* bSamFreqType (one)                  */
    (uint8_t)MAXIMUM_SAMPLE_RATE,               /* tSamFreq                            */
    (uint8_t)(MAXIMUM_SAMPLE_RATE >> 8),
    (uint8_t)(MAXIMUM_SAMPLE_RATE >> 16),

    /* Standard audio stream isochronous endpoint descriptor */

    USB_CA_STD_AS_ENDPOINT_DESCSZIE,            /* bLength                             */
    USB_ENDPOINT_DESCRIPTOR,                    /* bDescriptorType                     */
    MICROPHONE_EP_IN,                           /* bEndpointAddress                    */
    USB_EPTYPE_ISOC | USB_EPSYNC_SYNC,          /* bmAttributes                        */
    (uint8_t)MAXIMUM_BUFFER_SIZE,               /* wMaxPacketSize (LSB)                */
    (uint8_t)(MAXIMUM_BUFFER_SIZE >> 8),        /* wMaxPacketSize (MSB)                */
    1,                                          /* bInterval                           */
    0,                                          /* bRefresh                            */
    0,                                          /* bSynchAddress                       */

    /* Class-specific isochronous audio data endpoint descriptor */

    USB_CA_EP_GENERAL_DESCSIZE,                 /* bLength                              */
    USB_CS_ENDPOINT_DESCRIPTOR,                 /* bDescriptorType                      */
    USB_CA_EP_GENERAL_DESCRIPTOR,               /* bDescriptorSubtype                   */
    0,                                          /* bmAttributes                         */
    0,                                          /* bLockDelayUnits                      */
    0, 0                                        /* wLockDelay                           */

};

/* String descriptors */

STATIC_CONST_STRING_DESC_LANGID(langID, 0x04, 0x09);

STATIC_CONST_STRING_DESC(iManufacturer, 'o', 'p', 'e', 'n', 'a', 'c', 'o', 'u', 's', 't', 'i', 'c', 'd', 'e', 'v', 'i', 'c', 'e', 's', '.', 'i', 'n', 'f', 'o');

STATIC_CONST_STRING_DESC(iProduct, 'A', 'u', 'd', 'i', 'o', 'M', 'o', 't', 'h', ' ', 'U', 'S', 'B', ' ', 'M', 'i', 'c', 'r', 'o', 'p', 'h', 'o', 'n', 'e');

STATIC_CONST_STRING_DESC(iSerialNumber, '0', '1', '0', '0');

/* End-point buffer sizes */

static const uint8_t bufferingMultiplier[2] = {1, 2};

/* String array */

static const void* strings[] = {
    &langID,
    &iManufacturer,
    &iProduct,
    &iSerialNumber
};

/* USB callbacks */

static int setupCmd(const USB_Setup_TypeDef *setup);

static void stateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState);

static const USBD_Callbacks_TypeDef callbacks = {
    .usbReset        = NULL,
    .usbStateChange  = stateChange,
    .setupCmd        = setupCmd,
    .isSelfPowered   = NULL,
    .sofInt          = NULL
};

/* Initialisation data structure */

static const USBD_Init_TypeDef initstruct = {
    .deviceDescriptor    = &deviceDesc,
    .configDescriptor    = configDesc,
    .stringDescriptors   = strings,
    .numberOfStrings     = sizeof(strings)/sizeof(void*),
    .callbacks           = &callbacks,
    .bufferingMultiplier = bufferingMultiplier,
    .reserved            = 0
};

#endif /* __MICROPHONE_H */


