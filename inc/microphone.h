/****************************************************************************
 * microphone.h
 * openacousticdevices.info
 * August 2021
 *****************************************************************************/

#ifndef __MICROPHONE_H
#define __MICROPHONE_H

/* USB microphone constants */

#define USB_EP0_SIZE                            64
#define MICROPHONE_HID_BUFFER_SIZE              64

#define MICROPHONE_EP_IN                        0x82

/* Configuration descriptor constants */

#define USB_AC_INT_HEADER_DESCSIZE              9
#define USB_FEATURE_UNIT_DESCSIZE               9
#define USB_MONO_FORMAT_DESCSIZE                11

#define USB_AC_ITF_DESCSIZE                     (USB_AC_INT_HEADER_DESCSIZE + USB_CA_INPUT_TERMINAL_DESCSIZE + USB_FEATURE_UNIT_DESCSIZE + USB_CA_OUTPUT_TERMINAL_DESCSIZE)

#define USB_MICROPHONE_DESCSIZE                 (USB_CONFIG_DESCSIZE + USB_INTERFACE_DESCSIZE + USB_AC_ITF_DESCSIZE + 2 * USB_INTERFACE_DESCSIZE + USB_CA_AS_GENERAL_DESCSIZE \
                                                  + USB_MONO_FORMAT_DESCSIZE + USB_CA_STD_AS_ENDPOINT_DESCSZIE + USB_CA_EP_GENERAL_DESCSIZE + USB_INTERFACE_DESCSIZE + USB_HID_DESCSIZE + 2 * USB_ENDPOINT_DESCSIZE)

/* USB configuration constants */

#define MAXIMUM_SAMPLE_RATE                     384000

#define MAXIMUM_BUFFER_SIZE                     (MAXIMUM_SAMPLE_RATE * 2 / 1000)

#define SAMPLE_RATE_OFFSET                      (USB_MICROPHONE_DESCSIZE - USB_CA_EP_GENERAL_DESCSIZE - USB_CA_STD_AS_ENDPOINT_DESCSZIE - 3 - USB_INTERFACE_DESCSIZE - USB_HID_DESCSIZE - 2 * USB_ENDPOINT_DESCSIZE)

#define BUFFER_SIZE_OFFSET                      (USB_MICROPHONE_DESCSIZE - USB_CA_EP_GENERAL_DESCSIZE - 5 - USB_INTERFACE_DESCSIZE - USB_HID_DESCSIZE - 2 * USB_ENDPOINT_DESCSIZE)

/* Device descriptor */

SL_ALIGN(4)
static USB_DeviceDescriptor_TypeDef deviceDesc __attribute__ ((aligned(4))) = {
    .bLength            = USB_DEVICE_DESCSIZE,            /* Size of the Descriptor in Bytes          */
    .bDescriptorType    = USB_DEVICE_DESCRIPTOR,          /* Device Descriptor type                   */
    .bcdUSB             = 0x0200,                         /* USB 2.0 compliant                        */
    .bDeviceClass       = 0x00,                           /* Vendor unique device                     */
    .bDeviceSubClass    = 0x00,                           /* Ignored for vendor unique device         */
    .bDeviceProtocol    = 0x00,                           /* Ignored for vendor unique device         */
    .bMaxPacketSize0    = USB_EP0_SIZE,                   /* Max packet size for EP0                  */
    .idVendor           = 0x16D0,                         /* VID                                      */
    .idProduct          = 0x06F3,                         /* PID                                      */
    .bcdDevice          = 0x0000,                         /* Device Release number                    */
    .iManufacturer      = 0x01,                           /* Index of Manufacturer String Descriptor  */
    .iProduct           = 0x02,                           /* Index of Product String Descriptor       */
    .iSerialNumber      = 0x03,                           /* Index of Serial Number String Descriptor */
    .bNumConfigurations = 0x01                            /* Number of Possible Configurations        */

};

/* HID Report Descriptor */

SL_ALIGN(4)
static const char HID_ReportDescriptor[] __attribute__ ((aligned(4))) = {
    0x06, 0xAB, 0xFF,
    0x0A, 0x00, 0x02,
    0xA1, 0x01,                                 /* Collection 0x01                      */
    0x75, 0x08,                                 /* Report size = 8 bits                 */
    0x15, 0x00,                                 /* Logical minimum = 0                  */
    0x26, 0xFF, 0x00,                           /* logical maximum = 255                */
    0x95, 64,                                   /* Report count                         */
    0x09, 0x01,                                 /* Usage                                */
    0x81, 0x02,                                 /* Input (array)                        */
    0x95, 64,                                   /* Report count                         */
    0x09, 0x02,                                 /* Usage                                */
    0x91, 0x02,                                 /* Output (array)                       */
    0xC0                                        /* End collection                       */
};

/* HID Descriptor */

SL_ALIGN(4)
static const char HID_Descriptor[] __attribute__ ((aligned(4))) = {
    USB_HID_DESCSIZE,                           /* bLength                              */
    USB_HID_DESCRIPTOR,                         /* bDescriptorType                      */
    0x11,                                       /* bcdHID (LSB)                         */
    0x01,                                       /* bcdHID (MSB)                         */
    0x00,                                       /* bCountryCode                         */
    0x01,                                       /* bNumDescriptors                      */
    USB_HID_REPORT_DESCRIPTOR,                  /* bDecriptorType                       */
    sizeof(HID_ReportDescriptor),               /* wDescriptorLength(LSB)               */
    0x00                                        /* wDescriptorLength(MSB)               */
};

/* Configuration descriptor */

SL_ALIGN(4)
static uint8_t configDesc[] __attribute__ ((aligned(4)))= {

    /* Configuration descriptor */

    USB_CONFIG_DESCSIZE,                        /* bLength                             */
    USB_CONFIG_DESCRIPTOR,                      /* bDescriptorType                     */
    (uint8_t)USB_MICROPHONE_DESCSIZE,           /* wTotalLength (LSB)                  */
    (uint8_t)(USB_MICROPHONE_DESCSIZE >> 8),    /* wTotalLength (MSB)                  */
    0x03,                                       /* bNumInterfaces                      */
    0x01,                                       /* bConfigurationValue                 */
    0x00,                                       /* iConfiguration                      */
    CONFIG_DESC_BM_RESERVED_D7 |                /* bmAttrib                            */
    CONFIG_DESC_BM_SELFPOWERED |
    CONFIG_DESC_BM_REMOTEWAKEUP,
    CONFIG_DESC_MAXPOWER_mA(100),               /* bMaxPower: 100 mA                   */

    /* Interface descriptor */

    USB_INTERFACE_DESCSIZE,                     /* bLength                             */
    USB_INTERFACE_DESCRIPTOR,                   /* bDescriptorType                     */
    0x00,                                       /* bInterfaceNumber                    */
    0x00,                                       /* bAlternateSetting                   */
    0x00,                                       /* bNumEndpoints                       */
    USB_CLASS_AUDIO,                            /* bInterfaceClass                     */
    USB_CLASS_AUDIO_CONTROL,                    /* bInterfaceSubClass                  */
    0x00,                                       /* bInterfaceProtocol                  */
    0x00,                                       /* iInterface                          */

    /* Class-specific AC Interface Header descriptor */
  
    USB_AC_INT_HEADER_DESCSIZE,                 /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_HEADER_DESCRIPTOR,                   /* bDescriptorSubtype                  */
    0x00, 0x01,                                 /* bcdADC (1.0)                        */
    USB_AC_ITF_DESCSIZE,                        /* wTotalLength (LSB)                  */
    USB_AC_ITF_DESCSIZE >> 8,                   /* wTotalLength (MSB)                  */
    0x01,                                       /* bInCollection                       */
    0x01,                                       /* baInterfaceNr                       */

    /** Input Terminal ID1 descriptor */
  
    USB_CA_INPUT_TERMINAL_DESCSIZE,             /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_INPUT_TERMINAL_DESCRIPTOR,           /* bDescriptorSubtype                  */
    0x01,                                       /* bTerminalID                         */
    0x01, 0x02,                                 /* wTerminalType (Microphone)          */
    0x00,                                       /* bAssocTerminal                      */
    0x01,                                       /* bNrChannels                         */
    0x00, 0x00,                                 /* wChannelConfig (Mono)               */
    0x00,                                       /* iChannelNames                       */
    0x00,                                       /* iTerminal                           */

    /* Feature Unit ID2 descriptor */
  
    USB_FEATURE_UNIT_DESCSIZE,                  /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_FEATURE_UNIT_DESCRIPTOR,             /* bDescriptorSubtype                  */
    0x02,                                       /* bUnitID                             */
    0x01,                                       /* bSourceID                           */
    0x02,                                       /* bControlSize                        */
    0x01, 0x00,                                 /* bmaControls(0) (Mute)               */
    0x00,                                       /* iFeature                            */

    /* Output Terminal ID3 descriptor */
  
    USB_CA_OUTPUT_TERMINAL_DESCSIZE,            /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_OUTPUT_TERMINAL_DESCRIPTOR,          /* bDescriptorSubtype                  */
    0x03,                                       /* bTerminalID                         */
    0x01, 0x01,                                 /* wTerminalType (USB Streaming)       */
    0x00,                                       /* bAssocTerminal                      */
    0x02,                                       /* bSourceID                           */
    0x00,                                       /* iTerminal                           */

    /* Interface descriptor */

    USB_INTERFACE_DESCSIZE,                     /* bLength                             */
    USB_INTERFACE_DESCRIPTOR,                   /* bDescriptorType                     */
    0x01,                                       /* bInterfaceNumber                    */
    0x00,                                       /* bAlternateSetting                   */
    0x00,                                       /* bNumEndpoints                       */
    USB_CLASS_AUDIO,                            /* bInterfaceClass                     */
    USB_CLASS_AUDIO_STREAMING,                  /* bInterfaceSubClass                  */
    0x00,                                       /* bInterfaceProtocol                  */
    0x00,                                       /* iInterface                          */

    /* Alternate setting 1 (operational stream) */
  
    USB_INTERFACE_DESCSIZE,                     /* bLength                             */
    USB_INTERFACE_DESCRIPTOR,                   /* bDescriptorType                     */
    0x01,                                       /* bInterfaceNumber                    */
    0x01,                                       /* bAlternateSetting                   */
    0x01,                                       /* bNumEndpoints                       */
    USB_CLASS_AUDIO,                            /* bInterfaceClass                     */
    USB_CLASS_AUDIO_STREAMING,                  /* bInterfaceSubClass                  */
    0x00,                                       /* bInterfaceProtocol                  */
    0x00,                                       /* iInterface                          */

    /* Class-specific AS General Interface descriptor */
  
    USB_CA_AS_GENERAL_DESCSIZE,                 /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_AS_GENERAL_DESCRIPTOR,               /* bDescriptorSubtype                  */
    0x03,                                       /* bTerminalLink                       */
    0x00,                                       /* bDelay                              */
    0x01, 0x00,                                 /* wFormatTag (PCM)                    */

    /* Mono Type I Format interface descriptor */
  
    USB_MONO_FORMAT_DESCSIZE,                   /* bLength                             */
    USB_CS_INTERFACE_DESCRIPTOR,                /* bDescriptorType                     */
    USB_CA_FORMAT_TYPE_DESCRIPTOR,              /* bDescriptorSubtype                  */
    0x01,                                       /* bFormatType                         */
    0x01,                                       /* bNrChannels                         */
    0x02,                                       /* bSubFrameSize                       */
    0x10,                                       /* bBitResolution                      */
    0x01,                                       /* bSamFreqType (one)                  */
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
    0x01,                                       /* bInterval                           */
    0x00,                                       /* bRefresh                            */
    0x00,                                       /* bSynchAddress                       */

    /* Class-specific isochronous audio data endpoint descriptor */

    USB_CA_EP_GENERAL_DESCSIZE,                 /* bLength                              */
    USB_CS_ENDPOINT_DESCRIPTOR,                 /* bDescriptorType                      */
    USB_CA_EP_GENERAL_DESCRIPTOR,               /* bDescriptorSubtype                   */
    0x00,                                       /* bmAttributes                         */
    0x00,                                       /* bLockDelayUnits                      */
    0x00, 0x00,                                 /* wLockDelay                           */

    /* Interface descriptor */

    USB_INTERFACE_DESCSIZE,                     /* bLength                             */
    USB_INTERFACE_DESCRIPTOR,                   /* bDescriptorType                     */
    0x02,                                       /* bInterfaceNumber                    */
    0x00,                                       /* bAlternateSetting                   */
    0x02,                                       /* bNumEndpoints                       */
    0x03,                                       /* bInterfaceClass                     */
    0x00,                                       /* bInterfaceSubClass                  */
    0x00,                                       /* bInterfaceProtocol                  */
    0x00,                                       /* iInterface                          */

    /* HID descriptor */

    USB_HID_DESCSIZE,                           /* bLength                             */
    USB_HID_DESCRIPTOR,                         /* bDescriptorType                     */
    0x11,                                       /* bcdHID (LSB)                        */
    0x01,                                       /* bcdHID (MSB)                        */
    0x00,                                       /* bCountryCode                        */
    0x01,                                       /* bNumDescriptors                     */
    USB_HID_REPORT_DESCRIPTOR,                  /* bDecriptorType                      */
    sizeof(HID_ReportDescriptor),               /* wDescriptorLength(LSB)              */
    0x00,                                       /* wDescriptorLength(MSB)              */

    /* Interrupt End-point Descriptor (OUT) */

    USB_ENDPOINT_DESCSIZE,                      /* bLength                             */
    USB_ENDPOINT_DESCRIPTOR,                    /* bDescriptorType                     */
    HID_EP_OUT,                                 /* bEndpointAddress                    */
    USB_EPTYPE_INTR,                            /* bmAttributes                        */
    MICROPHONE_HID_BUFFER_SIZE,                 /* wMaxPacketSize (LSB)                */
    0x00,                                       /* wMaxPacketSize (MSB)                */
    0x0A,                                       /* bInterval                           */

    /* Interrupt End-point Descriptor (IN) */

    USB_ENDPOINT_DESCSIZE,                      /* bLength                             */
    USB_ENDPOINT_DESCRIPTOR,                    /* bDescriptorType                     */
    HID_EP_IN,                                  /* bEndpointAddress                    */
    USB_EPTYPE_INTR,                            /* bmAttributes                        */
    MICROPHONE_HID_BUFFER_SIZE,                 /* wMaxPacketSize (LSB)                */
    0x00,                                       /* wMaxPacketSize (MSB)                */
    0x0A                                        /* bInterval                           */

};

/* String descriptors */

STATIC_CONST_STRING_DESC_LANGID(langID, 0x04, 0x09);

STATIC_CONST_STRING_DESC(iManufacturer, 'o', 'p', 'e', 'n', 'a', 'c', 'o', 'u', 's', 't', 'i', 'c', 'd', 'e', 'v', 'i', 'c', 'e', 's', '.', 'i', 'n', 'f', 'o');

STATIC_CONST_STRING_DESC(iProduct, 'A', 'u', 'd', 'i', 'o', 'M', 'o', 't', 'h', ' ', 'U', 'S', 'B', ' ', 'M', 'i', 'c', 'r', 'o', 'p', 'h', 'o', 'n', 'e');

STATIC_CONST_STRING_DESC(iSerialNumber, '0', '0', '0', '0', '_', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0');

/* End-point buffer sizes */

static const uint8_t bufferingMultiplier[NUM_EP_USED + 1] = {
    1,  /* Control */
    2,  /* Isochronous */
    1,  /* Interrupt */
    1,  /* Interrupt */
    0   /* Unused */
};

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


