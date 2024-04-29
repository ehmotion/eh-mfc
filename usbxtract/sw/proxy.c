/*
 Copyright (c) 2015 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3

 2019 mirel.t.lazar@gmail.com
 adapted for USB message extraction
 */

#include <gusb.h>
#include <gserial.h>
#include <protocol.h>
#include <adapter.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gpoll.h>
#include <gtimer.h>
#include <names.h>
#include <prio.h>
#include <sys/time.h>

#define ENDPOINT_MAX_NUMBER USB_ENDPOINT_NUMBER_MASK

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"

#define PRINT_ERROR_OTHER(MESSAGE) fprintf(stderr, "%s:%d %s: %s\n", __FILE__, __LINE__, __func__, MESSAGE);
#define PRINT_TRANSFER_WRITE_ERROR(ENDPOINT,MESSAGE) fprintf(stderr, "%s:%d %s: write transfer failed on endpoint %hhu with error: %s\n", __FILE__, __LINE__, __func__, ENDPOINT & USB_ENDPOINT_NUMBER_MASK, MESSAGE);
#define PRINT_TRANSFER_READ_ERROR(ENDPOINT,MESSAGE) fprintf(stderr, "%s:%d %s: read transfer failed on endpoint %hhu with error: %s\n", __FILE__, __LINE__, __func__, ENDPOINT & USB_ENDPOINT_NUMBER_MASK, MESSAGE);

static int usb = -1;
static int adapter = -1;
static int init_timer = -1;

static s_usb_descriptors * descriptors = NULL;
static unsigned char desc[MAX_DESCRIPTORS_SIZE] = {};
static unsigned char * pDesc = desc;
static s_descriptorIndex descIndex[MAX_DESCRIPTORS] = {};
static s_descriptorIndex * pDescIndex = descIndex;
static s_endpointConfig endpoints[MAX_ENDPOINTS] = {};
static s_endpointConfig * pEndpoints = endpoints;

static uint8_t descIndexSent = 0;
static uint8_t endpointsSent = 0;

static uint8_t inPending = 0;

static uint8_t serialToUsbEndpoint[2][ENDPOINT_MAX_NUMBER] = {};
static uint8_t usbToSerialEndpoint[2][ENDPOINT_MAX_NUMBER] = {};

#define ENDPOINT_ADDR_TO_INDEX(ENDPOINT) (((ENDPOINT) & USB_ENDPOINT_NUMBER_MASK) - 1)
#define ENDPOINT_DIR_TO_INDEX(ENDPOINT) ((ENDPOINT) >> 7)
#define S2U_ENDPOINT(ENDPOINT) serialToUsbEndpoint[ENDPOINT_DIR_TO_INDEX(ENDPOINT)][ENDPOINT_ADDR_TO_INDEX(ENDPOINT)]
#define U2S_ENDPOINT(ENDPOINT) usbToSerialEndpoint[ENDPOINT_DIR_TO_INDEX(ENDPOINT)][ENDPOINT_ADDR_TO_INDEX(ENDPOINT)]

static struct {
  uint16_t length;
  s_endpointPacket packet;
} inPackets[ENDPOINT_MAX_NUMBER] = {};

static uint8_t inEpFifo[MAX_ENDPOINTS] = {};
static uint8_t nbInEpFifo = 0;

static volatile int done;

#define EP_PROP_IN    (1 << 0)
#define EP_PROP_OUT   (1 << 1)
#define EP_PROP_BIDIR (1 << 2)

#define EP_PROP_INT   (1 << 3)
#define EP_PROP_BLK   (1 << 4)
#define EP_PROP_ISO   (1 << 5)

/*
 * the atmega32u4 supports up to 6 non-control endpoints
 * that can be IN or OUT (not BIDIR),
 * and only the INTERRUPT type is supported.
 */
static uint8_t targetProperties[ENDPOINT_MAX_NUMBER] = {
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
};

static int send_next_in_packet()
{

  if (inPending)
  {
    return 0;
  }

  if (nbInEpFifo > 0)
  {
    uint8_t inPacketIndex = ENDPOINT_ADDR_TO_INDEX(inEpFifo[0]);
    int ret = adapter_send(adapter, E_TYPE_IN, (const void *)&inPackets[inPacketIndex].packet, inPackets[inPacketIndex].length);
    if(ret < 0)
    {
      return -1;
    }
    inPending = inEpFifo[0];
    --nbInEpFifo;
    memmove(inEpFifo, inEpFifo + 1, nbInEpFifo * sizeof(*inEpFifo));
  }

  return 0;
}

static int queue_in_packet(unsigned char endpoint, const void * buf, int transfered)
{

  if (nbInEpFifo == sizeof(inEpFifo) / sizeof(*inEpFifo))
  {
    PRINT_ERROR_OTHER("no more space in inEpFifo")
    return -1;
  }

  uint8_t inPacketIndex = ENDPOINT_ADDR_TO_INDEX(endpoint);
  inPackets[inPacketIndex].packet.endpoint = U2S_ENDPOINT(endpoint);
  memcpy(inPackets[inPacketIndex].packet.data, buf, transfered);
  inPackets[inPacketIndex].length = transfered + 1;
  inEpFifo[nbInEpFifo] = endpoint;
  ++nbInEpFifo;

  /*
   * TODO MLA: Poll the endpoint after registering the packet?
   */

  return 0;
}

/*
Logitech G923:
#USB extractor 0.02.1d                                                                                                                                                                                                                                                        
#i:initializing USB proxy with device 046d:c267                                                                                                                                                                                                                               
#i:using device: VID 0x046d PID 0xc267 PATH 01:01:04                                                                                                                                                                                                                          
#i:EP OUT INTERRUPT 3 -> 1                                                                                                                                                                                                                                                    
#i:EP IN INTERRUPT 4 -> 2                                                                                                                                                                                                                                                     
#i:EP IN INTERRUPT 2 -> 3                                                                                                                                                                                                                                                     #i:EP IN INTERRUPT 1 -> 4                                                                                                                                                                                                                                                     
#i:EP OUT INTERRUPT 1 -> 5                                                                                                                                                                                                                                                    
#i:starting
..
#d:adapter recv 2
#i@1663190.250:process pkt type 0x6
#i:next IN packet
#PKT@1663190.250:0B, type 0x06:
#i:usb_read_callback EP 4 user 0 status 0x40
#PKT@1663190.375:65B, type 0x06: 0x04 0x6d 0xc2 0x67 0x5b 0x45 0x22 0x63 0x06 0x41 0x00 0x82 0x01 0x80 0x80 0x80 0x80 0x08 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x6a 0x7f 0xff 0xff 0xff 0xff 0xff 0xff 0x00 0xff
#d:adapter sent 67B vs 65B
#d:adapter recv 2
#i@1663190.375:process pkt type 0x6
#i:next IN packet
#PKT@1663190.375:0B, type 0x06:
#i:usb_read_callback EP 4 user 0 status 0x40
#PKT@1663190.625:65B, type 0x06: 0x04 0x6d 0xc2 0x67 0x4f 0x46 0x22 0x63 0x06 0x41 0x00 0x82 0x01 0x80 0x80 0x80 0x80 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x6a 0x7f 0xff 0xff 0xff 0xff 0xff 0xff 0x00 0xff
#d:adapter sent 67B vs 65B
#d:adapter recv 2
#i@1663190.625:process pkt type 0x6
#i:next IN packet
#PKT@1663190.625:0B, type 0x06:
#i:usb_read_callback EP 2 user 0 status 0x14
#PKT@1663190.625:21B, type 0x06: 0x04 0x6d 0xc2 0x67 0x52 0x46 0x22 0x63 0x06 0x15 0x00 0x83 0x11 0xff 0x10 0x00 0x00 0x00 0x64 0x00 0x00
#d:adapter sent 23B vs 21B
#d:adapter recv 1
#d:adapter recv 1
#i@1663190.625:process pkt type 0x6
#i:next IN packet
#PKT@1663190.625:0B, type 0x06:
#i:usb_read_callback EP 2 user 0 status 0x14
#PKT@1663190.750:21B, type 0x06: 0x04 0x6d 0xc2 0x67 0xde 0x46 0x22 0x63 0x06 0x15 0x00 0x83 0x11 0xff 0x10 0x00 0x00 0x00 0x64 0x00 0x00
#d:adapter sent 23B vs 21B
#i:usb_read_callback EP 4 user 0 status 0x40
*/
extern int vid, pid;
int usb_read_callback(int user, unsigned char endpoint, const void * buf, int status)
{
  if (0) printf("\n#i:usb_read_callback EP %d user %d status 0x%x", endpoint & USB_ENDPOINT_NUMBER_MASK, user, status);
  //discard EP2 on Logitech G923 to avoid emu lock-up
  if ((endpoint & USB_ENDPOINT_NUMBER_MASK) == 2 && vid == 0x046d && pid == 0xc267)
  {
    //drop other packets
    printf("\n#w:usb_read_callback drop pkt for EP %d user %d status 0x%x", endpoint & USB_ENDPOINT_NUMBER_MASK, user, status);
    int i;
    printf ("\n#PKT:%d bytes T0x%02x <: ", status, E_TYPE_IN);
    for (i = 0; i < status; i++)
      printf ("%02x ", ((char *)buf)[i]);
    fflush (stdout);
    return 0;
  }
  switch (status)
  {
  case E_TRANSFER_TIMED_OUT:
    PRINT_TRANSFER_READ_ERROR (endpoint, "TIMEOUT")
    break;
  case E_TRANSFER_STALL:
    PRINT_TRANSFER_WRITE_ERROR (endpoint, "STALL")
    break;
  case E_TRANSFER_ERROR:
    PRINT_TRANSFER_WRITE_ERROR (endpoint, "OTHER ERROR")
    return -1;
  default:
    break;
  }

  if (endpoint == 0)
  {
    if (status > (int)MAX_PACKET_VALUE_SIZE)
    {
      PRINT_ERROR_OTHER ("too many bytes transfered")
      done = 1;
      return -1;
    }

    int ret = 0;
    if (status >= 0)
    {
      ret = adapter_send (adapter, E_TYPE_CONTROL, buf, status);
    }
    else
    {
      printf("\n#w:!usb_read_callback STALL pkt for EP %d user %d status 0x%x", endpoint & USB_ENDPOINT_NUMBER_MASK, user, status);
      //ret = adapter_send (adapter, E_TYPE_CONTROL_STALL, NULL, 0);
    }
    if(ret < 0)
    {
      return -1;
    }
  }
  else
  {
    if (status > MAX_PAYLOAD_SIZE_EP)
    {
      PRINT_ERROR_OTHER ("too many bytes transfered")
      done = 1;
      return -1;
    }
    /* Logitech G923
#i:queue done
#PKT@554269.062:021 bytes, type 6
##83 11 ff 10 00 00 00 64 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:send done
#PKT@554269.062:000 bytes, type 6
##
#i:queue done
#PKT@554269.188:021 bytes, type 6
##83 11 ff 10 00 00 00 64 00 00 01 00 00 00 00 00 00 00 00 00 00
-- -
#PKT@554464.125:021 bytes, type 6
##83 11 ff 10 00 00 00 64 00 00 01 00 00 00 00 00 00 00 00 00 00
#PKT@554464.125:021 bytes, type 6
##83 11 ff 10 00 00 00 64 00 00 01 00 00 00 00 00 00 00 00 00 00
#PKT@554464.188:000 bytes, type 6
##
#i:queue done
#PKT@554464.188:021 bytes, type 6
##83 11 ff 10 00 00 00 64 00 00 01 00 00 00 00 00 00 00 00 00 00
#i:send done
    */
    //if (status == 0x40) //TODO: fixme! only send IN report of size 64 bytes
    if (status >= 0) //TODO: fixme! only send IN report of size 64 bytes
    {
      int ret = queue_in_packet (endpoint, buf, status);
      if (ret < 0)
      {
        done = 1;
        return -1;
      }
      if (status != 0x40)
      {
        //printf("\n#i:queue done");
        fflush (stdout);
      }

      ret = send_next_in_packet ();
      if (ret < 0)
      {
        done = 1;
        return -1;
      }
      if (status != 0x40)
      {
        //printf("\n#i:send done");
        fflush (stdout);
      }
    }
    else
    {
      //drop other packets
      printf("\n#w:usb_read_callback drop pkt for EP %d user %d status 0x%x", endpoint & USB_ENDPOINT_NUMBER_MASK, user, status);
      int i;
      printf ("\n#PKT:%d bytes, type 0x%02x: ", status, E_TYPE_IN);
      for (i = 0; i < status; i++)
        printf ("%02x ", ((char *)buf)[i]);
      fflush (stdout);
    }
  }

  return 0;
}

int usb_write_callback (int user, unsigned char endpoint, int status)
{

  switch (status)
  {
  case E_TRANSFER_TIMED_OUT:
    PRINT_TRANSFER_WRITE_ERROR (endpoint, "TIMEOUT")
    break;
  case E_TRANSFER_STALL:
    if (endpoint == 0)
    {
      int ret = adapter_send (adapter, E_TYPE_CONTROL_STALL, NULL, 0);
      if (ret < 0)
      {
        done = 1;
        return -1;
      }
    }
    break;
  case E_TRANSFER_ERROR:
    PRINT_TRANSFER_WRITE_ERROR (endpoint, "OTHER ERROR")
    return -1;
  default:
    if (endpoint == 0)
    {
      int ret = adapter_send (adapter, E_TYPE_CONTROL, NULL, 0);
      if (ret < 0)
      {
        done = 1;
        return -1;
      }
    }
    break;
  }

  return 0;
}

int usb_close_callback(int user)
{
  done = 1;
  return 1;
}

int adapter_send_callback (int user, int transfered)
{
  if (transfered < 0)
  {
    done = 1;
    return 1;
  }

  return 0;
}

int adapter_close_callback(int user)
{
  done = 1;
  return 1;
}

static char * usb_select (int vid, int pid) 
{
  char * path = NULL;
  //
  s_usb_dev * usb_devs = gusb_enumerate(0x0000, 0x0000);
  if (usb_devs == NULL) 
  {
//	    fprintf (stderr, "\n#e:no USB device detected!");
    fprintf (stdout, "\n#e:no USB device detected!");
    return NULL;
  }
  //printf ("Available USB devices:\n");
  unsigned int index = 0;
  //char vendor[128], product[128];
  s_usb_dev * current;
  unsigned int choice = UINT_MAX;
  for (current = usb_devs; current != NULL; ++current) 
  {
    //get_vendor_string(vendor, sizeof(vendor), current->vendor_id);
    //get_product_string(product, sizeof(product), current->vendor_id, current->product_id);
    //printf("%2d", index);
    index++;
    //printf(" VID 0x%04x (%s)", current->vendor_id, strlen(vendor) ? vendor : "unknown vendor");
    //printf(" PID 0x%04x (%s)", current->product_id, strlen(product) ? product : "unknown product");
    //printf(" PATH %s\n", current->path);
    //fflush (stdout);
    //auto select T300RS
    if (current->product_id == pid && current->vendor_id == vid)
    {
      choice = index - 1;
    }
    if (current->next == 0) 
    {
      break;
    }
  }
  //
  //printf("Selected the USB device number: %d\n", choice);
  //fflush (stdout);
  if (choice < index) //scanf("%d", &choice) == 1 && choice < index) 
  {
    path = strdup (usb_devs[choice].path);
    if(path == NULL) 
    {
//        fprintf (stderr, "\n#e:USB device: can't duplicate path!");
        fprintf (stdout, "\n#e:USB device: can't duplicate path!");
    }
  } else 
  {
//	    fprintf (stderr, "\n#e:USB device not found!");
    fprintf (stdout, "\n#e:USB device not found!");
  }
  //
  gusb_free_enumeration (usb_devs);
  //
  return path;
}

void print_endpoint_properties(uint8_t epProps[ENDPOINT_MAX_NUMBER])
{
  unsigned char i;
  for (i = 0; i < ENDPOINT_MAX_NUMBER; ++i)
  {
    if (epProps[i] != 0)
    {
      printf("%hhu", i + 1);
      if (epProps[i] & EP_PROP_IN)
      {
        printf(" IN");
      }
      if (epProps[i] & EP_PROP_OUT)
      {
        printf(" OUT");
      }
      if (epProps[i] & EP_PROP_BIDIR)
      {
        printf(" BIDIR");
      }
      if (epProps[i] & EP_PROP_INT)
      {
        printf(" INTERRUPT");
      }
      if (epProps[i] & EP_PROP_BLK)
      {
        printf(" BULK");
      }
      if (epProps[i] & EP_PROP_ISO)
      {
        printf(" ISOCHRONOUS");
      }
      printf("\n");
    }
  }
}

void print_endpoints()
{
  //static uint8_t serialToUsbEndpoint[2][ENDPOINT_MAX_NUMBER] = {};
  //static uint8_t usbToSerialEndpoint[2][ENDPOINT_MAX_NUMBER] = {};
  for (int i = 0; i < 2; i++)
  {
    printf ("\n#s2u[%d]:", i);
    for (int j = 0; j < ENDPOINT_MAX_NUMBER; j++)
      printf (" %02X", serialToUsbEndpoint[i][j]);
    //
    printf ("\n#u2s[%d]:", i);
    for (int j = 0; j < ENDPOINT_MAX_NUMBER; j++)
      printf (" %02X", usbToSerialEndpoint[i][j]);
  }
}

void get_endpoint_properties (unsigned char configurationIndex, uint8_t epProps[ENDPOINT_MAX_NUMBER])
{
  struct p_configuration * pConfiguration = descriptors->configurations + configurationIndex;
  unsigned char interfaceIndex;
  for (interfaceIndex = 0; interfaceIndex < pConfiguration->descriptor->bNumInterfaces; ++interfaceIndex) 
  {
    struct p_interface * pInterface = pConfiguration->interfaces + interfaceIndex;
    unsigned char altInterfaceIndex;
    for (altInterfaceIndex = 0; altInterfaceIndex < pInterface->bNumAltInterfaces; ++altInterfaceIndex) 
    {
      struct p_altInterface * pAltInterface = pInterface->altInterfaces + altInterfaceIndex;
      unsigned char endpointIndex;
      for (endpointIndex = 0; endpointIndex < pAltInterface->bNumEndpoints; ++endpointIndex) 
      {
        struct usb_endpoint_descriptor * endpoint =
            descriptors->configurations[configurationIndex].interfaces[interfaceIndex].altInterfaces[altInterfaceIndex].endpoints[endpointIndex];
        uint8_t epIndex = ENDPOINT_ADDR_TO_INDEX(endpoint->bEndpointAddress);
        switch (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) 
        {
        case USB_ENDPOINT_XFER_INT:
          epProps[epIndex] |= EP_PROP_INT;
          break;
        case USB_ENDPOINT_XFER_BULK:
          epProps[epIndex] |= EP_PROP_BLK;
          break;
        case USB_ENDPOINT_XFER_ISOC:
          epProps[epIndex] |= EP_PROP_ISO;
          break;
        }
        if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) 
        {
          epProps[epIndex] |= EP_PROP_IN;
        }
        else 
        {
          epProps[epIndex] |= EP_PROP_OUT;
        }
        if ((epProps[epIndex] & (EP_PROP_IN | EP_PROP_OUT)) == (EP_PROP_IN | EP_PROP_OUT)) 
        {
          epProps[epIndex] |= EP_PROP_BIDIR;
        }
      }
    }
  }
}

int compare_endpoint_properties (uint8_t epPropsSource[ENDPOINT_MAX_NUMBER], uint8_t epPropsTarget[ENDPOINT_MAX_NUMBER])
{

  unsigned char i;
  for (i = 0; i < ENDPOINT_MAX_NUMBER; ++i) 
  {
    if (epPropsSource[i] != 0) 
    {
      if ((epPropsTarget[i] & epPropsSource[i]) != epPropsSource[i]) 
      {
        return 1;
      }
    }
  }

  return 0;
}

void fix_endpoints() 
{

  pEndpoints = endpoints;

  unsigned char configurationIndex;
  for (configurationIndex = 0; configurationIndex < descriptors->device.bNumConfigurations; ++configurationIndex) 
  {
    uint8_t sourceProperties[ENDPOINT_MAX_NUMBER] = {};
    get_endpoint_properties(configurationIndex, sourceProperties);
    /*
      print_endpoint_properties(usedEndpoints);
      print_endpoint_properties(endpointProperties);
    */
    int renumber = compare_endpoint_properties(sourceProperties, targetProperties);
    unsigned char endpointNumber = 0;
    struct p_configuration * pConfiguration = descriptors->configurations + configurationIndex;
    //printf("configuration: %hhu\n", pConfiguration->descriptor->bConfigurationValue);
    unsigned char interfaceIndex;
    for (interfaceIndex = 0; interfaceIndex < pConfiguration->descriptor->bNumInterfaces; ++interfaceIndex) 
    {
      struct p_interface * pInterface = pConfiguration->interfaces + interfaceIndex;
      unsigned char altInterfaceIndex;
      for (altInterfaceIndex = 0; altInterfaceIndex < pInterface->bNumAltInterfaces; ++altInterfaceIndex) 
      {
        struct p_altInterface * pAltInterface = pInterface->altInterfaces + altInterfaceIndex;
        //printf("  interface: %hhu:%hhu\n", pAltInterface->descriptor->bInterfaceNumber, pAltInterface->descriptor->bAlternateSetting);
        unsigned char endpointIndex;
        for (endpointIndex = 0; endpointIndex < pAltInterface->bNumEndpoints; ++endpointIndex) 
        {
          struct usb_endpoint_descriptor * endpoint =
              descriptors->configurations[configurationIndex].interfaces[interfaceIndex].altInterfaces[altInterfaceIndex].endpoints[endpointIndex];
          uint8_t originalEndpoint = endpoint->bEndpointAddress;
          if (renumber) 
          {
            ++endpointNumber;
            endpoint->bEndpointAddress = (endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) | endpointNumber;
          }
          if (1)
          {
            printf("\n#i:EP");
            printf(" %s", ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) ? "IN" : "OUT");
            printf(" %s",
                (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT ? "INTERRUPT" :
                (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK ? "BULK" :
                (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_ISOC ?
                    "ISOCHRONOUS" : "UNKNOWN");
            printf(" %hu", originalEndpoint & USB_ENDPOINT_NUMBER_MASK);
            
            if (originalEndpoint != endpoint->bEndpointAddress) 
            {
              //printf(KRED" -> %hu"KNRM, endpointNumber);
              printf(" -> %hu", endpointNumber);
            }
            printf(" intv %dms pkt %dB", endpoint->bInterval, endpoint->wMaxPacketSize);
          }
          if ((originalEndpoint & USB_ENDPOINT_NUMBER_MASK) == 0) 
          {
            PRINT_ERROR_OTHER("invalid endpoint number")
            continue;
          }
          if (configurationIndex > 0) 
          {
            continue;
          }
          if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_INT) 
          {
            if (1)
              printf("      endpoint %hu won't be configured (not an INTERRUPT endpoint)\n", endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
            continue;
          }
          if (endpoint->wMaxPacketSize > MAX_PAYLOAD_SIZE_EP) 
          {
            if (1)
              printf("      endpoint %hu won't be configured (max packet size %hu > %hu)\n", endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK, endpoint->wMaxPacketSize, MAX_PAYLOAD_SIZE_EP);
            continue;
          }
          if (endpointNumber > MAX_ENDPOINTS) 
          {
            if (1)
              printf("      endpoint %hu won't be configured (endpoint number %hhu > %hhu)\n", endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK, endpointNumber, MAX_ENDPOINTS);
            continue;
          }
          U2S_ENDPOINT(originalEndpoint) = endpoint->bEndpointAddress;
          S2U_ENDPOINT(endpoint->bEndpointAddress) = originalEndpoint;
          pEndpoints->number = endpoint->bEndpointAddress;
          pEndpoints->type = endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
          pEndpoints->size = endpoint->wMaxPacketSize;
          ++pEndpoints;
        }
      }
    }
  }
}

static int add_descriptor(uint16_t wValue, uint16_t wIndex, uint16_t wLength, void * data) 
{

  if (pDesc + wLength > desc + MAX_DESCRIPTORS_SIZE || pDescIndex >= descIndex + MAX_DESCRIPTORS) 
  {
    fprintf(stderr, "%s:%d %s: unable to add descriptor wValue=0x%04x wIndex=0x%04x wLength=%u (available=%u)\n",
        __FILE__, __LINE__, __func__, wValue, wIndex, wLength, (unsigned int)(MAX_DESCRIPTORS_SIZE - (pDesc - desc)));
    return -1;
  }

  pDescIndex->offset = pDesc - desc;
  pDescIndex->wValue = wValue;
  pDescIndex->wIndex = wIndex;
  pDescIndex->wLength = wLength;
  memcpy(pDesc, data, wLength);
  pDesc += wLength;
  ++pDescIndex;

  return 0;
}

int send_descriptors() 
{

  int ret;

  ret = add_descriptor ((USB_DT_DEVICE << 8), 0, sizeof(descriptors->device), &descriptors->device);
  if (ret < 0)
  {
    return -1;
  }

  ret = add_descriptor ((USB_DT_STRING << 8), 0, sizeof(descriptors->langId0), &descriptors->langId0);
  if (ret < 0)
  {
    return -1;
  }

  unsigned int descNumber;
  for(descNumber = 0; descNumber < descriptors->device.bNumConfigurations; ++descNumber) 
  {

    ret = add_descriptor ((USB_DT_CONFIG << 8) | descNumber, 0, descriptors->configurations[descNumber].descriptor->wTotalLength, descriptors->configurations[descNumber].raw);
    if (ret < 0)
    {
      return -1;
    }
  }

  for(descNumber = 0; descNumber < descriptors->nbOthers; ++descNumber) 
  {

    ret = add_descriptor (descriptors->others[descNumber].wValue, descriptors->others[descNumber].wIndex, descriptors->others[descNumber].wLength, descriptors->others[descNumber].data);
    if (ret < 0)
    {
      return -1;
    }
  }
  ret = adapter_send (adapter, E_TYPE_DESCRIPTORS, desc, pDesc - desc);
  if (ret < 0)
  {
    return -1;
  }

  return 0;
}

static int send_index ()
{

  if (descIndexSent)
  {
    return 0;
  }

  descIndexSent = 1;

  return adapter_send (adapter, E_TYPE_INDEX, (unsigned char *)&descIndex, (pDescIndex - descIndex) * sizeof(*descIndex));
}

static int send_endpoints() 
{

  if (endpointsSent)
  {
    return 0;
  }

  endpointsSent = 1;

  return adapter_send (adapter, E_TYPE_ENDPOINTS, (unsigned char *)&endpoints, (pEndpoints - endpoints) * sizeof(*endpoints));
}

static int poll_all_endpoints ()
{

  int ret = 0;
  unsigned char i;
  for (i = 0; i < sizeof(*serialToUsbEndpoint) / sizeof(**serialToUsbEndpoint) && ret >= 0; ++i)
  {
    uint8_t endpoint = S2U_ENDPOINT (USB_DIR_IN | i);
    if (endpoint)
    {
      ret = gusb_poll (usb, endpoint);
    }
  }
  return ret;
}

static int send_out_packet(s_packet * packet) 
{

  s_endpointPacket * epPacket = (s_endpointPacket *)packet->value;

  return gusb_write (usb, S2U_ENDPOINT(epPacket->endpoint), epPacket->data, packet->header.length - 1);
}

static int send_control_packet(s_packet * packet) 
{

  struct usb_ctrlrequest * setup = (struct usb_ctrlrequest *)packet->value;
  if ((setup->bRequestType & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) 
  {
    if (setup->wIndex != 0) {
      setup->wIndex = S2U_ENDPOINT(setup->wIndex);
    }
  }

  if (setup->bRequestType == USB_DIR_IN
      && setup->bRequest == USB_REQ_GET_DESCRIPTOR
      && (setup->wValue >> 8) == USB_DT_DEVICE_QUALIFIER) {
    // device qualifier descriptor is for high speed devices
    printf("force stall for get device qualifier\n");
    return adapter_send(adapter, E_TYPE_CONTROL_STALL, NULL, 0);
  }

  return gusb_write(usb, 0, packet->value, packet->header.length);
}

static void dump(unsigned char * data, unsigned char length)
{
  int i;
  for (i = 0; i < length; ++i)
  {
    printf("%02x ", data[i]);
  }
  fflush (stdout);
}

/*
#i@1207922.000:process pkt type 0x6
#i:next IN packet
#d:adapter sent 23B
#d:adapter recv 1
#d:adapter recv 1
#d:adapter recv 8
#i@1207923.000:process pkt type 0x4
#i:ready ctrl
#d:adapter sent 66B
#d:adapter recv 2
#d:adapter recv 8
#i@1207953.000:process pkt type 0x4
*/

const char* npktst[] = {"dsc", "idx", "eps", "rst", "ctl", "stl", "inn", "out", "dbg"};
static int process_packet(int user, s_packet * packet)
{
  extern unsigned long get_millis (char st);
  unsigned char type = packet->header.type;
  if (0 || adapter_debug (0xff) & 0x0f)
    fprintf (stdout, "\n#i@%.3f:process pkt type 0x%x/%s", get_fms(), type, npktst[type]);
  int ret = 0;

  switch (packet->header.type)
  {
  case E_TYPE_DESCRIPTORS:
    ret = send_index ();
    if (adapter_debug (0xff) & 0x0f)
    {
      fprintf (stdout, "\n#i:ready descriptors");
      fflush (stdout);
    }
    break;
  case E_TYPE_INDEX:
    ret = send_endpoints ();
    if (adapter_debug (0xff) & 0x0f)
    {
      fprintf (stdout, "\n#i:ready indexes");
      fflush (stdout);
    }
    break;
  case E_TYPE_ENDPOINTS:
    gtimer_close (init_timer);
    init_timer = -1;
    printf ("\n#i:ready");
    fflush (stdout);
    ret = poll_all_endpoints ();
    break;
  case E_TYPE_IN:
    if (inPending > 0) 
    {
      ret = gusb_poll (usb, inPending);
      inPending = 0;
      if (ret != -1) 
      {
        ret = send_next_in_packet ();
        if (adapter_debug (0xff) & 0x0f)
        {
          fprintf (stdout, "\n#i:next IN packet");
          fflush (stdout);
        }
      }
    }
    break;
  case E_TYPE_OUT:
    ret = send_out_packet (packet);
    if (adapter_debug (0xff) & 0x0f)
    {
      fprintf (stdout, "\n#i:ready out pkt");
      fflush (stdout);
    }
    break;
  case E_TYPE_CONTROL:
    ret = send_control_packet (packet);
    if (adapter_debug (0xff) & 0x0f)
    {
      fprintf (stdout, "\n#i:ready ctrl");
      fflush (stdout);
    }
    break;
  case E_TYPE_DEBUG:
    {
      printf ("\n#w:%.3f debug pkt %dB: ", get_millis (0)/1000.0f, packet->header.length);
      dump (packet->value, packet->header.length);
    }
    break;
  case E_TYPE_RESET:
    ret = -1;
    break;
  default:
    {
      fprintf (stdout, "\n#w:%.3f unhandled packet type=0x%02x: ", get_millis (0)/1000.0f, type);
      //dump (packet->value, packet->header.length);
    }
    break;
  }

  if (ret < 0)
  {
    done = 1;
  }
  return ret;
}

int proxy_init (int vid, int pid) 
{

  char * path = usb_select (vid, pid);

  if (path == NULL) 
  {
    return -1;
  }
  //
  usb = gusb_open_path (path);

  if (usb < 0) 
  {
    free (path);
    return -1;
  }

  descriptors = gusb_get_usb_descriptors (usb);
  if (descriptors == NULL) 
  {
    free (path);
    return -1;
  }

  printf("\n#i:using device: VID 0x%04x PID 0x%04x PATH %s", descriptors->device.idVendor, descriptors->device.idProduct, path);

  free(path);

  if (descriptors->device.bNumConfigurations == 0) {
    PRINT_ERROR_OTHER ("missing configuration")
    return -1;
  }

  if (descriptors->configurations[0].descriptor->bNumInterfaces == 0) {
    PRINT_ERROR_OTHER ("missing interface")
    return -1;
  }

  if (descriptors->configurations[0].interfaces[0].bNumAltInterfaces == 0) {
    PRINT_ERROR_OTHER ("missing altInterface")
    return -1;
  }

  fix_endpoints ();
  print_endpoints ();
  return 0;
}

static int timer_close (int user) 
{
  done = 1;
  return 1;
}

static int timer_read (int user) 
{
  /*
   * Returning a non-zero value will make gpoll return,
   * this allows to check the 'done' variable.
   */
  return 1;
}

int proxy_start (char * port) 
{

  int ret = set_prio ();
  if (ret < 0)
  {
    PRINT_ERROR_OTHER ("\n#e:failed to set process priority!")
    return -1;
  }

  adapter = adapter_open (port, process_packet, adapter_send_callback, adapter_close_callback);

  //adapter_send (adapter, E_TYPE_RESET, NULL, 0);

  if(adapter < 0) 
  {
    return -1;
  }
  if (adapter_debug (0xff) & 0x0f)
    printf ("\n#i:sending descriptors");
  if (send_descriptors () < 0) 
  {
    return -1;
  }

  init_timer = gtimer_start (0, 1000000, timer_close, timer_close, gpoll_register_fd);
  if (init_timer < 0) 
  {
    return -1;
  }

  if (adapter_debug (0xff) & 0x0f)
  {
    printf ("\n#i:started init timer");
  }
  ret = gusb_register (usb, 0, usb_read_callback, usb_write_callback, usb_close_callback, gpoll_register_fd);
  if (ret < 0)
  {
    return -1;
  }

  if (adapter_debug (0xff) & 0x0f)
  {
    printf ("\n#i:started polling timer");
  }
  int timer = gtimer_start (0, 10000, timer_read, timer_close, gpoll_register_fd);
  if (timer < 0) 
  {
    return -1;
  }

  while (!done) 
  {
    gpoll ();
  }

  printf ("\n#i:cleaning up");
  gtimer_close (timer);
  adapter_send (adapter, E_TYPE_RESET, NULL, 0);
  gusb_close (usb);
  adapter_close ();
  
  if (init_timer >= 0)
  {
    //PRINT_ERROR_OTHER("Failed to start the proxy: initialization timeout expired!")
    printf ("\n#e:failed to start the job, closing");
    gtimer_close (init_timer);
    return -1;
  }
  //

  return 0;
}

void proxy_stop ()
{
  done = 1;
}
