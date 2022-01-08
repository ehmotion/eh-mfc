/*
 Copyright (c) 2015 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3

 2019 mirel.t.lazar@gmail.com
 adapted for USB message extraction
 */

#include <adapter.h>
#include <gserial.h>
#include <gpoll.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_ADAPTERS 7

#define PRINT_ERROR_OTHER(msg) fprintf(stderr, "%s:%d %s: %s\n", __FILE__, __LINE__, __func__, msg);
//use flag 0x20 for capture
static char adapterDbg = 0;

static struct {
  s_packet packet;
  unsigned int bread;
  int serial;
  ADAPTER_READ_CALLBACK fp_packet_cb;
} adapters[MAX_ADAPTERS];

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

static int csock = -1;
static struct sockaddr_in mfc_si_other;
typedef struct PACKED {
  uint8_t type;
  uint16_t length;
} n_header;

typedef struct PACKED
{
  n_header header;
  uint8_t value[MAX_PACKET_VALUE_SIZE+10];
} n_packet;
static n_packet npkt; //network packet
static n_packet cpkt; //client data packet
// {<E_TYPE_DEBUG>, 5, vid[0], vid[1], pid[0], pid[1], <delta_ts>}
extern int vid, pid;
// when st is not 0, it returns the time delta from the first call
unsigned long get_millis (char st)
{
  static long st_ms = 0;
  struct timespec lts;
  long ctime;
  //get current time
  //clock_gettime (CLOCK_REALTIME, &lts);
  clock_gettime (CLOCK_BOOTTIME, &lts);
  ctime = lts.tv_sec * 1000L + lts.tv_nsec / 1000000L;
  if (st_ms == 0)
  {
    st_ms = ctime;
    if (st)
      return 0;
  }
  if (st)
    return ((int)ctime - st_ms);
  return (ctime);
}

unsigned long get_millis_delta ()
{
  static long lst_ms = 0;
  long ctime = get_millis(1);
  long dtime = ctime - lst_ms;
  //first time call, return delta time as 0
  if (lst_ms == 0)
    dtime = 0;
  lst_ms = ctime;
  return (dtime);
}

float get_fms()
{
  return get_millis (1) / 1000.0f;
  //return get_millis_delta () / 1000.0f;
}
//
static int client_init ()
{
  int s = -1;
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#E:socket");
    return 0;
  }
  //destination
  memset((char *) &mfc_si_other, 0, sizeof (mfc_si_other));
  mfc_si_other.sin_family = AF_INET;
  mfc_si_other.sin_port = htons (64405);
  mfc_si_other.sin_addr.s_addr = htonl(INADDR_LOOPBACK);//htonl(0x7f000001L); //htonl (INADDR_BROADCAST);
  csock = s;
  //prep debug data
  npkt.header.type = E_TYPE_DESCRIPTORS;//converts to PKTT_CTRL in MFC
  npkt.header.length = 8;
  npkt.value[0] = (vid >> 8) & 0xff;
  npkt.value[1] = vid & 0xff;
  npkt.value[2] = (pid >> 8) & 0xff;
  npkt.value[3] = pid & 0xff;
  unsigned long clts = get_millis (0) & 0xffffffff;
  //timestamp 4 bytes
  memcpy((void *)(npkt.value + 4), (void *)&clts, 4);
  //write debug pkt first
  (void)sendto (csock, (const void *)&npkt, npkt.header.length + 2, 0, (struct sockaddr*)&mfc_si_other, sizeof (mfc_si_other));
  //
  return s;
}
//
static int client_close ()
{
  if (csock > 0)
    close (csock);
  csock = -1;
  //
  return 1;
}
//
extern const char* npktst[];
static int client_send (n_packet *packet, char in)
{
  static int npkt_hdr = sizeof(n_header);
  if (csock < 0)
    client_init ();
  if (csock > 0)
  {
    //we need 4 bytes for the wheel vid+pid and another 4 for timestamp: we have a max of 1024 bytes buffer on reception
    unsigned long clts = get_millis (0) & 0xffffffff;
    memcpy((void *)(npkt.value + 8), (void *)packet, packet->header.length + npkt_hdr);
    //timestamp 4 bytes
    memcpy((void *)(npkt.value + 4), (void *)&clts, 4);
    //header
    npkt.header.type = packet->header.type;
    npkt.header.length = packet->header.length + 8 + npkt_hdr;//8 for the vid&pid+timestamp + packet header length
    //write USB data
    if (sendto (csock, (const void *)&npkt, npkt.header.length + npkt_hdr, 0, (struct sockaddr*)&mfc_si_other, sizeof (mfc_si_other)) < 0)
    {
      printf ("\n#E:client_send");
      fflush (stdout);
    }
  }
  //
  if (0 && packet->header.type != E_TYPE_IN)// || packet->header.length != 65)
  {
    int i;
    printf ("\n#PKT@%.3f%s%3dB/0x%02x/%s:", get_fms(), in?">":"<", packet->header.length, packet->header.type, npktst[packet->header.type]);
    for (i = 0; i < packet->header.length; i++)
      printf (" %02x", packet->value[i]);
    fflush (stdout);
  }
  #if 0
  if (packet->header.length > 0)
  {
    switch (packet->header.type)
    {
      case E_TYPE_IN:
      case E_TYPE_OUT:
        break;
      case E_TYPE_CONTROL:
        //check auth
        if (0 && packet->value[0] == 0xf1)
        {
          if (packet->value[2] == 0x00)
          {
            printf ("\n#i:auth stage %02X", packet->value[1]);
            fflush (stdout);
          }
        }
        if (0 && packet->header.length)
        {
          int i;
          printf ("\n#i:CTL %03d bytes, type %x\n##", packet->header.length, packet->header.type);
          for (i = 0; i < packet->header.length; i++)
            printf ("%02x ", packet->value[i]);
        }
        break;
      default:
        ;//not used
    } //switch
  } //if pkt len
  #endif
  return 1;
}

void adapter_init(void) __attribute__((constructor (101)));
void adapter_init(void) {
  unsigned int i;
  for (i = 0; i < sizeof(adapters) / sizeof(*adapters); ++i) 
  {
    adapters[i].serial = -1;
  }
}

unsigned char adapter_debug (unsigned char dbg)
{
  char ret = adapterDbg;
  if (dbg != 0xff)
    adapterDbg = dbg;
  return ret;
}

static inline int adapter_check(int adapter, const char * file, unsigned int line, const char * func) 
{
  if (adapter < 0 || adapter >= MAX_ADAPTERS) 
  {
    fprintf(stderr, "%s:%d %s: invalid device\n", file, line, func);
    return -1;
  }
  if (adapters[adapter].serial < 0) 
  {
    fprintf(stderr, "%s:%d %s: no such adapter\n", file, line, func);
    return -1;
  }
  return 0;
}
#define ADAPTER_CHECK(device,retValue) \
  if(adapter_check(device, __FILE__, __LINE__, __func__) < 0) { \
    return retValue; \
  }

static int adapter_recv(int adapter, const void * buf, int status) 
{
  if (0 || adapterDbg & 0x0f)
  {
    fprintf (stdout, "\n#d:adapter recv %d", status);
    fflush (stdout);
  }
  ADAPTER_CHECK(adapter, -1)

  if (status < 0) 
  {
    return -1;
  }

  int ret = 0;

  if (0 && (adapterDbg & 0x0f))
  {
    fprintf (stdout, "\n#d:adapter read %d", status);
    fflush (stdout);
  }
  if(adapters[adapter].bread + status <= sizeof(s_packet)) 
  {
    memcpy((unsigned char *)&adapters[adapter].packet + adapters[adapter].bread, buf, status);
    adapters[adapter].bread += status;
    unsigned int remaining;
    if(adapters[adapter].bread < sizeof(s_header))
    {
      remaining = sizeof(s_header) - adapters[adapter].bread;
    }
    else
    {
      remaining = adapters[adapter].packet.header.length - (adapters[adapter].bread - sizeof(s_header));
    }
    if(remaining == 0)
    {
      //store for network processing
      memcpy (&cpkt.value, &adapters[adapter].packet.value, adapters[adapter].packet.header.length);
      cpkt.header.type = adapters[adapter].packet.header.type;
      cpkt.header.length = adapters[adapter].packet.header.length;
      //send to network for processing
      client_send (&cpkt, 1);
      //
      ret = adapters[adapter].fp_packet_cb(adapter, &adapters[adapter].packet);
      adapters[adapter].bread = 0;
      gserial_set_read_size(adapters[adapter].serial, sizeof(s_header));
    }
    else
    {
      gserial_set_read_size(adapters[adapter].serial, remaining);
    }
  }
  else
  {
    // this is a critical error (no possible recovering)
    fprintf (stderr, "%s:%d %s: invalid data size (count=%u, available=%zu)\n", __FILE__, __LINE__, __func__, status, sizeof(s_packet) - adapters[adapter].bread);
    return -1;
  }

  return ret;
}

int adapter_send (int adapter, unsigned char type, const unsigned char * data, unsigned int count) 
{

  ADAPTER_CHECK(adapter, -1)

  if (count != 0 && data == NULL) 
  {
    PRINT_ERROR_OTHER("data is NULL")
    return -1;
  }

  do 
  {
    unsigned char length = MAX_PACKET_VALUE_SIZE;
    if (count < length) 
    {
      length = count;
    }
    s_packet packet = { .header = { .type = type, .length = length } };
    if (data) 
    {
      memcpy (packet.value, data, length);
    }
    data += length;
    count -= length;
    //store for network processing
    memcpy (&cpkt.value, &packet.value, length);
    cpkt.header.type = packet.header.type;
    cpkt.header.length = packet.header.length;
    //send to network for processing
    client_send (&cpkt, 0);
    //
    int ret = gserial_write (adapters[adapter].serial, &packet, 2 + length);
    if (adapterDbg & 0x0f)
    {
      fprintf (stdout, "\n#d:adapter sent %dB vs %dB", ret, cpkt.header.length);
      fflush (stdout);
    }
    if(ret < 0) 
    {
      return -1;
    }
  } while (count > 0);

  return 0;
}

int adapter_open(const char * port, ADAPTER_READ_CALLBACK fp_read, ADAPTER_WRITE_CALLBACK fp_write, ADAPTER_CLOSE_CALLBACK fp_close) 
{
  extern int sbaud;
  char tbuf[PATH_MAX];
  int serial = -1;
  if (realpath (port, tbuf))
    serial = gserial_open (tbuf, sbaud);
  if (serial < 0) 
  {
    if (adapterDbg & 0x0f)
    {
      fprintf (stdout, "\n#e:failed to open adapter '%s'->'%s':%d", port, tbuf, serial);
      fflush (stdout);
    }
    return -1;
  }

  unsigned int i;
  for (i = 0; i < sizeof(adapters) / sizeof(*adapters); ++i) 
  {
    if (adapters[i].serial < 0) 
    {
      adapters[i].serial = serial;
      adapters[i].fp_packet_cb = fp_read;
      int ret = gserial_register (serial, i, adapter_recv, fp_write, fp_close, gpoll_register_fd);
      if (ret < 0) 
      {
        return -1;
      }
      if (adapterDbg & 0x0f)
      {
        fprintf (stdout, "\n#d:adapter opened '%s'->'%s':%d", port, tbuf, serial);
        fflush (stdout);
      }
      return i;
    }
  }

  gserial_close(serial);

  return -1;
}

int adapter_close ()
{
  client_close ();
  return 0;
}

