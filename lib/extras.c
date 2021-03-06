/*
 Copyright (c) 2015 mirel lazar <mirel.t.lazar@gmail.com>
 License: GPLv3

 */

/*
modification history
--------------------
01a,08jan2015, mirel lazar <mirel.t.lazar@gmail.com>
               adapted for motion feedback controller
*/
#define _POSIX_C_SOURCE 199309L

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <termios.h>
#include <time.h>

#include <sys/ioctl.h>
#include <sys/uio.h>

#include <libintl.h>
#define _(STRING)    gettext(STRING)

#include <errno.h>

#include "extras.h"

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); if (DEBUG) fflush (stderr);} while (0)

/*
 * broadcast socket work
 */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int inet_aton(const char *cp, struct in_addr *inp);
//MFC platform control
/*
  int pkt_type;
  int pkt_dof_type;
  int data[6];    //{pitch, roll, yaw, surge, sway, heave}
  int speed;
  //
Pitch is the tilt of the car forwards or backwards in [°]
Roll is how much the car is dipped to the left or right in [°]
Yaw is the heading of the car (north, east, south, west) in [°]

Surge means the acceleration of the car in longitudinal direction [g]
Sway means the acceleration of the car in lateral direction [g]
Heave means the acceleration up and down [g]

//motion data packet
pkt[0] = PKTT_DATA;
pkt[1] = PKTT_2DOF;
//motion
pkt[2] = pl_pitch;  //pitch
pkt[3] = pl_roll;   //roll
pkt[4] = 0;         //yaw
pkt[5] = 0;         //surge
pkt[6] = 0;         //sway
pkt[7] = 0;         //heave
//speed
pkt[8] = 0;         //speed
*/
//#pragma pack(push)
//#pragma pack(1)
static int mfc_pkt[MFC_PKTSIZE] = {0};
//#pragma pack(pop)

static int mfc_bcast_sock = -1;
struct sockaddr_in mfc_si_other, si_me;
//
int mfc_bcast_prep (char *dst, int svr)
{
  //sockets
  if (!dst || !*dst)
    return 0;
  int s = -1;
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#ERR:socket");
    return 0;
  }
#if 0
  int broadcast = 1;
  setsockopt (s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
#endif
  if (svr)
  {
    memset ((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons (MFCSVR_PORT);
    si_me.sin_addr.s_addr = htonl (INADDR_ANY);
    if (bind (s, (struct sockaddr*)&si_me, sizeof (si_me)) == -1)
    {
      fprintf(stderr, "\n#ERR:bind");
      close (s);
      return 0;
    }
  }
  else
  {
    //destination
    memset((char *) &mfc_si_other, 0, sizeof (mfc_si_other));
    mfc_si_other.sin_family = AF_INET;
    mfc_si_other.sin_port = htons (MFCSVR_PORT);
  #if 1
    mfc_si_other.sin_addr.s_addr = inet_addr (dst); //htonl (INADDR_BROADCAST);
    if (mfc_si_other.sin_addr.s_addr == -1)
    {
      close (s);
      return 0;
    }
  #else
    if (inet_aton(dst, &mfc_si_other.sin_addr) == 0)
    {
      fprintf(stderr, "\n#!inet_aton() failed\n");
      close (s);
      return 0;
    }
  #endif
  }
  mfc_bcast_sock = s;
  mfc_pkt[0] = PKTT_CTRL;
  //
  return s;
}

int mfc_bcast_send ()
{
  int bs = 0;
  if (mfc_bcast_sock > 0)
  {
    bs = sendto (mfc_bcast_sock, &mfc_pkt, mfc_bcast_pktlen (), 0, (struct sockaddr*)&mfc_si_other, sizeof(mfc_si_other));
    if (bs < 0)
      printf("\n#ERR:sendto");
    else
      debug_print ("\n#i:%d<%dB sent", mfc_bcast_sock, bs);
  }
  fflush (stdout);
  return bs;
}

int mfc_bcast_receive ()
{
  //get network data
  if (mfc_bcast_sock < 0)
    return 0;
  int rb = 0, tb = 0, pks = sizeof (mfc_pkt), rtmo = 500;
  while (tb < pks && rtmo)
  {
    rb = recvfrom (mfc_bcast_sock, mfc_pkt + tb, pks - tb, 0, NULL, 0);
    if (rb > 0)
      tb += rb;
    else if (rb == 0)
      rtmo--;
    else
    {
      printf ("\n#E:recvfrom() failed");
      break;
    }
  }
  debug_print ("\n#i:%d>%dB vs %d", mfc_bcast_sock, tb, pks);
  if (1)
  {
    for (int i = 0; i < tb; i++)
      debug_print (", %d", mfc_pkt[i]);
  }
  //
  return tb;
}

int *mfc_bcast_pktget ()
{
  return mfc_pkt;
}

int mfc_bcast_pktlen ()
{
  return MFC_PKTSIZE*sizeof(int);
}

void mfc_bcast_close ()
{
  if (mfc_bcast_sock > 0)
    close (mfc_bcast_sock);
  mfc_bcast_sock = -1;
}

//DASH
/*
  int pkt_type;
  int pkt_dof_type;
  int data[6];    //{pitch, roll, yaw, surge, sway, heave}
  int speed;
  //
Pitch is the tilt of the car forwards or backwards in [°]
Roll is how much the car is dipped to the left or right in [°]
Yaw is the heading of the car (north, east, south, west) in [°]

Surge means the acceleration of the car in longitudinal direction [g]
Sway means the acceleration of the car in lateral direction [g]
Heave means the acceleration up and down [g]

//motion data packet
pkt[0] = PKTT_DATA;
pkt[1] = PKTT_2DOF;
//motion
pkt[2] = pl_pitch;  //pitch
pkt[3] = pl_roll;   //roll
pkt[4] = 0;         //yaw
pkt[5] = 0;         //surge
pkt[6] = 0;         //sway
pkt[7] = 0;         //heave
//speed
pkt[8] = 0;         //speed
*/
//#pragma pack(push)
//#pragma pack(1)
static int mfcdash_pkt[MFCDASH_PKTSIZE] = {0};
//#pragma pack(pop)

static int mfcdash_bcast_sock = -1;
struct sockaddr_in mfcdash_si_other, sidash_me;
//
int mfcdash_bcast_prep (char *dst, int svr)
{
  //sockets
  if (!dst || !*dst)
    return 0;
  int s = -1;
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#ERR:socket");
    return 0;
  }
#if 0
  int broadcast = 1;
  setsockopt (s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
#endif
  if (svr)
  {
    memset ((char *) &sidash_me, 0, sizeof(sidash_me));
    sidash_me.sin_family = AF_INET;
    sidash_me.sin_port = htons (MFCDASH_PORT);
    sidash_me.sin_addr.s_addr = htonl (INADDR_ANY);
    if (bind (s, (struct sockaddr*)&sidash_me, sizeof (sidash_me)) == -1)
    {
      fprintf(stderr, "\n#ERR:bind");
      close (s);
      return 0;
    }
  }
  else
  {
    //destination
    memset((char *) &mfcdash_si_other, 0, sizeof (mfcdash_si_other));
    mfcdash_si_other.sin_family = AF_INET;
    mfcdash_si_other.sin_port = htons (MFCDASH_PORT);
  #if 1
    mfcdash_si_other.sin_addr.s_addr = inet_addr (dst); //htonl (INADDR_BROADCAST);
    if (mfcdash_si_other.sin_addr.s_addr == -1)
    {
      close (s);
      return 0;
    }
  #else
    if (inet_aton(dst, &mfcdash_si_other.sin_addr) == 0)
    {
      fprintf(stderr, "\n#!inet_aton() failed\n");
      close (s);
      return 0;
    }
  #endif
  }
  mfcdash_bcast_sock = s;
  mfcdash_pkt[0] = PKTT_CTRL;
  //
  return s;
}

int mfcdash_bcast_send ()
{
  int bs = 0;
  if (mfcdash_bcast_sock > 0)
  {
    bs = sendto (mfcdash_bcast_sock, &mfcdash_pkt, mfcdash_bcast_pktlen (), 0, (struct sockaddr*)&mfcdash_si_other, sizeof(mfcdash_si_other));
    if (bs < 0)
      printf("\n#ERR:sendto");
    else
      debug_print ("\n#i:%d<%dB sent", mfcdash_bcast_sock, bs);
  }
  fflush (stdout);
  return bs;
}

int mfcdash_bcast_receive ()
{
  //get network data
  if (mfcdash_bcast_sock < 0)
    return 0;
  int rb = 0, tb = 0, pks = sizeof (mfcdash_pkt), rtmo = 500;
  while (tb < pks && rtmo)
  {
    rb = recvfrom (mfcdash_bcast_sock, mfcdash_pkt + tb, pks - tb, 0, NULL, 0);
    if (rb > 0)
      tb += rb;
    else if (rb == 0)
      rtmo--;
    else
    {
      printf ("\n#E:recvfrom() failed");
      break;
    }
  }
  debug_print ("\n#i:%d>%dB vs %d", mfcdash_bcast_sock, tb, pks);
  if (1)
  {
    for (int i = 0; i < tb; i++)
      debug_print (", %d", mfcdash_pkt[i]);
  }
  //
  return tb;
}

int *mfcdash_bcast_pktget ()
{
  return mfcdash_pkt;
}

int mfcdash_bcast_pktlen ()
{
  return MFCDASH_PKTSIZE*sizeof(int);
}

void mfcdash_bcast_close ()
{
  if (mfcdash_bcast_sock > 0)
    close (mfcdash_bcast_sock);
  mfcdash_bcast_sock = -1;
}
//-ends here

int bcast_sock = -1;
struct sockaddr_in si_other;
void bcast_prep (char *dst)
{
  //sockets
  if (!dst || !*dst)
    return;
  int s;
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#ERR:socket");
    return;
  }
#if 0
  int broadcast = 1;
  setsockopt (s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
#endif
  memset ((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons (65002);
  si_me.sin_addr.s_addr = htonl (INADDR_ANY);
  if (bind (s, (struct sockaddr*)&si_me, sizeof (si_me)) == -1)
  {
    fprintf(stderr, "\n#ERR:bind");
    close (s);
    return;
  }
  memset((char *) &si_other, 0, sizeof (si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons (65001);
#if 0
  si_other.sin_addr.s_addr = inet_addr (dst); //htonl (INADDR_BROADCAST);
  if (si_other.sin_addr.s_addr == -1)
  {
    close (s);
    return;
  }
#else
  if (inet_aton(dst, &si_other.sin_addr) == 0)
  {
    fprintf(stderr, "\n#!inet_aton() failed\n");
    close (s);
    return;
  }
#endif
  bcast_sock = s;
}

#if 0
void bcast_send ()
{
#if 1
  if (bcast_sock > 0 && sendto (bcast_sock, &pkt, sizeof(SEvtPacket), 0, (struct sockaddr*)&si_other, sizeof(si_other)) == -1)
  {
          printf("\n#ERR:sendto");
  }
  //else
  //    printf ("%d .pkt %d sent\n", pkt.gaugedata[gTimeStamp], pktk);
#endif
}
#endif

void bcast_close ()
{
  if (bcast_sock > 0)
    close (bcast_sock);
}

// https://stackoverflow.com/questions/2602823/in-c-c-whats-the-simplest-way-to-reverse-the-order-of-bits-in-a-byte
//Index 1==0b0001 => 0b1000
//Index 7==0b0111 => 0b1110
//etc
// Detailed breakdown of the math
//  + lookup reverse of bottom nibble
//  |       + grab bottom nibble
//  |       |        + move bottom result into top nibble
//  |       |        |     + combine the bottom and top results
//  |       |        |     | + lookup reverse of top nibble
//  |       |        |     | |       + grab top nibble
//  V       V        V     V V       V
// (lookup[n&0b1111] << 4) | lookup[n>>4]

unsigned char reverse_char(unsigned char n)
{
  static const char lookup[16] = {
  0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
  0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };
   // Reverse the top and bottom nibble then swap them.
   return (lookup[n&0b1111] << 4) | lookup[n>>4];
}

char count_ones (char byte)
{
  static const char lookup [16] =
  {
    0, 1, 1, 2, 1, 2, 2, 3,
    1, 2, 2, 3, 2, 3, 3, 4
  };
  return lookup[byte & 0x0F] + lookup[byte >> 4];
}

long get_map (long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
in 0, -10000, 10000, 0, 65535 -> 

*/
long get_cmap (long x, long in_min, long in_max, long out_min, long out_max)
{
  long rv = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  //cap the output in range
  if (out_max > out_min) //normal range
  {
    if (rv > out_max)
      rv = out_max;
    if (rv < out_min)
      rv = out_min;
  }
  else //reversed range
  {
    if (rv > out_min)
      rv = out_min;
    if (rv < out_max)
      rv = out_max;
  }
  return rv;
}

float get_map_f (float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float get_cmap_f (float x, float in_min, float in_max, float out_min, float out_max)
{
  float rv = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (rv > out_max)
    rv = out_max;
  if (rv < out_min)
    rv = out_min;
  return rv;
}

//[b3 b2 b1 b0]
float get_float (char *buf, int off)
{
  long lv = (long)((buf[off+3]&0xff)<<24 | (buf[off+2]&0xff)<<16 | (buf[off+1]&0xff)<<8 | (buf[off]&0xff));
  //printf ("\n#lv n %ld", lv);
  float rv;
  //lv = ntohl (lv);
  //printf ("\n#lv h %ld", lv);
  memcpy (&rv, &lv, 4);
  return rv;
}

unsigned int get_uint (char *buf, int off)
{
  long lv = (long)((buf[off+3]&0xff)<<24 | (buf[off+2]&0xff)<<16 | (buf[off+1]&0xff)<<8 | (buf[off]&0xff));
  //printf ("\n#lv n %ld", lv);
  unsigned int rv;
  //lv = ntohl (lv);
  //printf ("\n#lv h %ld", lv);
  memcpy (&rv, &lv, 4);
  return rv;
}

int get_int (char *buf, int off)
{
  long lv = (long)((buf[off+3]&0xff)<<24 | (buf[off+2]&0xff)<<16 | (buf[off+1]&0xff)<<8 | (buf[off]&0xff));
  //printf ("\n#lv n %ld", lv);
  int rv;
  //lv = ntohl (lv);
  //printf ("\n#lv h %ld", lv);
  memcpy (&rv, &lv, 4);
  return rv;
}

unsigned short get_ushort (char *buf, int off)
{
  return (unsigned short)(buf[off+1]<<8|buf[off]);
}

short get_short (char *buf, int off)
{
  return (short)(buf[off+1]<<8|buf[off]);
}

//axis is: -1..-32768<L R>32768..1 (or the reverse)
int normal_axis (int val, int max)
{
  int rv;
  if (val < 0)
    rv = get_map (val, -1, -32768, -32768, -1);//max + val;
  else
    rv = get_map (val, 32768, 0, 0, 32768);//val;
  //
  return rv;
}

int normal_pedal (int val, int max)
{
  int rv;
  if (val < 0)
    rv = get_map (val, -1, -32768, -32768, -1);//max + val;
  else
    rv = get_map (val, 32768, 0, 0, 32768);//val;
  //
  return rv;
}

/* brake reports 0 to -32768*/
int normal_brake (int val, int max)
{
  int rv;
  if (val < 0)
    rv = get_map (val, -1, -32768, 0, -16384);//max + val;
  else
    rv = get_map (val, 32768, 0, -16384, -32768);//val;
  //
  return rv;
}

/* accel reports 0 to 32768 */
int normal_accel (int val, int max)
{
  int rv;
  if (val < 0)
    rv = get_map (val, -1, -32768, 0, 16384);//max + val;
  else
    rv = get_map (val, 32768, 0, 16384, 32768);//val;
  //
  return rv;
}

//ffb is: 128..255<L R>1..127
//output: 128..0<>-1..-127
int normal_ffb (int val, int max)
{
  return (val > 127) ? (max - val) : (-val);
}

//ffb is: 128..255<L R>1..127
//output: delta from mid: -128..0<L R>1..127
int normal_ffb2 (int val, int mid)
{
  return (val > mid)?(val - mid) : -(mid - val);
}

//ffb is: 255..128<>127..1
//output: -128..0<L R>1..127
int normal_ffb3 (int val)
{
  return get_cmap(val, 255, 0, -128, 128);
}

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

//get delta time since the previous call
unsigned int dtime_ms ()
{
  static unsigned long lms = 0;
  unsigned long cms = get_millis (0);
  unsigned long ms = cms - lms;
  lms = cms;
  return (unsigned int)ms;
}
