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
            do { if (DEBUG) fprintf(stdout, fmt, __VA_ARGS__); if (DEBUG) fflush (stdout);} while (0)
#define debug_error(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); if (DEBUG) fflush (stderr);} while (0)

#define log_print(fmt, ...) \
            do { fprintf(stdout, fmt, __VA_ARGS__); fflush (stdout);} while (0)
#define log_error(fmt, ...) \
            do { fprintf(stderr, fmt, __VA_ARGS__); fflush (stderr);} while (0)

#include <sched.h>
#include <stdio.h>

int set_max_prio ()
{
  /*
   * Set highest priority & scheduler policy.
   */
  struct sched_param p =
  { .sched_priority = sched_get_priority_max(SCHED_FIFO) };

  if( sched_setscheduler(0, SCHED_FIFO, &p) < 0 )
  {
    perror("sched_setscheduler");
    return -1;
  }
  return 0;
}

/*
 * broadcast socket work
 */
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
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
struct in_addr mfc_si_mcast;
struct ip_mreq
{
  struct in_addr imr_multiaddr; /* IP multicast group
                                                address */
  struct in_addr imr_address;   /* IP address of local
                                                interface */
  int imr_ifindex;              /* interface index */
};

int goto_park(int *pkt)
{
  for(int i = MFC_PIPITCH; i <= MFC_PITLOSS; i++)
  {
    if (pkt[i] != 0)
      pkt[i] -= pkt[i]/10;
    //if (pkt[i] < 0)
    //  pkt[i] ;
  }
  return 1;
}

int goto_ramp(int *pkt, char flg)
{
  static int prc = 0;
  if (flg && prc == 100)
    return 0;
  if (flg)
    prc += 3;
  else
    prc = 0;
  if (prc >= 100)
    prc = 100;
  for(int i = MFC_PIPITCH; i <= MFC_PITLOSS; i++)
  {
    //printf("\n#ramping to %.3f", ((float)prc)/100.f);
    pkt[i] = pkt[i] * ((float)prc)/100.f;
  }
  return 1;
}

//
int mfc_bcast_prep (char *dst, int svr)
{
  //sockets
  if (!dst || !*dst)
    return 0;
  int s = -1;
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    log_print ("\n#ERR:socket", 0);
    return 0;
  }
#if 0
  int broadcast = 1;
  setsockopt (s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
#endif
  if (svr)
  {
    int sopt = 1;
    //SO_REUSEADDR
#if 1
    if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &sopt, sizeof(sopt)) < 0)
    {
      log_print("\n#WARN:sockopt SO_REUSEADDR", 0);
    }
    //else
      //fprintf(stderr, "\n#i:sockopt SO_REUSEADDR");
#endif
#if 1
    if (setsockopt(s, SOL_SOCKET, IP_MULTICAST_LOOP, &sopt, sizeof(sopt)) < 0)
    {
      log_print("\n#WARN:sockopt IP_MULTICAST_LOOP", 0);
    }
    //else
      //fprintf(stderr, "\n#i:sockopt IP_MULTICAST_LOOP");
#endif
#if 0
    //SO_REUSEPORT
    if (setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &sopt, sizeof(sopt)) < 0)
    {
      fprintf(stderr, "\n#WARN:sockopt SO_REUSEPORT");
    }
    else
      fprintf(stderr, "\n#i:sockopt SO_REUSEPORT");
#endif
#define MCASTADDR "224.0.0.1"
    memset ((char *)&si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons (MFCSVR_PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind (s, (struct sockaddr*)&si_me, sizeof (si_me)) == -1)
    {
      log_print("\n#ERR:bind", 0);
      close (s);
      return -1;
    }
    struct ip_mreq group;
    memset((char *)&group, 0, sizeof(group));
    group.imr_multiaddr.s_addr = inet_addr(MCASTADDR);
    group.imr_address.s_addr = inet_addr(dst);
    if (setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0)
    {
      log_print ("\n#ERR:IP_ADD_MEMBERSHIP on <%s> errno 0x%x (%s)", dst, errno, strerror(errno));
      close(s);
      return -1;
    }
    //else
      //fprintf (stderr, "\n#i:sockopt IP_ADD_MEMBERSHIP");
  }
  else
  {
    //destination
    memset ((char *) &mfc_si_other, 0, sizeof (mfc_si_other));
    mfc_si_other.sin_family = AF_INET;
    mfc_si_other.sin_port = htons (MFCSVR_PORT);
    mfc_si_other.sin_addr.s_addr = inet_addr(MCASTADDR); //htonl (INADDR_BROADCAST);
    if (mfc_si_other.sin_addr.s_addr == -1)
    {
      log_print ("\n#ERR:wrong mcast address %s", MCASTADDR);
      close(s);
      return -1;
    }
    memset((char *)&mfc_si_mcast, 0, sizeof(mfc_si_mcast));
    mfc_si_mcast.s_addr = inet_addr(dst);
    if (setsockopt (s, IPPROTO_IP, IP_MULTICAST_IF, (char *)&mfc_si_mcast, sizeof(mfc_si_mcast)) < 0)
    {
      log_print("\n#ERR:IP_MULTICAST_IF on <%s> errno 0x%x (%s)", dst, errno, strerror(errno));
      close(s);
      return -1;
    }
    //else
      //fprintf (stderr, "\n#i:sockopt IP_MULTICAST_IF");
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
      log_error("\n#ERR:sendto", 0);
    else
      debug_print ("\n#i:%d<%dB sent", mfc_bcast_sock, bs);
  }
  //fflush (stdout);
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
      log_print ("\n#E:recvfrom() failed", 0);
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
    log_print ("\n#ERR:socket", 0);
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
      log_print("\n#ERR:bind", 0);
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
      log_print("\n#ERR:sendto", 0);
    else
      debug_print ("\n#i:%d<%dB sent", mfcdash_bcast_sock, bs);
  }
  //fflush (stdout);
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
      log_print ("\n#E:recvfrom() failed", 0);
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
    log_print ("\n#ERR:socket", 0);
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
    log_print("\n#ERR:bind", 0);
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
    log_print("\n#!inet_aton() failed", 0);
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

#include <limits.h>
#include <termios.h>
int serial_dash_open(char *_dashport, int spd)
{
    char tbuf[PATH_MAX];
    struct termios options;
    int _dashportfd = -1;
    //open using realpath() symlink to actual /dev/ttyUSBx
    if (realpath (_dashport, tbuf))
      _dashportfd = open (tbuf, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_dashportfd == -1)
    {
      printf ("\n#E:serial dash not found on %s", tbuf);
      //_dashport = NULL;
      return _dashportfd;
    }
    //set options
    tcgetattr (_dashportfd, &options);
    //
    cfsetispeed (&options, spd);
    cfsetospeed (&options, spd);
    cfmakeraw (&options);
    //
    if (tcsetattr (_dashportfd, TCSANOW, &options) < 0)
    {
      printf ("\n#E:cannot set options for MFCxDOF adapter %s", tbuf);
      close (_dashportfd);
      _dashportfd = -1;
      //_dashport = NULL;
    }
    printf("\n#i:connected to serial dash '%s', on %d", tbuf, _dashportfd);
    return _dashportfd;
}

/*
simdash serial custom protocol

hex bytes
ArqSerial - proto
|-----|
| 01  | hdr fixed
| 01  | hdr fixed
| PID | packet id: 0..127 or 255
| LEN | length: 1..32 w/o header and w/o 1B CRC
| D00 |
| D01 |
| ..  |
| DLL |
| CRC | CRC8 from: PID+LEN+D00..DLL
//PID
nextpacketid = Arq_LastValidPacket > 127 ? 0 : Arq_LastValidPacket + 1;
//CRC
currentCrc = updateCrc(currentCrc, packetID);
currentCrc = updateCrc(currentCrc, length);
for (i = 0; i < length; i++) {
    currentCrc = updateCrc(currentCrc, partialdatabuffer[i]);
}

SimHubArduino serial - proto
|-----|
| 03  | hdr fixed
| CMD | command, see below: P - custom proto
| S00 |
| S01 |
| ..  |

//
void Command_CustomProtocolData() {
	shCustomProtocol.read();
	FlowSerialWrite(0x15);
}
//
	shCustomProtocol.loop();

	// Wait for data
	if (FlowSerialAvailable() > 0) {
		if (FlowSerialTimedRead() == MESSAGE_HEADER)
		{
			lastSerialActivity = millis();
			// Read command
			loop_opt = FlowSerialTimedRead();

			if (loop_opt == '1') Command_Hello();
			else if (loop_opt == '8') Command_SetBaudrate();
			else if (loop_opt == 'J') Command_ButtonsCount();
			else if (loop_opt == '2') Command_TM1638Count();
			else if (loop_opt == 'B') Command_SimpleModulesCount();
			else if (loop_opt == 'A') Command_Acq();
			else if (loop_opt == 'N') Command_DeviceName();
			else if (loop_opt == 'I') Command_UniqueId();
			else if (loop_opt == '0') Command_Features();
			else if (loop_opt == '3') Command_TM1638Data();
			else if (loop_opt == 'V') Command_Motors();
			else if (loop_opt == 'S') Command_7SegmentsData();
			else if (loop_opt == '4') Command_RGBLEDSCount();
			else if (loop_opt == '6') Command_RGBLEDSData();
			else if (loop_opt == 'R') Command_RGBMatrixData();
			else if (loop_opt == 'M') Command_MatrixData();
			else if (loop_opt == 'G') Command_GearData();
			else if (loop_opt == 'L') Command_I2CLCDData();
			else if (loop_opt == 'K') Command_GLCDData(); // Nokia | OLEDS
			else if (loop_opt == 'P') Command_CustomProtocolData();
			else if (loop_opt == 'X')
			{
				String xaction = FlowSerialReadStringUntil(' ', '\n');
				if (xaction == F("list")) Command_ExpandedCommandsList();
				else if (xaction == F("mcutype")) Command_MCUType();
				else if (xaction == F("tach")) Command_TachData();
				else if (xaction == F("speedo")) Command_SpeedoData();
				else if (xaction == F("boost")) Command_BoostData();
				else if (xaction == F("temp")) Command_TempData();
				else if (xaction == F("fuel")) Command_FuelData();
				else if (xaction == F("cons")) Command_ConsData();
				else if (xaction == F("encoderscount")) Command_EncodersCount();
			}
		}
	}

	if (millis() - lastSerialActivity > 5000) {
		Command_Shutdown();
	}
*/
static const uint8_t crc_table_crc8[256] = { 0,213,127,170,254,43,129,84,41,252,86,131,215,2,168,125,82,135,45,248,172,121,211,6,123,174,4,209,133,80,250,47,164,113,219,14,90,143,37,240,141,88,242,39,115,166,12,217,246,35,137,92,8,221,119,162,223,10,160,117,33,244,94,139,157,72,226,55,99,182,28,201,180,97,203,30,74,159,53,224,207,26,176,101,49,228,78,155,230,51,153,76,24,205,103,178,57,236,70,147,199,18,184,109,16,197,111,186,238,59,145,68,107,190,20,193,149,64,234,63,66,151,61,232,188,105,195,22,239,58,144,69,17,196,110,187,198,19,185,108,56,237,71,146,189,104,194,23,67,150,60,233,148,65,235,62,106,191,21,192,75,158,52,225,181,96,202,31,98,183,29,200,156,73,227,54,25,204,102,179,231,50,152,77,48,229,79,154,206,27,177,100,114,167,13,216,140,89,243,38,91,142,36,241,165,112,218,15,32,245,95,138,222,11,161,116,9,220,118,163,247,34,136,93,214,3,169,124,40,253,87,130,255,42,128,85,1,212,126,171,132,81,251,46,122,175,5,208,173,120,210,7,83,134,44,249 };
#define sh_crc(crc, val) crc_table_crc8[crc ^ val];

int sh_serial_write(int fd, char *buf, int blen)
{
  int ret=0;
  static uint8_t pid = 0xff;
  static uint8_t asb[37] = {0x01, 0x01, 0x00};//01 01 PID LEN DD00..DD31 CRC
  if (fd == -1)
    return -1;
  //static char* be = "\n\r";
  //pid
  asb[2] = pid;
  if (pid++ > 127)
    pid = 0;
  //len
  if (blen > 32)
    blen = 32;
  asb[3] = blen;
  //data
  for (int i = 0; i < blen; i++)
    asb[4+i] = buf[i];
  //crc: #i.sent to dash: 01 01 66 07 03 50 31 3b 33 33 0a f1
  uint8_t lcrc = 0;
  for (int i = 0; i < blen+2; i++)
    lcrc = sh_crc(lcrc, asb[2+i]);
  asb[blen + 4] = lcrc;//&0xff;
  //write buffer start
  ret = write(fd,        // Handle to the Serial port
                   asb,     // Data to be written to the port
                   blen + 5  //No of bytes to write
                   );
  printf("\n#i.send to dash:");
  for (int i = 0; i < blen+5; i++)
    printf(" %02x", asb[i]);

  return ret;
}

int dash_serial_write(int fd, char *_dpkt, int blen)
{
  int ret = 0;
  //write buffer start
  if (write(fd, "!!", 2) == 2)
    ret = write(fd, (char *)_dpkt, blen);
  return ret>0?ret+2:ret;
}

static int mfc_fltoutpos[MFC_PKTSIZE];
//Exponentially Weighted Moving Average filter
int mfc_ewmaf_set(int dof, int input)
{
  if (dof < 0 || dof >= MFC_PKTSIZE)
    return 0;
  mfc_fltoutpos[dof] = input;
  return input;
}

int mfc_ewmaf(int dof, int input, float wi)
{
  if (dof < 0 || dof >= MFC_PKTSIZE)
    return input;
  mfc_fltoutpos[dof] = (int)(wi * (float)(input - mfc_fltoutpos[dof])) + mfc_fltoutpos[dof];
  return mfc_fltoutpos[dof];
}
