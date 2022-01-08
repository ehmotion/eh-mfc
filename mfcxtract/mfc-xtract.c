/*
*
* MFC extractor: translates USB HID data into movement
*
* good with Thrustmaster T300RS, Fanatec Elite, Logitech
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/time.h>
#include <getopt.h>
#include <time.h>

#include "extras.h"
#include "ff_decode.h"

#define UDP_MAX_PACKETSIZE  1024
#define UDP_PORT            20777

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

int inet_aton (const char *cp, struct in_addr *inp);
//int usleep(long usec);
#if 0
//used by pkt processing logic
unsigned int dtime_ms ()
{
  static unsigned long lms = 0;
  unsigned long cms = get_millis ();
  unsigned long ms = cms - lms;
  lms = cms;
  return (unsigned int)ms;
}
#endif
#if 0
//used by motion computation logic
static unsigned int dtime2_ms ()
{
  static unsigned long lms2 = 0;
  unsigned long cms = get_millis ();
  unsigned long ms = cms - lms2;
  lms2 = cms;
  return (unsigned int)ms;
}
#endif

static int motion_reset (unsigned int mdt);
static int motion_compute (unsigned int mdt);
static int accel_pitch_get(int acc, int brk, int revs);

int motion_process_dummy (char *report, int retval, unsigned long mtime);
int motion_process_thrustmaster (char *report, int retval, unsigned long mtime);
int motion_process_fanatec (char *report, int retval, unsigned long mtime);
int motion_process_fanateclogitech (char *report, int retval, unsigned long mtime);
int motion_process_logitech (char *report, int retval, unsigned long mtime);

#include <sched.h>
#include <stdio.h>

int set_prio ()
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


char _done = 0;
void terminate (int sig)
{
  _done = 1;
}

//get number of millis from app start or first call
//first call will return 0
int ctime_ms(char val)
{
  static int st_ms = 0;
  struct timespec lts;
  //get current time
  //clock_gettime (CLOCK_REALTIME, &lts);
  clock_gettime (CLOCK_BOOTTIME, &lts);
  long long ms = lts.tv_sec * 1000L + lts.tv_nsec / 1000000L;
  //long long ms = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
//    ms = 999999;
  ms &= 0xffffffff;
  if (0)
      printf("milliseconds: %lld\n", ms);
  if (st_ms == 0)
  {
    st_ms = (int)ms;
    return 0;
  }
  return ((int)ms - st_ms);
}

/**
A. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral - add

B. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral (invert) - add
*/
int pf_shiftspd = 100; //[50 .. 100]
//
float _pitchprc  = -20.0f;   //pedals acc/brk
float _rollprc   = 30.0f;   //steering
float _yawprc    = 30.0f;   //steering
float _surgeprc  = 20.0f;   //gear changes
float _swayprc   = 25.0f;   //ffb force/roll
float _heaveprc  = 15.0f;   //revs shake / boating?!
float _trlossprc = 100.0f;  //computed from ffb delta and wheel pos
float _extra1prc = 100.0f;
float _extra2prc = 100.0f;
float _extra3prc = 100.0f;
//
char *_dashaddr = NULL;
//
char _odbg = 0;
char _mot = 0;        //don't process motion
//capture/debug
char _cap = 0;
#define CAP_FFB 0x02
#define CAP_WHL 0x04
#define CAP_ALL 0x06
char _fil = 0;        //capture in/out data to file
#define SWAY_CUTOFF   64
//act as a toy: use wheel input to control the platform
int _toyfd = -1;
unsigned char _nlat = 20; //20ms default network latency
//#define LINE_MAX  255

//supported devices
// 0eb7:0e04 Fanatec
// 046d:c260 Logitech
// 044f:b66d Thrustmaster
typedef struct {
  char *pdev;
  char *pdv;
  int (*pf)(char *buf, int bl, unsigned long dt);
} proc_list;

proc_list _procs[] = {
    {"0000:0000", "!Dummy",       motion_process_dummy},
    {"0eb7:0e04", "Fanatec",      motion_process_fanatec},
    {"046d:c260", "Logitech",     motion_process_logitech},
    {"044f:b66d", "Thrustmaster", motion_process_thrustmaster},
};
int _p_idx = 0; //dummy
//
char *_pdev = NULL;    //device used to extract USB data
static int *_cpkt = NULL;
static int _cpktl = 0;
static int *_dpkt = NULL;

static void usage()
{
  printf ("usage: sudo mfcxtract\n\n");
  printf ("--fanatec output processing protocol for Fanatec wheels\n");
  printf ("--logitech output processing protocol for Logitech wheels\n");
  printf ("--thrustmaster output processing protocol for Thrustmaster wheels\n");
  printf ("--roll <prc>  ;max roll percentage\n");
  printf ("--pitch <prc> ;max pitch percentage\n");
  printf ("--yaw <prc>   ;max yaw percentage\n");
  printf ("--surge <prc> ;max surge percentage\n");
  printf ("--sway <prc>  ;max sway percentage\n");
  printf ("--heave <prc> ;max heave percentage\n");
  printf ("--latency <ms> ;motion update frequency: 5..99ms\n");
  printf ("\n");
}

int env_init (int argc, char *argv[])
{
  /*
   * init params and stuff
   */
  //
  int c;

  struct option long_options[] = {
    /* These options don't set a flag. We distinguish them by their indices. */
    { "help",     no_argument,       0, 'H' },
    { "version",  no_argument,       0, 'V' },
    { "debug",    required_argument, 0, 'D' },
    { "capture",  required_argument, 0, 'c' },
    //
    { "roll",     required_argument, 0, 'r' },
    { "pitch",    required_argument, 0, 'p' },
    { "yaw",      required_argument, 0, 'y' },
    { "surge",    required_argument, 0, 's' },
    { "sway",     required_argument, 0, 'w' },
    { "heave",    required_argument, 0, 'h' },
    //
    { "tyre-slip",required_argument, 0, 't' },
    { "use-dash", required_argument, 0, 'a' },
    //
    { "fanatec",      no_argument,   0, 'f' },
    { "logitech",     no_argument,   0, 'g' },
    { "thrustmaster", no_argument,   0, 'm' },
    //{ "dummy",   no_argument,       0, 'y' },
    { 0, 0, 0, 0 }
  };

  while (1)
  {
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "a:c:r:p:y:s:w:h:l:HVD:fgt:", long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {

    case 'H':
      usage ();
      exit (0);
      break;
    //profiling params
    case 'r': //roll %
      _rollprc = atoi (optarg);
      //if (_rollprc < 0 || _rollprc > 100)
      //  _rollprc = 100;
      break;
    case 'p': //pitch %
      _pitchprc = atoi (optarg);
      //if (_pitchprc < 0 || _pitchprc > 100)
      //  _pitchprc = 100;
      break;
    case 'y': //yaw %
      _yawprc = atoi (optarg);
      //if (_yawprc < 0 || _yawprc > 100)
      //  _yawprc = 100;
      break;
    case 's': //surge %
      _surgeprc = atoi (optarg);
      //if (_surgeprc < 0 || _surgeprc > 100)
      //  _surgeprc = 100;
      break;
    case 'w': //sway %
      _swayprc = atoi (optarg);
      //if (_swayprc < 0 || _swayprc > 100)
      //  _swayprc = 100;
      break;
    case 'h': //heave %
      _heaveprc = atoi (optarg);
      //if (_heaveprc < 0 || _heaveprc > 100)
      //  _heaveprc = 100;
      break;
      //
    case 't': //tyre slip/traction loss
      _trlossprc = atoi (optarg);
      break;
    case 'a': //use dash to forward data
      _dashaddr = optarg;
      break;
    case 'f': //fanatec device
      _p_idx = 1;
      break;
    case 'g': //logitech device
      _p_idx = 2;
      break;
    case 'm': //thrustmaster device
      _p_idx = 3;
      break;
    case 'D': //debug
      printf ("\n#i.dbg %s", optarg);
      _odbg = atoi (optarg);
      break;
    case 'c': //data capture
      _cap = atoi (optarg);
      //if (_cap == 0)
      //  _cap = CAP_FFB | CAP_WHL;
      printf ("\n#i.cap %d", _cap);
      break;
    case 'V':
      printf("mfcxtract %s\n", MFC_VERSION);
      exit(0);
      break;
    case '?':
      usage();
      exit(-1);
      break;

    default:
      printf("unrecognized option: %c\n", c);
      break;
    }
  }

  /*
   * summary configuration
   *
   */
  //configuration summary
  printf ("\n# ##");
  printf ("\n#MFC extractor client %s", MFC_VERSION);
  printf ("\n#running configuration:");
  //printf ("\n#      roll range %d [1..10]", _rollspd);
  //printf ("\n#    accel. speed %d (-a%d) range [1..10]", _accspd, _accspd);
  //printf ("\n#shifter feedback %d (-s%d) range [1..10]", _shiftspd, _shiftspd);
  //printf ("\n#vibrat. feedback %d (-v%d) range [1..10]", _vibfbk, _vibfbk);
  printf ("\n#  pitch feedback %d%% (-p %d) - pedals input", (int)_pitchprc, (int)_pitchprc);
  printf ("\n#   roll feedback %d%% (-r %d) - steering input", (int)_rollprc, (int)_rollprc);
  printf ("\n#    yaw feedback %d%% (-y %d) - steering input", (int)_yawprc, (int)_yawprc);
  printf ("\n#  surge feedback %d%% (-s %d) - gear shift", (int)_surgeprc, (int)_surgeprc);
  printf ("\n#   sway feedback %d%% (-w %d) - FFB wheel pos", (int)_swayprc, (int)_swayprc);
  printf ("\n#  heave feedback %d%% (-h %d) - revs/engine", (int)_heaveprc, (int)_heaveprc);
  printf ("\n#   slip feedback %d%% (-t %d) - computed", (int)_trlossprc, (int)_trlossprc);
  printf ("\n# verbosity level %4d (-D %d)", _odbg, _odbg);
  printf ("\n#    capture mode %s   (-c)", _cap?((_cap&CAP_ALL)==CAP_ALL?"all":(_cap&CAP_FFB?"ffb":"whl")):"off");
  printf ("\n#motion processor %s", _procs[_p_idx].pdv);
  printf ("\n#motion frequency %ums (-l%u) (ms)", (unsigned int)_nlat, (unsigned int)_nlat);
  if (_dashaddr)
    printf ("\n#            dash %s (-a %s)", _dashaddr, _dashaddr);
  else
    printf ("\n#            dash <not in use> (-a <ipv4>)");
  printf ("\n# ##");
  //
  return 1;
}

/**
compute platform position data

A. right axis composition
a. g-force - longitudinal(pitch) - overwrite
b. g-force - lateral(roll) - add

B. right axis composition
a. g-force - longitudinal(pitch) - overwrite
b. g-force - lateral(roll) * invert - add

        float llv = (-1)*fv[1] + fv[0]; //(-)pitch + roll
        float lrv = (-1)*fv[1] - fv[0]; //(-)pitch - roll
*/
int max_accel = 0;
int pl_roll, pl_pitch = 0;  //entire platform roll/pitch
//platform acceleration leveling
int pw_roll, pw_pitch = 0;  //wheel forces
int pf_roll, pf_rolld, pf_pitch = 0, pf_tloss = 0;  //ffb forces
int pv_roll = 0;  //vibration forces
int pv_pitch = 0;
int pw_heave = 0, pw_heave_dir = 1; //used for heave 'boating' effect
int lpacc, lpbrk = 0;       //local values for accel+brake for axis combo
char _wd = 'W'; //wheel data source
int vib_k = 0;

//some inherent logic while we wait for more data
int pw_acc = 0, pw_brk = 0, pw_revs = 0;
static int extractor_update ()
{
  //update extractor logic as needed
  pw_pitch = accel_pitch_get (pw_acc, pw_brk, pw_revs);
  //pw_pitch = accel_pitch_get(lpacc, lpbrk, revsk);
  //
  return 0;
}

int main (int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int rc;//, timeout;
  int lport = MFCXTRACT_PORT;
  //
  env_init (argc, argv);
  //
  int cs;
  //uses dash? set it up
  if (_dashaddr)
  {
    int cs = mfcdash_bcast_prep (_dashaddr, 0);
    if (cs < 3)
    {
      printf ("\n#e:can't connect to DASH server at %s:%d", _dashaddr, MFCDASH_PORT);
      mfc_bcast_close ();
      exit(1);
    }
    _dpkt = mfcdash_bcast_pktget ();
  }
  //MFC server connection
  cs = mfc_bcast_prep ("127.0.0.1", 0);
  if (cs < 3)
  {
    printf ("\n#e:can't connect to MFC server on port %d", MFCSVR_PORT);
    exit(1);
  }
  printf ("\n#i:<%d:MFC server on port %d", cs, MFCSVR_PORT);
  if (_cpkt == NULL)
  {
    _cpkt = mfc_bcast_pktget ();
    _cpktl = mfc_bcast_pktlen ();
    //motion data packet
    _cpkt[MFC_PITYPE] = PKTT_DATA;
    _cpkt[MFC_PITDAT]  = PKTT_2DOF;
  }
  if (_cpkt == NULL)
  {
    printf ("\n#e:can't get MFC packet reference, aborting");
    exit(2);
  }
  //
#define POLL_TIMEOUT 5
  //timeout = POLL_TIMEOUT;

#if 0
  //uptime
  FILE* fp;
  double uptime, idle_time;
  /* Read the system uptime and accumulated idle time from /proc/uptime.  */
  fp = fopen ("/proc/uptime", "r");
#endif
  //sockets
  struct sockaddr_in si_me;
  int s, i;
  if((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#e:can't initialize network, aborting");
    exit(1);
  }
#if 0
  int broadcast=1;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
#endif
/* receiver */
  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(lport);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if(bind(s, (struct sockaddr*)&si_me, sizeof(si_me))==-1)
  {
    printf ("\n#e:can't set-up network, aborting");
    exit(1);
  }
#if 0
/* sender */
  memset((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(65001);
  pktk = 990;
  if (inet_aton("127.0.0.1", &si_other.sin_addr)==0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }
#endif
  printf ("\n#i:>%d:listening on port %d", s, lport);
  //only send 3 PAUSEd packets
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  (void) set_prio();
  //
  int ppkt = 1;
  char packetBuffer[UDP_MAX_PACKETSIZE];
  //
  int rlen = 0, tmoknt = 0;
  int ndts = -1, ldts = dtime_ms ();
  int ppid = 0, vvid = 0;
  int lts = ctime_ms(0);
  //motion measurements
  unsigned int ms_mot = 0;//(unsigned int)get_millis();
  unsigned int ms_now = 0;
  int *_cpkt = mfc_bcast_pktget ();
  int *_dpkt = mfcdash_bcast_pktget ();
  int pktl = mfc_bcast_pktlen ();
  //send control packet with profile % configuration
  _cpkt[MFC_PITYPE] = PKTT_CTRL;
  _cpkt[MFC_PITDAT] = PKTC_PRCFG;
  //
  _cpkt[MFC_PIPITCH] = _pitchprc;
  _cpkt[MFC_PISURGE] = _surgeprc;
  _cpkt[MFC_PIHEAVE] = _heaveprc;
  _cpkt[MFC_PIROLL]  = _rollprc;
  _cpkt[MFC_PISWAY]  = _swayprc;
  _cpkt[MFC_PIYAW]   = _yawprc;
  _cpkt[MFC_PITLOSS] = _trlossprc;
  _cpkt[MFC_PIEXTRA1]  = _extra1prc;
  _cpkt[MFC_PIEXTRA2]  = _extra2prc;
  _cpkt[MFC_PIEXTRA3]  = _extra3prc;
  mfc_bcast_send ();
  //send dash data
  if (_dashaddr)
  {
    memcpy(_dpkt, _cpkt, pktl);
    //set sim/app name
    snprintf((char *)(_dpkt + MFC_DIAPP14), MFC_DIAPPSZ, "MFC-XTRACT");
    mfcdash_bcast_send ();
  }
  _cpkt[MFC_PITYPE] = PKTT_DATA;
  _cpkt[MFC_PITDAT]  = PKTT_2DOF;
  //compute percentages here to avoid recompute
  _pitchprc = _pitchprc / 100.0f;
  _surgeprc = _surgeprc / 100.0f;
  _heaveprc = _heaveprc / 100.0f;
  _rollprc = _rollprc / 100.0f;
  _swayprc = _swayprc / 100.0f;
  _yawprc = _yawprc / 100.0f;
  _trlossprc = _trlossprc / 100.0f;
  _extra1prc = _extra1prc / 100.0f;
  _extra2prc = _extra2prc / 100.0f;
  _extra3prc = _extra3prc / 100.0f;
  printf("\n#i:ready.");
  fflush (stdout);
  //
  while (!_done)
  {
    memset ((void*)fdset, 0, sizeof(fdset));

    fdset[0].fd = s;
    fdset[0].events = POLLIN;
    if (_toyfd > 0)
    {
      fdset[1].fd = _toyfd;
      fdset[1].events = POLLIN;
    }
    ldts = _nlat; //POLL_TIMEOUT;
    rc = poll (fdset, nfds, _nlat /*POLL_TIMEOUT*/); //timeout every 5 millis
    lts = ctime_ms(0);
    if (rc < 0) 
    {
      printf("\n#e%.3f:poll() failed!", lts/1000.0f);
      _done = 1;
      break;
    }
    //timeout
    if (rc == 0)
    {
      //called every 5 millis
      tmoknt++;
      if (tmoknt >= 1000)
      {
        printf(".");
        fflush (stdout);
        tmoknt = 0;
      }
      //sleep (1);
    }
    //take the time
    ms_now = lts;//(unsigned int)get_millis();
    //printf ("\n#i:%04x.have data %d", ms_now, (rc == 0)?0:1);
    //>toy mode
    if (fdset[1].revents & POLLIN)
    {
      if ((rlen = read (_toyfd, packetBuffer + 3, UDP_MAX_PACKETSIZE)) > 0)
      {
        //prep motion packet
        packetBuffer[0] = PKTT_IN;
        packetBuffer[1] = rlen + 1;
        packetBuffer[2] = 0x84;
        rlen += 3;
        //motion process
        motion_process_fanatec (packetBuffer, rlen, ldts);
        //
        printf ("\n#i.WHL@%04d: ", ldts);
        for (i = 0; i < rlen; i++)
          printf ("%02x ", (unsigned char)packetBuffer[i]);
      }
    }
    //>network pkt from USB extractor
    if (fdset[0].revents & POLLIN)
    {
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      if ((rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL)) == -1)
      {
        printf("\n#w:recvfrom() failed");
      }
      else
      {
        ldts = dtime_ms ();
        ppkt++;
        //debug?!
        if (0)
          printf ("\n#i.PKT len %d", rlen);
        switch ((char) packetBuffer[0])
        {
          case (PKTT_DATA):  //USB pkt arrival timestamp data
          {
            ndts = packetBuffer[6] << 8 | packetBuffer[7];
            //printf ("\n#i.DBG@%04d: ", _dts);
            //select proper processing function based on VID+PID?!
            break;
          }
          //
          case (PKTT_CTRL): //wheel VID&PID
          {
            vvid = packetBuffer[2] << 8 | packetBuffer[3];
            ppid = packetBuffer[4] << 8 | packetBuffer[5];
            printf ("\n#i%.3f.WHL %04x:%04x ", lts/1000.0f, vvid, ppid);
            //select proper processing function based on VID+PID?!
            break;
          }
          //
          case (PKTT_OUT):  //FFB data
          {
            if (_cap&CAP_FFB || _odbg)
            {
              if (ndts != -1)
              {
                ldts = ndts;
                ndts = -1;  //reset network ts as we need another
              }
              fprintf (stdout, "\n#i.FFB@%04d: ", ldts);
              for (i = 0; i < rlen; i++)
                fprintf (stdout, "%02x ", packetBuffer[i]);
            }
            break;
          }
          case (PKTT_IN): //WHL data
          {
            if (_cap&CAP_WHL || _odbg)
            {
              if (ndts != -1)
              {
                ldts = ndts;
                ndts = -1;  //reset network ts as we need another
              }
              fprintf (stdout, "\n#i.WHL@%04d: ", ldts);
              for (i = 0; i < rlen; i++)
                fprintf (stdout, "%02x ", packetBuffer[i]);
            }
            //
            break;
          }
          default:
          {
            if (0)
            {
              printf ("\n#w.UNK@%04d: ", ldts);
              for (i = 0; i < rlen; i++)
                printf ("0x%02x, ", packetBuffer[i]);
            }
          }
        }//switch
        //
        if (packetBuffer[12] > 2) //report ask skip
          _procs[_p_idx].pf (packetBuffer + 11, packetBuffer[12], lts); //add 11 to skip envelope header+vid&pid+timestamp
        //
        if ((ppkt % 500) == 0)
          printf ("\n#i%.3f:received %dpkts", lts/1000.0f, ppkt);
      }//got network pkt
    }//read from network
    //fflush (stdout);
    motion_compute (ldts);
    //send the motion packet
    //if (rc && (ms_now - ms_mot >= _nlat))
    if (ms_now - ms_mot >= _nlat)
    {
      if (0 && _odbg)
        printf ("\n#i:%.3f.motion@%ums vs %ums p%06dr%06d", ms_now/1000.0f, ms_now - ms_mot,
          _nlat, _cpkt[MFC_PIPITCH], _cpkt[MFC_PIROLL]);
      //inc motion packets
      //mpktt++;
      //send the packet
      //_cpkt[MFC_PISPEED] = ms_now;  //hide the timestamp here, for future ref
      mfc_bcast_send ();
      if (_dashaddr)
      {
        memcpy(_dpkt, _cpkt, pktl);
        //
        mfcdash_bcast_send ();
      }
      //
      motion_reset (ms_now - ms_mot);
      //reset motion time to now
      ms_mot = ms_now;
      //update extractor logic every time, in case we get no data from the wheel
      extractor_update ();
    }
    else
    {
      //printf ("\n#w%.3f:dropped motion pkt", lts/1000.0f);
    }
  }
  //
  printf("\n#i:cleaning up.. done.\n");
  //
  close (s);
  mfc_bcast_close ();
  mfcdash_bcast_close ();
  if (_toyfd > 0)
    close (_toyfd);
  //
  return 0;
}

#if 0
static int get_accel_pitch (int lpacc)
{
  static int c_accel = 0;
  static const int pf_accspd = 350; //[50 .. 500]
  //deal with acceleration platform leveling
  if (max_accel == lpacc) //constant acceleration
  {
    //fprintf (stderr, "\n#i:1 lpacc %d \t max_acc %d \t c_acc %d", lpacc, max_accel, c_accel);
    c_accel -= 20;//this needs to drop / ramp down regardless of the steering movement
    //
    if (c_accel < 0)
      c_accel = 0;
    //fprintf (stderr, "\n#i:2 lpacc %d \t max_acc %d \t c_acc %d", lpacc, max_accel, c_accel);
  }
  else if (max_accel < lpacc)//accelerate || max_accel > lpacc/*deccelerate*/)
  {
    c_accel += pf_accspd;//this needs to grow / ramp up regardless of the steering movement
    if (c_accel > lpacc)
      c_accel = lpacc;
    max_accel = c_accel;
  }
  else
  {
    max_accel = lpacc;
    c_accel = lpacc;
    //fprintf (stderr, "\n#i:3 lpacc %d \t max_acc %d \t c_acc %d", lpacc, max_accel, c_accel);
  }
  //
  return c_accel + pf_accspd; //it should stay nose-up while accelerating
}
#endif
int acc_map[][2] = {
//knt,pitch level%
  {0, 0},
  {1, 20},
  {2, 40},
  {3, 50},
  {4, 60},
  {5, 70},
  {6, 80},
  {7, 85},
  {8, 90},
  {9, 95},
  {10, 100},
  {11, 100},
  {12, 80},
  {13, 60},
  {14, 50},
  {15, 40},
};
#define ACC_MAPIDX_MAX 15
#define ACC_MAPKNT_MAX 60

//implement an accel curve based on acc level
//  and the number of times this is called
static int accel_pitch_get(int acc, int brk, int revs)
{
  static int kntr = 0;
  static int ptch = 0;
  //
  static int lidx = 0;  //last used index
  static int fdt = (ACC_MAPKNT_MAX + ACC_MAPIDX_MAX) / ACC_MAPIDX_MAX;
  //static int lacc = 0;  //last accel value so we know if it was going up or down
  pw_acc = acc; pw_brk = brk; pw_revs = revs;
  //
  if (acc <= 0)
  {
    ptch = 0;
    //
    kntr = 0;
    lidx = 0;
    //lacc = 0;
  }
  //when brake is pressed
  if (brk < -5)
  {
    ptch = brk;  //cut off accel if brake pressed: goes to -MFC_WHL_MAX
    if (0)
      printf("\n#d.brk %d ", ptch);
    //reset all other metrics
    kntr = 0;
    lidx = 0;
    //lacc = 0;
  }
  else if (acc)
  {
    //pitch goes from 0 to MFC_WHL_MAX (32768)
    //our timeline is 100 iterations: kntr < 100
    //get accel map index
    //lidx = get_cmap (kntr++, 0, ACC_MAPKNT_MAX, 0, ACC_MAPIDX_MAX);
    lidx = kntr / fdt;
    if (lidx > ACC_MAPIDX_MAX)
      lidx = ACC_MAPIDX_MAX;
    //get pitch value from accel map: acc itself goes to MFC_WHL_MAX (32768)
    ptch = acc * acc_map[lidx][1] / 100;
    //add the discreet value in the interval if it is growing
    //e.g. we're at 20% and going to 25%
    if (lidx < ACC_MAPIDX_MAX)
    {
      int ptchn = acc_map[lidx + 1][1] * acc / 100;
      if (0)
        printf("\n#d.kntr %d fdt %d mod %d from %d (%d) to %d (%d)", kntr, fdt,
          kntr % fdt, ptch, acc_map[lidx][1], ptchn, acc_map[lidx + 1][1]);
      if (ptch <= ptchn)
        ptch = get_cmap(kntr % fdt, 0, fdt, ptch, ptchn);
      else
        ptch = get_cmap(kntr % fdt, fdt, 0, ptchn, ptch);
      #if 0
      int dpt = (acc_map[lidx + 1][1] * MFC_WHL_MAX / 100) - ptch;
      //going up or down?
      if (dpt > 0)
      {
        ptch += get_cmap(kntr % fdt, 0, fdt, 0, dpt);
      }
      else
      {
        ptch -= get_cmap(kntr % fdt, 0, fdt, 0, dpt);
      }
      #endif
    }
    if (0 && lidx <= ACC_MAPIDX_MAX)
    {
      printf("\n#d.acc map %didx %d%%map %dacc %d", lidx, acc_map[lidx][1], acc, ptch);
    }
    //add revs to nose pitch
    //if (revs == 9)
    //{
    //  ptch -= (kntr % 2) * 2500;
    //}
    //
    kntr++;
  }
  return ptch;
}

static int motion_reset (unsigned int mdt)
{
  //reset forces and vibrations
  //pf_roll = pf_pitch = pv_roll = 0;
  pf_pitch = 0;
  //pf_sway = 0;
  //pf_nudge = 0;
  pv_pitch = 0;
  pv_roll = 0;
  //
  return 0;
}

static int motion_compute (unsigned int mdt)
{
  static int sway_flag = 1;
  //int lpw_roll;
  /*
  * wheel roll should cap at -16k..+16k
  *
  * when we have vibrations, that needs to feel like 50%, 
  * then wheel 25%(20%) and ffb 25%(30%) otherwise, use 50% wheel and 50% ffb
PITCH	Pitch is the tilt of the car forwards or backwards in [°]             //UP AND DOWN HILLS	
SURGE	Surge means the acceleration of the car in longitudinal direction [g] //MAX= ACCERATION AND MIN = BRAKE	
HEAVE	Heave means the acceleration up and down [g]					                //BUMPS ON ROAD	
//
ROLL	Roll is how much the car is dipped to the left or right in [°]        //LEFT AND RIGHT, BANKS ,BUMPS WORKS OPPOSITE TO SWAY	
SWAY	Sway means the acceleration of the car in lateral direction [g]				//LEFT AND RIGHT BODY ROLL	
//
YAW	Yaw is the heading of the car (north, east, south, west) in [°]         //car heading
  *
  //pitch
  _cpkt[MFC_PIPITCH] = get_cmap_f (_cpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PISURGE] = get_cmap_f (_cpkt[MFC_PISURGE], mpkt[MFC_PISURGE], Mpkt[MFC_PISURGE], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PIHEAVE] = get_cmap_f (_cpkt[MFC_PIHEAVE], mpkt[MFC_PIHEAVE], Mpkt[MFC_PIHEAVE], MFC_POS_MIN, MFC_POS_MAX);
  //roll
  _cpkt[MFC_PIROLL]  = get_cmap_f (_cpkt[MFC_PIROLL],  mpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PISWAY]  = get_cmap_f (_cpkt[MFC_PISWAY],  mpkt[MFC_PISWAY],  Mpkt[MFC_PISWAY], MFC_POS_MIN, MFC_POS_MAX);
  //yaw
  _cpkt[MFC_PIYAW]   = get_cmap_f (_cpkt[MFC_PIYAW],   mpkt[MFC_PIYAW],   Mpkt[MFC_PIYAW], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PITLOSS] = get_cmap_f (_cpkt[MFC_PITLOSS],   mpkt[MFC_PITLOSS],   Mpkt[MFC_PITLOSS], MFC_POS_MIN, MFC_POS_MAX);
  */
  //--motion
  //>PITCH
  //pedals-based pitch
  _cpkt[MFC_PIPITCH] = -get_cmap (pw_pitch, -MFC_WHL_MAX, MFC_WHL_MAX, MFC_HPOS_MIN, MFC_HPOS_MAX);
  //gear changes
  _cpkt[MFC_PISURGE] = -get_cmap (pf_pitch, -100, 100, MFC_HPOS_MIN, MFC_HPOS_MAX);
  //revs jolt, others, pv_pitch
  _cpkt[MFC_PIHEAVE] = -get_cmap (pv_pitch, -128, 128, MFC_HPOS_MIN, MFC_HPOS_MAX);
  //>ROLL
  //ffb roll
  //if (_toyfd > 0)
  //{
  //  _cpkt[MFC_PIROLL]  = get_cmap (pw_roll, -MFC_HWHL_MAX, MFC_HWHL_MAX, MFC_POS_MIN, MFC_POS_MAX);
  //  //printf ("\n#d.pw roll %d", pw_roll);
  //}
  //else
  //steering wheel input for ROLL
  _cpkt[MFC_PIROLL] = get_cmap (pw_roll, -MFC_HWHL_MAX, MFC_HWHL_MAX, MFC_HPOS_MIN, MFC_HPOS_MAX);
  //printf ("\n#p.roll1 %d", _cpkt[MFC_PIROLL]);
  //force feedback input for sway
  //lpw_roll = get_cmap (pw_roll, -MFC_HWHL_MAX/2, MFC_HWHL_MAX/2, -MFC_HPOS_MAX/2, MFC_HPOS_MAX/2);
  //_cpkt[MFC_PISWAY]  = get_cmap (pf_roll, -128, 128, MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PISWAY]  = get_cmap (pf_rolld, -128, 128, MFC_HPOS_MIN, MFC_HPOS_MAX);
  //>YAW
  //steering direction
  _cpkt[MFC_PIYAW]  = get_cmap (pw_roll, -MFC_HWHL_MAX, MFC_HWHL_MAX, MFC_HPOS_MIN, MFC_HPOS_MAX);
  //traction loss when wheel pos command follows wheel pos
  if (abs (pf_rolld) > 1) 
  {
    if (pf_tloss)
    {
      //printf ("\n#d.stop TRL %d", pf_tloss);
      sway_flag = 1;
      pf_tloss = 0;
    }
  }
  else
  {
    if (sway_flag)
    {
      sway_flag = 0;
      //this may clip over the yaw but we want most effect so use 128
      pf_tloss = get_cmap (pw_roll, -MFC_HWHL_MAX, MFC_HWHL_MAX, -128, 128);
      if (!pf_tloss)
        pf_tloss = 1;
      //printf ("\n#d.start TRL on %d with %d", pf_rolld, pf_tloss);
    }
  }
  //yaw move: traction loss and strong ffb
  _cpkt[MFC_PITLOSS] = get_cmap (pf_tloss, -128, 128, MFC_HPOS_MIN, MFC_HPOS_MAX);
  //
  if (0 && _cpkt[MFC_PITLOSS])
    printf ("\n#d.tr loss %d on TRL %d with %d", _cpkt[MFC_PITLOSS], pf_rolld, pf_tloss);
  //
  if (0)
    printf ("\n#i@%04d.t1 pitch%% %d %d %d roll%% %d %d yaw%% %d %d", mdt,
      _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE],
      _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
      _cpkt[MFC_PIYAW], _cpkt[MFC_PITLOSS]);
  //adjust %
  _cpkt[MFC_PIPITCH] = (float)_cpkt[MFC_PIPITCH] * _pitchprc;
  _cpkt[MFC_PISURGE] = (float)_cpkt[MFC_PISURGE] * _surgeprc;
  _cpkt[MFC_PIHEAVE] = (float)_cpkt[MFC_PIHEAVE] * _heaveprc;
  //
  _cpkt[MFC_PIROLL]  = (float)_cpkt[MFC_PIROLL]  * _rollprc;
  _cpkt[MFC_PISWAY]  = (float)_cpkt[MFC_PISWAY]  * _swayprc;
  //
  _cpkt[MFC_PIYAW]   = (float)_cpkt[MFC_PIYAW]   * _yawprc;
  //
  _cpkt[MFC_PITLOSS] = (float)_cpkt[MFC_PITLOSS] * _trlossprc;
  if (0)
    printf ("\n#i@%04d.t2 pitch%% %d %d %d roll%% %d %d yaw%% %d %d", mdt,
      _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE],
      _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY],
      _cpkt[MFC_PIYAW], _cpkt[MFC_PITLOSS]);
  //
  return 1;
}

int motion_process_dummy (char *report, int rlen, unsigned long dtime)
{
  //update delta time
  if (dtime == -1)
    dtime = 4;
  _wd = 'U';
  //ffb wheel pos: ffb roll
  //
  switch ((char) report[0])
  {
    case (PKTT_DEBUG):  //debug data
      return 0;
    //
    case (PKTT_OUT):   //FFB data
      /*
       * FFB data processing
       */
    {
      //unhandled
      _wd = 'U';
      if (1||_odbg & 0x01)
      {
        printf ("\n#w!FFB@%04lu: ", dtime);
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
      return 0;
    }
      break;
    case (PKTT_IN):   //WHL data
      /*
       * WHL data processing
       */
    {
      //unhandled
      _wd = 'W';
      if (1||_odbg & 0x02)
      {
        printf ("\n#w!WHL@%04lu: ", dtime);
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
      return 0;
    }
      break;
  }
  return 1;
}

/*
 * Fanatec CSL - compat mode > Logitech G29 Driving Force Racing Wheel (PS4)
 * --vid 046d --pid c260
 * Bus 001 Device 031: ID 046d:c260 Logitech, Inc.
 *-------------
 *
 *
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c7 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a8 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 8a 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 7b 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 6c 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 4d 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 3e 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 20 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 11 81 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f2 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 f8 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 11 08 83 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 21 0c 05 00 05 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 11 08 83 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 f8 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 21 0c 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 f8 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i.FFB@0000: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 */
//#w!FFB@0004: 07 21 03 30 f8 12 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_logi_lights1[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x21, 0x03, 0x30, 0xf8, 0x12, 0xFF };

//#w!FFB@0004: 07 21 03 30 11 08 b3 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_logi_move1[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x21, 0x03, 0x30, 0x11, 0x08, 0xFF, 0x80 };

//#w!FFB@0004: 07 21 03 30 21 0c 04 00 04 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_logi_move2[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x21, 0x03, 0x30, 0x21, 0x0c, 0xFF, 0x00, 0xFF, 0x00, 0x01 };

//#w!FFB@0004: 07 21 03 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_logi_kpl1[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x21, 0x03, 0x30, 0x13, 0x00, 0x00 };

int motion_process_logitech_old (char *report, int rlen, unsigned long dtime)
{
  //update delta time
  if (dtime == -1)
    dtime = 4;
  //default pkt: unhandled
  _wd = 'U';
  //ffb wheel pos: ffb roll
  static int lwpos = 0, cwpos = 0;
  //
  switch ((char) report[0])
  {
    case (PKTT_DEBUG):  //debug data
      return 0;
    //
    case (PKTT_OUT):   //FFB data
      /*
       * FFB data processing
       */
      {
        _wd = 'F';
        if (memcmp ((const void *) (report + 2), (const void *) (ffb_logi_lights1 + 2), 4) == 0)
        {
          //TODO: process movement for lights
          _wd = 'F';
          if (0)
          {
            printf ("\n#i.FFB@%04lu: ", dtime);
            for (int i = 0; i < rlen; i++)
              printf ("%02x ", report[i]);
          }
        }
        else
        if (memcmp ((const void *) (report + 2), (const void *) (ffb_logi_move1 + 2), 4) == 0)
        {
          //TODO: process movement for wheel position
          _wd = 'F';
          if (0)
          {
            printf ("\n#i.FFB@%04lu: ", dtime);
            for (int i = 0; i < rlen; i++)
              printf ("%02x ", report[i]);
          }
          /* wheel ffb position
          * 128..255 | 0..127
          */
          cwpos = report[6]; //normal_ffb2 (((int) report[6]), 0x080);
          pf_roll = (cwpos > lwpos)? (cwpos - lwpos):-(lwpos - cwpos);
          lwpos = cwpos;
          if (_odbg > 2)
            printf ("\n#d.WHLPOS1 ffb roll %d / %d", pf_roll, ((int) report[6]));
        }
        else
        if (memcmp ((const void *) (report + 2), (const void *) (ffb_logi_move2 + 2), 4) == 0)
        {
          //TODO: process movement - unknown
          _wd = 'F';
          if (0)
          {
            printf ("\n#i.FFB@%04lu: ", dtime);
            for (int i = 0; i < rlen; i++)
              printf ("%02x ", report[i]);
          }
        }
        else
        if (memcmp ((const void *) (report + 2), (const void *) (ffb_logi_kpl1 + 2), 4) == 0)
        {
          //nothing to do here: keepalive
          _wd = 'U';
        }
        else
        if (_odbg)
        {
          _wd = 'U';
          printf ("\n#w!FFB@%04lu: ", dtime);
          for (int i = 0; i < rlen; i++)
            printf ("%02x ", report[i]);
        }
        return 0;
      }
      break;
    case (PKTT_IN):   //WHL data
      /*
       * WHL data processing
       */
      {
        //unhandled
        _wd = 'U';
        if (_odbg)
        {
          printf ("\n#i.WHL@%04lu: ", dtime);
          for (int i = 0; i < rlen; i++)
            printf ("%02x ", report[i]);
        }
        //get accel/brake
        /*
        * wheel: 1..32768<L R>-32768..-1
        *   acc: up: -1..-32768 > 32768..1 :dn
        * brake: up: -1..-32768 > 32768..1 :dn
        */
        pw_roll = normal_axis (get_short (report, 46), 0x0ffff); //from wheel turn
        lpacc   = normal_accel (get_short (report, 48), 0x0ffff);
        lpbrk   = normal_brake (get_short (report, 50), 0x0ffff);
        //lhbrk   = normal_brake (get_short (report, 54), 0x0ffff); //handbrake
        //
        if (_odbg > 2)
          printf ("\n#RAW whl %d acc %d brk %d", pw_roll, lpacc, lpbrk);
        //
        #if 0
        if (lpbrk < -5)
          pw_pitch = lpbrk;  //cut off accel if brake pressed
        else
        {
          //
          pw_pitch = accel_pitch_get(lpacc, lpbrk, 0); //get_accel_pitch (lpacc); //it should stay nose-up while accelerating
        }
        //pw_pitch = lpbrk; //accel-brake
        #endif
        pw_pitch = accel_pitch_get(lpacc, lpbrk, 0); //get_accel_pitch (lpacc); //it should stay nose-up while accelerating
        //
        //gear shifting: only when accelerating or braking
        if (lpbrk || lpacc)
        {
        //gear down
          if (report[9] & 0x01)
          {
            //printf ("\ngear down\n");
            max_accel = 0;
            if (lpbrk)
              //ffb_vib_gear[5] = pf_shiftspd;
              pf_pitch = pf_shiftspd;
            else
              pf_pitch = -pf_shiftspd;
              //ffb_vib_gear[5] = pf_shiftspd;
            //return motion_process (ffb_vib_gear, 0, mtime);
            //not reacheable
            /*
            mdelta += MOTION_FREQ_MS; //force event motion
            pv_pitch = 0x01;
            c_accel = lpacc;
            max_accel = lpacc;
            //pw_pitch = c_accel;
            */
          }
          //gear up
          if (report[9] & 0x02)
          {
            //printf ("\ngear up\n");
            max_accel = 0;
            if (lpbrk)
              //ffb_vib_gear[5] = pf_shiftspd;
              pf_pitch = pf_shiftspd;
            else
              //ffb_vib_gear[5] = -pf_shiftspd;
              pf_pitch = -pf_shiftspd;
            //return motion_process (ffb_vib_gear, 0, mtime);
          }
        }//gear shifting
      }//IN PKT: WHL
      break;
  }//switch pkt type
  //
  return (_wd != 'U' ? 1 : 0);
}

/*
 * Fanatec CSL - native mode
 * --vid 0eb7 --pid 0e04
 * Bus 001 Device 029: ID 0eb7:0e04 Endor AG
 */

/*
 * GTSport
 *
# center
dat: 07 (41) 03 30 01 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
# right
dat: 07 (41) 03 30 01 08 50 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
# left
dat: 07 (41) 03 30 01 08 a0 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
 *
 *
 *
#FFB
#i:FFB pkt: 07 41 03 30 f8 09 01 a0 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 09 01 06 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 03 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 03 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 03 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 23 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 23 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 23 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 33 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00                                                          #w!ffb pkt: 07 41 03 30 33 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 33 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 33 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 43 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 43 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 43 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 53 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 53 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 53 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 63 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 63 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 63 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 73 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 73 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 73 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 83 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 83 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 83 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 93 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 93 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 93 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 a3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 a3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 a3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 b3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 b3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 b3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 c3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 c3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 c3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 d3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 d3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 d3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 e3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 e3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00                                                          #w!ffb pkt: 07 41 03 30 e3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 e3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f3 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f3 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f3 0c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 09 07 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 09 08 01 ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 f8 09 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#--
#i:FFB pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#i:FFB pkt: 07 41 03 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#-- ---
#
# Dirt Rally
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 24 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 1c 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 0a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 1d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 17 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 17 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 38 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 28 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 41 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 41 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 0d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 2d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 2d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 32 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#w!ffb pkt: 07 21 03 05 01 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//-- ---
//Dirt 4
//-- start
#w!ffb pkt: 07 09 03 30 f8 81 ff ff 00 00 00
#w!ffb pkt: 07 09 03 30 f5 00 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 03 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 03 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 03 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 13 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 13 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 13 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 23 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 23 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 23 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 33 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 33 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 33 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 43 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 43 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 43 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 53 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 53 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 53 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 63 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 63 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 63 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 73 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 73 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 73 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 83 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 83 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 83 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 93 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 93 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 93 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 a3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 a3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 a3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 b3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 b3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 b3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 c3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 c3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 c3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 d3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 d3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 d3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 e3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 e3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 e3 0c 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 f3 08 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 f3 0b 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 f3 0c 00 00 00 00 00 ..
//-- wheel strength?!
#w!ffb pkt: 07 09 03 30 f8 13 01 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 03 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 07 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 0f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 1f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 3f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 14 00 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 7f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 7f 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 ff 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 ff 00 00 00 00
#w!ffb pkt: 07 09 03 30 f8 13 ff 01 00 00 00
#w!ffb pkt: 07 09 03 30 f8 14 00 00 00 00 00
 *
 */
//wheel position message
//also present as 9 bytes message
//#w!ffb pkt: 07 09 03 30 01 08 9c 00 00 00 00
unsigned char ffb_whlpos1[] =
//<type>,<len>,  03    30    01    08 <FF..80..01>
 { 0x07, 0x41, 0x03, 0x30, 0x01, 0x08, 0xFF };

 //also present as 9 bytes message
 //#w!ffb pkt: 07 09 03 30 f8 09 08 01 f8 00 00
unsigned char ffb_lights[] =
 //<type>,<len>,  03    30    f8    09    08, Y:1  Y2:Y3:R1:R2 R3:B1:B2:B3
  { 0x07, 0x41, 0x03, 0x30, 0xf8, 0x09, 0x08, 0x01, 0xFF };

//no impact on wheel led, just the rev leds
unsigned char ffb_lights2[] =
 //<type>,<len>,  03    30    f8    13  B2:B1:R3:R2 R1:Y3:Y2:Y1  B3    D1    D2    D3
  { 0x07, 0x09, 0x03, 0x30, 0xf8, 0x13, 0xff, 0x01, 0x00, 0x00, 0x00 };

/*
 * 9byte msg: the 3 digits
 *
//                                 "    5   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 6d 00
//                                 "    4   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 66 00
//                                 "    3   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 4f 00
//                                 "    2   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 5b 00
//                                 "    1   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 06 00
//                                 "    N   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 37 00
//                                 "    R   "
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 31 00
//                                 "0   0  0"
#w!ffb pkt: 07 09 03 30 f8 09 01 02 3f 3f 3f
//--
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 37 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 06 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 5b 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 4f 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 66 00
#w!ffb pkt: 07 09 03 30 f8 09 01 02 00 6d 00
//-- ---
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 01 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 1f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 0f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 07 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 03 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 01 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 00 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 01 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 03 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 07 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 0f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 1f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#i:received 5000pkts
#w!FFB@-001: 07 09 03 30 f8 13 ff 01 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 1f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#i:received 5500pkts
#w!FFB@-001: 07 09 03 30 f8 13 ff 01 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 ff 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 7f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 3f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 1f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 0f 00 00 00 00
#w!FFB@-001: 07 09 03 30 f8 13 00 00 00 00 00
 *
 *
 */
unsigned char ffb_digits[] =
 //<type>,<len>,  03    30    f8    09    01    02    D1    D2    D3
  { 0x07, 0x09, 0x03, 0x30, 0xf8, 0x09, 0x01, 0x02, 0xFF, 0xFF, 0xFF };

//wheel position message: weak force
unsigned char ffb_whlpos2[] =
 //<type>,<len>,  03    05    01    00    00    00  <FF..80..01>
  { 0x07, 0x21, 0x03, 0x05, 0x01, 0x00, 0x00, 0x00, 0xFF };

/*
 * Dirt4
 * 9byte msg: vibrations?
 *
#w!ffb pkt: 07 09 03 30 11 0c 06 00 06 00 ff
#w!ffb pkt: 07 09 03 30 11 0c 07 00 07 00 ff
#w!ffb pkt: 07 09 03 30 11 0c 06 00 06 00 ff
#w!ffb pkt: 07 09 03 30 11 0c 05 00 05 00 ff
 *
 * */
unsigned char ffb_whlvib1[] =
 //<type>,<len>,  03    30    11    0c    FF    00    FF    00    ff>
  { 0x07, 0x09, 0x03, 0x30, 0x11, 0x0c, 0xFF, 0x00, 0xFF, 0x00, 0xff };

/*
#d.digits 01 02 00 06 00 00 > _1_
#d.digits 01 02 00 06 00 00
#d.digits 01 02 00 5b 00 00 > _2_
#d.digits 01 02 00 5b 00 00
#d.digits 01 02 00 4f 00 00 > _3_
#d.digits 01 02 00 4f 00 00
#d.digits 01 02 00 66 00 00 > _4_
#d.digits 01 02 00 66 00 00.
#d.digits 01 02 00 6d 00 00 > _5_
#d.digits 01 02 00 6d 00 00
#d.digits 01 02 3f 3f 3f 00 > 000
#d.digits 01 02 3f 00 00 00 > 0__
#d.digits 01 02 00 54 00 00 > _n_
#d.digits 01 02 00 50 00 00 > _r_
bits schematic for the 7 segments:
  0
  -
5| |1
6 -
4| |2
  -
  3
*/
static char ff_fan_check_led_digit (char byte)
{
  switch (byte)
  {
    case 0x00:
      return ' ';
    case 0x3f:
      return '0';
    case 0x06:
      return '1';
    case 0x5b:
      return '2';
    case 0x4f:
      return '3';
    case 0x66:
      return '4';
    case 0x6d:
      return '5';
    case 0x7d:
      return '6';
    case 0x07:
      return '7';
    case 0x7f:
      return '8';
    case 0x6f:
      return '9';
    case 0x54:
      return 'n';
    case 0x50:
      return 'r';
    default:
      return '-';
  }
  return '*';
}

static int revsk = 0;      //count rev leds that are lit
//#i.WHL@0000: 06 4c 00 0e b7 0e 04 2d 6f dc 06 06 41 00 84 01 80 80 7f 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 e5 81 00 00 ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00                     
//#i.WHL@0000: 06 4c 00 - envelope header
//  0e b7 0e 04 - vid+pid
//  2d 6f dc 06 - timestamp
//  06 41 00 - pkt header, followed by pkt data x41/65 bytes
//  84 01 80 80 7f 80 08 00 00 
//   00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 e5 81 00 00 ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00                     

//#i.WHL@0030: 06 4c 00 0e b7 0e 04 02 b4 02 07 
//  06 41 00 
//   84 01 
//   80 80 7f 80 
//   08 00 00
//   00 00 00 00 00 00 00 00 00 00
//   00 00 00 00 00 00 00 00 00 00
//   00 00 00 00 00 00 00 00 00 00
//   00 00 00 00 00 
//   f0 7f 
//   ff ff 
//   ff ff 
//   ff ff 
//   00
//   ff ff
//   00 00 00 00 00 00 00 00 00 00

#define PACKED __attribute__((packed))
typedef struct PACKED {
  uint8_t epid;
  uint8_t rpid;
  uint8_t pad1[4];  //idx 2: 80 80 7f 80
  uint8_t hat;      //idx 6
  uint8_t btn1;     //idx 7
  uint8_t btn2;     //idx 8
  uint8_t pad2[35]; //idx 9
  uint16_t whl;     //idx 45
  uint16_t acc;     //idx 47
  uint16_t brk;     //idx 49
  uint16_t clt;     //idx 51
  uint8_t pad3;     //idx 53: 00
  uint16_t hbr;     //idx 54
  uint8_t pad4[10];  //idx 56
} fanatec_whl;

int motion_process_fanatec (char *report, int rlen, unsigned long dtime)
{
  //update delta time
  if (dtime == -1)
    dtime = 4;
  _wd = 'U';
  //ffb wheel pos: ffb roll
  //static int lwpos = 127, cwpos = 127, revs = 0; //wheel center pos default
  static int cgear = 0;     //current gear  - from 7seg digits
  static int csped = 0;     //current speed - from 7seg digits
  static int rollp = 0;     //previous roll target for computing roll delta used in traction loss
#if 1
  static char max_revs = 0; //overrev jolt and rev-based pitch
  //max revs
  if (revsk == 9)
  {
    max_revs = ~max_revs;
    if (max_revs)
      pv_pitch = pf_shiftspd/2;
    else
      pv_pitch = 0;
  }
#endif
  //
  //report ++; //drop the usb report id
  if (0 && report[0] != 0x04)
  {
    fprintf (stdout, "\n#i.DAT@%04d:", rlen);
    for (int i = 0; i < rlen; i++)
      fprintf (stdout, " %02x", report[i]);
  }
  switch ((char) report[0])
  {
    case (PKTT_DEBUG):  //debug data
      return 0;
    //
    case (PKTT_OUT):   //FFB data
      /*
       * FFB data processing
       */
    {
      //
      _wd = 'F';
      if (0)
      {
        fprintf (stdout, "\n#i.FFB2@%04d:", rlen);
        for (int i = 0; i < rlen; i++)
          fprintf (stdout, " %02x", report[i]);
      }
      //+FFB command decode
      //#i.FFB@0001: 07 4c 00 0e b7 0e 04 1e 6f dc 06 07 41 00 03 30 01 08 84 84 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
      //  DOWNLOAD_AND_PLAY  - VARIABLE - 0x84 0x84 0x00 0x00 0x01
      //#i.FFB@0001: 07 4c 00 - envelope header
      //  0e b7 0e 04 - vid+pid
      //  1e 6f dc 06 - timestamp
      //  07 41 00 - pkt header followed by pkt data (x41/65 bytes) and decoded message
      //  03 30 - endpoint 03, ?msgid? 30
      //  01 - DOWNLOAD_AND_PLAY
      //  08 - VARIABLE
      //  84 84 00 00 01 - params, followed by padding
      //  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
      //  DOWNLOAD_AND_PLAY - VARIABLE - 0x84 0x84 0x00 0x00 0x01
      char *ffbdata = report + 5; //3 for header, 1 for endpoint id, 1 for ?msgid?
      if (0 || _cap&CAP_FFB || _odbg)
        ff_lg_decode_command ((const unsigned char *)ffbdata);
      //uses almost identical message format as Logitech
      if(ffbdata[0] != FF_LG_CMD_EXTENDED_COMMAND)
      {
        unsigned char cmd = ffbdata[0] & FF_LG_CMD_MASK;
        switch(cmd)
        {
          case FF_LG_CMD_DOWNLOAD:
          case FF_LG_CMD_DOWNLOAD_AND_PLAY:
          case FF_LG_CMD_PLAY:
          case FF_LG_CMD_STOP:
          case FF_LG_CMD_REFRESH_FORCE:
          {
            if (0)
            {
              printf("\n#w:");
              ff_lg_decode_command ((const unsigned char *)ffbdata);
            }
            //printf("\n#i:convert command %s", ff_lg_get_cmd_name(cmd));
            if (cmd == FF_LG_CMD_STOP)
            {
              //should align/reset the platform now
              printf("\n#w:reset platform on %s", ff_lg_get_cmd_name(cmd));
              pf_roll = 0;
              rollp = 0;
              pf_rolld = 0;
            }
            else
            {
              //printf("\n#w:process force for %s", ff_lg_get_cmd_name(cmd));
              s_ff_lg_command * force = (s_ff_lg_command *)(ffbdata + 1);
              //process each force type
              switch (force->force_type)
              {
                case FF_LG_FTYPE_CONSTANT:
                {
                  //whlpos = force->parameters[0];
                  if (0||_odbg) printf ("\n#unused CONSTANT %02x %02x %02x %02x %02x", force->parameters[0], force->parameters[1], force->parameters[2], force->parameters[3], force->parameters[4]);
                  break;
                }
                case FF_LG_FTYPE_VARIABLE:
                {
                  //whlpos = force->parameters[1] * 255 + force->parameters[2]; //max precision
                  //platform roll is given by this value: left> FF..80,7F..00 <right
                  //80 and 7F are used for 0 roll - use only ~80% of the range for more immersiveness?!
                  if (0||_odbg) printf ("\n#VARIABLE %02x %02x %02x %02x %02x", force->parameters[0], force->parameters[1], force->parameters[2], force->parameters[3], force->parameters[4]);
                  //pf_roll = get_cmap(force->parameters[0], 0xff, 0x00, -128, 128);
                  if (force->parameters[1])//GT Sport uses finer control on 2 bytes
                    pf_roll = get_cmap((long)force->parameters[1], (long)0xe0, (long)0x20, -128, 128);
                  else  //classic control on 1 byte
                    pf_roll = get_cmap((long)force->parameters[0], (long)0xe0, (long)0x20, -128, 128);
                  pf_rolld = pf_roll - rollp;
                  rollp = pf_roll;  //reset prev roll
                  //
                  if (0||_odbg) printf ("\n#d.ffb roll %02x > %d delta %d prev %d",
                    force->parameters[1]?force->parameters[1]:force->parameters[0], pf_roll, pf_rolld, rollp);
                  break;
                }
                case FF_LG_FTYPE_SPRING:
                case FF_LG_FTYPE_HIGH_RESOLUTION_SPRING:
                {
                  if (0||_odbg) printf ("\n#unused SPRING %02x %02x %02x %02x %02x", force->parameters[0], force->parameters[1], force->parameters[2], force->parameters[3], force->parameters[4]);
                  break;
                }
                case FF_LG_FTYPE_DAMPER:
                case FF_LG_FTYPE_HIGH_RESOLUTION_DAMPER:
                {
                  if (0||_odbg) printf ("\n#unused DAMPER %02x %02x %02x %02x %02x", force->parameters[0], force->parameters[1], force->parameters[2], force->parameters[3], force->parameters[4]);
                  break;
                }
                default:
                {
                  if (0||_odbg)printf ("\n#w:skip force type: %s", ff_lg_get_ftype_name(force->force_type));
                }
              }//force type
            }//command
            break;
          }//DL, PLAY, STOP, REFRESH
          case FF_LG_CMD_DEFAULT_SPRING_ON:
          case FF_LG_CMD_DEFAULT_SPRING_OFF:
          {
            if (0)
            {
              printf("\n#w:");
              ff_lg_decode_command ((const unsigned char *)ffbdata);
            }
            if (0||_odbg) printf("\n#w:skip SPRING command %s", ff_lg_get_cmd_name(cmd));
            break;
          }
          default:
          {
            if (_odbg) printf("\n#w:skipping unsupported command %s", ff_lg_get_cmd_name(cmd));
          }
        }//cmd
      }
      else    //extended command
      {
        switch(ffbdata[1])
        {
          case FF_FT_EXT_CMD_RPM_LEDS:
          {
            s_ff_lg_command * leds = (s_ff_lg_command *)(ffbdata + 1);
            //revs = count_ones(report[6]) + (report[7] & 0x01);
            revsk = count_ones(leds->parameters[0]);
            revsk += count_ones(leds->parameters[1] & 0x01);
            if (_odbg) printf ("\n#d.rev2 level %d %02x %02x %02x %02x", revsk, leds->parameters[0], leds->parameters[1], leds->parameters[2], leds->parameters[3]);
            if (_dashaddr)
            {
              //compute revs data based on revs leds - 9 leds in total, don't care about accuracy
              _dpkt[MFC_DIRPM]  = -revsk * 1000;  //rpm - not real ones, use negative values
              _dpkt[MFC_DIRPMM] = -9000;         //max rpm for 9 leds
            }
            break;
          }
          case FF_FT_EXT_CMD_WHL_DIGITS:
          {
            s_ff_lg_command * leds = (s_ff_lg_command *)(ffbdata + 1);
            static char dgts[] = {0, 0, 0, 0};
            static int  ddat = 0;
            //is this 7 segment related?
            if (_dashaddr)
            {
              //do we have digits control message?
              if (leds->parameters[0] == 0x01 && leds->parameters[1] == 0x02)
              {
                dgts[0] = ff_fan_check_led_digit(leds->parameters[2]);
                dgts[1] = ff_fan_check_led_digit(leds->parameters[3]);
                dgts[2] = ff_fan_check_led_digit(leds->parameters[4]);
                ddat = atoi(dgts);
                //printf ("\n#d.digits <%s> vs <%d> %02x %02x %02x %02x %02x %02x", dgts, ddat, leds->parameters[0], leds->parameters[1], leds->parameters[2], leds->parameters[3], leds->parameters[4], leds->parameters[5]);
                //extract speed and gears
                if (ddat < 10 && dgts[1] != ' ')  // GT Sport style
                {
                  //this should be the gears
                  if (ddat > 0)
                    cgear = ddat;
                  else
                  {
                    if (dgts[1] == 'r')
                      cgear = -1;
                    else if (dgts[1] == 'n')
                      cgear = 0;
                  }
                }
                else
                {
                  if (dgts[2] != ' ') //GT Sport style
                    //this should be the speed since the gear are normally in the middle digit
                    csped = ddat;
                }
                //speed and gears
                _dpkt[MFC_DIGEAR] = cgear;
                _dpkt[MFC_DISPD] = csped;
                //printf ("\n#d.digits <%s> vs <%d> speed %d gear %d", dgts, ddat, csped, cgear);
              }
              else
              {
              /*
              off .. green .. orange .. purple .. orange .. green .. off
#d.digits/led 08 00 00 00 00 00 > off
#d.digits/led 08 01 80 00 00 00 > green
#d.digits/led 08 01 c0 00 00 00 > green
#d.digits/led 08 01 e0 00 00 00 > orange
#d.digits/led 08 01 f0 00 00 00 > orange
#d.digits/led 08 01 f8 00 00 00 > purple
#d.digits/led 08 01 fc 00 00 00 > ..
#d.digits/led 08 01 fe 00 00 00
#d.digits/led 08 01 ff 00 00 00
#d.digits/led 08 01 fe 00 00 00
#d.digits/led 08 01 fc 00 00 00
#d.digits/led 08 01 f8 00 00 00
#d.digits/led 08 01 f0 00 00 00
#d.digits/led 08 01 e0 00 00 00
#d.digits/led 08 01 c0 00 00 00
#d.digits/led 08 01 80 00 00 00
#d.digits/led 08 01 00 00 00 00
#d.digits/led 08 00 00 00 00 00.
              */
              //printf ("\n#d.digits/led %02x %02x %02x %02x %02x %02x", leds->parameters[0], leds->parameters[1], leds->parameters[2], leds->parameters[3], leds->parameters[4], leds->parameters[5]);
              }
            } //_dashaddr
            break;
          }
          default:
          {
            if (_odbg) printf("\n#w:skip ext command %s", ff_lg_get_ext_cmd_name(ffbdata[1]));
          }
        }
        //process rev leds
      }

    } // FFB data
      break;
      /*
       *
       */
    case (PKTT_IN):  //wheel input data
      /*
       * wheel data processing
       *
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f8 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 da 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 bd 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a0 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 82 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 65 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 48 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 2b 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0d 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f0 7d ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0d 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 2b 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 48 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 65 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 82 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a0 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 bd 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 da 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f8 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 32 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 50 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00       *
//DIY handbrake test
#i.WHL@0000: 06 40 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 40 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 ff ff ff ff ff ff 00 00 00 00 00 00 00 00 00 00 00 00 00

#i.DAT@0067:
06 41 00 84 01 80 80 7f 80 08
00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 
00 00 00 00 00 00 00 00 00 00 
00 00 00 00 00 00 00 95 7f ff 
ff ff ff ff ff 00 ff ff 00 00 
00 00 00 00 00 00 00
       */
    {
      //
      _wd = 'W';      //
      //get accel/brake
      report++; //skip extra byte due to header length
      if (0)
      {
        fprintf (stdout, "\n#i.WHL2@%04d:", rlen);
        for (int i = 0; i < rlen; i++)
          fprintf (stdout, " %02x", report[i]);
      }
      /*
      * wheel: 1..32768<L R>-32768..-1
      *   acc: up: -1..-32768 > 32768..1 :dn
      * brake: up: -1..-32768 > 32768..1 :dn
      */
      static fanatec_whl *fanrep;
      fanrep = (fanatec_whl *)(report + 2);
      pw_roll = -normal_axis (get_short (report, 46), 0x0ffff); //from wheel turn
      lpacc   = normal_accel (get_short (report, 48), 0x0ffff);
      lpbrk   = normal_brake (get_short (report, 50), 0x0ffff);
      //lhbrk   = normal_brake (get_short (report, 54), 0x0ffff); //handbrake
      //
      if (0 || _odbg > 2)
        printf ("\n#RAW whl %d/%d acc %d/%d brk %d/%d clt %d",
          fanrep->whl, pw_roll, fanrep->acc, lpacc, fanrep->brk, lpbrk, fanrep->clt);
      if (_odbg > 2)
        printf ("\n#RAW whl %d acc %d brk %d", pw_roll, lpacc, lpbrk);
      //
      pw_acc = lpacc; pw_brk = lpbrk; pw_revs = revsk;
      //pw_pitch = accel_pitch_get(lpacc, lpbrk, revsk);
      //gear shifting: only when accelerating or braking
      if (lpbrk || lpacc)
      {
      //gear down
        if (report[9] & 0x01)
        {
          //printf ("\n#i.gear down");
          pf_pitch = pf_shiftspd;
          //reset pitch on gear change
          pw_acc = 0;
          //pw_pitch = accel_pitch_get(0, lpbrk, revsk);
        }
        //gear up
        if (report[9] & 0x02)
        {
          //printf ("\n#i.gear up");
          pf_pitch = -pf_shiftspd;
          //reset pitch on gear change
          pw_acc = 0;//lpacc/4;
          //pw_pitch = accel_pitch_get(lpacc/2, lpbrk, revsk);
        }
      }
    } // wheel data
      break;
    default:
      return 0;
  }
  return 1;
}

int motion_process_logitech (char *report, int rlen, unsigned long dtime)
{
  //update delta time
  if (dtime == -1)
    dtime = 4;
  _wd = 'U';
  //ffb wheel pos: ffb roll
  //static int lwpos = 127, cwpos = 127, revs = 0; //wheel center pos default
  static int cgear = 0;     //current gear  - from 7seg digits
  static int csped = 0;     //current speed - from 7seg digits
  static int rollp = 0;     //previous roll target for computing roll delta used in traction loss
#if 1
  static char max_revs = 0; //overrev jolt and rev-based pitch
  //max revs
  if (revsk == 9)
  {
    max_revs = ~max_revs;
    if (max_revs)
      pv_pitch = pf_shiftspd/2;
    else
      pv_pitch = 0;
  }
#endif
  //
  switch ((char) report[0])
  {
    case (PKTT_DEBUG):  //debug data
      return 0;
    //
    case (PKTT_OUT):   //FFB data
      /*
       * FFB data processing
       */
    {
      //
      _wd = 'F';
      //+FFB command decode
      char *ffbdata = report + 4;
      if (1 || _cap&CAP_FFB || _odbg)
      {
        printf("\n#i:");
        ff_lg_decode_command ((const unsigned char *)ffbdata);
      }
      //uses almost identical message format as Logitech
      if(ffbdata[0] != FF_LG_CMD_EXTENDED_COMMAND)
      {
        unsigned char cmd = ffbdata[0] & FF_LG_CMD_MASK;
        switch(cmd)
        {
          case FF_LG_CMD_DOWNLOAD:
          case FF_LG_CMD_DOWNLOAD_AND_PLAY:
          case FF_LG_CMD_PLAY:
          case FF_LG_CMD_STOP:
          case FF_LG_CMD_REFRESH_FORCE:
          {
            if (0)
            {
              printf("\n#w:");
              ff_lg_decode_command ((const unsigned char *)ffbdata);
            }
            //printf("\n#i:convert command %s", ff_lg_get_cmd_name(cmd));
            if (cmd == FF_LG_CMD_STOP)
            {
              //should align/reset the platform now
              printf("\n#w:reset platform on %s", ff_lg_get_cmd_name(cmd));
              pf_roll = 0;
              rollp = 0;
              pf_rolld = 0;
            }
            else
            {
              //printf("\n#w:process force for %s", ff_lg_get_cmd_name(cmd));
              s_ff_lg_command * force = (s_ff_lg_command *)(ffbdata + 1);
              //process each force type
              switch (force->force_type)
              {
                case FF_LG_FTYPE_CONSTANT:
                {
                  //whlpos = force->parameters[0];
                  break;
                }
                case FF_LG_FTYPE_VARIABLE:
                {
                  //whlpos = force->parameters[1] * 255 + force->parameters[2]; //max precision
                  //platform roll is given by this value: left> FF..80,7F..00 <right
                  //80 and 7F are used for 0 roll - use only ~80% of the range for more immersiveness?!
                  if (0||_odbg) printf ("\n#VARIABLE %02x %02x %02x %02x %02x", force->parameters[0], force->parameters[1], force->parameters[2], force->parameters[3], force->parameters[4]);
                  //pf_roll = get_cmap(force->parameters[0], 0xff, 0x00, -128, 128);
                  if (force->parameters[1])//GT Sport uses finer control on 2 bytes
                    pf_roll = get_cmap((long)force->parameters[1], (long)0xe0, (long)0x20, -128, 128);
                  else  //classic control on 1 byte
                    pf_roll = get_cmap((long)force->parameters[0], (long)0xe0, (long)0x20, -128, 128);
                  pf_rolld = pf_roll - rollp;
                  rollp = pf_roll;  //reset prev roll
                  //
                  if (0||_odbg) printf ("\n#d.ffb roll %02x > %d delta %d prev %d",
                    force->parameters[1]?force->parameters[1]:force->parameters[0], pf_roll, pf_rolld, rollp);
                  break;
                }
                case FF_LG_FTYPE_SPRING:
                case FF_LG_FTYPE_HIGH_RESOLUTION_SPRING:
                {
                  if (_odbg) printf ("\n#unused SPRING %02x %02x %02x %02x %02x", force->parameters[0], force->parameters[1], force->parameters[2], force->parameters[3], force->parameters[4]);
                  break;
                }
                case FF_LG_FTYPE_DAMPER:
                case FF_LG_FTYPE_HIGH_RESOLUTION_DAMPER:
                {
                  if (_odbg) printf ("\n#unused DAMPER %02x %02x %02x %02x %02x", force->parameters[0], force->parameters[1], force->parameters[2], force->parameters[3], force->parameters[4]);
                  break;
                }
                default:
                {
                  printf ("\n#w:skip force type: %s", ff_lg_get_ftype_name(force->force_type));
                }
              }//force type
            }//command
            break;
          }//DL, PLAY, STOP, REFRESH
          case FF_LG_CMD_DEFAULT_SPRING_ON:
          case FF_LG_CMD_DEFAULT_SPRING_OFF:
          {
            if (0)
            {
              printf("\n#w:");
              ff_lg_decode_command ((const unsigned char *)ffbdata);
            }
            if (_odbg) printf("\n#w:skip command %s", ff_lg_get_cmd_name(cmd));
            break;
          }
          default:
          {
            if (_odbg) printf("\n#w:skipping unsupported command %s", ff_lg_get_cmd_name(cmd));
          }
        }//cmd
      }
      else    //extended command
      {
        switch(ffbdata[1])
        {
          case FF_FT_EXT_CMD_RPM_LEDS:
          {
            s_ff_lg_command * leds = (s_ff_lg_command *)(ffbdata + 1);
            //revs = count_ones(report[6]) + (report[7] & 0x01);
            revsk = count_ones(leds->parameters[0]);
            revsk += count_ones(leds->parameters[1] & 0x01);
            if (_odbg) printf ("\n#d.rev2 level %d %02x %02x %02x %02x", revsk, leds->parameters[0], leds->parameters[1], leds->parameters[2], leds->parameters[3]);
            if (_dashaddr)
            {
              //compute revs data based on revs leds - 9 leds in total, don't care about accuracy
              _dpkt[MFC_DIRPM]  = -revsk * 1000;  //rpm - not real ones, use negative values
              _dpkt[MFC_DIRPMM] = -9000;         //max rpm for 9 leds
            }
            break;
          }
          case FF_FT_EXT_CMD_WHL_DIGITS:
          {
            s_ff_lg_command * leds = (s_ff_lg_command *)(ffbdata + 1);
            static char dgts[] = {0, 0, 0, 0};
            static int  ddat = 0;
            //is this 7 segment related?
            if (_dashaddr)
            {
              //do we have digits control message?
              if (leds->parameters[0] == 0x01 && leds->parameters[1] == 0x02)
              {
                dgts[0] = ff_fan_check_led_digit(leds->parameters[2]);
                dgts[1] = ff_fan_check_led_digit(leds->parameters[3]);
                dgts[2] = ff_fan_check_led_digit(leds->parameters[4]);
                ddat = atoi(dgts);
                //printf ("\n#d.digits <%s> vs <%d> %02x %02x %02x %02x %02x %02x", dgts, ddat, leds->parameters[0], leds->parameters[1], leds->parameters[2], leds->parameters[3], leds->parameters[4], leds->parameters[5]);
                //extract speed and gears
                if (ddat < 10 && dgts[1] != ' ')  // GT Sport style
                {
                  //this should be the gears
                  if (ddat > 0)
                    cgear = ddat;
                  else
                  {
                    if (dgts[1] == 'r')
                      cgear = -1;
                    else if (dgts[1] == 'n')
                      cgear = 0;
                  }
                }
                else
                {
                  if (dgts[2] != ' ') //GT Sport style
                    //this should be the speed since the gear are normally in the middle digit
                    csped = ddat;
                }
                //speed and gears
                _dpkt[MFC_DIGEAR] = cgear;
                _dpkt[MFC_DISPD] = csped;
                //printf ("\n#d.digits <%s> vs <%d> speed %d gear %d", dgts, ddat, csped, cgear);
              }
              else
              {
              /*
              off .. green .. orange .. purple .. orange .. green .. off
#d.digits/led 08 00 00 00 00 00 > off
#d.digits/led 08 01 80 00 00 00 > green
#d.digits/led 08 01 c0 00 00 00 > green
#d.digits/led 08 01 e0 00 00 00 > orange
#d.digits/led 08 01 f0 00 00 00 > orange
#d.digits/led 08 01 f8 00 00 00 > purple
#d.digits/led 08 01 fc 00 00 00 > ..
#d.digits/led 08 01 fe 00 00 00
#d.digits/led 08 01 ff 00 00 00
#d.digits/led 08 01 fe 00 00 00
#d.digits/led 08 01 fc 00 00 00
#d.digits/led 08 01 f8 00 00 00
#d.digits/led 08 01 f0 00 00 00
#d.digits/led 08 01 e0 00 00 00
#d.digits/led 08 01 c0 00 00 00
#d.digits/led 08 01 80 00 00 00
#d.digits/led 08 01 00 00 00 00
#d.digits/led 08 00 00 00 00 00.
              */
              //printf ("\n#d.digits/led %02x %02x %02x %02x %02x %02x", leds->parameters[0], leds->parameters[1], leds->parameters[2], leds->parameters[3], leds->parameters[4], leds->parameters[5]);
              }
            } //_dashaddr
            break;
          }
          default:
          {
            if (_odbg) printf("\n#w:skip ext command %s", ff_lg_get_ext_cmd_name(ffbdata[1]));
          }
        }
        //process rev leds
      }

    } // FFB data
      break;
      /*
       *
       */
    case (PKTT_IN):  //wheel input data
      /*
       * wheel data processing
       *
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f8 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 da 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 bd 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a0 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 82 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 65 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 48 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 2b 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0d 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f0 7d ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0d 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 2b 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 48 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 65 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 82 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a0 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 bd 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 da 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f8 7e ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 32 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i:WHL pkt: 06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 50 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00       *
//DIY handbrake test
#i.WHL@0000: 06 40 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#i.WHL@0000: 06 40 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 ff ff ff ff ff ff 00 00 00 00 00 00 00 00 00 00 00 00 00
       */
    {
      //
      _wd = 'W';      //
      //get accel/brake
      /*
      * wheel: 1..32768<L R>-32768..-1
      *   acc: up: -1..-32768 > 32768..1 :dn
      * brake: up: -1..-32768 > 32768..1 :dn
      */
      pw_roll = normal_axis (get_short (report, 46), 0x0ffff); //from wheel turn
      lpacc   = normal_accel (get_short (report, 48), 0x0ffff);
      lpbrk   = normal_brake (get_short (report, 50), 0x0ffff);
      //lhbrk   = normal_brake (get_short (report, 54), 0x0ffff); //handbrake
      //
      if (_odbg > 2)
        printf ("\n#RAW whl %d acc %d brk %d", pw_roll, lpacc, lpbrk);
      //
      pw_pitch = accel_pitch_get(lpacc, lpbrk, revsk);
      //gear shifting: only when accelerating or braking
      if (lpbrk || lpacc)
      {
      //gear down
        if (report[9] & 0x01)
        {
          //printf ("\n#i.gear down");
          pf_pitch = -pf_shiftspd;
        }
        //gear up
        if (report[9] & 0x02)
        {
          //printf ("\n#i.gear up");
          pf_pitch = pf_shiftspd;
        }
      }
    } // wheel data
      break;
    default:
      return 0;
  }
  return 1;
}
/*
 * Thrustmaster T300RS
 *
* PS menu
#W:***pkt type: 0xee: ee 40 48 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 48 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#*
*/
unsigned char ffb_var1[] =
{ 0xee, 0x40, 0x39, 0x02, 0x41, 0x01, 0x00, 0x00 };

/*
* 35 spring/damper update effect
*
* 35 - update effect
* 30 - id
* 00 - unk
* ff - max intensity level: right
* ff - max intensity level: left
* ff - intensity level: signed, min=-103, max=103, center=0
* ff - direction: right = 0x00, left = 0xff
* 00 - unk
* 00 - unk
* 4c - unk
* 4c - unk
*
* ee 40 35 30 00 4c 4c 00 00 00 00 4c 4c 00 00
*/
unsigned char ffb_whlsprg1[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x4c, 0x4c, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x4c, 0x00, 0x00 };

/*
* 35 spring update effect
*
* 35 - update effect
* 50 - id
* 00 - unk
* ff - max intensity level: right
* ff - max intensity level: left
* ff - intensity level: signed, min=-103, max=103, center=0
* ff - direction: right = 0x00, left = 0xff
* 00 - unk
* 00 - unk
*
* ee 40 35 50 00 2b 2b 00 00 64 00 64 64 00 00
*/
unsigned char ffb_whlsprg2[] =
{ 0xee, 0x40, 0x35, 0x50, 0x00, 0x2b, 0x2b, 0x00, 0x00, 0x64, 0x00, 0x64, 0x64, 0x00, 0x00 };

unsigned char ffb_vib1[] =
//{ 0xee, 0x40, 0x34, 0x90, 0x00, 0xXX(intensity) };
//FFB@00072ms: ee 40 34 90 00 02 00 00 19 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
{ 0xee, 0x40, 0x34, 0x90, 0x00, 0xFF, 0x00 };

unsigned char ffb_vib1_off[] =
//{ 0xee, 0x40, 0x34, 0x90, 0x00, 0xXX(intensity) };
{ 0xee, 0x40, 0x34, 0x90, 0x00, 0x00, 0x00 };

//weak vibrations?!
//FFB@00027ms: ee 40 34 70 00 12 00 00 64 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
unsigned char ffb_vib2[] =
//{ 0xee, 0x40, 0x34, 0x70, 0x00, 0xXX(intensity) };
{ 0xee, 0x40, 0x34, 0x70, 0x00, 0xFF, 0x00 };

unsigned char ffb_vib2_off[] =
//{ 0xee, 0x40, 0x34, 0x70, 0x00, 0xXX(intensity) };
{ 0xee, 0x40, 0x34, 0x70, 0x00, 0x00, 0x00 };

unsigned char ffb_vib_on[] =
{ 0xee, 0x40, 0x39, 0x04, 0x01, 0x01, 0x00 };

unsigned char ffb_vib_off[] =
{ 0xee, 0x40, 0x39, 0x04, 0x00, 0x01, 0x00 };

unsigned char ffb_wrc5[] =
//{ 0xee, 0x40, 0x33, 0x40, 0x00, 0x15 };
{ 0xee, 0x40, 0x3d, 0xFF, 0x00, 0x00, 0x00 };

unsigned char ffb_wrc5_on[] =
{ 0xee, 0x40, 0x39, 0x04, 0x41, 0x00, 0x00 };

unsigned char ffb_wrc5_off[] =
{ 0xee, 0x40, 0x39, 0x04, 0x00, 0x00, 0x00 };

unsigned char ffb_pcars[] =
//{ 0xee, 0x40, 0x33, 0x40, 0x00, 0x15 };
//{ 0xee, 0x40, 0x33, 0x50, 0x00, 0x15 };
//{ 0xee, 0x40, 0x33, 0x30, 0x00, 0xFF };
{ 0xee, 0x40, 0x33, 0x10, 0x00, 0xFF };

unsigned char ffb_pcars_on[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x00, 0x00 };

unsigned char ffb_pcars_off[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x64, 0x64 };

/*
GT Sport update
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 b9 fd 00 49 53 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 08 fe 00 49 46 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 1c fe 00 49 58 24 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 f8 fd 00 49 47 7c 00 c8 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##07 41 03 3e 00 01 02 c0 fd 00 49 43 12 00 0b 00 07 00 0b 00 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
*/
unsigned char ffb_gtsp2[] =
//{ 0xee, 0x40, 0x33, 0x40, 0x00, 0x15 };
//{ 0xee, 0x40, 0x33, 0x50, 0x00, 0x15 };
{ 0xee, 0x40, 0x3e, 0x00, 0x01, 0x02, 0xFF, 0xFF, 0x00, 0x49 };

unsigned char ffb_dcvr[] =
//{ 0xee, 0x40, 0x33, 0x40, 0x00, 0x15 };
//{ 0xee, 0x40, 0x33, 0x50, 0x00, 0x15 };
{ 0xee, 0x40, 0x35, 0x20, 0x00, 0x07, 0x07, 0x00 };

unsigned char ffb_dcvr_on[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x00, 0x00 };

unsigned char ffb_dcvr_off[] =
{ 0xee, 0x40, 0x35, 0x30, 0x00, 0x64, 0x64 };
unsigned char ffb_byte_knt = 3;
unsigned char ffb_byte_pos = 7;

static int ff_tm_fpos(char *rep)
{
  static int lpos;
  lpos = 255; //not a force we can handle yet
  switch(rep[0] & FF_TM_CMD_MASK)
  {
    case FF_TM_FOR0E:
      if (rep[1] == 0x00)
      {
        if (rep[2] == 0x01 && rep[3] == 0x02)
        {
          //3e 00 01 02 f8 02 00 49 53 14 1f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
          lpos = normal_ffb (((int) rep[5]), 0x0ff);//get_cmap((long)rep[4], (long)0xe0, (long)0x20, -128, 128);
        }
        else if (rep[2] == 0x02 || rep[2] == 0x03)
        {
          //3e 00 02 0e 43 a1 2a a1 2a 03 0a c5 df 00 49 43 08 00 08 00 00 00 04 00 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
          //3e 00 03 0e 43 a4 2a a4 2a 02 0a 9b 12 00 49 47 9f 00 88 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
          //platform roll is given by this value: left> FF..80,7F..00 <right
          //80 and 7F are used for 0 roll - use only ~80% of the range for more immersiveness?!
          if (rep[3] == 0x0e && rep[4] == 0x43)
          {
            lpos = normal_ffb (((int) rep[12]), 0x0ff);//get_cmap((long)rep[5], (long)0xe0, (long)0x20, -128, 128);
          }
        }
      }
    break;
    case FF_TM_FOR03:
      if (rep[1] == 0x10 && rep[2] == 0x00)
      {
      //#i:FFB pkt: 07 41 03 
      // 33 10 00 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00       00 00 00 00 00 00 00 00
      //#FOR03 0x10 0x00 I:0x12 0x00 0x00 0x00
        /* wheel ffb position
        * 128..255 | 0..127
        */
        lpos = normal_ffb (((int) rep[3]), 0x0ff);//get_cmap((long)rep[4], (long)0xe0, (long)0x20, -128, 128);
      }
    break;
  }
  if (lpos == 255 && 0)
  {
    printf ("\n#i:UNK FFB pkt: ");
    for (int i = 0; i < 24; i++)
      printf ("%02x ", rep[i]);
  }
  return lpos;
}

int motion_process_thrustmaster (char *report, int rlen, unsigned long mtime)
{
  //update delta time
  _wd = 'U';
  static int lffbpos = 0;
  static int lrollp = 0;
  //
  switch ((char) report[0])
  {
    case (PKTT_DEBUG):  //debug data
    {
      unsigned int dts = report[6] << 8 | report[7];
      printf ("\n#i:DBG pkt @%04d: ", dts);
      for (int i = 0; i < rlen; i++)
        printf ("%02x ", report[i]);
    }
    break;
    //
    case (PKTT_OUT):   //FFB data
      /*
       * FFB data processing
       */
    {
      if (1)
      {
        printf ("\n#i@%.3f:FFB pkt: ", mtime/1000.0f);
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
      //ff_tm_decode_command((const unsigned char *)(report + 3));
      lffbpos = ff_tm_fpos(report + 3);
      if (lffbpos != 255) //only if we have proper force data/cmd
      {
        pf_roll = lffbpos;
        pf_rolld = (pf_roll - lrollp);
        if (0||_odbg) printf ("\n#d.ffb roll 0x%02x > %d delta %d prev %d",
          report[4], pf_roll, pf_rolld, lrollp);
        lrollp = pf_roll;  //reset prev roll
      }
      #if 0
      //advance the report data pointer
      report++;
      //
      _wd = 'F';
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_var1 + 2), 5) == 0)
      {
        //nothing to do here, standard FFB keepalive 1
        _wd = 'U';
      }
      else
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_whlsprg1 + 2), 3) == 0)
      {
        //nothing to do here, spring/damper data on 
        _wd = 'U';
      }
      else
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_whlsprg2 + 2), 3) == 0)
      {
        //nothing to do here, standard FFB keepalive 1
        _wd = 'U';
      }
      else
      //check vibrations START
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_vib_on + 2), 5) == 0)
      {
        //pv_pitch = ((int) report[5])>127?((int) report[5])-255:((int) report[5]);
        //printf ("\n#i:vibration START");
        vib_k = 1;
      }
      else
      //check vibrations
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_vib_off + 2), 5) == 0)
      {
        //pv_roll = -((int) report[5]);
        //pv_pitch = ((int) report[5])>127?((int) report[5])-255:((int) report[5]);
        //printf ("\n#i:vibration STOP");
        vib_k = 0;
        _wd = 'U';
      }
      else
      //check vibrations
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_vib1 + 2), 3) == 0)
      {
        //pv_roll = -((int) report[5]);
        pv_pitch = ((int) report[5])>127?((int) report[5])-255:((int) report[5]);
        //printf ("\n#i:vibration1 intensity %d", pv_pitch);
      }
      else
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_vib2 + 2), 3) == 0)
      {
        //pv_roll = -((int) report[5]);
        //pv_pitch = ((int) report[5]);
        pv_pitch = ((int) report[5])>127?((int) report[5])-255:((int) report[5]);
        //printf ("\n#i:vibration2 intensity %d", pv_pitch);
      }
      else
      /*
       * GT Sport
#W:***pkt type: 0xee: ee 40 48 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 03 1e 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 11 54 d5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 3b 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 03 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 11 ff ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 32 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 00 01 00 00 00 00 00 00 10 00 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 39 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 01 07 40 ff ff 00 ff ff 30 00 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 39 01 01 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 02 07 00 00 00 00 00 00 50 00 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 39 02 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 70 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 03 08 00 00 00 00 00 00 70 00 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 39 03 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
       * PCARS/PCARS2/GT Sport
       * inspect wheel vibration data
       * sample: [len: 0x40] 33 40 00 XX
       * -or-
       * sample: [len: 0x40] 33 50 00 XX
       * -or- GT Sport
       * sample: [len: 0x40] 33 10 00 XX
       * -or- PCARS2
       * sample: [len: 0x40] 33 30 00 XX
       */
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_pcars + 2), 3) == 0)
      {
        /* small values when wheel is turned right
         * large values when wheel is turned left
         * - both varies little for small vibs, very much for big vibs
         */
        /*
        * process vibrations
        */
        /* wheel ffb position
        * 128..255 | 0..127
        */
        pf_roll = normal_ffb (((int) report[5]), 0x0ff);
        if (_odbg > 2)
          printf ("\n# PCARS ffb roll %d / %d", pf_roll, ((int) report[5]));
        //
      }
      else
/*
* GT Sport update
#W:*pkt type: 0xee: ee 40 3e 00 01 02 4d 03 00 49 46 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 a2 04 00 49 58 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 c3 03 00 49 47 7c 00 c8 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 ee fd 00 49 43 12 00 0a 00 08 00 08 00 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 dd fd 00 49 53 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 c4 01 00 49 46 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 f0 ff 00 49 58 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 e9 f8 00 49 47 7c 00 c8 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 93 fa 00 49 43 12 00 0a 00 08 00 08 00 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 6f 00 00 49 53 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:*pkt type: 0xee: ee 40 3e 00 01 02 89 00 00 49 46 14 0f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
* inspect wheel vibration data
* sample: [len: 0x40]     3e 00 01 02 YY XX 00
*/
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_gtsp2 + 2), 4) == 0)
      {
        /* small values when wheel is turned right
         * large values when wheel is turned left
         * - both varies little for small vibs, very much for big vibs
         */
        /*
        * process vibrations
        */
        /* wheel ffb position
        * 128..255 | 0..127
        */
        pf_roll = normal_ffb (((int) report[7]), 0x0ff);
        if (_odbg > 2)
          printf ("\n# PCARS ffb roll %d / %d", pf_roll, ((int) report[5]));
        //
      }
      else
        /*
         * Dirt4
#W:***pkt type: 0xee: ee 40 48 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 3a 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 38 11 ff ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 35 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#W:***pkt type: 0xee: ee 40 31 02 08 40 10 27 00 ff ff 50 00 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
         * WRC5/6, Dirt Rally
         * inspect wheel vibration data
FFB@002ms: 0002 01 ee 40 3d 4a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 0a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FFB@002ms: 0002 01 ee 40 3d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
         */
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_wrc5 + 2), 1) == 0)
      {
        /* small values when wheel is turned right
         * large values when wheel is turned left
         * - both varies little for small vibs, very much for big vibs
         */
        pf_roll = - normal_ffb (((int) report[3]), 0x0ff);
        if (_odbg > 2)
          printf ("\n#i:wrc/drally ffb roll %d / %d", pf_roll, ((int) report[3]));
      }
      else
        /*
         * DriveClub VR
         * inspect wheel vibration data
FFB@00004ms: ee 40 35 20 00 06 06 f2 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00012ms: ee 40 35 20 00 06 06 f1 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00003ms: ee 40 35 20 00 06 06 f0 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00003ms: ee 40 35 20 00 06 06 ef ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00003ms: ee 40 35 20 00 06 06 f0 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
FFB@00003ms: ee 40 35 20 00 06 06 f1 ff 00 00 14 14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0
         */
      if (memcmp ((const void *) (report + 2), (const void *) (ffb_dcvr + 2), 2) == 0)
      {
        pf_roll = - normal_ffb (((int) report[7]), 0x0ff);
        if (_odbg > 2)
          printf ("\n#i:wrc/drally ffb roll %d / %d", pf_roll, ((int) report[7]));
      }
      else
      {
        //unhandled
        _wd = 'U';
        if (0)
        {
          printf ("\n#w!ffb pkt: ");
          for (int i = 0; i < rlen; i++)
            printf ("%02x ", report[i - 1]);
        }
      }
      #endif
    } // FFB data
      break;
      /*
       *
       */
    case (PKTT_IN):  //wheel input data
      /*
       * wheel data processing
       *
#pkt@000ms/067 bytes
##06 41 84 01 80 80 80 80 08 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0f 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##06 41 84 01 80 80 80 80 08 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0e 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##06 41 84 01 80 80 80 80 08 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0c 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#pkt@000ms/067 bytes
##06 41 84 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0c 80 ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
       *
#!WHL@out 64B:
 01 80 80 80 80 08 00 00
 00 00 00 00 00 00 00 00
 00 00 00 00 00 00 00 00
 00 00 00 00 00 00 00 00
 00 00 00 00 00 00 00 00
 00 00 00 a6 7f ff ff ff
 ff ff ff 00 ff ff 00 00
 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a3 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a4 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a6 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 a8 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ab 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 af 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 b2 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 b6 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 bb 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c0 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c3 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
#!WHL@out 64B: 01 80 80 80 80 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c8 7f ff ff ff ff ff ff 00 ff ff 00 00 00 00 00 00 00 00 00 00
       *
       */
    {
      //
      if (0)
      {
        printf ("\n#i:WHL pkt: ");
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
      //advance the report data pointer
      report++;
      _wd = 'W';      //
      //get accel/brake
      /*
        * wheel: 0..32768<L R>-32768..-1
        *   acc: up: -1..-32768 > 32768..1 :dn
        * brake: up: -1..-32768 > 32768..1 :dn
      */
      pw_roll = normal_axis (get_short (report, 45), 0x0ffff); //from wheel turn
      lpbrk   = normal_brake (get_short (report, 49), 0x0ffff);
      lpacc   = normal_accel (get_short (report, 47), 0x0ffff);
      //
      pw_pitch = accel_pitch_get(lpacc, lpbrk, 0);
      //
      if (0 || _odbg > 2)
        printf ("\n#RAW whl %d acc %d brk %d", get_short (report, 45), get_short (report, 47), get_short (report, 49));
      //
      //gear shifting: only when accelerating or braking
      if (lpbrk || lpacc)
      {
      //gear down
        if (report[8] & 0x01)
        {
          //printf ("\ngear down\n");
          pf_pitch = pf_shiftspd;
        }
        //gear up
        if (report[8] & 0x02)
        {
          pf_pitch = -pf_shiftspd;
        }
      }
    } // wheel data
      break;
    default:
      /*
       * wheel data processing
       */
      if (_odbg > 2)
      {
        printf ("\n#w!UNK pkt: ");
        for (int i = 0; i < rlen; i++)
          printf ("%02x ", report[i]);
      }
  }
  //
  return 0;
}
