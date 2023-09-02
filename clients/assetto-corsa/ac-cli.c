/*
* codemasters f1 dashboard test

gcc -o /opt/mfcc-ac clients/assetto_corsa/ac-cli.c -lrt -std=c11
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
#include <math.h>

#include <extras.h>
//from https://docs.google.com/document/d/1KfkZiIluXZ6mMhLWfDX1qAGbvhGRC3ZUzjVIt5FQpp4/pub

#if 0
struct RTCarInfo
{
int identifier; //char identifier;  //0
int size;                           //4
float speed_Kmh;                    //8
float speed_Mph;                    //12
float speed_Ms;                     //16
bool isAbsEnabled;                  //20
bool isAbsInAction;                 //21
bool isTcInAction;                  //22
bool isTcEnabled;                   //23
bool isInPit;                       //24
bool isEngineLimiterOn;             //25
float accG_vertical;                //26
float accG_horizontal;              //30
float accG_frontal;                 //34
int lapTime;                        //38
int lastLap;
int bestLap;
int lapCount;
float gas;
float brake;
float clutch;
float engineRPM;
float steer;
int gear;
float cgHeight;
float wheelAngularSpeed[4];
float slipAngle[4];
float slipAngle_ContactPatch[4];
float slipRatio[4];
float tyreSlip[4];
float ndSlip[4];
float load[4];
float Dy[4];
float Mz[4];
float tyreDirtyLevel[4];
float camberRAD[4];
float tyreRadius[4];
float tyreLoadedRadius[4];
float suspensionHeight[4];
float carPositionNormalized;
float carSlope;
float carCoordinates[3];

}

struct RTLap
{
int carIdentifierNumber;
int lap;
char driverName[50];
char carName[50];
int time;

};

struct handshackerResponse{
char carName[50];
char driverName[50];
int identifier;
int version;
char trackName[50];
char trackConfig[50];

}
#endif
#define bool unsigned char
struct RTCarInfo
{
        char identifier;
        int size;

        float speed_Kmh;
        float speed_Mph;
        float speed_Ms;

        bool isAbsEnabled;
        bool isAbsInAction;
        bool isTcInAction;
        bool isTcEnabled;
        bool isInPit;
        bool isEngineLimiterOn;


        float accG_vertical;
        float accG_horizontal;
        float accG_frontal;

        int lapTime;
        int lastLap;
        int bestLap;
        int lapCount;

        float gas;
        float brake;
        float clutch;
        float engineRPM;
        float steer;
        int gear;
        float cgHeight;

        float wheelAngularSpeed[4];
        float slipAngle[4];
        float slipAngle_ContactPatch[4];
        float slipRatio[4];
        float tyreSlip[4];
        float ndSlip[4];
        float load[4];
        float Dy[4];
        float Mz[4];
        float tyreDirtyLevel[4];

        float camberRAD[4];
        float tyreRadius[4];
        float tyreLoadedRadius[4];

        float suspensionHeight[4];

        float carPositionNormalized;

        float carSlope;

float carCoordinates[3];

} ;

#define UDP_MAX_PACKETSIZE  328
#define UDP_PORT            9996

//#define UDP_MAX_PACKETSIZE  2048

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

int inet_aton(const char *cp, struct in_addr *inp);
//int usleep(long usec);

//get number of millis from app start or first call
//first call will return 0
static int ctime_ms(char val)
{
  static int st_ms = 0;
    struct timeval te;
    gettimeofday (&te, NULL); // get current time
    long long ms = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
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

//+bcast network
int inet_aton(const char *cp, struct in_addr *inp);
/*
  int pkt_type;
  int pkt_dof_type;
  int data[6];    //{speed, roll, pitch, ..}
*/
//-
int mfc_pkt[MFC_PKTSIZE] = {0};
int mfcdash_pkt[MFCDASH_PKTSIZE] = {0};

char _done = 0;
char _odbg = 0;
char *_dashaddr = NULL;
char _learn = 0;

void terminate (int sig)
{
  _done = 1;
}

/**
A. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral - add

B. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral (invert) - add
*/
#if 0
float _pitchprc = -30.0f;
float _rollprc  = -20.0f;
float _yawprc   = 60.0f;
float _surgeprc = 30.0f;
float _swayprc  = -30.0f;
float _heaveprc = -30.0f;
float _trlossprc= -60.0f;
float _extra1prc= 100.0f;
float _extra2prc= 100.0f;
float _extra3prc= 100.0f;
#else
float _pitchprc = -30.0f;
float _rollprc  = 20.0f;
float _yawprc   = 60.0f;
float _surgeprc = 30.0f;
float _swayprc  = -30.0f;
float _heaveprc = 30.0f;
float _trlossprc= -60.0f;
float _extra1prc= 100.0f;
float _extra2prc= 100.0f;
float _extra3prc= 100.0f;
#endif
static int *_cpkt, *_dpkt, pktl;
/* original min/Max
//MAX values
static int Mpkt[MFC_PKTSIZE] = {1, 1,   127700,   27465,   68605,    79686,   20256,   1799992,  232291, 0, 0, 0};
//min values
static int mpkt[MFC_PKTSIZE] = {1, 1,  -127700,  -27465,  -68605,   -79686,  -20256,  -1799992, -232291, 0, 0, 0};
*/
/* learning mode
#i:new min/Max values:
static int mpkt = [0, 0, -55789, -17792, -25177, -39887, -27095, -1793308, -83005, 0, 0, 0];
static int Mpkt = [0, 0, 71807, 16820, 10602, 20307, 12984, 1795649, 94049, 100, 100, 100];
#i:new min/Max values:
static int mpkt[] = {0, 0, 0, -10652, -4007, 0, -9414, 0, 0, 0, 0, 0};
static int Mpkt[] = {0, 0, 0, 4875, 5460, 0, 8459, 0, 0, 100, 100, 100};
*/
//static int mpkt[] = {0, 0, 0, -11182, 0, 0, -6779, -11313, 0, 0, 0, 0};
//static int Mpkt[] = {0, 0, 0, 4605,   0, 0, 14565,  10878, 0, 100, 100, 100};
/*
static int mpkt[] = {0, 0, -55789, -17792, -25177, -39887, -27095, -1793308, -83005, 0, 0, 0};
static int Mpkt[] = {0, 0, 71807, 16820, 10602, 20307, 12984, 1795649, 94049, 100, 100, 100};
*/

int mfc_packet_use(char* packetBuffer, float rtime, float cltime)
{
  struct RTCarInfo *tpkt = (struct RTCarInfo *)packetBuffer;
  //static float ori0 = 0, ori1 = 0, ori2 = 0;
  //static float acc0 = 0, acc1 = 0, acc2 = 0;
  //static float vel0 = 0, vel1 = 0, vel2 = 0;
  //printf("\n#i.max axis at 60%%: %.3f", get_cmap_f(-137, -1795547, 1795547, -6000, 6000));
  //#define DOF_MAG (10000)
  #define DOF_MAG (5000)
  /* uses 10000 magnitude to not lose much fidelity on computation
                                
                                179.554748535156
  */
  //int Mpkt[MFC_PKT_SIZE] = {1, 1,   50483,   43371,   1795547,   11451,   14847,   5426, 1};
  // *!!!* careful with indexes as they need to correspond to the data pkt order
  //                                     //pitch  //surge  //heave  //roll   //sway   //yaw     //trloss
  static float dof_roll, dof_pitch, dof_yaw, dof_heave, dof_sway, dof_surge, dof_tloss;
  //static float fv[MFCDASH_PKTSIZE];
  //game telemetry data
  #define VERT_IDX 32
  #define HORI_IDX 28
  #define LONG_IDX 36
  //
  dof_pitch = 0;//-get_float (packetBuffer, VERT_IDX + 8);
  dof_surge = tpkt->accG_frontal;//get_float (packetBuffer, LONG_IDX);
  dof_heave = tpkt->accG_vertical;//get_float (packetBuffer, VERT_IDX);
  dof_roll = 0;//-get_float (packetBuffer, HORI_IDX + 4);
  dof_sway = tpkt->accG_horizontal; //get_float (packetBuffer, HORI_IDX);
  dof_yaw = 0; //get_float (packetBuffer, HORI_IDX); //TODO: compute direction
  dof_tloss = (int)(0 * DOF_MAG);
  //
  if (_odbg)
    printf ("\n#i@%.3f:t1 pitch %f %f %f roll %f %f yaw %f %f", cltime,
      dof_pitch, dof_surge, dof_heave, dof_roll, dof_sway, dof_yaw, dof_tloss);
  if (1 || _odbg)
    printf ("\n#i@%.3f:t1.1 pitch %f %f %f roll %f %f yaw %f %f", cltime,
      dof_pitch, tpkt->accG_frontal, tpkt->accG_vertical, dof_roll, tpkt->accG_horizontal, dof_yaw, dof_tloss);
  _cpkt[MFC_PIPITCH] = (int)(dof_pitch * DOF_MAG);
  _cpkt[MFC_PISURGE] = (int)(dof_surge * DOF_MAG);
  _cpkt[MFC_PIHEAVE] = (int)(dof_heave * DOF_MAG);
  _cpkt[MFC_PIROLL]  = (int)(dof_roll * DOF_MAG);
  _cpkt[MFC_PISWAY]  = (int)(dof_sway * DOF_MAG);
  _cpkt[MFC_PIYAW]   = (int)(dof_yaw * DOF_MAG);
  _cpkt[MFC_PITLOSS] = (int)(dof_tloss * DOF_MAG);
  //
  if (_learn)
  {
    char _newrange = 0;
    for (int i = MFC_PITDAT + 1; i < MFC_PKTSIZE; ++i)
    {
      if (_cpkt[i] < mpkt[i])
      {
        mpkt[i] = _cpkt[i];
        //_newrange = 1;
        printf ("\n#E:%d# [%d.. %d ..%d]", i, mpkt[i], _cpkt[i], Mpkt[i]);
      }
      if (_cpkt[i] > Mpkt[i])
      {
        Mpkt[i] = _cpkt[i];
        //_newrange = 1;
        printf ("\n#E:%d# [%d.. %d ..%d]", i, mpkt[i], _cpkt[i], Mpkt[i]);
      }
    }
    if (_newrange)
      for (int i = MFC_PITDAT + 1; i < MFC_PKTSIZE; ++i)
        printf ("\n#E:%d# [%d.. %d ..%d]", i, mpkt[i], _cpkt[i], Mpkt[i]);
  }
  //pitch
  _cpkt[MFC_PIPITCH] = 0;//get_cmap_f (_cpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  _cpkt[MFC_PISURGE] = get_cmap_f (_cpkt[MFC_PISURGE], mpkt[MFC_PISURGE], Mpkt[MFC_PISURGE], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  _cpkt[MFC_PIHEAVE] = 0;//get_cmap_f(_cpkt[MFC_PIHEAVE], mpkt[MFC_PIHEAVE], Mpkt[MFC_PIHEAVE], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  //roll
  _cpkt[MFC_PIROLL]  = 0;//get_cmap_f (_cpkt[MFC_PIROLL],  mpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  _cpkt[MFC_PISWAY]  = get_cmap_f (_cpkt[MFC_PISWAY],  mpkt[MFC_PISWAY],  Mpkt[MFC_PISWAY], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  //yaw
  _cpkt[MFC_PIYAW]   = get_cmap_f (_cpkt[MFC_PIYAW],   mpkt[MFC_PIYAW],   Mpkt[MFC_PIYAW], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  _cpkt[MFC_PITLOSS] = 0;//get_cmap_f (_cpkt[MFC_PITLOSS], mpkt[MFC_PITLOSS], Mpkt[MFC_PITLOSS], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  //adjust %
  //_cpkt[MFC_PIPITCH] = (float)_cpkt[MFC_PIPITCH] * _pitchprc;
  _cpkt[MFC_PISURGE] = (float)_cpkt[MFC_PISURGE] * _surgeprc;
  //
  //_cpkt[MFC_PIROLL]  = (float)_cpkt[MFC_PIROLL]  * _rollprc;
  _cpkt[MFC_PISWAY]  = (float)_cpkt[MFC_PISWAY]  * _swayprc;
  //
  _cpkt[MFC_PIYAW]   = (float)_cpkt[MFC_PIYAW] * _yawprc;
  //_cpkt[MFC_PITLOSS] = (float)_cpkt[MFC_PITLOSS] * _trlossprc;
  if (1 || _odbg)
    printf ("\n#i@%.3f.t3 pitch%% %d %d %d roll%% %d %d yaw%% %d %d", cltime,
      _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE],
      _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], 
      _cpkt[MFC_PIYAW], _cpkt[MFC_PITLOSS]);
  //share motion data
  if(!_learn)
    mfc_bcast_send ();
  //send dash data even when learning
  if (_dashaddr)
  {
    memcpy(_dpkt, _cpkt, pktl);
    mfcdash_bcast_send ();
  }
  //printf ("\r\n");
  return 1;
}
/**
A. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral - add

B. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral (invert) - add
*/
int _lport = UDP_PORT;
char *pp_ps4 = NULL;//"192.168.0.44";
//char _pp_ps4[50];

static void usage(char *app)
{
  //printf("\n%s %s\n", app, MFC_VERSION);
  printf("\n\nUsage: %s -g <game-ip-address>\n", app);
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
    { "help",     no_argument,       0, '?' },
    { "learn",    no_argument,       0, 'l' },
    { "debug",    required_argument, 0, 'd' },
    { "roll",     required_argument, 0, 'r' },
    { "pitch",    required_argument, 0, 'p' },
    { "yaw",      required_argument, 0, 'y' },
    { "surge",    required_argument, 0, 's' },
    { "sway",     required_argument, 0, 'w' },
    { "heave",    required_argument, 0, 'h' },
    { "tyre-slip",required_argument, 0, 't' },
    { "use-dash", required_argument, 0, 'a' },
    { "listen-on",required_argument, 0, 'c' },
    { "game-ip",  required_argument, 0, 'g' },
    { 0, 0, 0, 0 }
  };

  while (1)
  {
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "r:p:y:s:w:h:t:a:c:d:g:lhV?", long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {
    case '?':
      usage (argv[0]);
      exit (0);
      break;
    case 'r': //roll
      _rollprc = atoi (optarg);
      break;
    case 'p': //pitch
      _pitchprc = atoi (optarg);
      break;
    case 'y': //yaw
      _yawprc = atoi (optarg);
      break;
    case 's': //surge
      _surgeprc = atoi (optarg);
      break;
    case 'w': //sway
      _swayprc = atoi (optarg);
      break;
    case 'h': //heave
      _heaveprc = atoi (optarg);
      break;
    case 't': //tyre slip/traction loss
      _trlossprc = atoi (optarg);
      break;
    case 'a': //use dash to forward data
      _dashaddr = optarg;
      break;
    case 'l': //learn Max/min values
      _learn++;
      memset (Mpkt, 0, MFC_PKTSIZE*sizeof(int));
      memset (mpkt, 0, MFC_PKTSIZE*sizeof(int));
      break;
    case 'd': //debug
      _odbg++;
      break;
    case 'c': //listen port
      _lport = atoi (optarg);
      break;
    case 'g': //listen port
      pp_ps4 = optarg;
      break;
    default:
      printf("unrecognized option: %c\n", c);
    }
  }

  //configuration summary 
  printf ("\n# ##");
  printf ("\n#MFC Assetto Corsa client");
  printf ("\n#running configuration:");
  printf ("\n#  pitch feedback %d%% (-p %d)", (int)_pitchprc, (int)_pitchprc);
  printf ("\n#   roll feedback %d%% (-r %d)", (int)_rollprc, (int)_rollprc);
  printf ("\n#    yaw feedback %d%% (-y %d)", (int)_yawprc, (int)_yawprc);
  printf ("\n#  surge feedback %d%% (-s %d)", (int)_surgeprc, (int)_surgeprc);
  printf ("\n#   sway feedback %d%% (-w %d)", (int)_swayprc, (int)_swayprc);
  printf ("\n#  heave feedback %d%% (-h %d)", (int)_heaveprc, (int)_heaveprc);
  printf ("\n#   slip feedback %d%% (-t %d)", (int)_trlossprc, (int)_trlossprc);
  printf ("\n#   learning mode %s (-l)", _learn?"on":"off");
  printf ("\n#     server port %d", MFCSVR_PORT);
  printf ("\n#    game address %s (-g <ipv4>)", pp_ps4?pp_ps4:"<not-set>");
  printf ("\n# verbosity level %d (-d %d)", _odbg, _odbg);
  printf ("\n#     client port %d (-c %d)", _lport, _lport);
  if (_dashaddr)
    printf ("\n#            dash %s (-a %s)", _dashaddr, _dashaddr);
  else
    printf ("\n#            dash <not in use> (-a <ipv4>)");
  printf ("\n# ##");
  //
  return 1;
}


enum acstat {
    sthandshake = 0,
    stupdate = 1,
    stspot = 2,
    stdismiss = 3
};

//register for game data
int do_register(int s, struct sockaddr_in *si_other)
{
  char ac_pkt[1024];
  //handshake
  int bs = connect (s, (const struct sockaddr *)si_other, sizeof(struct sockaddr_in));
  if (bs < 0)
  {
    printf ("\n#E%d:connect %d (errno %d/%s) to game addr <%s>", s, bs, errno, strerror(errno), pp_ps4);
    //_done = 1;
    return 0;
  }
  else
    printf ("\n#i:%d<%dB connected to game addr <%s> (errno %d/%s)", s, bs, pp_ps4, errno, strerror(errno));
  int hshk[3] = {1, 1, sthandshake};
  bs = send (s, &hshk, 12, 0);
  if (bs < 0)
  {
    printf ("\n#E%d:send handshake %d (errno %d/%s)", s, bs, errno, strerror(errno));
    return 0;
  }
  else
    printf ("\n#i:%d<%dB sent for handshake", s, bs);
  //get handshake
  bs = recv (s, ac_pkt, 1024, 0);
  if (bs < 0)
  {
    printf ("\n#E%d:recv handshake %d (errno %d/%s)", s, bs, errno, strerror(errno));
    return 0;
  }
  else
    printf ("\n#i:%d<%dB recvd handshake", s, bs);
  //subscribe
  int sscr[3] = {1, 1, stupdate};
  bs = send (s, &sscr, 12, 0);
  if (bs < 0)
  {
    printf ("\n#E%d:subscribe %d (errno %d/%s)", s, bs, errno, strerror(errno));
    //_done = 1;
    return 0;
  }
  else
    printf ("\n#i:%d<%dB sent, subscribed", s, bs);
  return 1;
}
/**
A. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral - add

B. right axis composition
a. g-force - longitudinal/pitch - overwrite
b. g-force - lateral (invert) - add
*/
int main (int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int timeout, rc;
  //struct RTCarInfo *tpkt;
  //unsigned int gpio;

  env_init (argc, argv);
  if (pp_ps4 == NULL)
  {
    usage(argv[0]);
    exit (1);
  }
  if (0)
  {
    printf("\npkt data size %d\n", sizeof(struct RTCarInfo));
    exit (1);
  }
  //
  int cs = mfc_bcast_prep ("127.0.0.1", 0);
  if (cs < 3)
  {
    printf ("\n#e:can't connect to MFC server on port %d", MFCSVR_PORT);
    exit(1);
  }
  printf ("\n#i:<%d:MFC server on port %d", cs, MFCSVR_PORT);
  //
#define POLL_TIMEOUT 5000
  timeout = POLL_TIMEOUT;
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
  }

#if 0
  //uptime
  FILE* fp;
  double uptime, idle_time;
  /* Read the system uptime and accumulated idle time from /proc/uptime.  */
  fp = fopen ("/proc/uptime", "r");
#endif
  //sockets
  struct sockaddr_in si_other, si_me;
  int s;//, slen = sizeof(si_other);
  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#e:can't listen to game data on port %d, aborting", _lport);
    mfc_bcast_close ();
    mfcdash_bcast_close ();
    exit(1);
  }
  #if 0
  int broadcast = 0;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
  #endif
/* receiver */
  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(_lport);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (struct sockaddr*)&si_me, sizeof(si_me))==-1)
  {
    printf ("\n#e:can't listen to game data while binding to port %d, aborting", _lport);
    mfc_bcast_close ();
    mfcdash_bcast_close ();
    exit(1);
  }
  /* sender */
  memset ((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons (_lport);
  //TODO: add server IP param
  if (inet_aton (pp_ps4, &si_other.sin_addr) == 0)
  {
    fprintf (stderr, "inet_aton() for <%s> failed\n", pp_ps4);
    exit (1);
  }
  //
  do_register (s, &si_other);
  printf ("\n#i:>%d:listening on port %d", s, _lport);
  //setup internal timer ms
  ctime_ms (0);
  //learning values
  float cltime = -1.0f;
  //only send 3 PAUSEd packets
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  //
  int ppkt = 0;
  char packetBuffer[UDP_MAX_PACKETSIZE];
  //
  _cpkt = mfc_bcast_pktget ();
  _dpkt = mfcdash_bcast_pktget ();
  pktl = mfc_bcast_pktlen ();
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
    mfcdash_bcast_send ();
  }
  _cpkt[MFC_PITYPE] = PKTT_DATA;
  _cpkt[MFC_PITDAT]  = PKTT_2DOFN;
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

    rc = poll (fdset, nfds, timeout);      

    if (rc < 0) 
    {
      printf("\n#e:poll() failed!");
      _done = 1;
      break;
    }
        
    if (rc == 0) 
    {
      printf(".");
      fflush (stdout);
      sleep (1);
      do_register (s, &si_other);
    }
              
    if (fdset[0].revents & POLLIN)
    {
      int rlen = 0;
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      if ((rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL))==-1)
      {
        printf("\n#w:recvfrom() failed.");
      }
      else
      {
        cltime = (float)ctime_ms(0)/1000.0f;
        ppkt++;
        if ((ppkt % 500) == 0)
          printf ("\n#i:received %dpkts", ppkt);
        //
        if (1)
          printf("\r\n@%.3f received %dB packet (vs %d) from %s:%d <", 
                  cltime, rlen, UDP_MAX_PACKETSIZE, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        printf ("\n#i:pkt:%c / %d rpm %f gear %d spd %f %f %f", (char)get_int (packetBuffer, 0), get_int (packetBuffer, 4), 
          get_float (packetBuffer, 68), get_int (packetBuffer, 76), get_float (packetBuffer, 8), get_float (packetBuffer, 12), get_float (packetBuffer, 16));
        //printf ("\n#i:ffb:vert %f hori %f long%f", get_float (packetBuffer, 28), get_float (packetBuffer, 32), get_float (packetBuffer, 36));
        //
        mfc_packet_use (packetBuffer, cltime, cltime);
      }
    }
    fflush (stdout);
  }
  //
  printf("\n#i:cleaning up.. ");
  int diss[3] = {1, 1, stdismiss};
  int bs = send (s, &diss, 12, 0);
  if (bs < 0)
    printf ("\n#ERR:dismiss %d (errno %d/%s)", bs, errno, strerror(errno));
  else
    printf ("\n#i:%d<%dB sent, dismissed", s, bs);
  //
  close (s);
  mfc_bcast_close ();
  //
  //export learned values
  cltime = (float)ctime_ms(0)/1000.0f;
  if (_learn)
  {
    printf("\n#i:new min/Max values:");
    //min
    printf("\nstatic int mpkt[] = {");
    for (int i = 0; i < MFC_PKTSIZE; ++i)
    {
      if (i > 0)
        printf (", ");
      printf ("%d", mpkt[i]);
    }
    printf("};");
    //Max
    printf("\nstatic int Mpkt[] = {");
    for (int i = 0; i < MFC_PKTSIZE; ++i)
    {
      if (i > 0)
        printf (", ");
      printf ("%d", Mpkt[i]);
    }
    printf("};");
  }
  //
  printf("\n#i:done.\n");
  return 0;
}

#if 0
//from https://github.com/ottonello/AssettoCorsaTelemetry/blob/master/src/main/java/sample/corsa/TelemetryInterface.java
package sample.corsa;

import sample.StructWriter;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;

public class TelemetryInterface {
    DatagramSocket clientSocket;
    public static final int CORSA_PORT = 9996;
    private Status status = Status.init;
    InetAddress IPAddress;
    RTCarInfo telemetry;

    public void connect() {
        new Thread(new Runnable() {
            public void run() {
                try {
                    startHandshake();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }).start();

    }

    private void startHandshake() throws IOException {
        IPAddress = InetAddress.getByName("localhost");

        clientSocket = new DatagramSocket(9999);
        clientSocket.connect(IPAddress, CORSA_PORT);

        byte[] receiveData = new byte[1024];

        byte[] handShake = getHandshake();
        clientSocket.send(new DatagramPacket(handShake, handShake.length));

        DatagramPacket receivePacket = new DatagramPacket(receiveData, HandshakeResponse.SIZE);
        clientSocket.receive(receivePacket);

        HandshakeResponse handshakeResponse = new HandshakeResponse(receiveData);
        System.out.println("Handshake: " + handshakeResponse);

        status = Status.handshake;

        byte[] subscribe = getSubscribeUpdates();
        clientSocket.send(new DatagramPacket(subscribe, subscribe.length));

        System.out.println("Subscribed");
        status = Status.subscribed;

        while (!Status.dismissed.equals(status)) {
//            System.out.println("Receiving");

            DatagramPacket receivedLapData = new DatagramPacket(receiveData, 328);
            clientSocket.receive(receivedLapData);

            telemetry = new RTCarInfo(receiveData);
//            System.out.println("RECEIVED: " + telemetry);
        }
        System.out.println("Bye");

    }

    public RTCarInfo getTelemetry() {
        return telemetry;
    }

    private byte[] getHandshake() throws IOException {
        StructWriter structWriter = new StructWriter(12);
        structWriter.writeInt(1);
        structWriter.writeInt(1);
        structWriter.writeInt(OperationId.HANDSHAKE);
        return structWriter.toByteArray();
    }

    private byte[] getSubscribeUpdates() throws IOException {
        StructWriter structWriter = new StructWriter(12);
        structWriter.writeInt(1);
        structWriter.writeInt(1);
        structWriter.writeInt(OperationId.SUBSCRIBE_UPDATE);
        return structWriter.toByteArray();
    }

    private byte[] getSubscribeSpot() throws IOException {
        StructWriter structWriter = new StructWriter(12);
        structWriter.writeInt(1);
        structWriter.writeInt(1);
        structWriter.writeInt(OperationId.SUBSCRIBE_SPOT);
        return structWriter.toByteArray();
    }

    private byte[] getDismiss() throws IOException {
        StructWriter structWriter = new StructWriter(12);
        structWriter.writeInt(1);
        structWriter.writeInt(1);
        structWriter.writeInt(OperationId.DISMISS);
        return structWriter.toByteArray();
    }

    public void stop() throws IOException {
        System.out.println("Dismissing");

        byte[] dismiss = getDismiss();
        try {
            clientSocket.send(new DatagramPacket(dismiss, dismiss.length, IPAddress, CORSA_PORT));

        } catch (Exception e) {
            e.printStackTrace();
        }

        clientSocket.close();
        status = Status.dismissed;

    }
}
//
public enum Status {
    init,
    handshake,
    subscribed,
    dismissed
}
//
package sample.corsa;

import sample.StructReader;

import java.io.IOException;

public class HandshakeResponse {
    public static final int CAR_NAME_SIZE = 50;
    public static final int DRIVER_NAME_SIZE = 50;
    public static final int TRACK_NAME_SIZE = 50;
    public static final int TRACK_CONFIG_SIZE = 50;
    public static final int SIZE =
            CAR_NAME_SIZE *2 +
                    DRIVER_NAME_SIZE * 2 +
                    TRACK_NAME_SIZE * 2 +
                    TRACK_CONFIG_SIZE *2 +
                    4 +
                    4;

    String driverName;
    String carName;
    String trackName;
    String trackConfig;
    int identifier;
    int version;

    public HandshakeResponse(byte[] received) throws IOException {
        StructReader structReader = new StructReader(received);

        carName = structReader.readChars(CAR_NAME_SIZE);
        driverName =structReader.readChars(DRIVER_NAME_SIZE);
        identifier = structReader.readInt();
        version = structReader.readInt();
        trackName = structReader.readChars(TRACK_NAME_SIZE);
        trackConfig = structReader.readChars(TRACK_CONFIG_SIZE);
    }

    public String getDriverName() {
        return driverName;
    }

    public void setDriverName(String driverName) {
        this.driverName = driverName;
    }

    public String getCarName() {
        return carName;
    }

    public void setCarName(String carName) {
        this.carName = carName;
    }

    public String getTrackName() {
        return trackName;
    }

    public void setTrackName(String trackName) {
        this.trackName = trackName;
    }

    public String getTrackConfig() {
        return trackConfig;
    }

    public void setTrackConfig(String trackConfig) {
        this.trackConfig = trackConfig;
    }

    public int getIdentifier() {
        return identifier;
    }

    public void setIdentifier(int identifier) {
        this.identifier = identifier;
    }

    public int getVersion() {
        return version;
    }

    public void setVersion(int version) {
        this.version = version;
    }

    @Override
    public String toString() {
        return "HandshakeResponse{" +
                "carName='" + carName + '\'' +
                ", driverName='" + driverName + '\'' +
                ", identifier=" + identifier +
                ", version=" + version +
                ", trackName='" + trackName + '\'' +
                ", trackConfig='" + trackConfig + '\'' +
                '}';
    }
}
//
package sample.corsa;

public interface OperationId {
    public int HANDSHAKE = 0;
    public int SUBSCRIBE_UPDATE =1;
    public int SUBSCRIBE_SPOT = 2;
    public int DISMISS=3;
}
//
package sample.corsa;

import sample.StructReader;

import java.io.IOException;
import java.util.Arrays;

/**
 * Created by Marcos on 8/15/2015.
 */
public class RTCarInfo {

    int identifier;
    int size;

    float speed_Kmh;
    float speed_Mph;
    float speed_Ms;

    boolean isAbsEnabled;
    boolean isAbsInAction;
    boolean isTcInAction;
    boolean isTcEnabled;
    boolean isInPit;
    boolean isEngineLimiterOn;

    float accG_vertical;
    float accG_horizontal;
    float accG_frontal;

    int lapTime;
    int lastLap;
    int bestLap;
    int lapCount;

    float gas;
    float brake;
    float clutch;
    float engineRPM;
    float steer;
    int gear;
    float cgHeight;

    float wheelAngularSpeed[];
    float slipAngle[];
    float slipAngle_ContactPatch[];
    float slipRatio[];
    float tyreSlip[];
    float ndSlip[];
    float load[];
    float Dy[];
    float Mz[];
    float tyreDirtyLevel[];

    float camberRAD[];
    float tyreRadius[];
    float tyreLoadedRadius[];
    float suspensionHeight[];
    float carPositionNormalized;
    float carSlope;
    float carCoordinates[];

    public RTCarInfo(byte[] received) throws IOException {
        StructReader structReader = new StructReader(received);
        identifier = structReader.readInt();
        size = structReader.readInt();
        speed_Kmh = structReader.readFloat();
        speed_Mph = structReader.readFloat();
        speed_Ms = structReader.readFloat();
        isAbsEnabled = structReader.readBool();
        isAbsInAction = structReader.readBool();
        isTcInAction = structReader.readBool();
        isTcEnabled = structReader.readBool();
        isInPit = structReader.readBool();
        isEngineLimiterOn = structReader.readBool();

        accG_vertical = structReader.readFloat();
        accG_horizontal = structReader.readFloat();
        accG_frontal = structReader.readFloat();

        lapTime = structReader.readInt();
        lastLap = structReader.readInt();
        bestLap = structReader.readInt();
        lapCount = structReader.readInt();

        gas = structReader.readFloat();
        brake = structReader.readFloat();
        clutch = structReader.readFloat();
        engineRPM = structReader.readFloat();
        steer = structReader.readFloat();
        gear = structReader.readInt();
        cgHeight = structReader.readFloat();
        wheelAngularSpeed = structReader.readFloats(4);
        slipAngle = structReader.readFloats(4);
        slipAngle_ContactPatch = structReader.readFloats(4);
        slipRatio = structReader.readFloats(4);
        tyreSlip = structReader.readFloats(4);
        ndSlip = structReader.readFloats(4);
        load = structReader.readFloats(4);
        Dy = structReader.readFloats(4);
        Mz = structReader.readFloats(4);
        tyreDirtyLevel = structReader.readFloats(4);

        camberRAD = structReader.readFloats(4);
        tyreRadius = structReader.readFloats(4);
        tyreLoadedRadius = structReader.readFloats(4);
        suspensionHeight = structReader.readFloats(4);
        carPositionNormalized = structReader.readFloat();
        carSlope = structReader.readFloat();

        carCoordinates = structReader.readFloats(3);
    }

    public int getIdentifier() {
        return identifier;
    }

    public int getSize() {
        return size;
    }

    public float getSpeed_Kmh() {
        return speed_Kmh;
    }

    public float getSpeed_Mph() {
        return speed_Mph;
    }

    public float getSpeed_Ms() {
        return speed_Ms;
    }

    public boolean isAbsEnabled() {
        return isAbsEnabled;
    }

    public boolean isAbsInAction() {
        return isAbsInAction;
    }

    public boolean isTcInAction() {
        return isTcInAction;
    }

    public boolean isTcEnabled() {
        return isTcEnabled;
    }

    public boolean isInPit() {
        return isInPit;
    }

    public boolean isEngineLimiterOn() {
        return isEngineLimiterOn;
    }

    public float getAccG_vertical() {
        return accG_vertical;
    }

    public float getAccG_horizontal() {
        return accG_horizontal;
    }

    public float getAccG_frontal() {
        return accG_frontal;
    }

    public int getLapTime() {
        return lapTime;
    }

    public int getLastLap() {
        return lastLap;
    }

    public int getBestLap() {
        return bestLap;
    }

    public int getLapCount() {
        return lapCount;
    }

    public float getGas() {
        return gas;
    }

    public float getBrake() {
        return brake;
    }

    public float getClutch() {
        return clutch;
    }

    public float getEngineRPM() {
        return engineRPM;
    }

    @Override
    public String toString() {
        return "RTCarInfo{" +
                "identifier=" + identifier +
                ", size=" + size +
                ", speed_Kmh=" + speed_Kmh +
                ", speed_Mph=" + speed_Mph +
                ", speed_Ms=" + speed_Ms +
                ", isAbsEnabled=" + isAbsEnabled +
                ", isAbsInAction=" + isAbsInAction +
                ", isTcInAction=" + isTcInAction +
                ", isTcEnabled=" + isTcEnabled +
                ", isInPit=" + isInPit +
                ", isEngineLimiterOn=" + isEngineLimiterOn +
                ", accG_vertical=" + accG_vertical +
                ", accG_horizontal=" + accG_horizontal +
                ", accG_frontal=" + accG_frontal +
                ", lapTime=" + lapTime +
                ", lastLap=" + lastLap +
                ", bestLap=" + bestLap +
                ", lapCount=" + lapCount +
                ", gas=" + gas +
                ", brake=" + brake +
                ", clutch=" + clutch +
                ", engineRPM=" + engineRPM +
                ", steer=" + steer +
                ", gear=" + gear +
                ", cgHeight=" + cgHeight +
                ", wheelAngularSpeed=" + Arrays.toString(wheelAngularSpeed) +
                ", slipAngle=" + Arrays.toString(slipAngle) +
                ", slipAngle_ContactPatch=" + Arrays.toString(slipAngle_ContactPatch) +
                ", slipRatio=" + Arrays.toString(slipRatio) +
                ", tyreSlip=" + Arrays.toString(tyreSlip) +
                ", ndSlip=" + Arrays.toString(ndSlip) +
                ", load=" + Arrays.toString(load) +
                ", Dy=" + Arrays.toString(Dy) +
                ", Mz=" + Arrays.toString(Mz) +
                ", tyreDirtyLevel=" + Arrays.toString(tyreDirtyLevel) +
                ", camberRAD=" + Arrays.toString(camberRAD) +
                ", tyreRadius=" + Arrays.toString(tyreRadius) +
                ", tyreLoadedRadius=" + Arrays.toString(tyreLoadedRadius) +
                ", suspensionHeight=" + Arrays.toString(suspensionHeight) +
                ", carPositionNormalized=" + carPositionNormalized +
                ", carSlope=" + carSlope +
                ", carCoordinates=" + Arrays.toString(carCoordinates) +
                '}';
    }
}
//
package sample.corsa;

import sample.StructReader;

import java.io.IOException;
import java.util.Arrays;

/**
 * Created by Marcos on 8/15/2015.
 */
public class RTLap {
    int carIdentifierNumber;
    int lap;
    String driverName;
    String carName;
    int time;

    public RTLap(byte[] received) throws IOException {
        StructReader structReader = new StructReader(received);
        carIdentifierNumber = structReader.readInt();
        lap = structReader.readInt();
        driverName = structReader.readChars(50);
        carName = structReader.readChars(50);
        time = structReader.readInt();
    }

    public int getCarIdentifierNumber() {
        return carIdentifierNumber;
    }

    public int getLap() {
        return lap;
    }

    public String getDriverName() {
        return driverName;
    }

    public String getCarName() {
        return carName;
    }

    public int getTime() {
        return time;
    }

    @Override
    public String toString() {
        return "RTLap{" +
                "carIdentifierNumber=" + carIdentifierNumber +
                ", lap=" + lap +
                ", driverName='" + driverName + '\'' +
                ", carName='" + carName + '\'' +
                ", time=" + time +
                '}';
    }
}
//
#endif

#if 0
int main_dr2c (int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int timeout, rc;
  //unsigned int gpio;

  env_init (argc, argv);
  //
  int cs = mfc_bcast_prep ("127.0.0.1", 0);
  if (cs < 3)
  {
    printf ("\n#e:can't connect to MFC server on port %d", MFCSVR_PORT);
    exit(1);
  }
  printf ("\n#i:<%d:MFC server on port %d", cs, MFCSVR_PORT);
  //
#define POLL_TIMEOUT 5000
  timeout = POLL_TIMEOUT;
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
  }

#if 0
  //uptime
  FILE* fp;
  double uptime, idle_time;
  /* Read the system uptime and accumulated idle time from /proc/uptime.  */
  fp = fopen ("/proc/uptime", "r");
#endif
  //sockets
  struct sockaddr_in si_other, si_me;
  int s;//, slen = sizeof(si_other);
  int pktsz = -1;
  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#e:can't listen to game data on port %d, aborting", _lport);
    mfc_bcast_close ();
    mfcdash_bcast_close ();
    exit(1);
  }
  #if 0
  int broadcast = 1;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
  #endif
/* receiver */
  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = 0;//htons(_lport);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (struct sockaddr*)&si_me, sizeof(si_me))==-1)
  {
    printf ("\n#e:can't listen to game data while binding to port %d, aborting", _lport);
    mfc_bcast_close ();
    mfcdash_bcast_close ();
    exit(1);
  }
/* sender */
  memset((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(65001);
#if 0
  if (inet_aton("127.0.0.1", &si_other.sin_addr)==0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }
#endif
  //
  printf ("\n#i:>%d:listening on port %d", s, _lport);
  //setup internal timer ms
  ctime_ms (0);
  char fgamer = 1;  //game running or paused?
  //learning values
  float cltime, rtime;
  rtime = 0.0f;
  cltime = -1.0f;
  //only send 3 PAUSEd packets
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  //
  int ppkt = 0;
  char packetBuffer[UDP_MAX_PACKETSIZE];
  //
  _cpkt = mfc_bcast_pktget ();
  _dpkt = mfcdash_bcast_pktget ();
  pktl = mfc_bcast_pktlen ();
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
    snprintf((char *)(_dpkt + MFC_DIAPP14), MFC_DIAPPSZ, "A.Corsa");
    mfcdash_bcast_send ();
  }
  _cpkt[MFC_PITYPE] = PKTT_DATA;
  _cpkt[MFC_PITDAT]  = PKTT_2DOFN;
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

    rc = poll (fdset, nfds, timeout);

    if (rc < 0)
    {
      printf("\n#e:poll() failed!");
      _done = 1;
      break;
    }

    if (rc == 0)
    {
      printf(".");
      fflush (stdout);
      sleep (1);
    }

    if (fdset[0].revents & POLLIN)
    {
      int rlen = 0;//, idx;
      rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL);
      cltime = (float)ctime_ms(0)/1000.0f;
      if (rlen > 0)
      {
        char paused = (char)get_float (packetBuffer, 4 * 11); //custom paused/running
        if (pktsz == -1)
        {
          //first packet ever
          pktsz = rlen;
          printf ("\n#i@%f>%f:received pkt size %dbytes (%d fields)", cltime, rtime, pktsz, pktsz/4);
        }
        if (paused)
        {
          if (fgamer == 1)
          {
            fgamer = 0;
            printf ("\n#i@%f:game paused", cltime);
          }
        }
        else
        {
          if (fgamer == 0)
          {
            fgamer = 1;
            printf ("\n#i@%f:game running", cltime);
          }
          mfc_packet_use (packetBuffer, rtime, cltime);
          //
          ppkt++;
          if ((ppkt % 500) == 0)
            printf ("\n#i@%f>%f:received %dpkts", cltime, cltime, ppkt);
          //
          if (0)
            printf("\r\n@%f>%f:received %dB packet (vs %d) from %s:%d <",
                    cltime, rtime, rlen, UDP_MAX_PACKETSIZE, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        }
      } //rlen > 0
      if (rlen == -1)
      {
        printf("\n#w@%f:recvfrom() failed.", cltime);
      }
    }
    fflush (stdout);
  }
  //
  cltime = (float)ctime_ms(0)/1000.0f;
  if (_learn)
  {
    printf("\n#i:new min/Max values:");
    //min
    printf("\nstatic int mpkt[] = {");
    for (int i = 0; i < MFC_PKTSIZE; ++i)
    {
      if (i > 0)
        printf (", ");
      printf ("%d", mpkt[i]);
    }
    printf("};");
    //Max
    printf("\nstatic int Mpkt[] = {");
    for (int i = 0; i < MFC_PKTSIZE; ++i)
    {
      if (i > 0)
        printf (", ");
      printf ("%d", Mpkt[i]);
    }
    printf("};");
  }
  //
  printf("\n#i@%f:cleaning up.. done.\n", cltime);
  //
  close (s);
  mfc_bcast_close ();
  //
  return 0;
}
#endif
