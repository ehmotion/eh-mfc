/*
* pCars dashboard test

gcc -o /opt/mfcc-pcars2 clients/pcars/pcars-dash.c -lrt -std=c11
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
//#include <api_client.h>

#include "SMS_UDP_Definitions.hpp"

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

int inet_aton(const char *cp, struct in_addr *inp);
int usleep(long usec);

//get number of millis from app start or first call
//first call will return 0
int ctime_ms(char val)
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
int mfc_pkt[MFC_PKTSIZE] = {0};
int mfcdash_pkt[MFCDASH_PKTSIZE] = {0};

//-

char _done = 0;
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
#if 1
float _pitchprc = -30.0f;
float _rollprc  = 20.0f;
float _yawprc   = 60.0f;
float _surgeprc = -30.0f;
float _swayprc  = -30.0f;
float _heaveprc = -30.0f;
float _trlossprc= 100.0f;
float _extra1prc= 100.0f;
float _extra2prc= 100.0f;
float _extra3prc= 100.0f;
#else
float _pitchprc = -100.0f;
float _rollprc  = 100.0f;
float _yawprc   = 100.0f;
float _surgeprc = -100.0f;
float _swayprc  = -100.0f;
float _heaveprc = -100.0f;
float _trlossprc= 100.0f;
float _extra1prc= 100.0f;
float _extra2prc= 100.0f;
float _extra3prc= 100.0f;
#endif
//
char _odbg = 0;

char *_dashaddr = NULL;
int _lport = SMS_UDP_PORT; /* Project Cars sends to this port: 5605 */
char _learn = 0;

//int Mpkt[MFC_PKT_SIZE] = {1, 1,   50483,   43371,   1795547,   11451,   14847,   5426, 1};
// *!!!* careful with indexes as they need to correspond to the data pkt order
//                               //pitch  //surge  //heave  //roll   //sway   //yaw     //trloss
static int Mpkt[MFC_PKTSIZE] = {1, 1,   50483,   11451,   5426,    43371,   14847,   1795547,  4430, 0, 0, 0}; //MAX values
static int mpkt[MFC_PKTSIZE] = {1, 1,  -50483,  -11451,  -5426,   -43371,  -14847,  -1795547, -4430, 0, 0, 0}; //min values

static void usage(char *app)
{
  printf("%s %s\n", app, MFC_VERSION);
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
    { 0, 0, 0, 0 }
  };

  while (1)
  {
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "r:p:y:s:w:h:t:a:c:d:lhV?", long_options, &option_index);

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
    default:
      printf("unrecognized option: %c\n", c);
      break;
    }
  }

  //configuration summary 
  printf ("\n# ##");
  printf ("\n#MFC PCARS2 client");
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

int main(int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int timeout, rc;
  //
  env_init (argc, argv);

  //
  int cs = mfc_bcast_prep ("127.0.0.1", 0);
  if (cs < 3)
  {
    printf ("\n#e:can't connect to MFC server on port %d", MFCSVR_PORT);
    exit(1);
  }
  //printf ("\n--size of int %d\n", sizeof(int));
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
  int s;
  unsigned int slen = sizeof (si_other);
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
  {
    printf ("\n#e:can't listen to game data on port %d, aborting", _lport);
    mfc_bcast_close ();
    mfcdash_bcast_close ();
    exit(1);
  }
  int broadcast = 1;
  setsockopt (s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
/* receiver */
  memset((char *) &si_me, 0, sizeof (si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons (_lport);
  si_me.sin_addr.s_addr = htonl (INADDR_ANY);
  if (bind (s, (struct sockaddr*)&si_me, sizeof (si_me))==-1)
  {
    printf ("\n#e:can't listen to game data while binding to port %d, aborting", _lport);
    mfc_bcast_close ();
    mfcdash_bcast_close ();
    exit (1);
  }
  // sender storage - not really used
  memset ((char *) &si_other, 0, sizeof (si_other));

  //printf ("\n#i:>%d:listening on port %d", s, _lport);
  //ctime_ms (1);
  //learning values
  float fv[MFCDASH_PKTSIZE];
  //only send 3 PAUSEd packets
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  //
  int rlen = 0;
  int ppkt = 0;
  float lts = (float)ctime_ms (0)/1000.0f;
  char packetBuffer[SMS_UDP_MAX_PACKETSIZE];
  PacketBase packetHeader;
  memset(&packetHeader, 0, sizeof( PacketBase ));
  sParticipantsData pData;
  sParticipantsData pData2;
  sParticipantVehicleNamesData pVehicles;
  sParticipantVehicleNamesData pVehicles2;
  sVehicleClassNamesData pClasses;
  sGameStateData		stateData;
  sTelemetryData  sTelem;

  memset(&pData, 0, sizeof( sParticipantsData ));
  memset(&pData2, 0, sizeof( sParticipantsData ));
  memset(&stateData, 0, sizeof( sGameStateData ));
  memset(&pVehicles, 0, sizeof( sParticipantVehicleNamesData ));
  memset(&pVehicles2, 0, sizeof( sParticipantVehicleNamesData ));
  memset(&pClasses, 0, sizeof( sVehicleClassNamesData ));
  memset(&sTelem, 0, sizeof( sTelemetryData ));
  //int wpkt[MFC_PKT_SIZE] = {0};
  //local math vars
  static float ori0 = 0, ori1 = 0, ori2 = 0;
  static float acc0 = 0, acc1 = 0, acc2 = 0;
  static float vel0 = 0, vel1 = 0, vel2 = 0;
  //printf("\n#i.max axis at 60%%: %.3f", get_cmap_f(-137, -1795547, 1795547, -6000, 6000));
  #define DOF_MAG (10000)
  /* uses 10000 magnitude to not lose much fidelity on computation
                                
                                179.554748535156
  */
  //
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
    snprintf((char *)(_dpkt + MFC_DIAPP14), MFC_DIAPPSZ, "P.Cars 2");
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
      usleep (1000000);
    }
              
    if (fdset[0].revents & POLLIN)
    {
      //char participantsReceived = 0;
      //char participantsReceived2 = 0;
      //char stateReceived = 0;
      //char vehiclesReceived = 0;
      //char vehiclesReceived2 = 0;
      //char telemReceived = 0;
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      {
        printf("\n#w:recvfrom() failed.");
      }
      else
      {
        ++ppkt;
        lts = (float)ctime_ms (0)/1000.0f;
        if (0)
          printf("\r\n#i@%.3f.received %dB packet (vs %d) from %s:%d <", 
                  lts, rlen, SMS_UDP_MAX_PACKETSIZE, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        memcpy(&packetHeader, packetBuffer, sizeof( PacketBase ));
        //
        if ((ppkt % 500) == 0)
          printf ("\n#i@%.3f.received %dpkts", lts, ppkt);
        //printf ("\n#pkt type %d", packetHeader.mPacketType);
        switch ( packetHeader.mPacketType )
        {
            case eCarPhysics :
            {
              //float							sSuspensionTravel[4];							// 312 16
              //~0.080 still
              //- 0 front LT:
              //- 1 front RT:
              //- 2  back LT:
              //- 3  back RT:
              //float							sOrientation[3];									// 52 12
              //float							sAngularVelocity[3];							// 88 12
              //- 1  roll: +LT
              //float							sLocalVelocity[3];								// 64 12
              //- 0  roll: +LT,  -RT
              //- 2 pitch: +ACC, -BRK
              //float							sLocalAcceleration[3];						// 100 12
              // tenths
              //- 0  roll: +LT,  -RT
              //- 2 pitch: -ACC, +BRK
              //#i:telem  -7.79411411     -1.03792381     -9.79688072     0.00000000
              //#i:telem  -6.75643492     -1.22993863     -10.73717594    0.00000000
              //#i:telem  +12.45860386    +0.02436667     +3.99445176     0.00000000
              //#i:telem  +14.37643242    +0.14723888     +4.68823385     0.00000000
              //#i:telem  +12.37826157    -0.06973051     +7.29721689     0.00000000
              //float							sSpeed;														// 36 4
#if 1
/*
Pitch is the tilt of the car forwards or backwards in [°]
Roll is how much the car is dipped to the left or right in [°]
Yaw is the heading of the car (north, east, south, west) in [°]

Surge means the acceleration of the car in longitudinal direction [g]
Sway means the acceleration of the car in lateral direction [g]
Heave means the acceleration up and down [g]
			float							sOrientation[3];									// 52 12
			float							sLocalVelocity[3];								// 64 12
			float							sWorldVelocity[3];								// 76 12
			float							sAngularVelocity[3];							// 88 12
			float							sLocalAcceleration[3];						// 100 12
			float							sWorldAcceleration[3];						// 112 12
* Yaw, roll and pitch inputs from the Orientation values
* Sway, Surge, and Heave inputs we get from the Local Acceleration values
* Traction loss happens when the car is not pointing in the direction of travel.
Most of the time it takes math to get this answer.
But with PCars, it provides this info in one of the the LocalVelocity outputs I believe
**Most likely what you will want to run the sim with is Sway and Surge.
As Pitch and Roll will just tip the sim with the current angles of the track.
Where Sway and Surge will let you feel the gforces produced by the car.
**
PCars is one of the easiest: roll/pitch/yaw is the orientation vector and heave/sway/surge is the local acceleration vector.
--
example from LFS V3_Dash\AdditionPlugin\Plugin.vb: 186
With MyOutsim_Internal
    Roll_Output = (.sngOrientation2 * 180 / 3.14159)
    Pitch_Output = (.sngOrientation1 * 180 / 3.14159) * -1
    Heave_Output = (System.Math.Cos(.sngOrientation2) * .sngAcceleration2)
    Yaw_Output = (.sngOrientation0 * 180 / 3.14159)
    Sway_Output = ((System.Math.Cos(.sngOrientation0) * .sngAcceleration0) + (System.Math.Sin(.sngOrientation0) * .sngAcceleration1))
    Surge_Output = ((-System.Math.Sin(.sngOrientation0) * .sngAcceleration0) + (System.Math.Cos(.sngOrientation0) * .sngAcceleration1))
    Extra1_Output = (((System.Math.Sin(.sngOrientation0) * .sngAcceleration0) + (System.Math.Sin(.sngOrientation0) * .sngAcceleration1)) * -1)
End With
*/
  if (_cpkt)
  {
    //motion data packet
    /*
    PCars is one of the easiest.
    roll/pitch/yaw is the orientation vector and heave/sway/surge is the local acceleration vector
    */
    //motion
    #if 1
#define ORIENTATION_IDX 52
#define ORIENTATION_IDX0 56 //yaw   ori+4
#define ORIENTATION_IDX1 52 //pitch ori+0
#define ORIENTATION_IDX2 60 //roll  ori+8
#define LOCAL_ACCEL_IDX 100
#define LOCAL_ACCEL_IDX0 104
#define LOCAL_ACCEL_IDX1 100
#define LOCAL_ACCEL_IDX2 108
#define LOCAL_VELOC_IDX 64
#define LOCAL_VELOC_IDX0 64
#define LOCAL_VELOC_IDX1 68
#define LOCAL_VELOC_IDX2 72
    ori0 = get_float (packetBuffer, ORIENTATION_IDX0); //yaw
    ori1 = get_float (packetBuffer, ORIENTATION_IDX1); //pitch
    ori2 = get_float (packetBuffer, ORIENTATION_IDX2); //roll
    acc0 = get_float (packetBuffer, LOCAL_ACCEL_IDX0); //sway
    acc1 = get_float (packetBuffer, LOCAL_ACCEL_IDX1); //surge
    acc2 = get_float (packetBuffer, LOCAL_ACCEL_IDX2); //heave
    vel0 = get_float (packetBuffer, LOCAL_VELOC_IDX0);
    vel1 = get_float (packetBuffer, LOCAL_VELOC_IDX1);
    vel2 = get_float (packetBuffer, LOCAL_VELOC_IDX2);
    if (_odbg)
      printf ("\n#i@%.3f:t1 %f %f %f %f %f %f %f %f %f", lts,
        ori0, ori1, ori2, acc0, acc1, acc2, vel0, vel1, vel2);
    /*
    dof_heave = cos(ori2) * acc2;
    dof_sway  = cos(ori0) * acc0 + sin(ori0) * acc1;
    dof_surge = -sin(ori0) * acc0 + cos(ori0) * acc1;
    dof_tloss = sin(ori0) * acc0 + sin(ori0) * acc1;
    */
    // convert radian values to degrees
    fv[MFC_PIPITCH] = ori1 * RAD2DEG;
    _cpkt[MFC_PIPITCH]  = (int)(fv[MFC_PIPITCH] * DOF_MAG);
    //_cpkt[MFC_PIPITCH] = -get_cmap (pf_pitch, -100, 100, -1000, 1000);
    //
    // formula based, but wrong? -> fv[MFC_PISURGE] = (-sin(ori2) * acc0 + cos(ori2) * acc2) / 10.f;//acc1 * RAD2DEG;
    fv[MFC_PISURGE] = (cos(ori2) * acc2 + sin(ori2) * acc2) / GRAVACCEL;
    _cpkt[MFC_PISURGE]  = (int)(fv[MFC_PISURGE] * DOF_MAG);
    //_cpkt[MFC_PISURGE] = -get_cmap (pw_pitch, -32800, 32800, -7000, 7000);
    //
    // formula based, wrong -> fv[MFC_PIHEAVE] = (cos(ori2) * acc0) / 10.f;//acc2 * RAD2DEG;
    fv[MFC_PIHEAVE] = (cos(ori2) * acc0 + sin(ori2) * acc0) / GRAVACCEL;
    _cpkt[MFC_PIHEAVE]  = (int)(fv[MFC_PIHEAVE] * DOF_MAG);
    //_cpkt[MFC_PIHEAVE] = -get_cmap (pv_pitch, -128, 128, -2000, 2000);
    //
    fv[MFC_PIROLL] = ori2 * RAD2DEG;
    _cpkt[MFC_PIROLL]  = (int)(fv[MFC_PIROLL] * DOF_MAG);
    //_cpkt[MFC_PIROLL]  = get_cmap (pf_roll, -128, 128, -7000, 7000);
    //
    //-formula based -> fv[MFC_PISWAY] = (cos(ori1) * acc1 + sin(ori1) * acc0) / 10.f;//acc0 * RAD2DEG;
    fv[MFC_PISWAY] = (cos(ori2) * acc1 + sin(ori2) * acc1) / GRAVACCEL;
    _cpkt[MFC_PISWAY]  = (int)(fv[MFC_PISWAY] * DOF_MAG);
    //_cpkt[MFC_PISWAY]  = get_cmap (get_float (packetBuffer, local_accel_idx)*100, 8200, 8200, -3000, 3000);
    //
    fv[MFC_PIYAW] = ori0 * RAD2DEG;
    _cpkt[MFC_PIYAW]  = (int)(fv[MFC_PIYAW] * DOF_MAG);
    //_cpkt[MFC_PIYAW]   = get_cmap (pw_roll, -16400, 16400, -10000, 10000);
    //
    //fv[MFC_PITLOSS] = (sin(ori1) * acc1 + sin(ori1) * acc0);
    fv[MFC_PITLOSS] = vel0 / GRAVACCEL;
    //fv[MFC_PITLOSS] = (sin(ori0) * acc1) / 10.f;
    _cpkt[MFC_PITLOSS] = (int)(fv[MFC_PITLOSS] * DOF_MAG);
    //
    if (0)
      printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f",
        fv[MFC_PIPITCH]>=0.0f?" +":" ", fv[MFC_PIPITCH], fv[MFC_PISURGE]>=0.0f?" +":" ", fv[MFC_PISURGE],
        fv[MFC_PIHEAVE]>=0.0f?" +":" ", fv[MFC_PIHEAVE], fv[MFC_PIROLL]>=0.0f?" +":" ", fv[MFC_PIROLL], 
        fv[MFC_PISWAY]>=0.0f?" +":" ", fv[MFC_PISWAY], fv[MFC_PIYAW]>=0.0f?" +":" ", fv[MFC_PIYAW]);
    if (_odbg)
      printf ("\n#i@%.3f:t2 %f %f %f %f %f %f %f", lts,
        fv[MFC_PIPITCH], fv[MFC_PISURGE], fv[MFC_PIHEAVE], 
        fv[MFC_PIROLL], fv[MFC_PISWAY], 
        fv[MFC_PIYAW], fv[MFC_PITLOSS]);
    if (0 && _odbg)
      printf ("\n#i@%.3f:telem pitch %d %d %d roll %d %d yaw %d %d", lts,
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE], 
        _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], 
        _cpkt[MFC_PIYAW], _cpkt[MFC_PITLOSS]);
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
    _cpkt[MFC_PIPITCH] = get_cmap_f (_cpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], MFC_POS_MIN, MFC_POS_MAX);
    _cpkt[MFC_PISURGE] = get_cmap_f (_cpkt[MFC_PISURGE], mpkt[MFC_PISURGE], Mpkt[MFC_PISURGE], MFC_POS_MIN, MFC_POS_MAX);
    _cpkt[MFC_PIHEAVE] = get_cmap_f (_cpkt[MFC_PIHEAVE], mpkt[MFC_PIHEAVE], Mpkt[MFC_PIHEAVE], MFC_POS_MIN, MFC_POS_MAX);
    //roll
    _cpkt[MFC_PIROLL]  = get_cmap_f (_cpkt[MFC_PIROLL],  mpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL], MFC_POS_MIN, MFC_POS_MAX);
    _cpkt[MFC_PISWAY]  = get_cmap_f (_cpkt[MFC_PISWAY],  mpkt[MFC_PISWAY],  Mpkt[MFC_PISWAY], MFC_POS_MIN, MFC_POS_MAX);
    //yaw
    _cpkt[MFC_PIYAW]   = get_cmap_f (_cpkt[MFC_PIYAW],   mpkt[MFC_PIYAW],   Mpkt[MFC_PIYAW], MFC_POS_MIN, MFC_POS_MAX);
    //traction loss
    _cpkt[MFC_PITLOSS] = get_cmap_f (_cpkt[MFC_PITLOSS],   mpkt[MFC_PITLOSS],   Mpkt[MFC_PITLOSS], MFC_POS_MIN, MFC_POS_MAX);
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
    _cpkt[MFC_PIYAW]   = (float)_cpkt[MFC_PIYAW]   * _yawprc;
    _cpkt[MFC_PITLOSS] = (float)_cpkt[MFC_PITLOSS] * _trlossprc;
    
    if (1 && _odbg)
      printf ("\n#i@%.3f.t3 pitch%% %d %d %d roll%% %d %d yaw%% %d %d", lts,
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE],
        _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], 
        _cpkt[MFC_PIYAW], _cpkt[MFC_PITLOSS]);
    #else
    _cpkt[MFC_PIPITCH] = -get_cmap (pf_pitch, 0, 100, -500, 500);
    _cpkt[MFC_PISURGE] = -get_cmap (pw_pitch, -32800, 32800, -3500, 3500);
    _cpkt[MFC_PIHEAVE] = -get_cmap (pv_pitch, -125, 125, -1000, 1000);
    //
    _cpkt[MFC_PIROLL]  = get_cmap (pf_roll, -pf_roll_max, pf_roll_max, -3500, 3500);
    _cpkt[MFC_PISWAY]  = get_cmap (pw_roll, -16400, 16400, -1500, 1500);
    //
    _cpkt[MFC_PIYAW]   = 0;//get_cmap (pw_roll, -16400, 16400, -10000, 10000);
    #endif
    //
    mfc_bcast_send ();
    //send dash data
    if (_dashaddr)
    {
      memcpy(_dpkt, _cpkt, pktl);
      //dash data
      _dpkt[MFC_DITEMP]  = (int)get_short (packetBuffer, 22); //water temperature
      _dpkt[MFC_DIFUEL]  = (int)get_float (packetBuffer, 32); //fuel
      _dpkt[MFC_DISPD]  = (int)get_float (packetBuffer, 36);  //speed
      /*
      if you print it in HEX you'll see the system.
      eg. 6 gear
      R = 111 = 6F
      N = 96 = 60
      1 = 97 = 61
      2 = 98 = 66
      3 = 99 = 63
      4 = 100 = 64
      5 = 101 = 65
      6 = 102 = 66
      eg. 9 gear
      R = 159 = 9F
      N = 144 = 90
      1 = 145 = 91
      2 = 146 = 92
      3 = 147 = 93
      4 = 148 = 94
      5 = 149 = 95
      6 = 150 = 96
      7 = 151 = 97
      8 = 152 = 98
      9 = 153 = 99
      */
      _dpkt[MFC_DIGEARM] = (int)((packetBuffer[45]>>4) & 0x0f); //max gears
      _dpkt[MFC_DIGEAR] = (int)(packetBuffer[45] & 0x0f);     //gear
      if (_dpkt[MFC_DIGEAR] == 0x0f)
        _dpkt[MFC_DIGEAR] = -1;
      _dpkt[MFC_DIRPM]  = (int)get_short (packetBuffer, 40);  //rpm
      _dpkt[MFC_DIRPMM] = (int)get_short (packetBuffer, 42);  //max rpm
      //printf ("\n#i@%.3f:d1 gear %d", 0.0f, _dpkt[MFC_DIGEAR]);
      mfcdash_bcast_send ();
    }
  }
/**
              //float							sLocalAcceleration[3];						// 100 12
              // tenths
              //- 0  roll: +LT,  -RT
              //- 2 pitch: -ACC, +BRK

              float roll_out, pitch_out, heave_out, yaw_out, sway_out, surge_out;
              float ori0, ori1, ori2; //orientation/direction of travel
              float acc0, acc1, acc2; //acceleration of travel
              roll_out  = (((cos (ori0) * acc0) + (sin (ori0) * acc1)) * -1);
              pitch_out = (((-sin (ori0) * acc0) + (cos (ori0) * acc1)) * -1);
              heave_out = (cos (ori2) * acc2);
              yaw_out   = (((sin (ori0) * acc0) + (sin (ori0) * acc1)) *-1);
              sway_out  = ((cos (ori0) * acc0) + (sin (ori0) * acc1));
              surge_out = ((-sin (ori0) * acc0) + (cos (ori0) * acc1));

              A. right axis composition
              a. g-force - longitudinal/pitch - overwrite
              b. g-force - lateral - add

              B. left axis composition
              a. g-force - longitudinal/pitch - overwrite
              b. g-force - lateral (invert) - add

*/
#if 0
              if (_odbg)
                printf ("\n#i:%04d:telem %s%.8f \t %s%.8f", 
                  ppkt, fv[2]>0?" +":" ", fv[2], fv[0]>0?" +":" ", fv[0]);
              //llv = fv[2] + fv[0]; //pitch + roll
              //lrv = fv[2] - fv[0]; //pitch - roll
              wpkt[MFC_PIPITCH] = (int)(fv[2] * 1000.0f);
              //printf (" \t %06d ", wpkt[MFC_PIPITCH]);
              //range: min .. Max
              if (wpkt[MFC_PIPITCH] < mpkt[MFC_PIPITCH])
                mpkt[MFC_PIPITCH] = wpkt[MFC_PIPITCH];
              if (wpkt[MFC_PIPITCH] > Mpkt[MFC_PIPITCH])
                Mpkt[MFC_PIPITCH] = wpkt[MFC_PIPITCH];
              wpkt[MFC_PIPITCH] = (int)get_map (wpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], -10000, 10000);
              //
              wpkt[MFC_PIROLL]  = (int)(fv[0] * 1000.0f);
              //printf (" \t %06d ", wpkt[MFC_PIROLL]);
              //range: min .. Max
              if (wpkt[MFC_PIROLL] < mpkt[MFC_PIROLL])
                mpkt[MFC_PIROLL] = wpkt[MFC_PIROLL];
              if (wpkt[MFC_PIROLL] > Mpkt[MFC_PIROLL])
                Mpkt[MFC_PIROLL] = wpkt[MFC_PIROLL];
              wpkt[MFC_PIROLL]  = (int)get_map (wpkt[MFC_PIROLL], mpkt[MFC_PIROLL], Mpkt[MFC_PIROLL], -10000, 10000);
              //
              if (_odbg)
                printf ("\n#i:%04d:Mtelem [%06d \t %06d \t %06d] \t [%06d \t %06d \t %06d]", 
                  ppkt, 
                  mpkt[MFC_PIPITCH], wpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH],
                  mpkt[MFC_PIROLL],  wpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL]);
#endif
#if 0
              //autoadjust min
              if (lminl > llv)
                lminl = llv;
              if (lmaxl < llv)
                lmaxl = llv;
              //autoadjust max
              if (lminr > lrv)
                lminr = lrv;
              if (lmaxr < lrv)
                lmaxr = lrv;
              //map min/max on actuator range
              llv = get_map_f (llv, lminl, lmaxl, -10000.0f, 0.0f);
              lrv = get_map_f (lrv, lminr, lmaxr, -10000.0f, 0.0f);
              //smooth curve
              int ld = 50;
              llv -= (int)llv%ld;
              lrv -= (int)lrv%ld;
              //
              if (_odbg)
                printf ("\n#i:%04d:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f", 
                  ppkt, fv[0]>0?" +":" ", fv[0], fv[2]>0?" +":" ", fv[2], llv>0?" +":" ", llv, lrv>0?" +":" ", lrv);
#endif
#else //use suspention travel ;)
              idx = 312;//float							sSuspensionTravel[4];							// 312 16
              fv[0] = get_float (packetBuffer, idx);
              //fv[1] = 0.0f;//get_float (packetBuffer, idx + 4);
              fv[1] = get_float (packetBuffer, idx + 4);
              fv[2] = 0.0f;//get_float (packetBuffer, idx + 8);
              //fv[2] = get_float (packetBuffer, idx + 8);
              fv[3] = 0.0f;//get_float (packetBuffer, idx + 12);
              //fv[3] = get_float (packetBuffer, idx + 12);
              //printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f", fv[0]>0?" +":" ", fv[0], fv[1]>0?" +":" ", fv[1], fv[2]>0?" +":" ", fv[2], fv[3]>0?" +":" ", fv[3]);
              //
              float llv = fv[0];
              float lrv = fv[1];
              llv -= 0.080f;
              lrv -= 0.080f;
              //
              llv *= 100000.0f;
              lrv *= 100000.0f;
              //
              llv *= (-1);
              lrv *= (-1);
              //autoadjust min
              if (lminl > llv)
                lminl = llv;
              if (lmaxl < llv)
                lmaxl = llv;
              //autoadjust max
              if (lminr > lrv)
                lminr = lrv;
              if (lmaxr < lrv)
                lmaxr = lrv;
              //map min/max on actuator range
              llv = get_map_f (llv, lminl, lmaxl, -10000.0f, 0.0f);
              lrv = get_map_f (lrv, lminr, lmaxr, -10000.0f, 0.0f);
              //smooth curve
              int ld = 10;
              llv -= (int)llv%ld;
              lrv -= (int)lrv%ld;
              //
              printf ("\n#i:telem %s%.8f \t %s%.8f \t %s%.8f \t %s%.8f", fv[0]>0?" +":" ", fv[0], fv[1]>0?" +":" ", fv[1], llv>0?" +":" ", llv, lrv>0?" +":" ", lrv);
#endif
              //
              break;
            }
            #if 0
            case eParticipants :
            if (packetHeader.mPartialPacketIndex == 1)
            {
              memcpy(&pData, packetBuffer, sizeof( sParticipantsData ));
            }
            if (packetHeader.mPartialPacketIndex == 2)
            {
              memcpy(&pData2, packetBuffer, sizeof( sParticipantsData ));
              participantsReceived2 = 1;
            }
            if (packetHeader.mPartialPacketIndex == packetHeader.mPartialPacketNumber)
            {
              participantsReceived = 1;
            }

            break;
          case eGameState :
            memcpy(&stateData, packetBuffer, sizeof( sGameStateData ));
            stateReceived = 1;
            break;
          case eParticipantVehicleNames :
          {
            //last packet are always the vehicle class names
            if (packetHeader.mPartialPacketIndex == packetHeader.mPartialPacketNumber)
            {
              memcpy(&pClasses, packetBuffer, sizeof( sVehicleClassNamesData ));
              vehiclesReceived = 1;
            }
            else
            {
              if (packetHeader.mPartialPacketIndex == 1)
              {
                memcpy(&pVehicles, packetBuffer, sizeof( sParticipantVehicleNamesData ));
              }
              if (packetHeader.mPartialPacketIndex == 2)
              {
                memcpy(&pVehicles2, packetBuffer, sizeof( sParticipantVehicleNamesData ));
              }
            }
          }
          #endif
          default: break;
        }
        //
        #if 0
        if (telemReceived)
        {
          //printf ("\n#i:telem RPM %d", ntohs(sTelem.sRpm));
        }
        //
        if (stateReceived)
        {
          int gameState = stateData.mGameState & 7;
          int sessionState = stateData.mGameState >> 4;

          printf(" Game State %i, gameState  %i, sessionState %i \n", stateData.mGameState, gameState, sessionState );
          printf(" Race Participants  \n");
          if (participantsReceived)
          {
            for (int i=0;i<PARTICIPANTS_PER_PACKET;++i)
            {
              if (pData.sName[i][0] != '\0')
              {
                printf(" Name %S \n",pData.sName[i]);
              }
            }
            if (participantsReceived2)
            {
              for (int i=0;i<PARTICIPANTS_PER_PACKET;++i)
              {
                if (pData2.sName[i][0] != '\0')
                {
                  printf(" Name %S \n",pData2.sName[i]);
                }
              }
            }
          }
          if (vehiclesReceived)
          {
            printf("Vehicle Names\n");
            for (int i=0;i<VEHICLES_PER_PACKET;++i)
            {
              if (pVehicles.sVehicles[i].sName[0] != '\0')
              {
                printf("Vehicle Name %S, index %d, class %d \n",pVehicles.sVehicles[i].sName[i],pVehicles.sVehicles[i].sIndex, pVehicles.sVehicles[i].sClass);
              }
            }
            if (vehiclesReceived2)
            {
              for (int i=0;i<VEHICLES_PER_PACKET;++i)
              {
                if (pVehicles2.sVehicles[i].sName[0] != '\0')
                {
                  printf("Vehicle Name %S, index %d, class %d \n",pVehicles2.sVehicles[i].sName[i],pVehicles2.sVehicles[i].sIndex, pVehicles2.sVehicles[i].sClass);
                }
              }
            }
            printf("Class Names\n");
            for (int i=0;i<CLASSES_SUPPORTED_PER_PACKET;++i)
            {
              if (pClasses.sClasses[i].sName[0] != '\0')
              {
                printf("Class Name %S, index %d`\n",pClasses.sClasses[i].sName,pClasses.sClasses[i].sClassIndex);
              }
            }
          }
        }
        #endif
        //printf ("\r\n");
      }
    }
    fflush (stdout);
  }
  //
  //export learned values
  lts = (float)ctime_ms(0)/1000.0f;
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
  printf("\n#i:cleaning up.. done.\n");
  //
  close (s);
  mfc_bcast_close ();
  mfcdash_bcast_close ();
  //
  return 0;
}
