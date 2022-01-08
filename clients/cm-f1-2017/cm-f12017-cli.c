/*
* codemasters dirt rally 2
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

#if 0
Offset	http://forums.codemasters.com/discussion/53139/f1-2017-d-box-and-udp-output-specification/p1	
0	float m_time;	
4	float m_laptime;	
8	float m_lapDistance;	
12	float m_totalDistance;	
16	float m_x;	// World space position
20	float m_y;	// World space position
24	float m_z;	// World space position
28	float m_speed;	// Speed of car in MPH
32	float m_xv;	// Velocity in world space
36	float m_yv;	// Velocity in world space
40	float m_zv;	// Velocity in world space
44	float m_xr;	// World space right direction
48	float m_yr;	// World space right direction
52	float m_zr;	// World space right direction
56	float m_xd;	// World space forward direction
60	float m_yd;	// World space forward direction
64	float m_zd;	// World space forward direction
68	float m_susp_pos[4];	// Note: All wheel arrays have the order:
84	float m_susp_vel[4];	// RL, RR, FL, FR
100	float m_wheel_speed[4];	
116	float m_throttle;	
120	float m_steer;	
124	float m_brake;	
128	float m_clutch;	
132	float m_gear;	
136	float m_gforce_lat;	
140	float m_gforce_lon;	
144	float m_lap;	
148	float m_engineRate;	
152	float m_sli_pro_native_support;	// SLI Pro support
156	float m_car_position;	// car race position
160	float m_kers_level;	// kers energy left
164	float m_kers_max_level;	// kers maximum energy
168	float m_drs;	// 0 = off, 1 = on
172	float m_traction_control;	// 0 (off) - 2 (high)
176	float m_anti_lock_brakes;	// 0 (off) - 1 (on)
180	float m_fuel_in_tank;	// current fuel mass
184	float m_fuel_capacity;	// fuel capacity
188	float m_in_pits;	// 0 = none, 1 = pitting, 2 = in pit area
192	float m_sector;	// 0 = sector1, 1 = sector2, 2 = sector3
196	float m_sector1_time;	// time of sector1 (or 0)
200	float m_sector2_time;	// time of sector2 (or 0)
204	float m_brakes_temp[4];	// brakes temperature (centigrade)
220	float m_tyres_pressure[4];	// tyres pressure PSI
236	float m_team_info;	// team ID
240	float m_total_laps;	// total number of laps in this race
244	float m_track_size;	// track size meters
248	float m_last_lap_time;	// last lap time
252	float m_max_rpm;	// cars max RPM, at which point the rev limiter will kick in
256	float m_idle_rpm;	// cars idle RPM
260	float m_max_gears;	// maximum number of gears
264	float m_sessionType;	// 0 = unknown, 1 = practice, 2 = qualifying, 3 = race
268	float m_drsAllowed;	// 0 = not allowed, 1 = allowed, -1 = invalid / unknown
272	float m_track_number;	// -1 for unknown, 0-21 for tracks
276	float m_vehicleFIAFlags;	// -1 = invalid/unknown, 0 = none, 1 = green, 2 = blue, 3 = yellow, 4 = red
280	float m_era;	// era, 2017 (modern) or 1980 (classic)
284	float m_engine_temperature;	// engine temperature (centigrade)
288	float m_gforce_vert;	// vertical g-force component
292	float m_ang_vel_x;	// angular velocity x-component
296	float m_ang_vel_y;	// angular velocity y-component
300	float m_ang_vel_z;	// angular velocity z-component
304	byte  m_tyres_temperature[4];	// tyres temperature (centigrade)
308	byte  m_tyres_wear[4];	// tyre wear percentage
312	byte  m_tyre_compound;	// compound of tyre – 0 = ultra soft, 1 = super soft, 2 = soft, 3 = medium, 4 = hard, 5 = inter, 6 = wet
313	byte  m_front_brake_bias;         // front brake bias (percentage)	
314	byte  m_fuel_mix;                 // fuel mix - 0 = lean, 1 = standard, 2 = rich, 3 = max	
315	byte  m_currentLapInvalid;	// current lap invalid - 0 = valid, 1 = invalid
316	byte  m_tyres_damage[4];	// tyre damage (percentage)
320	byte  m_front_left_wing_damage;	// front left wing damage (percentage)
321	byte  m_front_right_wing_damage;	// front right wing damage (percentage)
322	byte  m_rear_wing_damage;	// rear wing damage (percentage)
323	byte  m_engine_damage;	// engine damage (percentage)
324	byte  m_gear_box_damage;	// gear box damage (percentage)
325	byte  m_exhaust_damage;	// exhaust damage (percentage)
326	byte  m_pit_limiter_status;	// pit limiter status – 0 = off, 1 = on
327	byte  m_pit_speed_limit;	// pit speed limit in mph
328	float m_session_time_left;  // NEW: time left in session in seconds	
332	byte  m_rev_lights_percent;  // NEW: rev lights indicator (percentage)	
333	byte  m_is_spectating;  // NEW: whether the player is spectating	
334	byte  m_spectator_car_index;  // NEW: index of the car being spectated	
	// Car data	
335	byte  m_num_cars;	// number of cars in data
336	byte  m_player_car_index;	// index of player's car in the array
337	CarUDPData  m_car_data[20];   // data for all cars on track	900
1237	float m_yaw;  // NEW (v1.8)	
1241	float m_pitch;  // NEW (v1.8)	
1245	float m_roll;  // NEW (v1.8)	
1249	float m_x_local_velocity;          // NEW (v1.8) Velocity in local space	
1253	float m_y_local_velocity;          // NEW (v1.8) Velocity in local space	
1257	float m_z_local_velocity;          // NEW (v1.8) Velocity in local space	
1261	float m_susp_acceleration[4];   // NEW (v1.8) RL, RR, FL, FR	
1277	float m_ang_acc_x;                 // NEW (v1.8) angular acceleration x-component	
1281	float m_ang_acc_y;                 // NEW (v1.8) angular acceleration y-component	
1285	float m_ang_acc_z;                 // NEW (v1.8) angular acceleration z-component	
1289	};
#endif

#define UDP_MAX_PACKETSIZE  2048
#define UDP_PORT            20777

#define DEBUG 0
#define debug_print(fmt, ...) \
            do { if (DEBUG) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

int inet_aton(const char *cp, struct in_addr *inp);
//int usleep(long usec);

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
float _rollprc  = 20.0f;
float _yawprc   = 60.0f;
float _surgeprc = 30.0f;
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
float _surgeprc = 100.0f;
float _swayprc  = -100.0f;
float _heaveprc = -100.0f;
float _trlossprc= 100.0f;
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
*/
static int mpkt[] = {0, 0, -55789, -17792, -25177, -39887, -27095, -1793308, -83005, 0, 0, 0};
static int Mpkt[] = {0, 0, 71807, 16820, 10602, 20307, 12984, 1795649, 94049, 100, 100, 100};

int mfc_packet_use(char* packetBuffer, float rtime, float cltime)
{
  #define DOF_MAG (10000)
  /* uses 10000 magnitude to not lose much fidelity on computation
  //movement
136	float m_gforce_lat;	
140	float m_gforce_lon;	
288	float m_gforce_vert;	// vertical g-force component
1237	float m_yaw;  // NEW (v1.8)	
1241	float m_pitch;  // NEW (v1.8)	
1245	float m_roll;  // NEW (v1.8)	
1249	float m_x_local_velocity;          // NEW (v1.8) Velocity in local space	
1253	float m_y_local_velocity;          // NEW (v1.8) Velocity in local space	
1257	float m_z_local_velocity;          // NEW (v1.8) Velocity in local space	
  */
  static float dof_roll, dof_pitch, dof_yaw, dof_heave, dof_sway, dof_surge, dof_tloss;
  //
  dof_roll  = get_float (packetBuffer, 1245);  //roll.ori2
  dof_pitch = get_float (packetBuffer, 1241);  //pitch.ori1
  dof_heave = get_float (packetBuffer,  288);  //heave
  dof_yaw   = get_float (packetBuffer, 1237);  //yaw.ori0
  dof_sway  = get_float (packetBuffer, 136);  //sway
  dof_surge = get_float (packetBuffer, 140);  //surge
  dof_tloss = get_float (packetBuffer, 1249);  //traction loss computation
  if (_odbg)
    printf ("\n#i@%.3f:t1 pitch %f %f %f roll %f %f yaw %f %f", cltime,
      dof_pitch, dof_surge, dof_heave, dof_roll, dof_sway, dof_yaw, dof_tloss);
  dof_roll  = dof_roll * RAD2DEG;
  dof_pitch = dof_pitch * RAD2DEG;
  dof_yaw   = dof_yaw * RAD2DEG;
  //dof_heave = dof_heave;
  //dof_tloss = dof_tloss;// / GRAVACCEL;
  if (_odbg)
    printf ("\n#i@%.3f:t2 pitch %f %f %f roll %f %f yaw %f %f", cltime, 
      dof_pitch, dof_surge, dof_heave, dof_roll, dof_sway, dof_yaw, dof_tloss);
/**
Pitch is the tilt of the car forwards or backwards in [°]
Roll is how much the car is dipped to the left or right in [°]
Yaw is the heading of the car (north, east, south, west) in [°]

Surge means the acceleration of the car in longitudinal direction (front/back) [g]
Sway means the acceleration of the car in lateral direction (left/right) [g]
Heave means the acceleration up and down [g]
 * Yaw, roll and pitch inputs from orientation values
 * Sway, Surge, and Heave inputs from local acceleration values
 * local velocity provides traction loss
--when we have roll, yaw, pitch
  Roll_Output = ((.sngOrientation2 * 180 / 3.14159) - 90)
  Pitch_Output = (.sngOrientation1 * 180 / 3.14159) * -1
  Heave_Output = (System.Math.Cos(.sngOrientation2) * .sngAcceleration2)
  Yaw_Output = (.sngOrientation0 * 180 / 3.14159)
  Sway_Output = ((System.Math.Cos(.sngOrientation0) * .sngAcceleration0) + (System.Math.Sin(.sngOrientation0) * .sngAcceleration1))
  Surge_Output = ((-System.Math.Sin(.sngOrientation0) * .sngAcceleration0) + (System.Math.Cos(.sngOrientation0) * .sngAcceleration1))
  Extra1_Output = (((System.Math.Sin(.sngOrientation0) * .sngAcceleration0) + (System.Math.Sin(.sngOrientation0) * .sngAcceleration1)) * -1)
  -or-
  roll = (roll * 180 / 3.14)
  pitch = (pitch * 180 / 3.14) * -1
  yaw = (yaw * 180 / 3.14)
  heave = (cos(roll) * acceleration2)
  sway = ((cos(yaw) * acceleration0) + (sin(yaw) * acceleration1))
  surge = ((-sin(yaw) * acceleration0) + (cos(yaw) * acceleration1))
  traction_loss = (((sin(yaw) * acceleration0) + (sin(yaw) * acceleration1)) * -1)
--
  Dim RadtoDeg As Double = 45.0 / Atan(1)
  Dim pitch As Single = -CSng(Atan2(-SimTele.mOriY.z, Sqrt((-SimTele.mOriX.z * -SimTele.mOriX.z) + (-SimTele.mOriZ.z * -SimTele.mOriZ.z)))) * radsToDeg
  Dim yaw As Single = -CSng(Atan2(SimTele.mOriX.z, SimTele.mOriZ.z)) * radsToDeg
  Dim roll As Single = CSng(Atan2(SimTele.mOriY.x, Sqrt((SimTele.mOriX.x * SimTele.mOriX.x) + (SimTele.mOriZ.x * SimTele.mOriZ.x)))) * radsToDeg
  Dim speed As Single = CSng(Sqrt(SimTele.mLocalVel.x * SimTele.mLocalVel.x + SimTele.mLocalVel.y * SimTele.mLocalVel.y + SimTele.mLocalVel.z * SimTele.mLocalVel.z))
  Roll_MemMap = roll
  Pitch_MemMap = pitch
  Heave_MemMap = SimTele.mLocalAccel.y
  Yaw_MemMap = yaw
  Sway_MemMap = -SimTele.mLocalAccel.x
  Surge_MemMap = -SimTele.mLocalAccel.z
 * 
A. right axis composition
60p surge + 60p sway - 60p pitch - 60p roll - 20p heave
//a. g-force - longitudinal(pitch) - overwrite
//b. g-force - lateral(roll) - add

B. right axis composition
60p surge - 60p sway - 60p pitch + 60p roll - 20p heave
//a. g-force - longitudinal(pitch) - overwrite
//b. g-force - lateral(roll) * invert - add

C. traction loss
- 60p extra1 (local velocity x)
*/
/*
  int pkt_type;
  int pkt_dof_type;
  int data[6];    //{pitch, roll, yaw, surge, sway, heave}
  int speed;
  //
Pitch is the tilt of the car forwards or backwards in [°]
Roll is how much the car is dipped to the left or right in [°]
Yaw is the heading of the car (north, east, south, west) in [°]

Surge means the acceleration of the car in longitudinal direction (front/back) [g]
Sway means the acceleration of the car in lateral direction (left/right) [g]
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
pkt[8] = (int)lrv;  //speed
*/
  // convert radian values to degrees
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
  _cpkt[MFC_PIPITCH] = get_cmap_f (_cpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PISURGE] = get_cmap_f (_cpkt[MFC_PISURGE], mpkt[MFC_PISURGE], Mpkt[MFC_PISURGE], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PIHEAVE] = get_cmap_f (_cpkt[MFC_PIHEAVE], mpkt[MFC_PIHEAVE], Mpkt[MFC_PIHEAVE], MFC_POS_MIN, MFC_POS_MAX);
  //roll
  _cpkt[MFC_PIROLL]  = get_cmap_f (_cpkt[MFC_PIROLL],  mpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PISWAY]  = get_cmap_f (_cpkt[MFC_PISWAY],  mpkt[MFC_PISWAY],  Mpkt[MFC_PISWAY], MFC_POS_MIN, MFC_POS_MAX);
  //yaw
  _cpkt[MFC_PIYAW]   = get_cmap_f (_cpkt[MFC_PIYAW],   mpkt[MFC_PIYAW],   Mpkt[MFC_PIYAW], MFC_POS_MIN, MFC_POS_MAX);
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
  _cpkt[MFC_PITLOSS] = (float)_cpkt[MFC_PITLOSS] * _trlossprc;
  if (1 && _odbg)
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
    /*
    //gauges
    4	float m_laptime;
    16	float m_x;	// World space position
    20	float m_y;	// World space position
    24	float m_z;	// World space position
    28	float m_speed;	// Speed of car in MPH
    132	float m_gear;	
    148	float m_engineRate;	
    252	float m_max_rpm;	// cars max RPM, at which point the rev limiter will kick in
    256	float m_idle_rpm;	// cars idle RPM
    260	float m_max_gears;	// maximum number of gears
    284	float m_engine_temperature;	// engine temperature (centigrade)
    */
   _dpkt[MFC_DIPOSX]  = (int)get_float (packetBuffer, 16);
   _dpkt[MFC_DIPOSY]  = (int)get_float (packetBuffer, 20);
   _dpkt[MFC_DIPOSZ]  = (int)get_float (packetBuffer, 24);
   _dpkt[MFC_DIGEAR]  = (int)get_float (packetBuffer, 132);
   _dpkt[MFC_DIGEARM] = (int)get_float (packetBuffer, 260);
   _dpkt[MFC_DIRPM]   = (int)get_float (packetBuffer, 148);
   _dpkt[MFC_DIRPMM]  = (int)get_float (packetBuffer, 252);
   _dpkt[MFC_DIRPMI]  = (int)get_float (packetBuffer, 256);
   _dpkt[MFC_DITEMP]  = (int)get_float (packetBuffer, 284);
   _dpkt[MFC_DIRUNTS] = (int)get_float (packetBuffer, 4);
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
  printf ("\n#MFC F1 2017 client");
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

int main (int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int timeout, rc;

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
  int s, pktsz = 0;//slen = sizeof(si_other);
  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#e:can't listen to game data on port %d, aborting", _lport);
    mfc_bcast_close ();
    mfcdash_bcast_close ();
    exit(1);
  }
  int broadcast=1;
  setsockopt (s, SOL_SOCKET, SO_BROADCAST,
          &broadcast, sizeof (broadcast));
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
  float ltime, cltime, rtime;
  ltime = rtime = 0.0f;
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
    snprintf((char *)(_dpkt + MFC_DIAPP14), MFC_DIAPPSZ, "F1 2017");
    mfcdash_bcast_send ();
  }
  _cpkt[MFC_PITYPE] = PKTT_DATA;
  _cpkt[MFC_PITDAT]  = PKTT_2DOFN;
  //compute percentages here to avoid recompute
  _pitchprc  = _pitchprc / 100.0f;
  _surgeprc  = _surgeprc / 100.0f;
  _heaveprc  = _heaveprc / 100.0f;
  _rollprc   = _rollprc / 100.0f;
  _swayprc   = _swayprc / 100.0f;
  _yawprc    = _yawprc / 100.0f;
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
      int rlen = 0;
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL);
      cltime = (float)ctime_ms(0)/1000.0f;
      if (rlen > 0)
      {
        //pkt timestamps
        rtime = get_float (packetBuffer, 0);//run time
        ltime = get_float (packetBuffer, 4);//lap time - current
        if (pktsz == 0)
        {
          //first packet ever
          pktsz = rlen;
          printf ("\n#i@%.3f>%f:received pkt size %dbytes (%d fields)", cltime, rtime, pktsz, pktsz/sizeof(float));
        }
        //game paused or running
        if (rtime == ltime) //game paused
        {
          if (fgamer == 1)
          {
            fgamer = 0;
            printf ("\n#i@%.3f>%f:game paused", cltime, rtime);
            //process packet once only
            //mfc_packet_use (packetBuffer, rtime, cltime);
          }
        }
        else //game running
        {
          if (fgamer == 0)
          {
            fgamer = 1;
            printf ("\n#i@%.3f>%f:game running", cltime, rtime);
          }
          if (ltime == -1.0f)
          {
            //skip the first one
            ltime = rtime;
          }
          else
          {
            ltime = rtime;
            //process packet
            //mfc_packet_use (packetBuffer, rtime, cltime);
            //
            ppkt++;
            if ((ppkt % 500) == 0)
              printf ("\n#i@%f>%f:received %dpkts", cltime, rtime, ppkt);
            //
            if (0)
              printf("\r\n@%f>%f:received %dB packet (vs %d) from %s:%d <",
                      cltime, rtime, rlen, UDP_MAX_PACKETSIZE, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
          }
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
  printf("\n#i@%f:cleaning up.. done.\n", cltime);
  //
  close (s);
  mfc_bcast_close ();
  mfcdash_bcast_close ();
  //
  return 0;
}
