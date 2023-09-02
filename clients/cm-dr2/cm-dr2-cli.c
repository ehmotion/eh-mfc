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
/*
Pitch is the tilt of the car forwards or backwards in [°]
Roll is how much the car is dipped to the left or right in [°]
Yaw is the heading of the car (north, east, south, west) in [°]

Surge means the acceleration of the car in longitudinal direction (front/back) [g]
Sway means the acceleration of the car in lateral direction (left/right) [g]
Heave means the acceleration up and down [g]


class model from: https://github.com/lmirel/dr2_logger/blob/master/source/dirt_rally/udp_data.py
class Fields(Enum):
    run_time =            0
    lap_time =            1
    distance =            2
    progress =            3
    pos_x =               4
    pos_y =               5
    pos_z =               6
    speed_ms =            7
    vel_x =               8
    vel_y =               9
    vel_z =               10
    roll_x =              11
    roll_y =              12
    roll_z =              13
    pitch_x =             14
    pitch_y =             15
    pitch_z =             16
    susp_rl =             17
    susp_rr =             18
    susp_fl =             19
    susp_fr =             20
    susp_vel_rl =         21
    susp_vel_rr =         22
    susp_vel_fl =         23
    susp_vel_fr =         24
    wsp_rl =              25
    wsp_rr =              26
    wsp_fl =              27
    wsp_fr =              28
    throttle =            29
    steering =            30
    brakes =              31
    clutch =              32
    gear =                33
    g_force_lat =         34
    g_force_lon =         35
    current_lap =         36
    rpm =                 37  # / 10
    sli_pro_support =     38  # ignored
    car_pos =             39
    kers_level =          40  # ignored
    kers_max_level =      41  # ignored
    drs =                 42  # ignored
    traction_control =    43  # ignored
    anti_lock_brakes =    44  # ignored
    fuel_in_tank =        45  # ignored
    fuel_capacity =       46  # ignored
    in_pit =              47  # ignored
    sector =              48
    sector_1_time =       49
    sector_2_time =       50
    brakes_temp_rl =      51
    brakes_temp_rr =      52
    brakes_temp_fl =      53
    brakes_temp_fr =      54
    tyre_pressure_rl =    55  # ignored
    tyre_pressure_rr =    56  # ignored
    tyre_pressure_fl =    57  # ignored
    tyre_pressure_fr =    58  # ignored
    laps_completed =      59
    total_laps =          60
    track_length =        61
    last_lap_time =       62
    max_rpm =             63  # / 10
    idle_rpm =            64  # / 10
    max_gears =           65


num_fields = 66
*/


#define PACKED __attribute__((packed))

typedef struct PACKED {
  float     run_time ; //            0
  float     lap_time ; //            1
  float     distance ; //            2
  float     progress ; //            3
  float     pos_x ; //               4
  float     pos_y ; //               5
  float     pos_z ; //               6
  float     speed_ms ; //            7
  float     vel_x ; //               8
  float     vel_y ; //               9
  float     vel_z ; //               10
  float     roll_x ; //              11
  float     roll_y ; //              12
  float     roll_z ; //              13
  float     pitch_x ; //             14
  float     pitch_y ; //             15
  float     pitch_z ; //             16
  float     susp_rl ; //             17
  float     susp_rr ; //             18
  float     susp_fl ; //             19
  float     susp_fr ; //             20
  float     susp_vel_rl ; //         21
  float     susp_vel_rr ; //         22
  float     susp_vel_fl ; //         23
  float     susp_vel_fr ; //         24
  float     wsp_rl ; //              25
  float     wsp_rr ; //              26
  float     wsp_fl ; //              27
  float     wsp_fr ; //              28
  float     throttle ; //            29
  float     steering ; //            30
  float     brakes ; //              31
  float     clutch ; //              32
  float     gear ; //                33
  float     g_force_lat ; //         34
  float     g_force_lon ; //         35
  float     current_lap ; //         36
  float     rpm ; //                 37  # / 10
  float     sli_pro_support ; //     38  # ignored
  float     car_pos ; //             39
  float     kers_level ; //          40  # ignored
  float     kers_max_level ; //      41  # ignored
  float     drs ; //                 42  # ignored
  float     traction_control ; //    43  # ignored
  float     anti_lock_brakes ; //    44  # ignored
  float     fuel_in_tank ; //        45  # ignored
  float     fuel_capacity ; //       46  # ignored
  float     in_pit ; //              47  # ignored
  float     sector ; //              48
  float     sector_1_time ; //       49
  float     sector_2_time ; //       50
  float     brakes_temp_rl ; //      51
  float     brakes_temp_rr ; //      52
  float     brakes_temp_fl ; //      53
  float     brakes_temp_fr ; //      54
  float     tyre_pressure_rl ; //    55  # ignored
  float     tyre_pressure_rr ; //    56  # ignored
  float     tyre_pressure_fl ; //    57  # ignored
  float     tyre_pressure_fr ; //    58  # ignored
  float     laps_completed ; //      59
  float     total_laps ; //          60
  float     track_length ; //        61
  float     last_lap_time ; //       62
  float     max_rpm ; //             63  # / 10
  float     idle_rpm ; //            64  # / 10
  float     max_gears ; //           65
  //float num_fields ; // 66
} UDPPacket;


#if 0

struct UDPPacket
{
    float m_time;
    float m_lapTime;
    float m_lapDistance;
    float m_totalDistance;
    float m_x;      // World space position
    float m_y;      // World space position
    float m_z;      // World space position
    float m_speed;
    float m_xv;      // Velocity in world space
    float m_yv;      // Velocity in world space
    float m_zv;      // Velocity in world space
    float m_xr;      // World space right direction
    float m_yr;      // World space right direction
    float m_zr;      // World space right direction
    float m_xd;      // World space forward direction
    float m_yd;      // World space forward direction
    float m_zd;      // World space forward direction
    float m_susp_pos_bl;
    float m_susp_pos_br;
    float m_susp_pos_fl;
    float m_susp_pos_fr;
    float m_susp_vel_bl;
    float m_susp_vel_br;
    float m_susp_vel_fl;
    float m_susp_vel_fr;
    float m_wheel_speed_bl;
    float m_wheel_speed_br;
    float m_wheel_speed_fl;
    float m_wheel_speed_fr;
    float m_throttle;
    float m_steer;
    float m_brake;
    float m_clutch;
    float m_gear;
    float m_gforce_lat;   //sway
    float m_gforce_lon;   //surge
    float m_lap;
    float m_engineRate;
    float m_sli_pro_native_support; // SLI Pro support
    float m_car_position;   // car race position
    float m_kers_level;    // kers energy left
    float m_kers_max_level;   // kers maximum energy
    float m_drs;     // 0 = off, 1 = on
    float m_traction_control;  // 0 (off) - 2 (high)
    float m_anti_lock_brakes;  // 0 (off) - 1 (on)
    float m_fuel_in_tank;   // current fuel mass
    float m_fuel_capacity;   // fuel capacity
    float m_in_pits;    // 0 = none, 1 = pitting, 2 = in pit area
    float m_sector;     // 0 = sector1, 1 = sector2; 2 = sector3
    float m_sector1_time;   // time of sector1 (or 0)
    float m_sector2_time;   // time of sector2 (or 0)
    float m_brakes_temp[4];   // brakes temperature (centigrade)
    float m_wheels_pressure[4];  // wheels pressure PSI
    float m_team_info;    // team ID 
    float m_total_laps;    // total number of laps in this race
    float m_track_size;    // track size meters
    float m_last_lap_time;   // last lap time
    float m_max_rpm;    // cars max RPM, at which point the rev limiter will kick in
    float m_idle_rpm;    // cars idle RPM
    float m_max_gears;    // maximum number of gears
    float m_sessionType;   // 0 = unknown, 1 = practice, 2 = qualifying, 3 = race
    float m_drsAllowed;    // 0 = not allowed, 1 = allowed, -1 = invalid / unknown
    float m_track_number;   // -1 for unknown, 0-21 for tracks
    float m_vehicleFIAFlags;  // -1 = invalid/unknown, 0 = none, 1 = green, 2 = blue, 3 = yellow, 4 = red
 };
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
char *_bcastaddr = NULL;
int debug_data = 0;

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
float _pitchprc = 30.0f;
float _rollprc  = 20.0f;
float _yawprc   = 60.0f;
float _surgeprc = -30.0f;
float _swayprc  = -30.0f;
float _heaveprc = 30.0f;
float _trlossprc= 100.0f;
float _extra1prc= 100.0f;
float _extra2prc= 100.0f;
float _extra3prc= 100.0f;
#else
float _pitchprc = 50.0f;
float _rollprc  = 50.0f;
float _yawprc   = 0.0f;
float _surgeprc = -50.0f;
float _swayprc  = -50.0f;
float _heaveprc = 50.0f;
float _trlossprc= 50.0f;
float _extra1prc= 100.0f;
float _extra2prc= 100.0f;
float _extra3prc= 100.0f;
#endif
static int *_cpkt, *_dpkt, pktl;
char _learn = 0;

#define XX 0
#define YY 1
#define ZZ 2
void gforceFromVelocity(float *gforce, const float *velocity)
{
  static float lfv = 0.0f;
  static float llv = 0.0f;
  static float lvv = 0.0f;
  //heading in radians
  float hh = atan2f(velocity[XX], velocity[YY]);
  //forward velocity
  float fv = sin(hh) * velocity[XX] + cos(hh) * velocity[YY];
  //lateral velocity
  float lv = cos(hh) * velocity[XX] - sin(hh) * velocity[YY];
  //vertical velocity
  float vv = velocity[ZZ];
  //g forces
  gforce[XX] = ((fv - lfv) * 60.0f) * (1.0f/9.81f);
  gforce[YY] = ((lv - llv) * 60.0f) * (1.0f/9.81f);
  gforce[ZZ] = ((vv - lvv) * 60.0f) * (1.0f/9.81f);
  //save velocities
  lfv = fv; llv = lv; lvv = vv;
}

// https://gamedev.stackexchange.com/questions/172147/convert-3d-direction-vectors-to-yaw-pitch-roll-angles
#define PITCH 0 // up/down
#define YAW   1 // left/right
#define ROLL  2 // fall over
#define sign(x) ((x > 0) - (x < 0))
void anglesFromVectorsFwUp(float *angles, const float *forward, const float *up)
{
    // Yaw is the bearing of the forward vector's shadow in the xy plane.
    float yaw = atan2(forward[1], forward[0]);

    // Pitch is the altitude of the forward vector off the xy plane, toward the down direction.
    float pitch = -asin(forward[2]);

    // Find the vector in the xy plane 90 degrees to the right of our bearing.
    float planeRightX = sin(yaw);
    float planeRightY = -cos(yaw);

    // Roll is the rightward lean of our up vector, computed here using a dot product.
    float roll;// = asin(up[1]);
    if (1)
    {
      roll = asin(up[0]*planeRightX + up[1]*planeRightY);
      // If we're twisted upside-down, return a roll in the range +-(pi/2, pi)
      if(up[2] < 0)
          roll = sign(roll) * M_PI - roll;
    }
    // Convert radians to degrees.
    angles[YAW]   =   yaw * 180 / M_PI;
    angles[PITCH] = pitch * 180 / M_PI;
    angles[ROLL]  =  roll * 180 / M_PI;
}

//static int Mpkt[] = {0, 0, 11696, 130, 1108,    0,       191, 93943, 90000, 100, 100, 100};
//static int mpkt[] = {0, 0, -8958, -134, -3811, -101291, -176, -36455, -14839, 0, 0, 0};
//                          pit    srg    hev    rol    sway   yaw      tl
static int Mpkt[] = {0, 0, 10600,  132,   1340, 105000, 191, 93000, 90000, 100, 100, 100};
static int mpkt[] = {0, 0, -8000, -154,  -3800,  75000,-170,-36000, -90000, 0, 0, 0};

int mfc_packet_use(char* packetBuffer, float rtime, float cltime)
{
  #define DOF_MAG (1000.f)
  /* uses 10000 magnitude to not lose much fidelity on computation
                                
                                179.554748535156
  */
  //game telemetry data
  static float dof_roll, dof_pitch, dof_yaw, dof_heave, dof_sway, dof_surge, dof_tloss;
  UDPPacket *tpkt = (UDPPacket*)packetBuffer;
  //only when racing
  static float phv = 0.0f; //heave - for road detail
  static float pst = 0.0f; //front suspention travel - for road detail - not used
  static float prlv = 0.0f;
  static float pflv = 0.0f;
  if (tpkt->lap_time == 0.0f)
  {
    //reset previous values as we start new race
    pst = 0.0f;
    prlv = 0.0f;
    pflv = 0.0f;
  }
  //
  float angs[3];
  float fwd[3] = {tpkt->pitch_x, tpkt->pitch_z, tpkt->pitch_y};
  float rgt[3] = {tpkt->roll_x, tpkt->roll_z, tpkt->roll_y};
  anglesFromVectorsFwUp(angs, fwd, rgt);
  float gf[3];
  float vel[] = {tpkt->vel_x, tpkt->vel_z, tpkt->vel_y};
  gforceFromVelocity(gf, vel);
  //suspention travel for road texture
  float stdt = tpkt->susp_vel_fr + tpkt->susp_vel_fl - pst;
  pst = tpkt->susp_vel_fr + tpkt->susp_vel_fl;
  //yaw or traction loss: accel / pkts-per-sec + previous velocity
  float rlv = tpkt->g_force_lat / 60.0f + prlv; //getLVel4Accel(tpkt->m_gforce_lat, prlv);
  prlv = rlv;
  float flv = tpkt->g_force_lon / 60.0f + pflv;//getLVel4Accel(tpkt->m_gforce_lon, pflv);
  pflv = flv;
  //dofs
  dof_pitch = -angs[PITCH];//rpy[1] * RAD2DEG;//MATH_PI * tpkt->rx * RAD2DEG; //raw rotation rx is -1..1 and needs PI to transform to radians
  dof_surge = -tpkt->g_force_lon * (1.0f/9.81f);//gf[XX];//-G[1];//fa / GRAVACCEL; //from local acc to G
  dof_sway  = tpkt->g_force_lat * (1.0f/9.81f);//gf[YY] * 100000.0f;//G[0];//ra / GRAVACCEL; //from local acc to G
  dof_heave = -gf[ZZ];//G[2];//va / GRAVACCEL; //from local acc to G
  dof_roll  = -angs[ROLL];//stdt;//angs[ROLL];//-rpy[0] * RAD2DEG;//MATH_PI * tpkt->rz * RAD2DEG;
  dof_yaw   = angs[YAW];//(Vl[2]==0.0f)?0.0f:-(RAD2DEG * atanf(Vl[0] / abs(Vl[2])));//(MATH_PI * tpkt->ry) * RAD2DEG;
  if (tpkt->lap_time > 0.0f)
    dof_tloss = (flv==0.0f)?0.0f:-(RAD2DEG * atanf(rlv / abs(flv)));
  else
    dof_tloss = 0.0f;
  //
  if (0 || _odbg)
    printf ("\n#i@%.3f:tr %f fwd %f %f %f rgt %f %f %f ang %f %f %f", cltime, tpkt->lap_time,
      tpkt->pitch_x, tpkt->pitch_y, tpkt->pitch_z, tpkt->roll_x, tpkt->roll_y, tpkt->roll_z, angs[ROLL], angs[PITCH], angs[YAW]);
  if (0 || _odbg)
    printf ("\n#i@%.3f:tr %f vel %f %f %f gf %f %f %f", cltime, tpkt->lap_time,
      tpkt->vel_x, tpkt->vel_y, tpkt->vel_z, gf[ROLL], gf[PITCH], gf[YAW]);
  if (0 || _odbg)
    printf ("\n#i@%.3f:t0 %f pitch %f %f %f roll %f %f yaw %f %f", cltime, tpkt->lap_time,
      dof_pitch, dof_surge, dof_heave, dof_roll, dof_sway, dof_yaw, dof_tloss);
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
  if (0 || _odbg)
    printf ("\n#i@%.3f.t1 pitch%% %5d %5d %5d roll%% %5d %5d yaw%% %5d %5d", cltime,
      _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE],
      _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], 
      _cpkt[MFC_PIYAW], _cpkt[MFC_PITLOSS]);
  //pitch
  _cpkt[MFC_PIPITCH] = get_cmap_f (_cpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  _cpkt[MFC_PISURGE] = get_cmap_f (_cpkt[MFC_PISURGE], mpkt[MFC_PISURGE], Mpkt[MFC_PISURGE], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  _cpkt[MFC_PIHEAVE] = get_cmap_f(_cpkt[MFC_PIHEAVE], mpkt[MFC_PIHEAVE], Mpkt[MFC_PIHEAVE], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  //roll
  _cpkt[MFC_PIROLL]  = get_cmap_f (_cpkt[MFC_PIROLL],  mpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  _cpkt[MFC_PISWAY]  = get_cmap_f (_cpkt[MFC_PISWAY],  mpkt[MFC_PISWAY],  Mpkt[MFC_PISWAY], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  //yaw
  _cpkt[MFC_PIYAW]   = get_cmap_f (_cpkt[MFC_PIYAW],   mpkt[MFC_PIYAW],   Mpkt[MFC_PIYAW], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  _cpkt[MFC_PITLOSS] = get_cmap_f (_cpkt[MFC_PITLOSS], mpkt[MFC_PITLOSS], Mpkt[MFC_PITLOSS], -MFC_HPOS_MAX, MFC_HPOS_MAX);
  if (0 || _odbg)
    printf ("\n#i@%.3f.t2 pitch%% %d %d %d roll%% %d %d yaw%% %d %d", cltime,
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
  _cpkt[MFC_PITLOSS] = (float)_cpkt[MFC_PITLOSS] * _trlossprc;
  if (0 || _odbg)
    printf ("\n#i@%.3f.t3 pitch%% %d %d %d roll%% %d %d yaw%% %d %d", cltime,
      _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE],
      _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], 
      _cpkt[MFC_PIYAW], _cpkt[MFC_PITLOSS]);
  //share motion data
  if (!_learn)
    mfc_bcast_send ();
  //send dash data
  if (_dashaddr)
  {
    //dash data
    _dpkt[MFC_DISPD]  = (int)(tpkt->speed_ms * 2.24f);  //speed: 1 m/s, 3.60 kph - 1 m/s, 2.24 mph
    _dpkt[MFC_DIGEAR] = (int)tpkt->gear;  //gear
    _dpkt[MFC_DIRPM]  = (int)tpkt->rpm * 10;  //rpm x10
    _dpkt[MFC_DIRPMM] = (int)tpkt->max_rpm * 10 - 1000; //max rpm x10
    //printf ("\n#i@%.3f:d1 rpm %d rpmm %d rpmM %d", cltime, _dpkt[MFC_DIRPM], tpkt->mrpm, tpkt->Mrpm);
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

static void usage(char *app)
{
  printf("%s %s\n", app, MFC_VERSION);
  printf ("--scn output processing protocol specific to SCN5/6 controllers\n");
  printf ("--arduino output processing protocol specific to Arduino controllers\n"\
      "\t it uses the command model 'XL<bin-left-pos>CXR<bin-right-pos>C\n");
  printf ("--kangaroo output processing protocol specific to Kangaroo controllers\n"\
      "\t it uses the command model 'L,P<left-pos> S<left-speed>' and 'R,P<right-pos> S<right-speed>'\n");
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
    { "debug-data",required_argument, 0, 'D' },
    { "motion-ip", required_argument, 0, 'm' },
    { 0, 0, 0, 0 }
  };

  while (1)
  {
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "r:p:y:s:w:h:t:a:c:d:D:m:lhV?", long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {
    case '?':
      usage (argv[0]);
      exit (0);
      break;
    case 'D': //roll
      debug_data = atoi (optarg);
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
    case 'm': //server address for telemetry data broadcast
      if(inet_addr(optarg) <= 0)
      {
        printf("\n#E:invalid server address %s", optarg);
        exit(-1);
      }
      _bcastaddr = optarg;
      break;
    default:
      printf("unrecognized option: %c\n", c);
    }
  }

  //configuration summary 
  printf ("\n# ##");
  printf ("\n#MFC Dirt Rally 2 client");
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
  printf ("\n#motion server IP %s (-m %s)", _bcastaddr?_bcastaddr:"127.0.0.1", _bcastaddr?_bcastaddr:"127.0.0.1");
  printf ("\n# ##");
  //printf ("\n-- sizeof float %d", sizeof(float));
  printf ("\n-- debug data %d", debug_data);
  //
  return 1;
}

int main (int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int gpio_fd, timeout, rc;
  unsigned int gpio;
  int len;

  env_init (argc, argv);
  //
  if (1)
  {
    printf("\n#i:pkt data size %d", sizeof(UDPPacket));
  }
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
  int s, i, slen = sizeof(si_other), pktk = 0, pktsz = 0;
  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#e:can't listen to game data on port %d, aborting", _lport);
    mfc_bcast_close ();
    mfcdash_bcast_close ();
    exit(1);
  }
  int broadcast=1;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST,
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
  pktk = 990;
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
  float lminl, lmaxl, lminr, lmaxr, ltime, cltime, rtime;
  lminl = lmaxl = lminr = lmaxr = ltime = rtime = 0.0f;
  ltime = cltime = -1.0f;
  //only send 3 PAUSEd packets
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  //
  int ppkt = 0;
  long lts = ctime_ms (0);
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
    snprintf((char *)(_dpkt + MFC_DIAPP14), MFC_DIAPPSZ, "CM-DR2");
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
      lminl = lmaxl = lminr = lmaxr = 0.0f;
      printf(".");
      fflush (stdout);
      sleep (1);
    }

    if (fdset[0].revents & POLLIN)
    {
      int rlen = 0, idx;
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL);
      cltime = get_fms();
      if (rlen > 0)
      {
        //pkt timestamps
        rtime = get_float (packetBuffer, 0);//run time
        //ltime = get_float (packetBuffer, 4);//lap time - current
        if (ltime == -1.0f)
        {
          //first packet ever
          pktsz = rlen;
          printf ("\n#i@%.3f>%f:received pkt size %dbytes (%d fields)", cltime, rtime, pktsz, pktsz/4);
        }
        //process packet
        mfc_packet_use (packetBuffer, rtime, cltime);
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
    //Max
    printf("\nstatic int Mpkt[] = {");
    for (int i = 0; i < MFC_PKTSIZE; ++i)
    {
      if (i > 0)
        printf (", ");
      printf ("%d", Mpkt[i]);
    }
    printf("};");
    //min
    printf("\nstatic int mpkt[] = {");
    for (int i = 0; i < MFC_PKTSIZE; ++i)
    {
      if (i > 0)
        printf (", ");
      printf ("%d", mpkt[i]);
    }
    printf("};");
  }
  cltime = (float)ctime_ms(0)/1000.0f;
  printf("\n#i@%f:cleaning up.. done.\n", cltime);
  //
  close (s);
  mfc_bcast_close ();
  //
  return 0;
}
