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
#define UDP_PORT            20778

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
#if 1
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
#else
float _pitchprc = -100.0f;
float _rollprc  = -100.0f;
float _yawprc   = 100.0f;
float _surgeprc = 100.0f;
float _swayprc  = -100.0f;
float _heaveprc = 100.0f;
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
/*DR2*/
/*
static int mpkt[] = {0, 0, -55789, -17792, -25177, -39887, -27095, -1793308, -83005, 0, 0, 0};
static int Mpkt[] = {0, 0, 71807, 16820, 10602, 20307, 12984, 1795649, 94049, 100, 100, 100};
*/
//#i:new min/Max values GRID2019
static int mpkt[] = {0, 0, -66872, -20720, -10199, -56510, -20614, -1799075, -33074, 0, 0, 0};
static int Mpkt[] = {0, 0, 82035, 12073, 7435, 59804, 17895, 1799995, 64906, 100, 100, 100};

int mfc_packet_use(char* packetBuffer, float rtime, float cltime)
{
  //static float ori0 = 0, ori1 = 0, ori2 = 0;
  //static float acc0 = 0, acc1 = 0, acc2 = 0;
  //static float vel0 = 0, vel1 = 0, vel2 = 0;
  //printf("\n#i.max axis at 60%%: %.3f", get_cmap_f(-137, -1795547, 1795547, -6000, 6000));
  #define DOF_MAG (10000)
  /* uses 10000 magnitude to not lose much fidelity on computation
                                
                                179.554748535156
  */
  //int Mpkt[MFC_PKT_SIZE] = {1, 1,   50483,   43371,   1795547,   11451,   14847,   5426, 1};
  // *!!!* careful with indexes as they need to correspond to the data pkt order
  //                                     //pitch  //surge  //heave  //roll   //sway   //yaw     //trloss
  //static float fv[MFCDASH_PKTSIZE];
  //game telemetry data
  //static float posi[3], velo[3], roll[3], pitch[3], gforce[2];
  static float dof_roll, dof_pitch, dof_yaw, dof_heave, dof_sway, dof_surge, dof_tloss;
  //static float safr, safl, sabr, sabl;
  //
        /*
<float channel="roll" scale="1.0" />
<float channel="pitch" scale="1.0" />
<float channel="gforce_vertical" scale="1.0" />
<float channel="yaw" scale="1.0" />
<float channel="gforce_lateral" scale="1.0" />
<float channel="gforce_longitudinal" scale="1.0" />
<float channel="local_velocity_x" scale="1.0" />
<float channel="speed" scale="1.0" />
<float channel="gear" scale="1.0" />
<float channel="engine_rate" scale="1.0" />
<float channel="max_rpm" scale="1.0" />
<float channel="paused" scale="1.0" />
<float channel="race_position" scale="1.0" />
<float channel="suspension_acceleration_fr" scale="1.0" />
<float channel="suspension_acceleration_fl" scale="1.0" />
<float channel="suspension_acceleration_br" scale="1.0" />
<float channel="suspension_acceleration_bl" scale="1.0" />
        */
    //position at 4*4
    dof_roll  = get_float (packetBuffer, 4 * 0);  //roll.ori2
    dof_pitch = get_float (packetBuffer, 4 * 1);  //pitch.ori1
    dof_heave = get_float (packetBuffer, 4 * 2);  //heave
    dof_yaw   = get_float (packetBuffer, 4 * 3);  //yaw.ori0
    dof_sway  = get_float (packetBuffer, 4 * 4);  //sway
    dof_surge = get_float (packetBuffer, 4 * 5);  //surge
    dof_tloss = get_float (packetBuffer, 4 * 6);  //traction loss computation
    //safr      = get_float (packetBuffer, 4 * 13); //suspension_acceleration_fr
    //safl      = get_float (packetBuffer, 4 * 14); //suspension_acceleration_fl
    //sabr      = get_float (packetBuffer, 4 * 15); //suspension_acceleration_br
    //sabl      = get_float (packetBuffer, 4 * 16); //suspension_acceleration_bl
    //dash data
    if (_dashaddr)
    {
      _dpkt[MFC_DISPD]  = (int)get_float (packetBuffer, 4 * 7);  //speed
      _dpkt[MFC_DIGEAR] = (int)get_float (packetBuffer, 4 * 8);  //gear
      _dpkt[MFC_DIRPM]  = (int)get_float (packetBuffer, 4 * 9) * 10;  //rpm x10
      _dpkt[MFC_DIRPMM] = (int)get_float (packetBuffer, 4 * 10) * 10; //max rpm x10
      //printf ("\n#i@%.3f:d1 rpm %d", cltime, _dpkt[MFC_DIRPM]);
    }
    //-
    if (_odbg)
      printf ("\n#i@%.3f:t1 pitch %f %f %f roll %f %f yaw %f %f", cltime,
        dof_pitch, dof_surge, dof_heave, dof_roll, dof_sway, dof_yaw, dof_tloss);
      //printf ("\n#i@%.3f:t1 pitch %f %f %f roll %f %f yaw %f %f sa %f %f %f %f", cltime,
      //  dof_pitch, dof_surge, dof_heave, dof_roll, dof_sway, dof_yaw, dof_tloss, safr, safl, sabr, sabl);
          //compute dofs here - not making much sense during testing..
          /*
        With MyOutsim_Internal
            Roll_Output = ((.sngOrientation2 * 180 / 3.14159) - 90)
            Pitch_Output = (.sngOrientation1 * 180 / 3.14159) * -1
            Yaw_Output = (.sngOrientation0 * 180 / 3.14159)
            Heave_Output = (System.Math.Cos(.sngOrientation2) * .sngAcceleration2)
            Sway_Output = ((System.Math.Cos(.sngOrientation0) * .sngAcceleration0) + (System.Math.Sin(.sngOrientation0) * .sngAcceleration1))
            Surge_Output = ((-System.Math.Sin(.sngOrientation0) * .sngAcceleration0) + (System.Math.Cos(.sngOrientation0) * .sngAcceleration1))
            Extra1_Output = (((System.Math.Sin(.sngOrientation0) * .sngAcceleration0) + (System.Math.Sin(.sngOrientation0) * .sngAcceleration1)) * -1)
        End With
          */
    dof_roll  = dof_roll * RAD2DEG;
    dof_pitch = dof_pitch * RAD2DEG;
    dof_yaw   = dof_yaw * RAD2DEG;
    dof_heave = cos(dof_roll) * dof_heave;
    dof_tloss = dof_tloss;// / GRAVACCEL;
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
  //_cpkt[MFC_PIPITCH]  = (int)(fv[MFC_PIPITCH] * DOF_MAG);
  _cpkt[MFC_PIPITCH]  = (int)(dof_pitch * DOF_MAG);
  //_cpkt[MFC_PIPITCH] = -get_cmap (pf_pitch, -100, 100, -1000, 1000);
  //
  // formula based, but wrong? -> fv[MFC_PISURGE] = (-sin(ori2) * acc0 + cos(ori2) * acc2) / 10.f;//acc1 * RAD2DEG;
  //fv[MFC_PISURGE] = (cos(ori2) * acc2 + sin(ori2) * acc2) / 10.f;
  _cpkt[MFC_PISURGE]  = (int)(dof_surge * DOF_MAG);
  //_cpkt[MFC_PISURGE] = -get_cmap (pw_pitch, -32800, 32800, -7000, 7000);
  //
  // formula based, wrong -> fv[MFC_PIHEAVE] = (cos(ori2) * acc0) / 10.f;//acc2 * RAD2DEG;
  //fv[MFC_PIHEAVE] = (cos(ori2) * acc0 + sin(ori2) * acc0) / 10.f;
  _cpkt[MFC_PIHEAVE]  = (int)(dof_heave * DOF_MAG);
  //_cpkt[MFC_PIHEAVE] = -get_cmap (pv_pitch, -128, 128, -2000, 2000);
  //
  //fv[MFC_PIROLL] = ori2 * RAD2DEG;
  _cpkt[MFC_PIROLL]  = (int)(dof_roll * DOF_MAG);
  //_cpkt[MFC_PIROLL]  = get_cmap (pf_roll, -128, 128, -7000, 7000);
  //
  //-formula based -> fv[MFC_PISWAY] = (cos(ori1) * acc1 + sin(ori1) * acc0) / 10.f;//acc0 * RAD2DEG;
  //fv[MFC_PISWAY] = (cos(ori2) * acc1 + sin(ori2) * acc1) / 10.f;
  _cpkt[MFC_PISWAY]  = (int)(dof_sway * DOF_MAG);
  //_cpkt[MFC_PISWAY]  = get_cmap (get_float (packetBuffer, local_accel_idx)*100, 8200, 8200, -3000, 3000);
  //
  //fv[MFC_PIYAW] = ori0 * RAD2DEG;
  _cpkt[MFC_PIYAW]  = (int)(dof_yaw * DOF_MAG);
  //
  //fv[MFC_PITLOSS] = (sin(ori1) * acc1 + sin(ori1) * acc0);
  //fv[MFC_PITLOSS] = velo[0] / 9.8f;
  //fv[MFC_PITLOSS] = (sin(ori0) * acc1) / 10.f;
  _cpkt[MFC_PITLOSS] = (int)(dof_tloss * DOF_MAG);
  //
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
  printf ("\n#MFC DIRT RALLY 2 client");
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
  pktl  = mfc_bcast_pktlen ();
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
    /*
    #i.got message 100001:
    <00 00 00 00><02 00 00 00>
    <e2 ff ff ff><1e 00 00 00><1e 00 00 00><ec ff ff ff>
    <e2 ff ff ff><3c 00 00 00><c4 ff ff ff><64 00 00 00>
    <64 00 00 00><64 00 00 00><00 00 00 00><00 00 00 00>
    <00 00 00 00><00 00 00 00><00 00 00 00><00 00 00 00>
    <00 00 00 00><00 00 00 00><00 00 00 00><00 00 00 00>
    <00 00 00 00><00 00 00 00><00 00 00 00><00 00 00 00>
    <00 00 00 00>
    <44 69 72 74><20 52 61 6c><63 73 5b 66>
    */
    memcpy(_dpkt, _cpkt, pktl);
    //set sim/app name
    snprintf((char *)(_dpkt + MFC_DIAPP14), MFC_DIAPPSZ, "DirtRally2");
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
            printf ("\n#i@%f>%f:received %dpkts", cltime, rtime, ppkt);
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
