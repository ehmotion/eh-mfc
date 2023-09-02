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

--
https://forums.forzamotorsport.net/turn10_postst128499_Forza-Motorsport-7--Data-Out--feature-details.aspx

It seems that the values that were added in "Car Dash" mode in FM7 were moved towards the end of the Horizon 4 packet.
The structure of the Horizon packet is:
[0]-[231] FM7 Sled data (as mentioned before)
[232]-[243] FH4 new unknown data
[244]-[322] FM7 Car Dash data
[323] FH4 new unknown data
--

Input Parameters
The following parameters are now located in the HUD options menu.

• Data Out: Toggles the data output function on and off.
• Data Out IP Address: The target IP address of the remote machine receiving data.
• Data Out IP Port: The target IP port of the remote machine receiving data.

Output Structure
Data is output in the following structure.

s32 IsRaceOn; // = 1 when race is on. = 0 when in menus/race stopped …

u32 TimestampMS; //Can overflow to 0 eventually

f32 EngineMaxRpm;
f32 EngineIdleRpm;
f32 CurrentEngineRpm;

f32 AccelerationX; //In the car's local space; X = right, Y = up, Z = forward
f32 AccelerationY;
f32 AccelerationZ;

f32 VelocityX; //In the car's local space; X = right, Y = up, Z = forward
f32 VelocityY;
f32 VelocityZ;

f32 AngularVelocityX; //In the car's local space; X = pitch, Y = yaw, Z = roll
f32 AngularVelocityY;
f32 AngularVelocityZ;

f32 Yaw;
f32 Pitch;
f32 Roll;

f32 NormalizedSuspensionTravelFrontLeft; // Suspension travel normalized: 0.0f = max stretch; 1.0 = max compression
f32 NormalizedSuspensionTravelFrontRight;
f32 NormalizedSuspensionTravelRearLeft;
f32 NormalizedSuspensionTravelRearRight;

f32 TireSlipRatioFrontLeft; // Tire normalized slip ratio, = 0 means 100% grip and |ratio| > 1.0 means loss of grip.
f32 TireSlipRatioFrontRight;
f32 TireSlipRatioRearLeft;
f32 TireSlipRatioRearRight;

f32 WheelRotationSpeedFrontLeft; // Wheel rotation speed radians/sec.
f32 WheelRotationSpeedFrontRight;
f32 WheelRotationSpeedRearLeft;
f32 WheelRotationSpeedRearRight;

s32 WheelOnRumbleStripFrontLeft; // = 1 when wheel is on rumble strip, = 0 when off.
s32 WheelOnRumbleStripFrontRight;
s32 WheelOnRumbleStripRearLeft;
s32 WheelOnRumbleStripRearRight;

f32 WheelInPuddleDepthFrontLeft; // = from 0 to 1, where 1 is the deepest puddle
f32 WheelInPuddleDepthFrontRight;
f32 WheelInPuddleDepthRearLeft;
f32 WheelInPuddleDepthRearRight;

f32 SurfaceRumbleFrontLeft; // Non-dimensional surface rumble values passed to controller force feedback
f32 SurfaceRumbleFrontRight;
f32 SurfaceRumbleRearLeft;
f32 SurfaceRumbleRearRight;

f32 TireSlipAngleFrontLeft; // Tire normalized slip angle, = 0 means 100% grip and |angle| > 1.0 means loss of grip.
f32 TireSlipAngleFrontRight;
f32 TireSlipAngleRearLeft;
f32 TireSlipAngleRearRight;

f32 TireCombinedSlipFrontLeft; // Tire normalized combined slip, = 0 means 100% grip and |slip| > 1.0 means loss of grip.
f32 TireCombinedSlipFrontRight;
f32 TireCombinedSlipRearLeft;
f32 TireCombinedSlipRearRight;

f32 SuspensionTravelMetersFrontLeft; // Actual suspension travel in meters
f32 SuspensionTravelMetersFrontRight;
f32 SuspensionTravelMetersRearLeft;
f32 SuspensionTravelMetersRearRight;

s32 CarOrdinal; //Unique ID of the car make/model
s32 CarClass; //Between 0 (D -- worst cars) and 7 (X class -- best cars) inclusive
s32 CarPerformanceIndex; //Between 100 (slowest car) and 999 (fastest car) inclusive
s32 DrivetrainType; //Corresponds to EDrivetrainType; 0 = FWD, 1 = RWD, 2 = AWD
s32 NumCylinders; //Number of cylinders in the engine

[Sept. 10, 2018 UPDATE: NEW DATA OUT STRUCTURE]

V1 is now called Sled
V2 is V1 then these added in this order at the bottom.
//Position (meters)
f32 PositionX;
f32 PositionY;
f32 PositionZ;

f32 Speed; // meters per second
f32 Power; // watts
f32 Torque; // newton meter

f32 TireTempFrontLeft;
f32 TireTempFrontRight;
f32 TireTempRearLeft;
f32 TireTempRearRight;

f32 Boost;
f32 Fuel;
f32 DistanceTraveled;
f32 BestLap;
f32 LastLap;
f32 CurrentLap;
f32 CurrentRaceTime;

u16 LapNumber;
u8 RacePosition;

u8 Accel;
u8 Brake;
u8 Clutch;
u8 HandBrake;
u8 Gear;
s8 Steer;

s8 NormalizedDrivingLine;
s8 NormalizedAIBrakeDifference;
*/

#define s32 int32_t
#define u32 uint32_t
#define f32 float

#define u16 uint16_t
#define u8 uint8_t
#define s8 int8_t

#define PACKED __attribute__((packed))
//Forza Horizon 4 UDP data
typedef struct PACKED
{
  s32 IsRaceOn; // = 1 when race is on. = 0 when in menus/race stopped …

  u32 TimestampMS; //Can overflow to 0 eventually

  f32 EngineMaxRpm;
  f32 EngineIdleRpm;
  f32 CurrentEngineRpm;

  f32 AccelerationX; //In the car's local space; X = right, Y = up, Z = forward
  f32 AccelerationY;
  f32 AccelerationZ;

  f32 VelocityX; //In the car's local space; X = right, Y = up, Z = forward
  f32 VelocityY;
  f32 VelocityZ;

  f32 AngularVelocityX; //In the car's local space; X = pitch, Y = yaw, Z = roll
  f32 AngularVelocityY;
  f32 AngularVelocityZ;

  f32 Yaw;
  f32 Pitch;
  f32 Roll;

  f32 NormalizedSuspensionTravelFrontLeft; // Suspension travel normalized: 0.0f = max stretch; 1.0 = max compression
  f32 NormalizedSuspensionTravelFrontRight;
  f32 NormalizedSuspensionTravelRearLeft;
  f32 NormalizedSuspensionTravelRearRight;

  f32 TireSlipRatioFrontLeft; // Tire normalized slip ratio, = 0 means 100% grip and |ratio| > 1.0 means loss of grip.
  f32 TireSlipRatioFrontRight;
  f32 TireSlipRatioRearLeft;
  f32 TireSlipRatioRearRight;

  f32 WheelRotationSpeedFrontLeft; // Wheel rotation speed radians/sec.
  f32 WheelRotationSpeedFrontRight;
  f32 WheelRotationSpeedRearLeft;
  f32 WheelRotationSpeedRearRight;

  s32 WheelOnRumbleStripFrontLeft; // = 1 when wheel is on rumble strip, = 0 when off.
  s32 WheelOnRumbleStripFrontRight;
  s32 WheelOnRumbleStripRearLeft;
  s32 WheelOnRumbleStripRearRight;

  f32 WheelInPuddleDepthFrontLeft; // = from 0 to 1, where 1 is the deepest puddle
  f32 WheelInPuddleDepthFrontRight;
  f32 WheelInPuddleDepthRearLeft;
  f32 WheelInPuddleDepthRearRight;

  f32 SurfaceRumbleFrontLeft; // Non-dimensional surface rumble values passed to controller force feedback
  f32 SurfaceRumbleFrontRight;
  f32 SurfaceRumbleRearLeft;
  f32 SurfaceRumbleRearRight;

  f32 TireSlipAngleFrontLeft; // Tire normalized slip angle, = 0 means 100% grip and |angle| > 1.0 means loss of grip.
  f32 TireSlipAngleFrontRight;
  f32 TireSlipAngleRearLeft;
  f32 TireSlipAngleRearRight;

  f32 TireCombinedSlipFrontLeft; // Tire normalized combined slip, = 0 means 100% grip and |slip| > 1.0 means loss of grip.
  f32 TireCombinedSlipFrontRight;
  f32 TireCombinedSlipRearLeft;
  f32 TireCombinedSlipRearRight;

  f32 SuspensionTravelMetersFrontLeft; // Actual suspension travel in meters
  f32 SuspensionTravelMetersFrontRight;
  f32 SuspensionTravelMetersRearLeft;
  f32 SuspensionTravelMetersRearRight;

  s32 CarOrdinal;          //Unique ID of the car make/model
  s32 CarClass;            //Between 0 (D -- worst cars) and 7 (X class -- best cars) inclusive
  s32 CarPerformanceIndex; //Between 100 (slowest car) and 999 (fastest car) inclusive
  s32 DrivetrainType;      //Corresponds to EDrivetrainType; 0 = FWD, 1 = RWD, 2 = AWD
  s32 NumCylinders;        //Number of cylinders in the engine
} FSLEDUDPPacket;

// https://github.com/nettrom/forza_motorsport/blob/master/fdp.py
//Forza Horizon 4 UDP data
typedef struct PACKED
{
#define hzn uint32_t
s32 IsRaceOn; // = 1 when race is on. = 0 when in menus/race stopped …
u32 TimestampMS; //Can overflow to 0 eventually
f32 EngineMaxRpm;
f32 EngineIdleRpm;
f32 CurrentEngineRpm;
f32 AccelerationX; //In the car's local space; X = right, Y = up, Z = forward
f32 AccelerationY;
f32 AccelerationZ;
f32 VelocityX; //In the car's local space; X = right, Y = up, Z = forward
f32 VelocityY;
f32 VelocityZ;
f32 AngularVelocityX; //In the car's local space; X = pitch, Y = yaw, Z = roll
f32 AngularVelocityY;
f32 AngularVelocityZ;
f32 Yaw;
f32 Pitch;
f32 Roll;
f32 NormalizedSuspensionTravelFrontLeft; // Suspension travel normalized: 0.0f = max stretch; 1.0 = max compression
f32 NormalizedSuspensionTravelFrontRight;
f32 NormalizedSuspensionTravelRearLeft;
f32 NormalizedSuspensionTravelRearRight;
f32 TireSlipRatioFrontLeft; // Tire normalized slip ratio, = 0 means 100% grip and |ratio| > 1.0 means loss of grip.
f32 TireSlipRatioFrontRight;
f32 TireSlipRatioRearLeft;
f32 TireSlipRatioRearRight;
f32 WheelRotationSpeedFrontLeft; // Wheel rotation speed radians/sec.
f32 WheelRotationSpeedFrontRight;
f32 WheelRotationSpeedRearLeft;
f32 WheelRotationSpeedRearRight;
s32 WheelOnRumbleStripFrontLeft; // = 1 when wheel is on rumble strip, = 0 when off.
s32 WheelOnRumbleStripFrontRight;
s32 WheelOnRumbleStripRearLeft;
s32 WheelOnRumbleStripRearRight;
f32 WheelInPuddleDepthFrontLeft; // = from 0 to 1, where 1 is the deepest puddle
f32 WheelInPuddleDepthFrontRight;
f32 WheelInPuddleDepthRearLeft;
f32 WheelInPuddleDepthRearRight;
f32 SurfaceRumbleFrontLeft; // Non-dimensional surface rumble values passed to controller force feedback
f32 SurfaceRumbleFrontRight;
f32 SurfaceRumbleRearLeft;
f32 SurfaceRumbleRearRight;
f32 TireSlipAngleFrontLeft; // Tire normalized slip angle, = 0 means 100% grip and |angle| > 1.0 means loss of grip.
f32 TireSlipAngleFrontRight;
f32 TireSlipAngleRearLeft;
f32 TireSlipAngleRearRight;
f32 TireCombinedSlipFrontLeft; // Tire normalized combined slip, = 0 means 100% grip and |slip| > 1.0 means loss of grip.
f32 TireCombinedSlipFrontRight;
f32 TireCombinedSlipRearLeft;
f32 TireCombinedSlipRearRight;
f32 SuspensionTravelMetersFrontLeft; // Actual suspension travel in meters
f32 SuspensionTravelMetersFrontRight;
f32 SuspensionTravelMetersRearLeft;
f32 SuspensionTravelMetersRearRight;
s32 CarOrdinal; //Unique ID of the car make/model
s32 CarClass; //Between 0 (D -- worst cars) and 7 (X class -- best cars) inclusive
s32 CarPerformanceIndex; //Between 100 (slowest car) and 999 (fastest car) inclusive
s32 DrivetrainType; //Corresponds to EDrivetrainType; 0 = FWD, 1 = RWD, 2 = AWD
s32 NumCylinders; //Number of cylinders in the engine
//hzn HorizonPlaceholder; // unknown FH4 values
char padding[12];
f32 PositionX;
f32 PositionY;
f32 PositionZ;
f32 Speed; // meters per second
f32 Power; // watts
f32 Torque; // newton meter
f32 TireTempFrontLeft;
f32 TireTempFrontRight;
f32 TireTempRearLeft;
f32 TireTempRearRight;
f32 Boost;
f32 Fuel;
f32 DistanceTraveled;
f32 BestLap;
f32 LastLap;
f32 CurrentLap;
f32 CurrentRaceTime;
u16 LapNumber;
u8 RacePosition;
u8 Accel;
u8 Brake;
u8 Clutch;
u8 HandBrake;
u8 Gear;
s8 Steer;
s8 NormalizedDrivingLine;
s8 NormalizedAIBrakeDifference;
//s8 padding[8];
} FH4UDPPacket;

//Forza Motorsport 7 UDP data
typedef struct PACKED {
s32 IsRaceOn; // = 1 when race is on. = 0 when in menus/race stopped …

u32 TimestampMS; //Can overflow to 0 eventually

f32 EngineMaxRpm;
f32 EngineIdleRpm;
f32 CurrentEngineRpm;

f32 AccelerationX; //In the car's local space; X = right, Y = up, Z = forward
f32 AccelerationY;
f32 AccelerationZ;

f32 VelocityX; //In the car's local space; X = right, Y = up, Z = forward
f32 VelocityY;
f32 VelocityZ;

f32 AngularVelocityX; //In the car's local space; X = pitch, Y = yaw, Z = roll
f32 AngularVelocityY;
f32 AngularVelocityZ;

f32 Yaw;
f32 Pitch;
f32 Roll;

f32 NormalizedSuspensionTravelFrontLeft; // Suspension travel normalized: 0.0f = max stretch; 1.0 = max compression
f32 NormalizedSuspensionTravelFrontRight;
f32 NormalizedSuspensionTravelRearLeft;
f32 NormalizedSuspensionTravelRearRight;

f32 TireSlipRatioFrontLeft; // Tire normalized slip ratio, = 0 means 100% grip and |ratio| > 1.0 means loss of grip.
f32 TireSlipRatioFrontRight;
f32 TireSlipRatioRearLeft;
f32 TireSlipRatioRearRight;

f32 WheelRotationSpeedFrontLeft; // Wheel rotation speed radians/sec.
f32 WheelRotationSpeedFrontRight;
f32 WheelRotationSpeedRearLeft;
f32 WheelRotationSpeedRearRight;

s32 WheelOnRumbleStripFrontLeft; // = 1 when wheel is on rumble strip, = 0 when off.
s32 WheelOnRumbleStripFrontRight;
s32 WheelOnRumbleStripRearLeft;
s32 WheelOnRumbleStripRearRight;

f32 WheelInPuddleDepthFrontLeft; // = from 0 to 1, where 1 is the deepest puddle
f32 WheelInPuddleDepthFrontRight;
f32 WheelInPuddleDepthRearLeft;
f32 WheelInPuddleDepthRearRight;

f32 SurfaceRumbleFrontLeft; // Non-dimensional surface rumble values passed to controller force feedback
f32 SurfaceRumbleFrontRight;
f32 SurfaceRumbleRearLeft;
f32 SurfaceRumbleRearRight;

f32 TireSlipAngleFrontLeft; // Tire normalized slip angle, = 0 means 100% grip and |angle| > 1.0 means loss of grip.
f32 TireSlipAngleFrontRight;
f32 TireSlipAngleRearLeft;
f32 TireSlipAngleRearRight;

f32 TireCombinedSlipFrontLeft; // Tire normalized combined slip, = 0 means 100% grip and |slip| > 1.0 means loss of grip.
f32 TireCombinedSlipFrontRight;
f32 TireCombinedSlipRearLeft;
f32 TireCombinedSlipRearRight;

f32 SuspensionTravelMetersFrontLeft; // Actual suspension travel in meters
f32 SuspensionTravelMetersFrontRight;
f32 SuspensionTravelMetersRearLeft;
f32 SuspensionTravelMetersRearRight;

s32 CarOrdinal; //Unique ID of the car make/model
s32 CarClass; //Between 0 (D -- worst cars) and 7 (X class -- best cars) inclusive
s32 CarPerformanceIndex; //Between 100 (slowest car) and 999 (fastest car) inclusive
s32 DrivetrainType; //Corresponds to EDrivetrainType; 0 = FWD, 1 = RWD, 2 = AWD
s32 NumCylinders; //Number of cylinders in the engine
//v1 num_fields ; // 58

//[Sept. 10, 2018 UPDATE: NEW DATA OUT STRUCTURE]
//V1 is now called Sled
//V2 is V1 then these added in this order at the bottom.
//Position (meters)
f32 PositionX;
f32 PositionY;
f32 PositionZ;

f32 Speed; // meters per second
f32 Power; // watts
f32 Torque; // newton meter

f32 TireTempFrontLeft;
f32 TireTempFrontRight;
f32 TireTempRearLeft;
f32 TireTempRearRight;

f32 Boost;
f32 Fuel;
f32 DistanceTraveled;
f32 BestLap;
f32 LastLap;
f32 CurrentLap;
f32 CurrentRaceTime;

u16 LapNumber;
u8 RacePosition;

u8 Accel;
u8 Brake;
u8 Clutch;
u8 HandBrake;
u8 Gear;
s8 Steer;

s8 NormalizedDrivingLine;
s8 NormalizedAIBrakeDifference;
//v2:total num_fields ; // 77 ints of 4 bytes
} FM7UDPPacket;

//#Forza v1 / SLED pkt 232B
#define PKTSZ_FV1 232
//#Forza Motorsport 7 pkt 311B
#define PKTSZ_FM7 311
//#Forza Horizon 4 pkt 315B
#define PKTSZ_FH4 324

#define UDP_MAX_PACKETSIZE  2048
#define UDP_PORT            10001

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
float _pitchprc = 0.0f;
float _rollprc  = 0.0f;
float _yawprc   = 0.0f;
float _surgeprc = 30.0f;
float _swayprc  = -30.0f;
float _heaveprc = -20.0f;
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
//                          pit    srg    hev    rol    sway   yaw      tl
static int mpkt[] = {0, 0, -5042, -198968, -24046, -1174, -157804, -4104, 0, 0, 0, 0};
static int Mpkt[] = {0, 0, 4092, 92609, 35579, 992, 157219, 2485, 0, 100, 100, 100};

int mfc_packet_use(char* packetBuffer, int pktsz, float rtime, float cltime)
{
  static FM7UDPPacket *dpktfm7;
  static FH4UDPPacket *dpktfh4;
  dpktfm7 = (FM7UDPPacket *)packetBuffer;
  dpktfh4 = (FH4UDPPacket *)packetBuffer;
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
  if (pktsz == PKTSZ_FH4)
  {
    dof_roll  = dpktfh4->AngularVelocityX;//get_float (packetBuffer, 4 * 0);  //roll.ori2
    dof_pitch = dpktfh4->AngularVelocityY;//get_float (packetBuffer, 4 * 1);  //pitch.ori1
    dof_heave = dpktfh4->AccelerationY;//(cos(ori2) * acc0 + sin(ori2) * acc0) / GRAVACCEL;//get_float (packetBuffer, 4 * 2);  //heave
    dof_yaw   = dpktfh4->AngularVelocityZ;//get_float (packetBuffer, 4 * 3);  //yaw.ori0
    dof_sway  = dpktfh4->AccelerationX;//(cos(ori2) * acc1 + sin(ori2) * acc1) / GRAVACCEL;//get_float (packetBuffer, 4 * 4);  //sway
    dof_surge = dpktfh4->AccelerationZ;//(cos(ori2) * acc2 + sin(ori2) * acc2) / GRAVACCEL;//get_float (packetBuffer, 4 * 5);  //surge
    dof_tloss = 0;//get_float (packetBuffer, 4 * 6);  //traction loss computation
  }
  else
  {
    dof_roll = dpktfm7->Roll;   //get_float (packetBuffer, 4 * 0);  //roll.ori2
    dof_pitch = dpktfm7->Pitch; //get_float (packetBuffer, 4 * 1);  //pitch.ori1
    dof_heave = 0;           //(cos(ori2) * acc0 + sin(ori2) * acc0) / GRAVACCEL;//get_float (packetBuffer, 4 * 2);  //heave
    dof_yaw = dpktfm7->Yaw;     //get_float (packetBuffer, 4 * 3);  //yaw.ori0
    dof_sway = 0;            //(cos(ori2) * acc1 + sin(ori2) * acc1) / GRAVACCEL;//get_float (packetBuffer, 4 * 4);  //sway
    dof_surge = 0;           //(cos(ori2) * acc2 + sin(ori2) * acc2) / GRAVACCEL;//get_float (packetBuffer, 4 * 5);  //surge
    dof_tloss = 0;           //get_float (packetBuffer, 4 * 6);  //traction loss computation
  }
    //-
    if (1||_odbg)
      printf ("\n#i@%.3f:%s:t1 pitch %f %f %f roll %f %f yaw %f %f", cltime, ((pktsz == PKTSZ_FH4)?"FH4":"FM7"),
        dof_pitch, dof_surge, dof_heave, dof_roll, dof_sway, dof_yaw, dof_tloss);
        
    _cpkt[MFC_PIPITCH] = (int)(dof_pitch * DOF_MAG);
    _cpkt[MFC_PISURGE] = (int)(dof_surge * DOF_MAG);
    _cpkt[MFC_PIHEAVE] = (int)(dof_heave * DOF_MAG);
    _cpkt[MFC_PIROLL]  = (int)(dof_roll * DOF_MAG);
    _cpkt[MFC_PISWAY]  = (int)(dof_sway * DOF_MAG);
    _cpkt[MFC_PIYAW]   = (int)(dof_yaw * DOF_MAG);
    _cpkt[MFC_PITLOSS] = (int)(dof_tloss * DOF_MAG);
    if (_odbg)
      printf ("\n#i@%.3f:t2 pitch %f %f %f roll %f %f yaw %f %f", cltime, 
        dof_pitch, dof_surge, dof_heave, dof_roll, dof_sway, dof_yaw, dof_tloss);
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
  if (0 || _odbg)
    printf ("\n#i@%.3f.t2 pitch%% %d %d %d roll%% %d %d yaw%% %d %d", cltime,
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
  if (0 || _odbg)
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
    if (pktsz == PKTSZ_FH4)
    {
      _dpkt[MFC_DISPD] = (int)(dpktfh4->Speed * 2.24f);   //speed: 1 m/s, 3.60 kph - 1 m/s, 2.24 mph
      _dpkt[MFC_DIGEAR] = dpktfh4->Gear;          //gear
      _dpkt[MFC_DIRPM] = dpktfh4->CurrentEngineRpm;          //rpm x10
      _dpkt[MFC_DIRPMM] = dpktfh4->EngineMaxRpm - 1500;         //max rpm x10
      //printf ("\n#i@%.3f:FH4 spd %d gear %x", cltime, (int)(dpktfh4->Speed*2.24f), dpktfh4->Gear);
    }
    else
    {
      _dpkt[MFC_DISPD] = dpktfm7->Speed * 2.24f;   //speed: 1 m/s, 3.60 kph - 1 m/s, 2.24 mph
      _dpkt[MFC_DIGEAR] = dpktfm7->Gear; //gear
      _dpkt[MFC_DIRPM] = dpktfm7->CurrentEngineRpm; //rpm x10
      _dpkt[MFC_DIRPMM] = dpktfm7->EngineMaxRpm; //max rpm x10
    }
    //printf ("\n#i@%.3f:FH4dd spd %d gear %x", cltime, _dpkt[MFC_DISPD], _dpkt[MFC_DIGEAR]);
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
      break;
    default:
      printf("unrecognized option: %c\n", c);
      break;
    }
  }

  //configuration summary 
  printf ("\n# ##");
  printf ("\n#MFC MS Forza Motorsport 7/Forza Horizon 4 client");
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
  if (0)
  {
    printf("\n#Forza v1/SLED pkt      %dB", sizeof(FSLEDUDPPacket));
    printf("\n#Forza Motorsport 7 pkt %dB", sizeof(FM7UDPPacket));
    printf("\n#Forza Horizon 4 pkt    %dB", sizeof(FH4UDPPacket));
  }
  printf("\n# ##");
  //
  if (sizeof (float) != 4)
  {
    printf ("\nERR:game data packet will be wrong size (float %d), bailing out\n", sizeof (float));
    exit(-1);
  }
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
  //int fm7pktsz = sizeof(FM7UDPPacket);
  //int fh4pktsz = sizeof(FH4UDPPacket);
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
    snprintf((char *)(_dpkt + MFC_DIAPP14), MFC_DIAPPSZ, "FH4/FM7");
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
      static int rlen;
      rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL);
      cltime = (float)ctime_ms(0)/1000.0f;
      if (rlen > 0)
      {
        if (rlen != PKTSZ_FM7 && rlen != PKTSZ_FH4)
        {
          printf ("\n#i@%f>%f:W:received wrong pkt size %dB vs expected %dB or %dB",
            cltime, rtime, rlen, PKTSZ_FH4, PKTSZ_FM7);
        }
        char paused = ((FSLEDUDPPacket *)(packetBuffer))->IsRaceOn?0:1; //custom paused/running
        if (pktsz == -1)
        {
          //first packet ever
          pktsz = rlen;
          printf ("\n#i@%f>%f:%s:received pkt size %dbytes (%d fields)", cltime, rtime,
            rlen==PKTSZ_FM7?"FM7":"FH4",pktsz, pktsz/4);
          if (0)//debug
          {
            printf ("\n#i@%f>%f:data %dB:", cltime, rtime, pktsz);
            for (int i = 0; i < rlen; ++i)
            {
              if (i > 0)
                printf (" ");
              printf ("%02x", packetBuffer[i]);
            }
          }
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
          if (0)//debug
          {
            printf ("\n#i@%f>%f:data %dB:", cltime, rtime, pktsz);
            for (int i = 0; i < rlen; ++i)
            {
              if (i > 0)
                printf (" ");
              printf ("%02x", packetBuffer[i]);
            }
          }
          mfc_packet_use (packetBuffer, rlen, rtime, cltime);
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
