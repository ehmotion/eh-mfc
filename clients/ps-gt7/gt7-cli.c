/*
* Playstation Gran Turismo 7/Sport/6

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

#define bool unsigned char

struct gt7_rd {
#if 0
            Position = new Vector3(sr.ReadSingle(), sr.ReadSingle(), sr.ReadSingle());
            Velocity = new Vector3(sr.ReadSingle(), sr.ReadSingle(), sr.ReadSingle());
            Rotation = new Vector3(sr.ReadSingle(), sr.ReadSingle(), sr.ReadSingle());
            RelativeOrientationToNorth = sr.ReadSingle();
            AngularVelocity = new Vector3(sr.ReadSingle(), sr.ReadSingle(), sr.ReadSingle());
            BodyHeight = sr.ReadSingle();
            EngineRPM = sr.ReadSingle();
            sr.Position += sizeof(int); // Skip IV
            GasLevel = sr.ReadSingle();
            GasCapacity = sr.ReadSingle();
            MetersPerSecond = sr.ReadSingle();
#endif
/* https://www.gtplanet.net/forum/threads/gt7-is-compatible-with-motion-rig.410728/post-13799643
Pos	  Data Type	Range	Meaning	Notes
0x04	3 float	[-inf,inf]	Position X,Y,Z	Y points up, XZ is the race track plane Origin varies by track
0x10	3 float	[-inf,inf]	Velocity X,Y,Z	in meter/second, normalized gives vehicle direction
0x1C	3 float	[-1,1]	Rotation X,Y,Z	seems to be the cosine of rotation half? angle about X,Y,Z axis, acos(Y)*360/PI seems to be rotation angle of car direction about track north direction
UPDATE: Seems to be the imaginary part of a unit quaternion that gives the rotation of the car relative to the track coordinate system, will add more info
0x28	1 float	[0,1]	Rotation ?	seems to be 1 when pointing north and 0 when pointing south, don't know if there is any benefit to Rotation values
UPDATE: Seems to be the real part of a unit quaternion that gives the rotation of the car relative to the track coordinate system, will add more info
0x2C	3 float	[-inf,inf]	Angular Velocity X,Y,Z	in radians/second, how fast the car turns about an axis
0x38	1 float	?	Ride Height	seems to include suspension effects, not completely sure what is measured here
0x3C	1 float	[0,inf]	RPM	Engine RPM
0x40	4 byte	-	IV	decoding data
0x44	1 float	[0,100?]	Fuel Level	amount of of fuel left, starts at Fuel Capacity at start of race, TDB for EVs
0x48	1 float	[5,100?]	Fuel Capacity	amount of fuel that fits into the tank, usually 100 for fossil fuel cars, 5 for the kart, TBD for EVs
0x4C	1 float	[0,inf]	Speed	in meter/second, positive even when going backwards
0x50	1 float	[0,inf?]	Turbo Boost	this value - 1 gives the Turbo Boost display
0x54	1 float	[1.6?,8?]	Oil Pressure	in Bar?
0x58	1 float	?	?	seems to be always at 85.0, maybe water temp. check with other cars
0x5C	1 float	?	?	seems to be always at 110.0, maybe oil temp. check with other cars
0x60	4 float	[-273.15,inf]	Tire Temperature FL,FR,RL,RR	in degree celsius
0x70	1 int	-	Ticks	increases for every sent packet even when paused, packet number
0x74	1 short	[0,65535]	Current Lap	Which lap we are in, 0 before starting first lap
0x76	1 short	[0,65535]	Total Race Laps	How many laps the race will have, 0 for TTs
0x78	1 int	-	Best Lap Time	millisecond timestamp
0x7C	1 int	-	Last Lap Time	millisecond timestamp
0x80	1 int	-	Day Time Progression	millisecond timestamp, time of day indicates race start time of day, affected by Variable Time Speed Ratio, useless for timing when time speed ratio is not 1
0x84	1 short	TBD	Race Position	position of the car in the race, seems to be always -1 after race start
0x86	1 short	TBD	Number Of Cars	total number of cars in the race, seems to be always -1 after race start
0x88	1 short	[0,65535]	RPM Flashing	in RPM, indicates RPM when rev indicator starts flashing
0x8A	1 short	[0,65535]	RPM Rev. Limiter	in RPM, indicates RPM when rev limiter is hit
0x8C	1 short	[0,65535]	Car Maximum Speed	in speed units (km/h,mph,etc) chosen by game display setting, affected by car and tuning options
0x8E	1 short	-	Flags	each of the bits indicates something is active when 1
0: in race
1: paused
2: loading/processing
3: in a gear, 0 when shifting or out of gear, standing
4: has turbo
5: rev limiter flashing
6: handbrake
7: lights
8: lowbeam
9: highbeam
10: ASM
11: TCS
12:?
13:?
14:?
15:?
0x90	1 byte	[0,15]	Gears	two nibbles/4bit integers:
0-4: current gear, 0 is reverse, -1 is neutral
4-8: suggested gear, -1 when no suggested gear
0x91	1 byte	[0,255]	Throttle	throttle input
0x92	1 byte	[0,255]	Brake	brake input
0x93	1 byte	-	Reserved	always set to 0
0x94	4 float	-	Road Plane	coefficients of plane equation of the road below? or nearest? to the car, first three floats are the normal of the plane, distance of Position to this plane matches Ride Height
0xA4	4 float	[-inf,inf]	Wheel Speed FL,FR,RL,RR	angular speed in radians/second, sign opposite to vehicle speed
0xB4	4 float	[0,inf]	Tire Radius FL,FR,RL,RR	in meter, multiply with Wheel Speed to get Tire Speed in meter/second
0xC4	4 float	?	Tire Suspension Travel FL,FR,RL,RR	in meter?
0xD4	32 bytes	?	Reserved?	always set to 0
0xF4	1 float	[0,1]	Clutch	TBD
0xF8	1 float	[0,1]	Clutch Engagement	TBD, seems to be 1 - Clutch
0xFC	1 float	[0,inf]	RPM through Clutch	TBD
0x100	1 float	[0,inf]	Trans. Top Speed	corresponds to the Top Speed setting of a customizable gear box in the car settings, given as gear ratio
0x104	8 float	[0,inf]	Gear Ratios	ratios for gears 1-8 (what about reverse?)
there is space for 8 gears, if car has more gears than that 9th gear ratio will overwrite the Car Code and other gear ratios are lost
0x124	1 int		Car Code	*/
  // https://github.com/Nenkai/PDTools/blob/master/PDTools.SimulatorInterface/SimulatorPacket.cs
  //#00 magic int32
  #if 0
            int magic = sr.ReadInt32();
            if (magic == 0x30533647) // G6S0 - GT6
                sr.Endian = Syroot.BinaryData.Core.Endian.Big; // GT6 is on PS3, it'll be sending a big endian packet
            else if (magic == 0x47375330) // 0S7G - GTSport/GT7
                sr.Endian = Syroot.BinaryData.Core.Endian.Little;
  #endif
  uint32_t magic;
  //#01 position x,y,z (float) // Position on the track. Track units are in meters.
  float px, py, pz;
  //#04 velocity x, y, z (float) // Velocity in track units (which are meters) for each axis.
  float vx, vy, vz;
  //#07 rotation Pitch/Yaw/Roll all -1 to 1 (float)
  float rx, ry, rz;
  //#10 RelativeOrientationToNorth (float) // Orientation to North. 1.0 is north, 0.0 is south.
  float rorin;
  //#11 angular velocity a, b, c (float) // How fast the car turns around axes. (In radians/second, -1 to 1).
  float ax, ay, az;
  //#14 body heigth (float)
  float bheigth;
  //#15 rpm (float)
  float rpm;
  //#16 IV - 8 bytes
  uint32_t oiv;
  //#20 gas level (float)
  float glevel;
  //#24 gas capacity (float)
  float gcapa;
  //#28 speed mps (float)
  float mps;
  //#32 boost (float)
#if 0
            TurboBoost = sr.ReadSingle();
            OilPressure = sr.ReadSingle();
            WaterTemperature = sr.ReadSingle();
            OilTemperature = sr.ReadSingle();
            TireFL_SurfaceTemperature = sr.ReadSingle();
            TireFR_SurfaceTemperature = sr.ReadSingle();
            TireRL_SurfaceTemperature = sr.ReadSingle();
            TireRR_SurfaceTemperature = sr.ReadSingle();
            PacketId = sr.ReadInt32();
            LapCount = sr.ReadInt16();
            LapsInRace = sr.ReadInt16();
            BestLapTime = TimeSpan.FromMilliseconds(sr.ReadInt32());
            LastLapTime = TimeSpan.FromMilliseconds(sr.ReadInt32());
            TimeOfDayProgression = TimeSpan.FromMilliseconds(sr.ReadInt32());
            PreRaceStartPositionOrQualiPos = sr.ReadInt16();
            NumCarsAtPreRace = sr.ReadInt16();
            MinAlertRPM = sr.ReadInt16();
            MaxAlertRPM = sr.ReadInt16();
            CalculatedMaxSpeed = sr.ReadInt16();
            Flags = (SimulatorFlags)sr.ReadInt16();
#endif
  float boost;
  //#36 oil pressure (float)
  float oilpres;
  //#40 water temp (float)
  float watertemp;
  //#44 oil temp (float)
  float oiltemp;
  //#48 tire temp fl (float)
  //#52 tire temp fr (float)
  //#56 tire temp rl (float)
  //#60 tire temp rr (float)
  float ttemp[4];
  //#64 total time ticks (int)
  uint32_t pktid;
  //#68 lap (short)
  uint16_t lap;
  //#70 laps total (short)
  uint16_t laps;
  //#72 best lap ms (int)
  uint32_t blap;
  //#76 last lap ms (int)
  uint32_t llap;
  //#80 day progress ms (int)
  uint32_t dprog;
  //#84 start pos (short)
  uint16_t spos;
  //#86 num cars (short)
  uint16_t ncars;
  //#88 min rpm alert (short)
  uint16_t mrpm;
  //#90 max rpm alert (short)
  uint16_t Mrpm;
  //#92 max speed (short)
  uint16_t Mspeed;
  //#94 flags (short)
  // 0Z19 - replay
  // 0Z18 - stand by
  // 0Z00 - game menu
  // 0Z98 - loading/race wait
  // 0Z99 - starting
  // 0Z91 - racing
  // 0Z93 - pause
  /* https://www.gtplanet.net/forum/threads/gt7-is-compatible-with-motion-rig.410728/post-13799643
  each of the bits indicates something is active when 1
  0: in race
  1: paused
  2: loading/processing
  3: in a gear, 0 when shifting or out of gear, standing
  4: has turbo
  5: rev limiter flashing
  6: handbrake
  7: lights
  8: lowbeam
  9: highbeam
  10: ASM
  11: TCS
  12:?
  13:?
  14:?
  15:?
  */
  uint16_t flags;
  //#96 gear (byte) - CurrentGear = (byte)(bits & 0b1111); SuggestedGear = (byte)(bits >> 4);
#if 0
            int bits = sr.ReadByte();
            CurrentGear = (byte)(bits & 0b1111); // 4 bits
            SuggestedGear = (byte)(bits >> 4); // Also 4 bits

            Throttle = sr.ReadByte();
            Brake = sr.ReadByte();
#endif
  uint8_t gear;
  //#97 accel (byte)
  uint8_t accel;
  //#98 brake (byte)
  uint8_t brake;
  //#99 unk (byte)
  //#not relevant now
#if 0
            Empty_0x93 = sr.ReadByte();

            RoadPlane = new Vector3(sr.ReadSingle(), sr.ReadSingle(), sr.ReadSingle());
            RoadPlaneDistance = sr.ReadSingle();

            WheelFL_RevPerSecond = sr.ReadSingle();
            WheelFR_RevPerSecond = sr.ReadSingle();
            WheelRL_RevPerSecond = sr.ReadSingle();
            WheelRR_RevPerSecond = sr.ReadSingle();
            TireFL_TireRadius = sr.ReadSingle();
            TireFR_TireRadius = sr.ReadSingle();
            TireRL_TireRadius = sr.ReadSingle();
            TireRR_TireRadius = sr.ReadSingle();
            TireFL_SusHeight = sr.ReadSingle();
            TireFR_SusHeight = sr.ReadSingle();
            TireRL_SusHeight = sr.ReadSingle();
            TireRR_SusHeight = sr.ReadSingle();

            sr.Position += sizeof(int) * 8; // Seems to be reserved - server does not set that

            ClutchPedal = sr.ReadSingle();
            ClutchEngagement = sr.ReadSingle();
            RPMFromClutchToGearbox = sr.ReadSingle();

            TransmissionTopSpeed = sr.ReadSingle();

            // Always read as a fixed 7 gears
            for (var i = 0; i < 7; i++)
                GearRatios[i] = sr.ReadSingle();

            // Normally this one is not set at all. The game memcpy's the gear ratios without bound checking
            // The LC500 which has 10 gears even overrides the car code 😂
            float empty_or_gearRatio8 = sr.ReadSingle();

            CarCode = sr.ReadInt32();
#endif
};

#define UDP_MAX_PACKETSIZE  0x128
#define UDP_PORT            33739

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
char *_bcastaddr = NULL;
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
//learning mode
//#i:new min/Max values:
//                          pit    srg    hev    rol    sway   yaw    tl
//static int Mpkt[] = {0, 0, 68, 1000, 1000, 86, 1000, 1000, 0, 100, 100, 100};
//static int mpkt[] = {0, 0,-58, -1000, -1000, -78, -1000, -1000, 0, 0, 0, 0};
static int Mpkt[] = {0, 0,  9000,  2200,  1100,  9000,  1100,  6000,  6000, 100, 100, 100};
static int mpkt[] = {0, 0, -9000, -2200, -1100, -9000, -1100, -6000, -6000,   0,   0,   0};

int debug_data = 0;

float *quat_conj(float *Q)
{
  Q[0] = -Q[0];
  Q[1] = -Q[1];
  Q[2] = -Q[2];
  //Q[3] = Q[3];
  return Q;
}
float *quat_vec(float *Q)
{
  return Q;
}
float quat_scalar(float *Q)
{
  return Q[3];
}
float *cross(float *A, float *B)
{
  static float cr[3];
  cr[0] = A[1]*B[2]-A[2]*B[1];
  cr[1] = A[2]*B[0]-A[0]*B[2];
  cr[2] = A[0]*B[1]-A[1]*B[0];
  return cr;
}
float *add(float *A, float *B)
{
  static float ar[3];
  ar[0] = A[0]+B[0];
  ar[1] = A[1]+B[1];
  ar[2] = A[2]+B[2];
  return ar;
}
float *sub(float *A, float *B)
{
  static float sr[3];
  sr[0] = A[0]-B[0];
  sr[1] = A[1]-B[1];
  sr[2] = A[2]-B[2];
  return sr;
}
float *scale(float *A, float s)
{
  static float sr[3];
  sr[0] = A[0]*s;
  sr[1] = A[1]*s;
  sr[2] = A[2]*s;
  return sr;
}
float *quat_rot(float *V, float *Q)
{
  static float Rr[3];
  float *Qv = quat_vec(Q);
  float *Up = cross(Qv, V);
  float  U[3]; U[0] = Up[0]; U[1] = Up[1]; U[2] = Up[2];
  float  w  = quat_scalar(Q);
  float *Pp  = add(U, scale(V, w));
  float  P[3]; P[0] = Pp[0]; P[1] = Pp[1]; P[2] = Pp[2];
  float *Tp = scale(Qv, 2.0f);
  float T[3]; T[0] = Tp[0]; T[1] = Tp[1]; T[2] = Tp[2];
  float *Rp = cross(T, P);
  float RR[3]; RR[0] = Rp[0]; RR[1] = Rp[1]; RR[2] = Rp[2];
  float *R  = add(V, RR);
  Rr[0] = R[0]; Rr[1] = R[1]; Rr[2] = R[2];
  return Rr;
}
/*
It's not just the rotation angles -- the range of [-1,1] indicates that they are sin/cos of an angle.
So need to do an asin/acos to get to the angles in radians, then multiply with 180/pi to get to the angle in degrees.
If you do that, you will notice that the angles are just half the angles of the actual rotation in the game.
If you rotate the car 90° you will notice that the rotation angles will only indicate 45°.
All of this indicates that the values are coefficients of a unit quaternion -- if you read up on that here, -- https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
you see that the quaternion coefficients are sin/cos of half the rotation angle about a certain axis which is also encoded in the coefficients.

Here is an example python script that does the velocity vector rotation to get to the local velocity.
The Z component of the local vector seems the main direction of travel, and the sign of the velocity is reversed.
I guess the X component corresponds to sideways velocity, with positive meaning leftways,
so all in all it seems that PD is using a right handed coordinate system. Here is the script,
including Acceleration and G-Force calculation as described by @Skinny McLean:

#see https://www.gtplanet.net/forum/threads/gt7-is-compatible-with-motion-rig.410728/post-13823560

def quat_conj(Q):
    return (-Q[0],-Q[1],-Q[2],Q[3])
def quat_vec(Q):
    return (Q[0],Q[1],Q[2])
def quat_scalar(Q):
    return Q[3]
def cross(A,B):
    return (A[1]*B[2]-A[2]*B[1],A[2]*B[0]-A[0]*B[2],A[0]*B[1]-A[1]*B[0])
def add(A,B):
    return (A[0]+B[0],A[1]+B[1],A[2]+B[2])
def sub(A,B):
    return (A[0]-B[0],A[1]-B[1],A[2]-B[2])
def scale(A,s):
    return (A[0]*s,A[1]*s,A[2]*s)
def quat_rot(V,Q):
    Qv = quat_vec(Q)
    U = cross(Qv,V)
    w = quat_scalar(Q)
    P = add(U,scale(V,w))
    T = scale(Qv,2)
    RR = cross(T,P)
    R = add(V,RR)
    return R

print("Ctrl+C to exit the program")
dp_prev = 0
pknt = 0
Vp = (0,0,0)
while True:
    try:
        data, address = s.recvfrom(4096)
        pknt = pknt + 1
        print("received: %d bytes" % len(data))
        ddata = salsa20_dec(data)
        if len(ddata) > 0:
            magic = struct.unpack_from('i',ddata,0)
            P = struct.unpack_from('fff',ddata,0x4)
            print('Position',P)
            V = struct.unpack_from('fff',ddata,0x10)
            print('Global Velocity',V)
            Q = struct.unpack_from('ffff',ddata,0x1C)
            Qc = quat_conj(Q)
            Vl = quat_rot(V,Qc)
            print("Local Velocity:",Vl)
            dV = sub(Vl,Vp)
            A = scale(dV,60)
            print("Acceleration:",A)
            G = scale(A,1.0/9.81)
            print("G-Forces:",G)
            Vp = Vl # save current local velocity for next packet

        if pknt > 900:
            send_hb(s)
            pknt = 0
    except Exception as e:
        print(e)
        send_hb(s)
        pknt = 0
        pass
*/
typedef struct {
    float w, x, y, z;
} Quaternion;

typedef struct {
    float roll, pitch, yaw;
} EulerAngles;

EulerAngles *ToEulerAngles(float w, float x, float y, float z)
{
    static EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        angles.pitch = copysignf(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asinf(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.yaw = atan2f(siny_cosp, cosy_cosp);

    return &angles;
}
//quaternions to roll/pitch/yaw
/*
def roll_pitch_yaw(Q):
# see http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/Quaternions.pdf
# permute quaternion coefficients to determine
# order of roll/pitch/yaw rotation axes, scalar comes first always
P=(Q[3],Q[2],Q[0],Q[1])
# e is for handedness of axes
e = -1
x_p = 2*(P[0]*P[2] + e*P[1]*P[3])
pitch = math.asin(x_p)
if math.isclose(math.fabs(pitch),math.pi):
    # handle singularity when pitch is +-90°
    yaw = 0
    roll = math.atan2(P[1],P[0])
else:
    y_r = 2*(P[0]*P[1] - e*P[2]*P[3])
    x_r = 1 - 2*(P[1]*P[1] + P[2]*P[2])
    roll = math.atan2(y_r,x_r)
    y_y = 2*(P[0]*P[3] - e*P[1]*P[2])
    x_y = 1 - 2*(P[2]*P[2] + P[3]*P[3])
    yaw =math.atan2(y_y,x_y)
return (roll,pitch,yaw)
*/
// https://www.gtplanet.net/forum/threads/gt7-is-compatible-with-motion-rig.410728/post-13829730
float *quats_to_rpy(float *Q) // roll/pitch/yaw
{
  static float rpyr[3];
  float P[4];
  // see http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/Quaternions.pdf
  // permute quaternion coefficients to determine
  //  order of roll/pitch/yaw rotation axes, scalar comes first always
  P[0] = Q[3]; P[1] = Q[2]; P[2] = Q[0]; P[3] = Q[1];
  float e = -1.0f;
  float x_p = 2.0f*(P[0]*P[2] + e*P[1]*P[3]);
  float pitch = asinf(x_p);
  float yaw = 0.0f;
  float roll = 0.0f;
  if ((int)(abs(pitch * 1000.0f)) == (int)(MATH_PI * 1000.0f))
  {
    // handle singularity when pitch is +-90°
    roll = atan2f(P[1], P[0]);
  }
  else
  {
    float y_r = 2.0f*(P[0]*P[1] - e*P[2]*P[3]);
    float x_r = 1.0f - 2.0f*(P[1]*P[1] + P[2]*P[2]);
    roll = atan2f(y_r, x_r);
    float y_y = 2.0f*(P[0]*P[3] - e*P[1]*P[2]);
    float x_y = 1.0f - 2.0f*(P[2]*P[2] + P[3]*P[3]);
    yaw = atan2f(y_y, x_y);
  }
  rpyr[0] = roll;
  rpyr[1] = pitch;
  rpyr[2] = yaw;
  return rpyr;
}

int mfc_packet_use(char* packetBuffer, float rtime, float cltime)
{
  struct gt7_rd *tpkt = (struct gt7_rd *)packetBuffer;
  //float *fpkt = (float*)packetBuffer;
  //static float pvx = 0.0f, pvy = 0.0f, pvz = 0.0f, dt = 0.0f;//previous values for vx, vy, vz
  static float pltime = 0.0f, dt = 0.0f;
  //static float ori0 = 0, ori1 = 0, ori2 = 0;
  //static float acc0 = 0, acc1 = 0, acc2 = 0;
  //static float vel0 = 0, vel1 = 0, vel2 = 0;
  //printf("\n#i.max axis at 60%%: %.3f", get_cmap_f(-137, -1795547, 1795547, -6000, 6000));
  //#define DOF_MAG (10000)
  #define DOF_MAG (1000.0f)
  /* uses 10000 magnitude to not lose much fidelity on computation
                                
                                179.554748535156
  */
  //int Mpkt[MFC_PKT_SIZE] = {1, 1,   50483,   43371,   1795547,   11451,   14847,   5426, 1};
  // *!!!* careful with indexes as they need to correspond to the data pkt order
  //                                     //pitch  //surge  //heave  //roll   //sway   //yaw     //trloss
  static float dof_roll, dof_pitch, dof_yaw, dof_heave, dof_sway, dof_surge, dof_tloss;
  static float V[3], Q[4], Vp[3] = {0, 0, 0};//, ptl = 0.0f; //and prev traction loss for accel compute
  //
  //int sidx = debug_data;//4, 6;
  //
  //if((tpkt->flags & 0x000A) == 0 && (tpkt->flags & 0x0081)) // exclude motion during replay
  dt = cltime - pltime;//using seconds
  pltime = cltime;
  if (dt > 0.1f) //don't use this one since it is old data
  {
    //save last values
    //pvx = tpkt->vx; pvy = tpkt->vy; pvz = tpkt->vz;
    if (1 || _odbg)
      printf ("\n#i@%.3f:00fvals 0x%04x dt %f dir %f val %f %f %f mps %f", cltime, tpkt->flags,
        dt, tpkt->rorin, tpkt->vx, tpkt->vy, tpkt->vz, tpkt->mps);
    dt = 0;
  }
  /*
-game menu
#i:pkt:0x0000 / 65535 rpm 7350.000000 gear 07 spd 302.793823 rpy -0.006580 0.016017 -0.030721
-loading
#i:pkt:0x0198 / 1 rpm 5941.000000 gear 05 spd 188.704025 rpy 0.109757 0.088692 0.047300
-race menu
#i:pkt:0x01b8 / 1 rpm 7329.000000 gear 07 spd 301.951691 rpy 0.062090 0.035818 0.165117
#i:pkt@115.196/0.018 0x0098 / 20 rpm 7185.000000 gear 05 spd 165.724808 rpy -0.025166 -0.048868 -0.007500
#i:pkt@5.994/0.017 0x00b8 / 20 rpm 7614.000000 gear 04 spd 141.511597 rpy -0.007178 -0.295699 0.080580
-race countdown
#i:pkt:0x019b / 65535 rpm 5560.000000 gear 04 spd 155.562698 rpy 0.070308 -0.026675 0.156877
-racing
#i:pkt:0x0199 / 65535 rpm 3228.000000 gear 04 spd 92.303497 rpy 0.067701 0.016735 0.014449
#i:pkt@239.942/0.017 0x0189 / 65535 rpm 5230.000000 gear 04 spd 229.976669 rpy 0.011394 0.015060 0.028492
#i:pkt@71.751/0.018 0x0009 / 65535 rpm 3293.000000 gear 01 spd 2.500520 rpy -0.073604 0.474158 -0.536907
-pits
#i:pkt@112.920/0.016 0x0091 / 65535 rpm 1023.000000 gear 01 spd 0.000000 rpy -0.000000 0.000039 -0.000004
-replay
#i:pkt@13.338/0.017 0x0008 / 65535 rpm 8148.541504 gear 01 spd 117.605621 rpy 0.012991 -0.495956 -0.033759

  */
  if (1 ||  !(tpkt->flags & 0x2))
    printf ("\n#i:pkt@%.3f/%.3f 0x%04x / %d rpm %f gear %02x spd %f rpy %f %f %f",
      cltime, dt, tpkt->flags, tpkt->spos, tpkt->rpm, tpkt->gear&0xf, tpkt->mps * 3.6f, tpkt->ax, tpkt->ay, tpkt->az);
  //
  #define is_set_bit(a, b) ((a&0xf)&(1<<b))
  #define bit_set(a, b) ((a&0xf)&(b))
  #define gt7_running 0x8 //also on replay
  #define gt7_racing  0x1 //only when driving
  #define gt7_autorun 0x2 //auto drive, no control
  #define gt7_race_running(flg) (bit_set(flg, 0x08) && bit_set(flg, 0x01) && !bit_set(flg, 0x02))
  //if(!(tpkt->flags & 0x0006) && (tpkt->flags & 0x0001) && dt) // exclude motion during replay or menu
  if(/*dt && */gt7_race_running(tpkt->flags)) // exclude motion during replay or menu
  {
    /*
    // https://stackoverflow.com/questions/20615962/2d-world-velocity-to-2d-local-velocity
    // get the heading angle in radians:
    float hf = atan2f(tpkt->vx, tpkt->vz); //frontal
    float hr = atan2f(tpkt->vz, tpkt->vx); //lateral
    float hv = atan2f(tpkt->vy, tpkt->vz); //vertical
    // get the longitudinal acceleration: surge
    float fwa = (tpkt->vx - pvx) / dt;
    // get the lateral acceleration: sway
    float rta = (tpkt->vz - pvz) / dt;
    // get the vertical acceleration: heave
    float vta = (tpkt->vy - pvy) / dt;
    //
    float fa =   sin(hf) * fwa + cos(hf) * rta;  // forward acceleration: surge
    float ra =  -sin(hr) * fwa + cos(hr) * rta;  // lateral acceleration: sway
    float va =  -cos(hv) * vta;  // vertical acceleration: heave
    //
    float fwv =  sin(hf) * tpkt->vx + cos(hf) * tpkt->vz;
    float rtv =  cos(hf) * tpkt->vx - sin(hf) * tpkt->vz;
    //Car Slip Angle = - arctan ( Car_Velocity_Right / | Car_velocity_Forward | ) -or- https://en.wikipedia.org/wiki/Slip_angle
    float da =  -atan2f (abs(rtv + fwv), rtv); //-atanf(fwv / abs(rtv)); // lateral drift: traction loss? 
    //save last values
    pvx = tpkt->vx; pvy = tpkt->vy; pvz = tpkt->vz;
    if (0 || _odbg)
      printf ("\n#i@%.3f:fvals 0x%04x dt %f wr %f %f %f > %f " \
        "wv %f %f %f av %f %f %f " \
        "dval %f %f %f %f %f ",
        cltime, tpkt->flags, dt, tpkt->rx, tpkt->ry, tpkt->rz, hf,
        tpkt->vx, tpkt->vy, tpkt->vz, tpkt->ax, tpkt->ay, tpkt->az,
        fa, ra, tpkt->mps, fwv, rtv);
    */
    /*
            V = struct.unpack_from('fff',ddata,0x10)
            print('Global Velocity',V)
            Q = struct.unpack_from('ffff',ddata,0x1C)
            Qc = quat_conj(Q)
            Vl = quat_rot(V,Qc)
            print("Local Velocity:",Vl)
            dV = sub(Vl,Vp)
            A = scale(dV,60)
            print("Acceleration:",A)
            G = scale(A,1.0/9.81)
            print("G-Forces:",G)
            Vp = Vl # save current local velocity for next packet
    */
   // https://www.gtplanet.net/forum/threads/gt7-is-compatible-with-motion-rig.410728/post-13829730
    V[0] = tpkt->vx; V[1] = tpkt->vy; V[2] = tpkt->vz;
    Q[0] = tpkt->rx; Q[1] = tpkt->ry; Q[2] = tpkt->rz; Q[3] = tpkt->rorin;
    float *Qc = quat_conj(Q);
    float *Vl = quat_rot(V, Qc);  //local velocity
    float *dV = sub(Vl, Vp);
    float *A  = scale(dV, 60.0f); //local acceleration: 60pkts/s
    float *G  = scale(A, 1.0f/9.81f); //G forces
    Vp[0] = Vl[0]; Vp[1] = Vl[1]; Vp[2] = Vl[2];//save current local velocity for next packet
    //
    //EulerAngles *ea = ToEulerAngles(tpkt->rorin, -tpkt->rx, -tpkt->rz, -tpkt->ry);
    Q[0] = tpkt->rx; Q[1] = tpkt->ry; Q[2] = tpkt->rz; Q[3] = tpkt->rorin;
    float *rpy = quats_to_rpy(Q); //quaternions to roll/pitch/yaw
    //
    dof_pitch = rpy[1] * RAD2DEG;//MATH_PI * tpkt->rx * RAD2DEG; //raw rotation rx is -1..1 and needs PI to transform to radians
    dof_surge = -G[2];//fa / GRAVACCEL; //from local acc to G
    dof_sway  = G[0];//ra / GRAVACCEL; //from local acc to G
    dof_heave = G[1];//va / GRAVACCEL; //from local acc to G
    dof_roll  = -rpy[0] * RAD2DEG;//MATH_PI * tpkt->rz * RAD2DEG;
    dof_yaw   = 0;
    dof_tloss = (Vl[2]==0.0f)?0.0f:-(RAD2DEG * atanf(Vl[0] / abs(Vl[2])));//(MATH_PI * tpkt->ry) * RAD2DEG; //(Vl[2]==0.0f)?0.0f:-(RAD2DEG * atanf(Vl[0] / abs(Vl[2])));//da; // / GRAVACCEL
    //
    if (0 || _odbg)
      printf ("\n#i@%.3f:t0 0x%04x pitch %f %f %f roll %f %f yaw %f %f", cltime, tpkt->flags,
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
    _cpkt[MFC_PIPITCH] = get_cmap_f (_cpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], MFC_POS_MIN, MFC_HPOS_MAX);
    _cpkt[MFC_PISURGE] = get_cmap_f (_cpkt[MFC_PISURGE], mpkt[MFC_PISURGE], Mpkt[MFC_PISURGE], MFC_POS_MIN, MFC_HPOS_MAX);
    _cpkt[MFC_PIHEAVE] = get_cmap_f(_cpkt[MFC_PIHEAVE], mpkt[MFC_PIHEAVE], Mpkt[MFC_PIHEAVE], MFC_POS_MIN, MFC_HPOS_MAX);
    //roll
    _cpkt[MFC_PIROLL]  = get_cmap_f (_cpkt[MFC_PIROLL],  mpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL], MFC_POS_MIN, MFC_HPOS_MAX);
    _cpkt[MFC_PISWAY]  = get_cmap_f (_cpkt[MFC_PISWAY],  mpkt[MFC_PISWAY],  Mpkt[MFC_PISWAY], MFC_POS_MIN, MFC_HPOS_MAX);
    //yaw
    _cpkt[MFC_PIYAW]   = get_cmap_f (_cpkt[MFC_PIYAW],   mpkt[MFC_PIYAW],   Mpkt[MFC_PIYAW], MFC_POS_MIN, MFC_HPOS_MAX);
    _cpkt[MFC_PITLOSS] = get_cmap_f (_cpkt[MFC_PITLOSS], mpkt[MFC_PITLOSS], Mpkt[MFC_PITLOSS], MFC_POS_MIN, MFC_HPOS_MAX);
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
  }
  else
  {
    //park platform
    if (0 && !_learn)
    {
      goto_park(_cpkt);
      mfc_bcast_send ();
    }
    //don't process these packets
    if (0)
      printf ("\n#i:drop pkt:%04x / %d rpm %f gear %02x spd %f rpy %f %f %f",
      tpkt->flags, tpkt->spos, (float)tpkt->rpm, tpkt->gear&0xf, tpkt->mps * 3.6f, tpkt->ax, tpkt->ay, tpkt->az);
  }
  //send dash data even when learning
  if (_dashaddr)
  {
    //dash data
    _dpkt[MFC_DISPD]  = (int)(tpkt->mps * 2.24f);  //speed: 1 m/s, 3.60 kph - 1 m/s, 2.24 mph
    _dpkt[MFC_DIGEAR] = (int)tpkt->gear & 0xf;  //gear
    _dpkt[MFC_DIRPM]  = (int)tpkt->rpm;  //rpm x10
    _dpkt[MFC_DIRPMM] = (int)tpkt->Mrpm; //max rpm x10
    if (0)
      printf ("\n#i@%.3f:d1 rpm %d rpmm %d rpmM %d", cltime, _dpkt[MFC_DIRPM], _dpkt[MFC_DIRPMM], tpkt->Mrpm);
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
char *pp_ps = NULL;//"192.168.0.44";
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
    { "debug-data",required_argument, 0, 'D' },
    { "motion-ip", required_argument, 0, 'm' },
    { 0, 0, 0, 0 }
  };

  while (1)
  {
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "r:p:y:s:w:h:t:a:c:d:g:D:m:lhV?", long_options, &option_index);

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
    case 'g': //listen port
      pp_ps = optarg;
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
  printf ("\n#MFC Gran Turismo 7 client");
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
  printf ("\n#    game address %s (-g <ipv4>)", pp_ps?pp_ps:"<n/a>");
  printf ("\n# verbosity level %d (-d %d)", _odbg, _odbg);
  printf ("\n#     client port %d (-c %d)", _lport, _lport);
  if (_dashaddr)
    printf ("\n#            dash %s (-a %s)", _dashaddr, _dashaddr);
  else
    printf ("\n#            dash <n/a> (-a <ipv4>)");
  printf ("\n#motion server IP %s (-m %s)", _bcastaddr?_bcastaddr:"127.0.0.1", _bcastaddr?_bcastaddr:"127.0.0.1");
  printf ("\n# ##");
  //printf ("\n-- sizeof float %d", sizeof(float));
  printf ("\n-- debug data %d", debug_data);
  //
  return 1;
}

//https://github.com/alexwebr/salsa20
#include "salsa20.h"
int do_decrypt(char *buf, int blen)
{
  uint8_t *key = (uint8_t *)"Simulator Interface Packet GT7 ver 0.0";
  uint8_t nonce[8];
  uint32_t si = 0;
  uint32_t iv1, iv2;
  memcpy(&iv1, buf+0x40, 4);
  iv2 = iv1 ^ 0xDEADBEAF;
  memcpy(nonce, &iv2, 4);
  memcpy(nonce+4, &iv1, 4);
  s20_crypt(key, S20_KEYLEN_256, nonce, si, (uint8_t *)buf, blen);
  uint32_t *magic = (uint32_t *)buf;
  //printf("\n#i:got magic 0x%x", *magic);
  if (*magic != 0x47375330)
    return 0;
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
  int bs = sendto (s, "A", 1, 0, (struct sockaddr*)si_other, sizeof(struct sockaddr));
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
  if (pp_ps == NULL)
  {
    usage(argv[0]);
    exit (1);
  }
  if (0)
  {
    printf("\npkt data size %d\n", sizeof(struct gt7_rd));
    exit (1);
  }
  //
  int cs = mfc_bcast_prep (_bcastaddr?_bcastaddr:"127.0.0.1", 0); //mfc_bcast_prep ("192.168.0.24", 0);
  if (cs < 3)
  {
    printf ("\n#e:can't connect to MFC server on port %d", MFCSVR_PORT);
    exit(1);
  }
  printf ("\n#i:<%d:MFC server at %s:%d", cs, _bcastaddr?_bcastaddr:"127.0.0.1", MFCSVR_PORT);
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
  si_me.sin_port = htons(_lport + 1);
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
  if (inet_aton (pp_ps, &si_other.sin_addr) == 0)
  {
    fprintf (stderr, "inet_aton() for <%s> failed\n", pp_ps);
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
        {
          do_register (s, &si_other);
          printf ("\n#i:received %dpkts", ppkt);
        }
        if (0)
          printf("\r\n@%.3f received %dB packet (vs %d) from %s:%d <", 
                  cltime, rlen, UDP_MAX_PACKETSIZE, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        //
        if (do_decrypt (packetBuffer, rlen))
        {
          struct gt7_rd *pkt = (struct gt7_rd *)packetBuffer;
          if (0 && pkt->flags & 0x2) //0x2 is paused
          {
            //game paused
          }
          else
          {
            if (0 && !(pkt->flags & 0x2))
              printf ("\n#i:pkt:%04x / %d rpm %f gear %02x spd %f rpy %f %f %f", pkt->flags, pkt->spos, pkt->rpm, pkt->gear&0xf, pkt->mps * 3.6f, pkt->ax, pkt->ay, pkt->az);
            //printf ("\n#i:pkt:%c / %d rpm %f gear %d spd %f %f %f", (char)get_int (packetBuffer, 0), get_int (packetBuffer, 4), 
            //  get_float (packetBuffer, 68), get_int (packetBuffer, 76), get_float (packetBuffer, 8), get_float (packetBuffer, 12), get_float (packetBuffer, 16));
            //printf ("\n#i:ffb:vert %f hori %f long%f", get_float (packetBuffer, 28), get_float (packetBuffer, 32), get_float (packetBuffer, 36));
            //
            mfc_packet_use (packetBuffer, cltime, cltime);
          }
        }
      }
    }
    fflush (stdout);
  }
  //
  printf("\n#i:cleaning up.. ");
  //
  close (s);
  mfc_bcast_close ();
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
  //
  printf("\n#i:done.\n");
  return 0;
}
