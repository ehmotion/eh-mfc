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

struct wrcg_rd
{
  float m_time;
  //Total	Time	(not	reset	after	stage	restart)
  float m_lapTime;
  //Current	Lap/Stage	Time	(starts	on	Go!)
  float m_lapDistance;
  //Current	Lap/Stage	Distance	(meters)
  float m_totalDistance;
  //?	(starts	from	0)	-0	if	distance	then	not	equal	to	above!
  float m_x; //	World	space	position
  //Position	X
  float m_y; //	World	space	position
  //Position	Y
  float m_z; //	World	space	position
  //Position	Z
  float m_speed;
  //Velocity	(Speed)	[m/s]
  float m_xv; //	Velocity	in	world	space
  //Velocity	X
  float m_yv; //	Velocity	in	world	space
  //Velocity	Y
  float m_zv; //	Velocity	in	world	space
  //Velocity	Z
  float m_xr; //	World	space	right	direction
  //Roll	Vector	X
  float m_yr; //	World	space	right	direction
  //Roll	Vector	Y
  float m_zr; //	World	space	right	direction
  //Roll	Vector
  //Z	Pitch	Vector	X	Pitch	Vector	Y	Pitch	Vector	Z
  float m_xd; //	World	14	space	forward	direction
  float m_yd; //	World	15	space	forward	direction
  float m_zd; //	World	16	space	forward	direction
  //Position	of	Suspension	Rear	Left	
  //Position	of	Suspension	Rear	Right	
  //Position	of	Suspension	Front	Left	
  //Position	of	Suspension	Front	Right	
  float m_susp_pos_bl;
  float m_susp_pos_br;
  float m_susp_pos_fl;
  float m_susp_pos_fr;
  //Velocity	of	Suspension	Rear	Left	
  //Velocity	of	Suspension	Rear	Right	
  //Velocity	of	Suspension	Front	Left	
  //Velocity	of	Suspension	Front	Right	
  float m_susp_vel_bl;
  float m_susp_vel_br;
  float m_susp_vel_fl;
  float m_susp_vel_fr;
  //Velocity	of	Wheel	Rear	Left	
  //Velocity	of	Wheel	Rear	Right	
  //Velocity	of	Wheel	Front	Left	
  //Velocity	of	Wheel	Front	Right	
  float m_wheel_speed_bl;
  float m_wheel_speed_br;
  float m_wheel_speed_fl;
  float m_wheel_speed_fr;
  //Position	Throttle
  float m_throttle;
  //Position	Steer
  float m_steer;
  //Position	Brake	
  float m_brake;
  //Position	Clutch
  float m_clutch;
  //Gear	[0	=	Neutral,	1	=	1,	2	=	2,	...,	-1	=	Reverse]
  float m_gear;
  //G-Force	Lateral	G-Force	Longitudinal	Current	Lap	(rx	only)	Engine	Speed	[rpm	/	10]
  float m_gforce_lat;
  float m_gforce_lon;
  float m_lap;
  float m_engineRate;
  float m_sli_pro_native_support;
  //	SLI	Pro	support
  float m_car_position;       //	car	race	position
                              //			Current	Position	(rx	only)
  float m_kers_level;         //	kers	energy	left
  float m_kers_max_level;     //	kers	maximum	energy
  float m_drs;                //	0	=	off,	1	=	on
  float m_traction_control;   //	0	(off)	-0	2	(high)
  float m_anti_lock_brakes;   //	0	(off)	-0	1	(on)
  float m_fuel_in_tank;       //	current	fuel	mass
  float m_fuel_capacity;      //	fuel	capacity
  float m_in_pits;            //	0	=	none,	1	=	pitting,	2	=	in	pit	area
  float m_sector;             //	0	=	sector1,	1	=	sector2;	2	=	sector3
  float m_sector1_time;       //	time	of	sector1	(or	0)
  float m_sector2_time;       //	time	of	sector2	(or	0)
  float m_brakes_temp[4];     //	brakes	temperature	(centigrade)
  float m_wheels_pressure[4]; //	wheels	pressure	PSI
  float m_team_info;          //	team	ID
  float m_total_laps;         //	total	number	of	laps	in	this	race
  float m_track_size;         //track	size	meters
  float m_last_lap_time;      //	last	lap	time
  float m_max_rpm;            //	cars	max	RPM,	at	which	point	the	rev	limiter	will	kick	in
  float m_idle_rpm;           //	cars	idle	RPM
  float m_max_gears;          //	maximum	number	of	gears
  float m_sessionType;        //	0	=	unknown,	1	=	practice,	2	=	qualifying,	3	=	race
  float m_drsAllowed;         //	0	=	not	allowed,	1	=	allowed,	-1	=	invalid	/	unknown
  float m_track_number;       //	-1	for	unknown,	0-21	for	tracks
  float m_vehicleFIAFlags;    //	-1	=	invalid/unknown,	0	=	none,	1	=	green,	2	=	blue,	3	=	yellow,	4	=	red
};

#define PKTSZ 280

#define UDP_MAX_PACKETSIZE  PKTSZ
#define UDP_PORT            20778

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
char *_dashaddr = NULL; //virtual dash ui
char *_dashport = NULL; //arduino serial dash
int _dashportfd = -1;
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
float _pitchprc = 15.0f;
float _rollprc  = -50.0f;
float _yawprc   = 0.0f;
float _surgeprc = 50.0f;
float _swayprc  = 50.0f;
float _heaveprc = 25.0f;
float _trlossprc= 75.0f;
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

/*
https://answers.unity.com/questions/228203/getting-vector-which-is-pointing-to-the-rightleft.html
In a 2D space - Vector2, or Vector3 with one coordinate fixed at 0 - you can get 
the right and left vectors with simple transformations:

  // original vector in the xz plane (y = 0):
  var forward: Vector3 = Vector3(0.5, 0.0, 0.8);
  // transformed vectors right and left:
  var right = Vector3(forward.z, forward.y, -forward.x);
  var left = -right;
--
For a full 3D vector, however, there's no predefined left/right directions - you must 
set some reference, usually the up direction. Having the up direction, you can calculate 
the right and left vectors using a cross product:

  // original vector:
  var forward: Vector3 = Vector3(0.5, 0.7, 0.8);
  // up direction:
  var up: Vector3 = Vector3(0.0, 1.0, 0.0);
  // find right vector:
  var right = Vector3.Cross(forward.normalized, up.normalized);
  var left = -right;

--
https://gamedev.stackexchange.com/questions/121654/getting-the-right-vector-from-the-forward-vector
You can't get the "right" vector from just a "forward" vector. Any particular "forward" vector 
could have an infinite number of different legal "up" and "right" vectors.

For example, if I am looking forward along the z axis forwardVector = vec3(0,0,1), then 
I could have up be along the y axis upVector = vec3(0,1,0) and right therefore be along 
the x axis rightVector = vec3(1,0,0), or I could have up be along the -y axis 
upVector = vec3(0,-1,0) and therefore right would be along the -x axis 
rightVector = vec3(-1,0,0). Or I could even have 'up' be along the x axis 
upVector = vec3(1,0,0), which would mean that right is along the y axis 
rightVector = vec3(0,1,0). And so forth. No one "up" or "right" vector can be 
considered "correct" if all you have is a forward vector, without imposing some 
extra constraints on the system.

One common constraint to add to this kind of system is that "up" should be pointing 
as close as possible to some objective "up" in the scene. Lots of 3D platform games do this, 
for example. In these, you can calculate your "right" vector by taking the cross product of 
the forward vector against vec3(0,1,0) (or whatever your world-space 'up' is), and then 
the cross product of the right vector against the forward vector.

*/

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

//To calculate the angle between two 3d coordinates, in degrees you can use this CalcAngle
typedef struct
{
    float x, y, z;
} vec3;

vec3 Subtract(vec3 src, vec3 dst)
{
    vec3 diff;
    diff.x = src.x - dst.x;
    diff.y = src.y - dst.y;
    diff.z = src.z - dst.z;
    return diff;
}

float Magnitude(vec3 vec)
{
    return sqrtf(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

float Distance(vec3 src, vec3 dst)
{
    vec3 diff = Subtract(src, dst);
    return Magnitude(diff);
}

vec3 CalcAngle(vec3 src, vec3 dst)
{
    vec3 angle;
    angle.x = -atan2f(dst.x - src.x, dst.y - src.y) / M_PI * 180.0f + 180.0f;
    angle.y = asinf((dst.z - src.z) / Distance(src, dst)) * 180.0f / M_PI;
    angle.z = 0.0f;

    return angle;
}

float getLVel4Accel(float acc, float plvel)
{
  //acc = (cv - pv) * dt
  //cv = acc / dt + pv
  return  acc / 60.0f + plvel;
}

void susp_travel_comp(float ts, float tm, float sl, float sr, float *sffbroll, float *sffbheave)
{
  static float phv = 0.0f;
  if (0 || _odbg)
    printf ("\n#i@%.3f:tr %f susp %f %f ", ts, tm, sl, sr);
  if (sl != sr)
  {
    //roll
    *sffbroll = sr - sl;
    *sffbroll *= 10.f;
    //printf ("\n#i@%.3f:tr %f susp %f %f ROLL %f", ts, tm, sl, sr, *sffbroll);
  }
  else
  {
    //heave
    *sffbheave = phv - sr;
    //printf ("\n#i@%.3f:tr %f susp %f %f HEAVE %f", ts, tm, sl, sr, phv - sr);
    phv = sr;
  }
}
//learning mode
//static int Mpkt[] = {0, 0,  4735,  1152,  3730, 100324, 1536, 156726, 90000, 100, 100, 100};
//static int mpkt[] = {0, 0, -5844, -1243, -4024, 0,     -1159, 0,     -28970, 0, 0, 0};
//                          pit    srg    hev    rol    sway   yaw      tl
static int Mpkt[] = {0, 0,  2204,  1200,  1500,  105000, 1534,  150000,  60000,  100, 100, 100};
static int mpkt[] = {0, 0, -2200, -1240, -1525,  75000, -1559,  30000,  -60000, 0, 0, 0};

int mfc_packet_use(char* packetBuffer, float rtime, float cltime)
{
  struct wrcg_rd *tpkt = (struct wrcg_rd *)packetBuffer;
  static long pktk = 0;
  //float *fpkt = (float*)packetBuffer;
  //static float pvx = 0.0f, pvy = 0.0f, pvz = 0.0f, dt = 0.0f;//previous values for vx, vy, vz
  static float pltime = 0.0f, dt = 0.0f, crtime = 1.0f;
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
  //
  //int sidx = debug_data;//4, 6;
  //
  //if((tpkt->flags & 0x000A) == 0 && (tpkt->flags & 0x0081)) // exclude motion during replay
  pktk++;
  dt = cltime - pltime;//using seconds
  pltime = cltime;
  if (dt > 0.1f) //don't use this one since it is old data
  {
    //save last values
    //pvx = tpkt->vx; pvy = tpkt->vy; pvz = tpkt->vz;
    if (0 || _odbg)
      printf ("\n#i@%.3f:00fvals %f dt %f dir %f val %f %f %f mps %f", cltime, tpkt->m_lapTime,
        dt, tpkt->m_x, tpkt->m_xv, tpkt->m_yv, tpkt->m_zv, tpkt->m_speed);
    dt = 0;
  }
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
  #if 0
  static float V[3], Q[4], Vp[3] = {0, 0, 0};//, ptl = 0.0f; //and prev traction loss for accel compute
  V[0] = tpkt->m_xv; V[1] = tpkt->m_yv; V[2] = tpkt->m_zv;
  Q[0] = tpkt->m_xr; Q[1] = tpkt->m_yr; Q[2] = tpkt->m_zr; Q[3] = 0;//tpkt->rorin;
  float *Qc = quat_conj(Q);
  float *Vl = quat_rot(V, Qc);  //local velocity
  float *dV = sub(Vl, Vp);
  float *A  = scale(dV, 60.0f); //local acceleration: 60pkts/s
  float *G  = scale(A, 1.0f/9.81f); //G forces
  Vp[0] = Vl[0]; Vp[1] = Vl[1]; Vp[2] = Vl[2];//save current local velocity for next packet
  //
  //EulerAngles *ea = ToEulerAngles(tpkt->rorin, -tpkt->rx, -tpkt->rz, -tpkt->ry);
  Q[0] = tpkt->m_xr; Q[1] = tpkt->m_yr; Q[2] = tpkt->m_zr; Q[3] = 0;//tpkt->rorin;
  float *rpy = quats_to_rpy(Q); //quaternions to roll/pitch/yaw
  //
  #endif
  //only when racing
  //static float phv = 0.0f; //heave - for road detail
  //static float pstl = 0.0f, pstr = 0.0f; //front suspention travel - for road detail - not used
  static float prlv = 0.0f;
  static float pflv = 0.0f;
  if (tpkt->m_lapTime == 0.0f)
  {
    //reset previous values as we start new race
    //pstl = pstr = 0.0f;//tpkt->m_susp_pos_fl;
    prlv = 0.0f;
    pflv = 0.0f;
    //pktk = 0;
  }
  if (1 || (tpkt->m_lapTime > 0.0f && crtime != tpkt->m_lapTime))//time doesn't advance beyond stage end
  {
    if (tpkt->m_lapTime > 0.0f)
      crtime = tpkt->m_lapTime;
    float angs[3];
    float fwd[3] = {tpkt->m_xd, tpkt->m_yd, tpkt->m_zd};
    float rgt[3] = {tpkt->m_xr, tpkt->m_yr, tpkt->m_zr};
    anglesFromVectorsFwUp(angs, fwd, rgt);
    float gf[3];
    float vel[] = {tpkt->m_xv, tpkt->m_yv, tpkt->m_zv};
    gforceFromVelocity(gf, vel);
    //suspention travel for road texture
    //float stdt = tpkt->m_susp_vel_fr + tpkt->m_susp_vel_fl - pst;
    //float stdt = 7.5f*fmodf(pst - tpkt->m_susp_pos_fr - tpkt->m_susp_pos_fl, 1.0f);
    //road feedback based on suspention travel - added to heave
    float stfbroll = 0.0f;
    float stfbheave = 0.0f;
    if (!_learn)
    {
      susp_travel_comp(cltime, tpkt->m_lapTime, tpkt->m_susp_pos_fl, tpkt->m_susp_pos_fr, &stfbroll, &stfbheave);
    }
    //yaw or traction loss: accel / pkts-per-sec + previous velocity
    float rlv = tpkt->m_gforce_lat / 60.0f + prlv; //getLVel4Accel(tpkt->m_gforce_lat, prlv);
    prlv = rlv;
    float flv = tpkt->m_gforce_lon / 60.0f + pflv;//getLVel4Accel(tpkt->m_gforce_lon, pflv);
    pflv = flv;
    /*
    float dhv = -gf[ZZ] - phv;
    phv = -gf[ZZ];//previous heave
    */
    //dofs
    dof_pitch = angs[PITCH];//rpy[1] * RAD2DEG;//MATH_PI * tpkt->rx * RAD2DEG; //raw rotation rx is -1..1 and needs PI to transform to radians
    dof_surge = -tpkt->m_gforce_lon * (1.0f/9.81f);//gf[XX];//-G[1];//fa / GRAVACCEL; //from local acc to G
    dof_sway  = tpkt->m_gforce_lat * (1.0f/9.81f) + stfbroll;// + mfc_ewmaf(MFC_PIROLL, (int)stfbroll, 1.f);//gf[YY] * 100000.0f;//G[0];//ra / GRAVACCEL; //from local acc to G
    dof_heave = -gf[ZZ];// + mfc_ewmaf(MFC_PIHEAVE, (int)stdt, 0.3f);//dhv * 1.0f;
    dof_roll  = angs[ROLL];// + stdt;//angs[ROLL];//-rpy[0] * RAD2DEG;//MATH_PI * tpkt->rz * RAD2DEG;
    dof_yaw   = -angs[YAW];//(Vl[2]==0.0f)?0.0f:-(RAD2DEG * atanf(Vl[0] / abs(Vl[2])));//(MATH_PI * tpkt->ry) * RAD2DEG;
    if (tpkt->m_lapTime > 0.0f && crtime != tpkt->m_lapTime) 
      dof_tloss = (flv==0.0f)?0.0f:-(RAD2DEG * atanf(rlv / abs(flv)));
    else
      dof_tloss = 0;
    //
    if (0 || _odbg)
      printf ("\n#i@%.3f:tr %f fwd %f %f %f rgt %f %f %f ang %f %f %f", cltime, tpkt->m_lapTime,
        tpkt->m_xd, tpkt->m_yd, tpkt->m_zd, tpkt->m_xr, tpkt->m_yr, tpkt->m_zr, angs[ROLL], angs[PITCH], angs[YAW]);
    if (0 || _odbg)
      printf ("\n#i@%.3f:tr %f vel %f %f %f gf %f %f %f", cltime, tpkt->m_lapTime,
        tpkt->m_xv, tpkt->m_yv, tpkt->m_zv, gf[ROLL], gf[PITCH], gf[YAW]);
    if (0 || _odbg)
      printf ("\n#i@%.3f:t0 %f pitch %f %f %f roll %f %f yaw %f %f", cltime, tpkt->m_lapTime,
        dof_pitch, dof_surge, dof_heave, dof_roll, dof_sway, dof_yaw, dof_tloss);

    _cpkt[MFC_PIPITCH] = (int)(dof_pitch * DOF_MAG);
    _cpkt[MFC_PISURGE] = (int)(dof_surge * DOF_MAG);
    _cpkt[MFC_PIHEAVE] = (int)(dof_heave * DOF_MAG);
    _cpkt[MFC_PIROLL]  = (int)(dof_roll * DOF_MAG);
    _cpkt[MFC_PISWAY]  = (int)(dof_sway * DOF_MAG);
    _cpkt[MFC_PIYAW]   = (int)(dof_yaw * DOF_MAG);
    _cpkt[MFC_PITLOSS] = (int)(dof_tloss * DOF_MAG);
    _cpkt[MFC_PITLOSS] = mfc_ewmaf(MFC_PITLOSS, _cpkt[MFC_PITLOSS], 0.1);
    //
    if (0 || _odbg)
      printf ("\n%.3f, %5d, %5d, %5d, %5d, %5d, %5d, %5d", cltime,
        _cpkt[MFC_PIPITCH], _cpkt[MFC_PISURGE], _cpkt[MFC_PIHEAVE],
        _cpkt[MFC_PIROLL], _cpkt[MFC_PISWAY], 
        _cpkt[MFC_PIYAW], _cpkt[MFC_PITLOSS]);
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
  }
  //send dash data even when learning
  if (_dashaddr || _dashport)
  {
    //dash data
    _dpkt[MFC_DISPD]  = (int)(tpkt->m_speed * 2.24f);  //speed: 1 m/s, 3.60 kph - 1 m/s, 2.24 mph
    _dpkt[MFC_DIGEAR] = (int)tpkt->m_gear;  //gear
    _dpkt[MFC_DIRPM]  = (int)tpkt->m_engineRate * 10;  //rpm x10
    _dpkt[MFC_DIRPMM] = (int)tpkt->m_max_rpm * 10 - 1000; //max rpm x10
    //printf ("\n#i@%.3f:d1 rpm %d rpmm %d rpmM %d", cltime, _dpkt[MFC_DIRPM], tpkt->mrpm, tpkt->Mrpm);
    memcpy(_dpkt, _cpkt, pktl);
    if (_dashaddr)
      mfcdash_bcast_send ();
    if (_dashportfd > 0 && pktk%5 == 0)
    {
      //write(_dashportfd, "!!", 2);
      //write(_dashportfd, (char *)_dpkt, mfcdash_bcast_pktlen());
      //simdash style
      static char ldbuf[10] = {0x03, 'P','N',';','1','\n'};
      switch(_dpkt[MFC_DIGEAR])
      {
        case -1:
          ldbuf[2] = 'R';
          break;
        case 0:
          ldbuf[2] = 'N';
          break;
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
          ldbuf[2] = '0' + _dpkt[MFC_DIGEAR];
          break;
        default:
          ldbuf[2] = '9';
          break;
      }
      ldbuf[4] = '0' + (int)(_dpkt[MFC_DIRPM] / 1000);
      sh_serial_write(_dashportfd, ldbuf, 6);
    }
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
    { "debug-data",  required_argument, 0, 'D' },
    { "motion-ip",   required_argument, 0, 'm' },
    { "simhub-dash1",required_argument, 0, 'b' },
    { 0, 0, 0, 0 }
  };

  while (1)
  {
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "r:p:y:s:w:h:t:a:c:d:D:m:b:lhV?", long_options, &option_index);

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
    case 'b': //use arduino serial dash to forward data
      _dashport = optarg;
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
  printf ("\n#MFC WRC Generations client");
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
  printf ("\n#serial port dash %s (-b %s)", _dashport?_dashport:"n/a", _dashport?_dashport:"/dev/ttyUSBx");
  printf ("\n# ##");
  //printf ("\n-- sizeof float %d", sizeof(float));
  printf ("\n-- debug data %d", debug_data);
  //
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
#include <limits.h>
#include <termios.h>
int main (int argc, char **argv, char **envp)
{
  struct pollfd fdset[3];
  int nfds = 1;
  int timeout, rc;
  //struct RTCarInfo *tpkt;
  //unsigned int gpio;

  env_init (argc, argv);
  if (1)
  {
    printf("\n#i:pkt data size %d", sizeof(struct wrcg_rd));
    if (PKTSZ != sizeof(struct wrcg_rd))
    {
      printf(" differs from %d\n", PKTSZ);
      exit (1);
    }
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
#define POLL_TIMEOUT 3000 //use 4500 for simdash keepalive
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
  //
  printf ("\n#i:>%d:listening on port %d", s, _lport);
  //serial port dash
  if (_dashport)
  {
    //B19200 for simhub dash custom proto
    //B500000 for others maybe
    _dashportfd = serial_dash_open(_dashport, B19200);
  }
  //setup internal timer ms
  ctime_ms (0);
  //learning values
  float cltime = -1.0f;
  //only send 3 PAUSEd packets
  (void) signal(SIGINT, terminate);
  (void) signal(SIGTERM, terminate);
  (void) signal(SIGHUP, terminate);
  //
  //mfc_ewmaf_set(MFC_PITLOSS, 0);
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
  if (_dashaddr||_dashport)
  {
    memcpy(_dpkt, _cpkt, pktl);
    //set sim/app name
    snprintf((char *)(_dpkt + MFC_DIAPP14), MFC_DIAPPSZ, "WRCG");
    mfcdash_bcast_send ();
    if (_dashportfd > 0)
    {
      _dpkt[MFC_DIGEAR] = 0;
      _dpkt[MFC_DISPD]  = 0;
      _dpkt[MFC_DIRPM]  = 855;
      //dash_serial_write(_dpkt, mfcdash_bcast_pktlen());
      //simdash style
      char ldbuf[10] = {0x03, 'P','N',';','1','\n'};
      sh_serial_write(_dashportfd, ldbuf, 6);
    }
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
      if (_dashportfd > 0)
      {
        //simdash style - keepalive
        char ldbuf[10] = {0x03, 'P','N',';','1','\n'};
        sh_serial_write(_dashportfd, ldbuf, 6);
      }
      sleep (1);
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
          printf ("\n#i:received %dpkts", ppkt);
        }
        if (0)
          printf("\r\n@%.3f received %dB packet (vs %d) from %s:%d <", 
                  cltime, rlen, UDP_MAX_PACKETSIZE, inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        //
        if (rlen == 280)
        {
          if (0)
          {
            struct wrcg_rd *pkt = (struct wrcg_rd *)packetBuffer;
            printf ("\n#i:pkt:%f rpm %f gear %d spd %f rpy %f %f %f", pkt->m_lapTime, pkt->m_engineRate, (int)pkt->m_gear, pkt->m_speed * 3.6f, pkt->m_gforce_lat, pkt->m_gforce_lon, pkt->m_xv);
          }
          //printf ("\n#i:pkt:%c / %d rpm %f gear %d spd %f %f %f", (char)get_int (packetBuffer, 0), get_int (packetBuffer, 4), 
          //  get_float (packetBuffer, 68), get_int (packetBuffer, 76), get_float (packetBuffer, 8), get_float (packetBuffer, 12), get_float (packetBuffer, 16));
          //printf ("\n#i:ffb:vert %f hori %f long%f", get_float (packetBuffer, 28), get_float (packetBuffer, 32), get_float (packetBuffer, 36));
          //
          mfc_packet_use (packetBuffer, cltime, cltime);
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
  if (_dashportfd > 0)
    close (_dashportfd);
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
