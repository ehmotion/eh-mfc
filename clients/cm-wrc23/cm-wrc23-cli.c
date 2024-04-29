/*
* codemasters wrc 23
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

#define PACKED __attribute__((packed))
#define float32 float
#define uint64 uint64_t
#define float64 float
#define boolean uint8_t
#define uint8 uint8_t

typedef struct PACKED {
	uint64 packet_uid;
			float32	game_total_time;
			float32	game_delta_time;
			uint64	game_frame_count;
			float32	shiftlights_fraction;
			float32	shiftlights_rpm_start;
			float32	shiftlights_rpm_end;
			boolean	shiftlights_rpm_valid;
			uint8	vehicle_gear_index;
			uint8	vehicle_gear_index_neutral;
			uint8	vehicle_gear_index_reverse;
			uint8	vehicle_gear_maximum;
			float32	vehicle_speed;
			float32	vehicle_transmission_speed;
			float32	vehicle_position_x;
			float32	vehicle_position_y;
			float32	vehicle_position_z;
			float32	vehicle_velocity_x;
			float32	vehicle_velocity_y;
			float32	vehicle_velocity_z;
			float32	vehicle_acceleration_x;
			float32	vehicle_acceleration_y;
			float32	vehicle_acceleration_z;
			float32	vehicle_left_direction_x;
			float32	vehicle_left_direction_y;
			float32	vehicle_left_direction_z;
			float32	vehicle_forward_direction_x;
			float32	vehicle_forward_direction_y;
			float32	vehicle_forward_direction_z;
			float32	vehicle_up_direction_x;
			float32	vehicle_up_direction_y;
			float32	vehicle_up_direction_z;
			float32	vehicle_hub_position_bl;
			float32	vehicle_hub_position_br;
			float32	vehicle_hub_position_fl;
			float32	vehicle_hub_position_fr;
			float32	vehicle_hub_velocity_bl;
			float32	vehicle_hub_velocity_br;
			float32	vehicle_hub_velocity_fl;
			float32	vehicle_hub_velocity_fr;
			float32	vehicle_cp_forward_speed_bl;
			float32	vehicle_cp_forward_speed_br;
			float32	vehicle_cp_forward_speed_fl;
			float32	vehicle_cp_forward_speed_fr;
			float32	vehicle_brake_temperature_bl;
			float32	vehicle_brake_temperature_br;
			float32	vehicle_brake_temperature_fl;
			float32	vehicle_brake_temperature_fr;
			float32	vehicle_engine_rpm_max;
			float32	vehicle_engine_rpm_idle;
			float32	vehicle_engine_rpm_current;
			float32	vehicle_throttle;
			float32	vehicle_brake;
			float32	vehicle_clutch;
			float32	vehicle_steering;
			float32	vehicle_handbrake;
			float32	stage_current_time;
			float64	stage_current_distance;
			float64	stage_length
} UDPPacket;

#define UDP_MAX_PACKETSIZE  2048
#define UDP_PORT            20779

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

typedef struct {
    float x, y, z;
} Vector3;

void vec3Normalize(Vector3 *vec)
{
    float len = sqrt(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);

    if (len != 0.f)
    {
        vec->x /= len;
        vec->y /= len;
        vec->z /= len;
    }
}

typedef struct {
    Vector3 forward, left, up;
} TransformFLU;

#define N 4

typedef struct {
    float m[4][4];
} Matrix4x4;

void printMatrix(float matrix[N][N]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            printf("%f\t", matrix[i][j]);
        }
        printf("\n");
    }
}

void swapRows(float matrix[N][N], int row1, int row2) {
    for (int i = 0; i < N; i++) {
        float temp = matrix[row1][i];
        matrix[row1][i] = matrix[row2][i];
        matrix[row2][i] = temp;
    }
}

void scaleRow(float matrix[N][N], int row, float scalar) {
    for (int i = 0; i < N; i++) {
        matrix[row][i] *= scalar;
    }
}

void addScaledRow(float matrix[N][N], int targetRow, int sourceRow, float scalar) {
    for (int i = 0; i < N; i++) {
        matrix[targetRow][i] += scalar * matrix[sourceRow][i];
    }
}

void invertMatrix(float matrix[N][N], float result[N][N]) {
    // Initialize result matrix as an identity matrix
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            result[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Perform Gaussian elimination
    for (int i = 0; i < N; i++) {
        // Make the diagonal element equal to 1
        float diagonalElement = matrix[i][i];
        scaleRow(matrix, i, 1.0 / diagonalElement);
        scaleRow(result, i, 1.0 / diagonalElement);

        // Eliminate other elements in the current column
        for (int j = 0; j < N; j++) {
            if (j != i) {
                float factor = -matrix[j][i];
                addScaledRow(matrix, j, i, factor);
                addScaledRow(result, j, i, factor);
            }
        }
    }
}

void transformVector(Vector3 *inputVector,  float transformMatrix[4][4], Vector3 *outputVector)
{
    // Convert the 3D vector to a 4D homogeneous coordinate
    float homogeneousCoord[4] = {inputVector->x, inputVector->y, inputVector->z, 1.0f};

    // Perform matrix-vector multiplication
    for (int i = 0; i < 4; ++i) {
        outputVector->x += transformMatrix[0][i] * homogeneousCoord[i];
        outputVector->y += transformMatrix[1][i] * homogeneousCoord[i];
        outputVector->z += transformMatrix[2][i] * homogeneousCoord[i];
    }

    // Extract the transformed 3D vector
    float w = transformMatrix[3][3];
    outputVector->x /= w;
    outputVector->y /= w;
    outputVector->z /= w;
}

int worldToLocal2(Vector3 *world, Vector3 *up, Vector3 *fwd, Vector3 *lft, Vector3 *out)
{
  //4x4 matrix
  float m44[4][4] = {
    {lft->x, lft->y, lft->z, 0},
    {up->x, up->y, up->z, 0},
    {fwd->x, fwd->y, fwd->z, 0},
    {0, 0, 0, 1}
    };
  //invert
  float r44[4][4];
  invertMatrix(m44, r44);
  //transform
  transformVector(world, r44, out);
  return 1;
}

#if 0
Function WorldToLocal(worldVector As Vector3, upVector As Vector3, forwardVector As Vector3, leftVector As Vector3) As Vector3
    ' Create the transformation matrix
    Dim matrix As New Matrix4x4(leftVector.X, leftVector.Y, leftVector.Z, 0,
                                  upVector.X, upVector.Y, upVector.Z, 0,
                                  forwardVector.X, forwardVector.Y, forwardVector.Z, 0,
                                  0, 0, 0, 1)
    ' Invert the matrix to get the world-to-local transformation
    Dim success As Boolean = Matrix4x4.Invert(matrix, matrix)
    ' Transform the world vector to local space
    Dim localVector As Vector3 = Vector3.Transform(worldVector, matrix)
    Return localVector
End Function
#endif

Vector3 worldToLocal(Vector3 wrld, Vector3 fwd, Vector3 ltd, Vector3 upd) {
    Vector3 loc;
    TransformFLU trns = {fwd, ltd, upd};
    
    loc.x = wrld.x * trns.forward.x + wrld.y * trns.forward.y + wrld.z * trns.forward.z;
    loc.y = wrld.x * trns.left.x + wrld.y * trns.left.y + wrld.z * trns.left.z;
    loc.z = wrld.x * trns.up.x + wrld.y * trns.up.y + wrld.z * trns.up.z;

    return loc;
}

#if 0
int main() {
    // Example usage
    Vector3 worldAccel = {1.0, 2.0, 3.0};
    Transform objectTransform = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // Identity matrix for simplicity
    
    Vector3 localAccel = worldToLocal(worldAccel, objectTransform);

    printf("Local Acceleration: (%.2f, %.2f, %.2f)\n", localAccel.x, localAccel.y, localAccel.z);

    return 0;
}
#endif

//                          pit    srg    hev    rol    sway   yaw      tl
//#i:new min/Max values:
//static int Mpkt[] = {0, 0, 9856, 1775,   892,  2992,  1707,  170979, 90000, 100, 100, 100};
//static int mpkt[] = {0, 0, 0,   -1853,  -571, -5033, -1710, -46827, -90000, 0, 0, 0};
static int Mpkt[] = {0, 0,  10000,  1500,   1000,  9000,  1507,  180000, 10000, 100, 100, 100};
static int mpkt[] = {0, 0, -10000, -1500,  -1000, -9000, -1510, -180000, -10000, 0, 0, 0};

int mfc_packet_use(char* packetBuffer, float rtime, float cltime)
{
  /* uses 1000 magnitude to not lose much fidelity on computation */
  #define DOF_MAG (1000.f)
  //game telemetry data
  static float dof_roll, dof_pitch, dof_yaw, dof_heave, dof_sway, dof_surge, dof_tloss;
  static float lcltime = -2.f;
  static int rampprc = 0;
  if (cltime - lcltime > 1.f) //no packets for more than 1sec, ramp to position
  {
    rampprc = goto_ramp(_cpkt, 0);
    printf("\n#i@%.3f:ramp to pos!", cltime);
  }
  lcltime = cltime;
  UDPPacket *tpkt = (UDPPacket*)packetBuffer;
  //world to local
  Vector3 wfwd  = {tpkt->vehicle_forward_direction_x, tpkt->vehicle_forward_direction_y, tpkt->vehicle_forward_direction_z};
  Vector3 wltd  = {tpkt->vehicle_left_direction_x,    tpkt->vehicle_left_direction_y,    tpkt->vehicle_left_direction_z};
  Vector3 wupd  = {tpkt->vehicle_up_direction_x,      tpkt->vehicle_up_direction_y,      tpkt->vehicle_up_direction_z};
  vec3Normalize(&wfwd);
  vec3Normalize(&wltd);
  vec3Normalize(&wupd);
  Vector3 wacc = {tpkt->vehicle_acceleration_x,      tpkt->vehicle_acceleration_y,      tpkt->vehicle_acceleration_z};
  Vector3 lacc = worldToLocal(wacc, wfwd, wltd, wupd);
  Vector3 wvel = {tpkt->vehicle_velocity_x, tpkt->vehicle_velocity_y, tpkt->vehicle_velocity_z};
  Vector3 lvel = worldToLocal(wvel, wfwd, wltd, wupd);
  //worldToLocal2(&wvel, &wupd, &wfwd, &wltd, &lvel);
  //dofs
  dof_pitch = atan2(wfwd.y, wupd.y) * RAD2DEG;
  dof_roll  = asin(wltd.y) * RAD2DEG;
  dof_yaw   = atan2(wltd.z, wltd.x) * RAD2DEG;
  dof_surge = lacc.x / GRAVACCEL;
  dof_sway  = lacc.y / GRAVACCEL;
  dof_heave = -lacc.z / GRAVACCEL;
  if (fabs(lvel.x) > 3.0f)
    dof_tloss = atan2f(lvel.x, lvel.z) * RAD2DEG - 90.0f;
  else
    dof_tloss = 0.0f;
  if (0 || _odbg)
    printf ("\n#i@%.3f:sts %.3f vel x %.3f vel z %.3f tloss %.3f", 
      cltime, tpkt->stage_current_time, lvel.x, lvel.z, dof_tloss);
  #if 0
  {
    dof_tloss = 90 - atan2f(lvel.x, -lvel.z) * RAD2DEG;
  }
  #endif
  if (0 || _odbg)
    printf ("\n#i@%.3f:t0 %f pitch %f %f %f roll %f %f yaw %f %f", cltime, tpkt->stage_current_time,
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
  _cpkt[MFC_PIPITCH] = get_cmap_f (_cpkt[MFC_PIPITCH], mpkt[MFC_PIPITCH], Mpkt[MFC_PIPITCH], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PISURGE] = get_cmap_f (_cpkt[MFC_PISURGE], mpkt[MFC_PISURGE], Mpkt[MFC_PISURGE], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PIHEAVE] = get_cmap_f(_cpkt[MFC_PIHEAVE], mpkt[MFC_PIHEAVE], Mpkt[MFC_PIHEAVE], MFC_POS_MIN, MFC_POS_MAX);
  //roll
  _cpkt[MFC_PIROLL]  = get_cmap_f (_cpkt[MFC_PIROLL],  mpkt[MFC_PIROLL],  Mpkt[MFC_PIROLL], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PISWAY]  = get_cmap_f (_cpkt[MFC_PISWAY],  mpkt[MFC_PISWAY],  Mpkt[MFC_PISWAY], MFC_POS_MIN, MFC_POS_MAX);
  //yaw
  _cpkt[MFC_PIYAW]   = get_cmap_f (_cpkt[MFC_PIYAW],   mpkt[MFC_PIYAW],   Mpkt[MFC_PIYAW], MFC_POS_MIN, MFC_POS_MAX);
  _cpkt[MFC_PITLOSS] = get_cmap_f (_cpkt[MFC_PITLOSS], mpkt[MFC_PITLOSS], Mpkt[MFC_PITLOSS], MFC_POS_MIN, MFC_POS_MAX);
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
  {
    if (rampprc > 0)
      rampprc = goto_ramp(_cpkt, rampprc);
    mfc_bcast_send ();
  }
  //send dash data
  if (_dashaddr)
  {
    //dash data
    _dpkt[MFC_DISPD]  = (int)(tpkt->vehicle_speed * 2.24f);  //speed: 1 m/s, 3.60 kph - 1 m/s, 2.24 mph
    _dpkt[MFC_DIGEAR] = (int)tpkt->vehicle_gear_index;  //gear
    _dpkt[MFC_DIRPM]  = (int)tpkt->vehicle_engine_rpm_current;  //rpm x10
    _dpkt[MFC_DIRPMM] = (int)tpkt->vehicle_engine_rpm_max - 1000; //max rpm x10
    //printf ("\n#i@%.3f:d1 rpm %d rpmm %.3f rpmM %.3f", cltime, _dpkt[MFC_DIRPM], tpkt->vehicle_engine_rpm_current, tpkt->vehicle_engine_rpm_max);
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
    printf("\n#i:float size %d", sizeof(float));
    printf("\n#i:float size %d", sizeof(float));
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
  float uptime, idle_time;
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
  long tmok = 0;
  char fgtmo = 0;
  while (!_done)
  {
    memset ((void*)fdset, 0, sizeof(fdset));

    fdset[0].fd = s;
    fdset[0].events = POLLIN;

    rc = poll (fdset, nfds, 25/*timeout*/);

    if (rc < 0)
    {
      printf("\n#e:poll() failed!");
      _done = 1;
      break;
    }

    if (rc == 0)
    {
      tmok++;
      if (tmok > 200) //200x25=5000ms
      {
        tmok = 0;
        fgtmo = 1;//timed out, stop sending
        lminl = lmaxl = lminr = lmaxr = 0.0f;
        printf(".");
      }
      //park platform
      if (!fgtmo && !_learn)
      {
        goto_park(_cpkt);
        mfc_bcast_send ();
      }
      //sleep (1);
    }

    if (fdset[0].revents & POLLIN)
    {
      fgtmo = 0;
      int rlen = 0, idx;
      //recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
      //if ((rlen = recvfrom (s, (void *)&packetBuffer, SMS_UDP_MAX_PACKETSIZE, 0, (struct sockaddr*)&si_other, &slen))==-1)
      rlen = recvfrom (s, (void *)&packetBuffer, UDP_MAX_PACKETSIZE, 0, NULL, NULL);
      cltime = get_fms();
      if (rlen > 0)
      {
        //process packet
        mfc_packet_use (packetBuffer, rtime, cltime);
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
