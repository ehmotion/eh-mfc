/*
 Copyright (c) 2015 mirel lazar <mirel.t.lazar@gmail.com>
 License: GPLv3
 */

/*
modification history
--------------------
01a,08jan2015, mirel lazar <mirel.t.lazar@gmail.com>
               adapted for motion feedback controller
*/

#define MFC_VERSION "0.02.1d"

#define UDP_MAX_PKTSIZE  (65507)  //theoretical

/*
  int pkt_type;
  int pkt_dof_type;
  int data[6];    //{pitch, roll, yaw, surge, sway, heave}
  int speed;
  //
Pitch is the tilt of the car forwards or backwards in [°]
Roll is how much the car is dipped to the left or right in [°]
Yaw is the heading of the car (north, east, south, west) in [°]

Surge means the acceleration of the car in longitudinal direction [g]
Sway means the acceleration of the car in lateral direction [g]
Heave means the acceleration up and down [g]
//motion: all values range from -10000 to +10000 as they are mapped server-side
pkt[2] = pl_pitch;  //pitch
pkt[3] = pl_roll;   //roll
pkt[4] = 0;         //yaw
pkt[5] = 0;         //surge
pkt[6] = 0;         //sway
pkt[7] = 0;         //heave
//speed
pkt[8] = 0;         //speed
*/
#define MFC_PKTSIZE   12
//
#define MFC_PITYPE    0 //pkt type spec
#define MFC_PITDAT    1 //pkt type data
#define MFC_PIPITCH   2
#define MFC_PISURGE   3
#define MFC_PIHEAVE   4
#define MFC_PIROLL    5
#define MFC_PISWAY    6
#define MFC_PIYAW     7
#define MFC_PITLOSS   8  //tyre slip
#define MFC_PIEXTRA1  9
#define MFC_PIEXTRA2  10
#define MFC_PIEXTRA3  11

//pkt_type
#define PKTT_CTRL   (0x00)
#define PKTT_DATA   (0x01)
#define PKTT_WEPS   2
#define PKTT_WRESET 3
#define PKTT_WCTRL  4
#define PKTT_WCSTL  5
#define PKTT_IN     6
#define PKTT_OUT    7
#define PKTT_DEBUG  8
//pktck: c0ae00XX on type and DOF type
#define PKTCK_CKPT1 0xc0
#define PKTCK_CKPT2 0xae
//
#define MFC_POS_MIN  (-10000)
#define MFC_POS_MAX  (10000)
#define MFC_HPOS_MIN (-5000)
#define MFC_HPOS_MAX (5000)
#define MFC_WHL_MIN  (-32768)
#define MFC_WHL_MAX  (32768)
#define MFC_HWHL_MIN (-16384)
#define MFC_HWHL_MAX (16384)
#define MFC_QWHL_MIN (-4096)
#define MFC_QWHL_MAX (4096)
#define MFC_BYTE_MIN (-127)
#define MFC_BYTE_MAX (127)
#define MFC_HBYTE_MIN (-64)
#define MFC_HBYTE_MAX (64)

//pkt control types, used with PKTT_CTRL
#define PKTC_UNUSED  (0x00)
#define PKTC_PCCFG   (0x01) //platform configuration data
#define PKTC_PRCFG   (0x02) //profile configuration data

//pkt_dof_type
//in 6DOF mode, all 6 forces are available discretly: wheel, ffb, vibration
//  wroll, wpitch, froll, fpitch, vroll, vpitch
#define PKTT_0DOF    (0x00)
#define PKTT_1DOF    (0x01)
#define PKTT_2DOF    (0x02)
#define PKTT_2DOFN   (0x82)
#define PKTT_3DOF    (0x03)
#define PKTT_4DOF    (0x04)
#define PKTT_5DOF    (0x04)
#define PKTT_6DOF    (0x06)
#define PKTT_6DOFN   (0x86)

#define MFCSVR_PORT (64401)
//dash
/*
motion: all values range from -10000 to +10000 as they are mapped server-side
int pkt_type;     //includes first chk bytes
int pkt_dof_type; //includes first chk bytes
int platf[10];     //{pitch, roll, yaw, surge, sway, heave, tyre_slip, ext1, ext2, ext3}
int position[3];  //world position: x, y z
int object[11];   //gear(-1 reverse), gears_max, rpm, rpm_max, rpm_idle, speed, speed_max, fuel, fuel_max, temp, temp_max
int appname[3];   //12 bytes for app name
*/
#define MFC_DIPOSX  12
#define MFC_DIPOSY  13
#define MFC_DIPOSZ  14
#define MFC_DIGEAR  15  //gear: -1, 0, 1..
#define MFC_DIGEARM 16  //max gears
#define MFC_DIRPM   17  //rpm
#define MFC_DIRPMM  18  //max rpm
#define MFC_DIRPMI  19  //idle rpm
#define MFC_DISPD   20  //speed
#define MFC_DISPDM  21  //max speed
#define MFC_DIFUEL  22  //fuel
#define MFC_DIFUELM 23  //max fuel
#define MFC_DITEMP  24  //temp
#define MFC_DITEMPM 25  //max temp
#define MFC_DIRUNTS 26  //running timestamp
#define MFC_DIAPP14 27  //sim/app name 1..4
#define MFC_DIAPP58 28  //sim/app name 5..8
#define MFC_DIAPP9C 29  //sim/app name 9..12
//-
#define MFC_DIAPPSZ 12  //sim/app name len including '\0'

#define MFCDASH_PORT (64402)
#define MFCDASH_PKTSIZE  30

//other stuff
#define MFCXTRACT_PORT (64405)

//math stuff
#define MATH_PI (3.14159f)
#define RAD2DEG (57.2958279088f)  // 45/ATAN(1) or 180/3.14159
#define GRAVACCEL (9.80665f)      //grav accel

#define print_info(...) \
            do { printf("\n#i."); printf(__VA_ARGS__); } while (0)
#define print_warn(...) \
            do { printf("\n#!."); printf(__VA_ARGS__); } while (0)
#define print_errr(...) \
            do { printf("\n#E."); printf(__VA_ARGS__); } while (0)

int mfcdash_bcast_prep (char *dst, int svr);
int mfcdash_bcast_send ();
int mfcdash_bcast_receive ();
int *mfcdash_bcast_pktget ();
int mfcdash_bcast_pktlen ();
void mfcdash_bcast_close ();

//MFC platform control
int mfc_bcast_prep (char *dst, int svr);
int mfc_bcast_send ();
int mfc_bcast_receive ();
int *mfc_bcast_pktget ();
int mfc_bcast_pktlen ();
void mfc_bcast_close ();

void bcast_prep ();
void bcast_send ();
void bcast_close ();

//gradually move to park
int goto_park(int *pkt);
//gradually ramp to the current pos based on prc weight (to 100%)
int goto_ramp(int *pkt, char flg);

int s7led_connect(char* portname);
int s7led_close();
/*
 * Send a data to the serial port.
 */
int s7ledS_send (char* pd);
int s7ledN_send (int pd);

void s7led_gear_max (int g);
void s7led_gear (int g);

int opt_gear_up ();
int opt_gear_dn ();
int opt_gear_ready ();
int opt_gear_max_up ();
int opt_gear_max_dn ();
int opt_gear_max_ready ();

int opt_gear_get ();
int opt_rpm_set (int rpm);
int opt_rpm_get ();

long get_map (long x, long in_min, long in_max, long out_min, long out_max);
long get_cmap (long x, long in_min, long in_max, long out_min, long out_max);
float get_map_f (float x, float in_min, float in_max, float out_min, float out_max);
float get_cmap_f (float x, float in_min, float in_max, float out_min, float out_max);

float get_float (char *buf, int off);
int get_int (char *buf, int off);
unsigned int get_uint (char *buf, int off);
unsigned short get_ushort (char *buf, int off);
short get_short (char *buf, int off);

int normal_axis (int val, int max);
int normal_pedal (int val, int max);
int normal_brake (int val, int max);
int normal_accel (int val, int max);
//ffb is: 128..255<L R>1..127
//output: 128..0<>-1..-127
int normal_ffb (int val, int mid);
//ffb is: 128..255<L R>1..127
//output: delta from mid: -128..0<L R>1..127
int normal_ffb2 (int val, int mid);
//ffb is: 128..255<>1..127
//output: -128..0<L R>1..127
int normal_ffb3 (int val);

char count_ones (char byte);

unsigned char reverse_char(unsigned char n);
//get millis since the Epoch, 1970-01-01 00:00:00 +0000 (UTC)
// when st is not 0, it returns the time delta from the first call
unsigned long get_millis (char st);
//delta millis from first call
unsigned long get_millis_delta ();
//fraction millis from first call
float get_fms();
//get delta time since the previous call
unsigned int dtime_ms ();

int set_max_prio ();

//serial dash
int serial_dash_open(char *_dashport, int spd);
int sh_serial_write(int fd, char *buf, int blen);
int dash_serial_write(int fd, char *_dpkt, int blen);

void _check_bkey ( char *_bkey, char *_busr);
// define a type using a typedef so that we can declare the externally
// visible struct in this include file and then use the same type when
// defining the externally visible struct in the implementation file which
// will also have the definitions for the actual functions which will have
// file visibility only because we will use the static modifier to restrict
// the functions' visibility to file scope only.
typedef struct {
  void (*p1)(char *a, char *b);
  void (*p2)(char *a, char *b);
  void (*p3)(char *a, char *b);
  void (*p4)(char *a, char *b);
  void (*p5)(char *a, char *b);
  void (*p6)(char *a, char *b);
  void (*p7)(char *a, char *b);
  void (*p8)(char *a, char *b);
  void (*p9)(char *a, char *b);
} FuncList;

void mFLi (int);

/*
* Exponentially Weighted Moving Average filter
* @params
* [in]    dof - ROLL,PITCH,YAW..
* [in]  input - current position value
* [in] weigth - desired weigth
* @returns input weighted value
*/
int mfc_ewmaf(int dof, int input, float wi);
int mfc_ewmaf_set(int dof, int input);
