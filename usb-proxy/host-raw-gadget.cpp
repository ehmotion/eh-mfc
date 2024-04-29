#include <assert.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/types.h>

#include "host-raw-gadget.h"
#include "device-libusb.h"

#ifndef gettid
#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)
#endif

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
static int csock = -1;
static struct sockaddr_in mfc_si_other;
#define PACKED __attribute__((packed))
typedef struct PACKED
{
	uint8_t type;
	uint16_t length;
} n_header;

// this is the max serial packet size
#define MAX_PACKET_SIZE 1048
#define MAX_PACKET_VALUE_SIZE (MAX_PACKET_SIZE - sizeof(n_header))

typedef struct PACKED
{
	n_header header;
	uint8_t value[MAX_PACKET_VALUE_SIZE + 10];
} n_packet;
static n_packet npkt, cpkt, cpkto; //network packet
// {<E_TYPE_DEBUG>, 5, vid[0], vid[1], pid[0], pid[1], <delta_ts>}
extern int vid, pid;
// when st is not 0, it returns the time delta from the first call
unsigned long get_millis(char st)
{
	static long st_ms = 0;
	struct timespec lts;
	long ctime;
	//get current time
	//clock_gettime (CLOCK_REALTIME, &lts);
	clock_gettime(CLOCK_BOOTTIME, &lts);
	ctime = lts.tv_sec * 1000L + lts.tv_nsec / 1000000L;
	if (st_ms == 0)
	{
		st_ms = ctime;
		if (st)
			return 0;
	}
	if (st)
		return ((int)ctime - st_ms);
	return (ctime);
}

unsigned long get_millis_delta()
{
	static long lst_ms = 0;
	long ctime = get_millis(1);
	long dtime = ctime - lst_ms;
	//first time call, return delta time as 0
	if (lst_ms == 0)
		dtime = 0;
	lst_ms = ctime;
	return (dtime);
}

float get_fms()
{
	return get_millis(1) / 1000.0f;
	//return get_millis_delta () / 1000.0f;
}
//
int client_init (const char *ipaddr)
{
  int s = -1;
  if ((s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf ("\n#E:socket");
    return 0;
  }
	extern int vendor_id;
	extern int product_id;
  //destination
	#define MFCXTRACT_PORT (64405)
  memset((char *) &mfc_si_other, 0, sizeof (mfc_si_other));
  mfc_si_other.sin_family = AF_INET;
  mfc_si_other.sin_port = htons (MFCXTRACT_PORT);
  mfc_si_other.sin_addr.s_addr = inet_addr(ipaddr);//htonl(0x7f000001L); //htonl (INADDR_BROADCAST);
  //mfc_si_other.sin_addr.s_addr = htonl(INADDR_LOOPBACK);//htonl(0x7f000001L); //htonl (INADDR_BROADCAST);
	printf("Sending USB data to %s:%d\n", ipaddr, MFCXTRACT_PORT);
  csock = s;
  //prep debug data
  npkt.header.type = 0;//converts to PKTT_CTRL in MFC
  npkt.header.length = 8;
  npkt.value[0] = (vendor_id >> 8) & 0xff;
  npkt.value[1] = vendor_id & 0xff;
  npkt.value[2] = (product_id >> 8) & 0xff;
  npkt.value[3] = product_id & 0xff;
  unsigned long clts = get_millis (0) & 0xffffffff;
  //timestamp 4 bytes
  memcpy((void *)(npkt.value + 4), (void *)&clts, 4);
  //write debug pkt first
  (void)sendto (csock, (const void *)&npkt, npkt.header.length + 2, 0, (struct sockaddr*)&mfc_si_other, sizeof (mfc_si_other));
  //
  return s;
}
//
int client_close ()
{
  if (csock > 0)
    close (csock);
  csock = -1;
  //
  return 1;
}
/*
#PKT@48.161>545B/0x07/out: 03 
30 01 08 d0 16 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
30 01 08 de 1a 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
30 01 08 e0 17 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
30 01 08 ca 14 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
30 01 08 86 22 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
30 f8 09 01 02 06 06 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
30 01 08 b3 2c 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
30 01 08 2e 31 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
*/
const char *npktst[] = {"dsc", "idx", "eps", "rst", "ctl", "stl", "inn", "out", "dbg"};
static int client_send(n_packet *packet, char in)
{
	static int npkt_hdr = sizeof(n_header);
#if 1
	//if (csock < 0)
		//client_init();
	if (csock > 0)
	{
		//we need 4 bytes for the wheel vid+pid and another 4 for timestamp: we have a max of 1024 bytes buffer on reception
		unsigned long clts = get_millis(0) & 0xffffffff;
		memcpy((void *)(npkt.value + 8), (void *)packet, packet->header.length + npkt_hdr);
		//timestamp 4 bytes
		memcpy((void *)(npkt.value + 4), (void *)&clts, 4);
		//header
		npkt.header.type = packet->header.type;
		npkt.header.length = packet->header.length + 8 + npkt_hdr; //8 for the vid&pid+timestamp + packet header length
		//write USB data
		if (sendto(csock, (const void *)&npkt, npkt.header.length + npkt_hdr, 0, (struct sockaddr *)&mfc_si_other, sizeof(mfc_si_other)) < 0)
		{
			printf("\n#E:client_send");
			fflush(stdout);
		}
	}
#endif
	//
	if (0) // || packet->header.length != 65)
	{
		int i;
		printf("\n#PKT@%.3f%s%3dB/0x%02x/%s:", get_fms(), in ? ">" : "<", packet->header.length, packet->header.type, npktst[packet->header.type]);
		for (i = 0; i < packet->header.length; i++)
			printf(" %02x", packet->value[i]);
		//fflush(stdout);
	}
#if 0
  if (packet->header.length > 0)
  {
    switch (packet->header.type)
    {
      case E_TYPE_IN:
      case E_TYPE_OUT:
        break;
      case E_TYPE_CONTROL:
        //check auth
        if (0 && packet->value[0] == 0xf1)
        {
          if (packet->value[2] == 0x00)
          {
            printf ("\n#i:auth stage %02X", packet->value[1]);
            fflush (stdout);
          }
        }
        if (0 && packet->header.length)
        {
          int i;
          printf ("\n#i:CTL %03d bytes, type %x\n##", packet->header.length, packet->header.type);
          for (i = 0; i < packet->header.length; i++)
            printf ("%02x ", packet->value[i]);
        }
        break;
      default:
        ;//not used
    } //switch
  } //if pkt len
#endif
	return 1;
}
//

struct usb_device_descriptor		host_device_desc;
struct raw_gadget_config_descriptor	*host_config_desc;

struct endpoint_thread *ep_thread_list;

/*----------------------------------------------------------------------*/

int usb_raw_open() {
	int fd = open("/dev/raw-gadget", O_RDWR);
	if (fd < 0) {
		perror("open() /dev/raw-gadget");
		exit(EXIT_FAILURE);
	}
	return fd;
}

void usb_raw_init(int fd, enum usb_device_speed speed,
			const char *driver, const char *device) {
	struct usb_raw_init arg;
	strcpy((char *)&arg.driver_name[0], driver);
	strcpy((char *)&arg.device_name[0], device);
	arg.speed = speed;
	int rv = ioctl(fd, USB_RAW_IOCTL_INIT, &arg);
	if (rv < 0) {
		perror("ioctl(USB_RAW_IOCTL_INIT)");
		exit(EXIT_FAILURE);
	}
}

void usb_raw_run(int fd) {
	int rv = ioctl(fd, USB_RAW_IOCTL_RUN, 0);
	if (rv < 0) {
		perror("ioctl(USB_RAW_IOCTL_RUN)");
		exit(EXIT_FAILURE);
	}
}

void usb_raw_event_fetch(int fd, struct usb_raw_event *event) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EVENT_FETCH, event);
	if (rv < 0) {
		if (errno == EINTR) {
			event->length = 4294967295;
			return;
		}
		perror("ioctl(USB_RAW_IOCTL_EVENT_FETCH)");
		exit(EXIT_FAILURE);
	}
}

int usb_raw_ep0_read(int fd, struct usb_raw_ep_io *io) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EP0_READ, io);
	if (rv < 0) {
		if (errno == EBUSY)
			return rv;
		perror("ioctl(USB_RAW_IOCTL_EP0_READ)");
		exit(EXIT_FAILURE);
	}
	return rv;
}

int usb_raw_ep0_write(int fd, struct usb_raw_ep_io *io) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EP0_WRITE, io);
	if (rv < 0) {
		perror("ioctl(USB_RAW_IOCTL_EP0_WRITE)");
		exit(EXIT_FAILURE);
	}
	return rv;
}

int usb_raw_ep_enable(int fd, struct usb_endpoint_descriptor *desc) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EP_ENABLE, desc);
	if (rv < 0) {
		perror("ioctl(USB_RAW_IOCTL_EP_ENABLE)");
		exit(EXIT_FAILURE);
	}
	return rv;
}

int usb_raw_ep_disable(int fd, uint32_t num) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EP_DISABLE, num);
	if (rv < 0) {
		perror("ioctl(USB_RAW_IOCTL_EP_DISABLE)");
		exit(EXIT_FAILURE);
	}
	return rv;
}

int usb_raw_ep_read(int fd, struct usb_raw_ep_io *io) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EP_READ, io);
	if (rv < 0) {
		if (errno == EINPROGRESS) {
			// Ignore failures caused by the test that halts endpoints.
			return rv;
		}
		else if (errno == EBUSY)
			return rv;
		perror("ioctl(USB_RAW_IOCTL_EP_READ)");
		exit(EXIT_FAILURE);
	}
	return rv;
}

int usb_raw_ep_write(int fd, struct usb_raw_ep_io *io) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EP_WRITE, io);
	if (rv < 0) {
		if (errno == EINPROGRESS) {
			// Ignore failures caused by the test that halts endpoints.
			return rv;
		}
		else if (errno == EBUSY)
			return rv;
		perror("ioctl(USB_RAW_IOCTL_EP_WRITE)");
		exit(EXIT_FAILURE);
	}
	return rv;
}

void usb_raw_configure(int fd) {
	int rv = ioctl(fd, USB_RAW_IOCTL_CONFIGURE, 0);
	if (rv < 0) {
		perror("ioctl(USB_RAW_IOCTL_CONFIGURED)");
		exit(EXIT_FAILURE);
	}
}

void usb_raw_vbus_draw(int fd, uint32_t power) {
	int rv = ioctl(fd, USB_RAW_IOCTL_VBUS_DRAW, power);
	if (rv < 0) {
		perror("ioctl(USB_RAW_IOCTL_VBUS_DRAW)");
		exit(EXIT_FAILURE);
	}
}

int usb_raw_eps_info(int fd, struct usb_raw_eps_info *info) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EPS_INFO, info);
	if (rv < 0) {
		perror("ioctl(USB_RAW_IOCTL_EPS_INFO)");
		exit(EXIT_FAILURE);
	}
	return rv;
}

void usb_raw_ep0_stall(int fd) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EP0_STALL, 0);
	if (rv < 0) {
		if (errno == EBUSY)
			return;
		perror("ioctl(USB_RAW_IOCTL_EP0_STALL)");
		exit(EXIT_FAILURE);
	}
}

void usb_raw_ep_set_halt(int fd, int ep) {
	int rv = ioctl(fd, USB_RAW_IOCTL_EP_SET_HALT, ep);
	if (rv < 0) {
		perror("ioctl(USB_RAW_IOCTL_EP_SET_HALT)");
		exit(EXIT_FAILURE);
	}
}

/*----------------------------------------------------------------------*/

void log_control_request(struct usb_ctrlrequest *ctrl) {
	printf("  bRequestType: 0x%02x %5s, bRequest: 0x%02x, wValue: 0x%04x,"
		" wIndex: 0x%04x, wLength: %d\n", ctrl->bRequestType,
		(ctrl->bRequestType & USB_DIR_IN) ? "(IN)" : "(OUT)",
		ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);

	switch (ctrl->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_STANDARD:
		printf("  type = USB_TYPE_STANDARD\n");
		break;
	case USB_TYPE_CLASS:
		printf("  type = USB_TYPE_CLASS\n");
		break;
	case USB_TYPE_VENDOR:
		printf("  type = USB_TYPE_VENDOR\n");
		break;
	default:
		printf("  type = unknown = %d\n", (int)ctrl->bRequestType);
		break;
	}

	switch (ctrl->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_STANDARD:
		switch (ctrl->bRequest) {
		case USB_REQ_GET_DESCRIPTOR:
			printf("  req = USB_REQ_GET_DESCRIPTOR\n");
			switch (ctrl->wValue >> 8) {
			case USB_DT_DEVICE:
				printf("  desc = USB_DT_DEVICE\n");
				break;
			case USB_DT_CONFIG:
				printf("  desc = USB_DT_CONFIG\n");
				break;
			case USB_DT_STRING:
				printf("  desc = USB_DT_STRING\n");
				break;
			case USB_DT_INTERFACE:
				printf("  desc = USB_DT_INTERFACE\n");
				break;
			case USB_DT_ENDPOINT:
				printf("  desc = USB_DT_ENDPOINT\n");
				break;
			case USB_DT_DEVICE_QUALIFIER:
				printf("  desc = USB_DT_DEVICE_QUALIFIER\n");
				break;
			case USB_DT_OTHER_SPEED_CONFIG:
				printf("  desc = USB_DT_OTHER_SPEED_CONFIG\n");
				break;
			case USB_DT_INTERFACE_POWER:
				printf("  desc = USB_DT_INTERFACE_POWER\n");
				break;
			case USB_DT_OTG:
				printf("  desc = USB_DT_OTG\n");
				break;
			case USB_DT_DEBUG:
				printf("  desc = USB_DT_DEBUG\n");
				break;
			case USB_DT_INTERFACE_ASSOCIATION:
				printf("  desc = USB_DT_INTERFACE_ASSOCIATION\n");
				break;
			case USB_DT_SECURITY:
				printf("  desc = USB_DT_SECURITY\n");
				break;
			case USB_DT_KEY:
				printf("  desc = USB_DT_KEY\n");
				break;
			case USB_DT_ENCRYPTION_TYPE:
				printf("  desc = USB_DT_ENCRYPTION_TYPE\n");
				break;
			case USB_DT_BOS:
				printf("  desc = USB_DT_BOS\n");
				break;
			case USB_DT_DEVICE_CAPABILITY:
				printf("  desc = USB_DT_DEVICE_CAPABILITY\n");
				break;
			case USB_DT_WIRELESS_ENDPOINT_COMP:
				printf("  desc = USB_DT_WIRELESS_ENDPOINT_COMP\n");
				break;
			case USB_DT_PIPE_USAGE:
				printf("  desc = USB_DT_PIPE_USAGE\n");
				break;
			case USB_DT_SS_ENDPOINT_COMP:
				printf("  desc = USB_DT_SS_ENDPOINT_COMP\n");
				break;
			default:
				printf("  desc = unknown = 0x%x\n",
							ctrl->wValue >> 8);
				break;
			}
			break;
		case USB_REQ_SET_CONFIGURATION:
			printf("  req = USB_REQ_SET_CONFIGURATION\n");
			break;
		case USB_REQ_GET_CONFIGURATION:
			printf("  req = USB_REQ_GET_CONFIGURATION\n");
			break;
		case USB_REQ_SET_INTERFACE:
			printf("  req = USB_REQ_SET_INTERFACE\n");
			break;
		case USB_REQ_GET_INTERFACE:
			printf("  req = USB_REQ_GET_INTERFACE\n");
			break;
		case USB_REQ_GET_STATUS:
			printf("  req = USB_REQ_GET_STATUS\n");
			break;
		case USB_REQ_CLEAR_FEATURE:
			printf("  req = USB_REQ_CLEAR_FEATURE\n");
			break;
		case USB_REQ_SET_FEATURE:
			printf("  req = USB_REQ_SET_FEATURE\n");
			break;
		default:
			printf("  req = unknown = 0x%x\n", ctrl->bRequest);
			break;
		}
		break;
	case USB_TYPE_CLASS:
		switch (ctrl->bRequest) {
		default:
			printf("  req = unknown = 0x%x\n", ctrl->bRequest);
			break;
		}
		break;
	default:
		printf("  req = unknown = 0x%x\n", ctrl->bRequest);
		break;
	}
}

void log_event(struct usb_raw_event *event) {
	switch (event->type) {
	case USB_RAW_EVENT_CONNECT:
		printf("event: connect, length: %u\n", event->length);
		break;
	case USB_RAW_EVENT_CONTROL:
		printf("event: control, length: %u\n", event->length);
		log_control_request((struct usb_ctrlrequest *)&event->data[0]);
		break;
	default:
		printf("event: unknown, length: %u\n", event->length);
	}
}

/*----------------------------------------------------------------------*/

void *ep_loop_write(void *arg) {
	struct thread_info ep_thread_info = *((struct thread_info*) arg);
	int fd = ep_thread_info.fd;
	int ep_num = ep_thread_info.ep_num;
	struct usb_endpoint_descriptor ep = ep_thread_info.endpoint;
	std::string transfer_type = ep_thread_info.transfer_type;
	std::string dir = ep_thread_info.dir;
	std::deque<data_queue_info> *data_queue = ep_thread_info.data_queue;
	std::mutex *data_mutex = ep_thread_info.data_mutex;

	printf("Start writing thread for EP%02x, thread id(%ld)\n",
		ep.bEndpointAddress, gettid());

	while (!please_stop_eps) {
		assert(ep_num != -1);
		if (data_queue->size() == 0) {
			usleep(100);
			continue;
		}

		data_mutex->lock();
		struct data_queue_info temp_data = data_queue->front();
		data_queue->pop_front();
		data_mutex->unlock();
		struct usb_raw_transfer_io io = temp_data.io;

		if (ep.bEndpointAddress & 0x80) {
			int rv = usb_raw_ep_write(fd, (struct usb_raw_ep_io *)&io);
			if (rv >= 0) {
				//printf("EP%x(%s_%s): wrote %d bytes to host\n", ep.bEndpointAddress,
					//transfer_type.c_str(), dir.c_str(), rv);
				//store for network processing
				//cpkt.header.type = 6; //PKTT_IN //WHL data
				//cpkt.header.length = rv;
				//memcpy(&cpkt.value, &io.data, rv);
				//send to network for processing
				//client_send(&cpkt, 1);
			}
		}
		else {
			int length = temp_data.length;
			unsigned char *data = new unsigned char[length];
			memcpy(data, io.data, length);
			send_data(ep.bEndpointAddress, ep.bmAttributes, data, length);
			//printf("EP%x(%s_%s): send %d bytes\n", ep.bEndpointAddress,
				//transfer_type.c_str(), dir.c_str(), length);
			delete[] data;
		}
	}

	printf("End writing thread for EP%02x, thread id(%ld)\n",
		ep.bEndpointAddress, gettid());
	return NULL;
}

void *ep_loop_read(void *arg) {
	struct thread_info ep_thread_info = *((struct thread_info*) arg);
	int fd = ep_thread_info.fd;
	int ep_num = ep_thread_info.ep_num;
	struct usb_endpoint_descriptor ep = ep_thread_info.endpoint;
	std::string transfer_type = ep_thread_info.transfer_type;
	std::string dir = ep_thread_info.dir;
	std::deque<data_queue_info> *data_queue = ep_thread_info.data_queue;
	std::mutex *data_mutex = ep_thread_info.data_mutex;

	printf("Start reading thread for EP%02x, thread id(%ld)\n",
		ep.bEndpointAddress, gettid());

	while (!please_stop_eps) {
		assert(ep_num != -1);
		struct data_queue_info temp_data;

		if (ep.bEndpointAddress & 0x80) {
			unsigned char *data;
			int nbytes = 0;

			if (data_queue->size() >= 32) {
				usleep(200);
				continue;
			}

			receive_data(ep.bEndpointAddress, ep.bmAttributes, ep.wMaxPacketSize, &data, &nbytes, 500);

			if (nbytes > 0)
			{
				memcpy(temp_data.io.data, data, nbytes);
				temp_data.io.inner.ep = ep_num;
				temp_data.io.inner.flags = 0;
				temp_data.io.inner.length = nbytes;
				temp_data.length = nbytes;

				data_mutex->lock();
				data_queue->push_back(temp_data);
				data_mutex->unlock();
				if (verbose_level)
					printf("EP%x(%s_%s):R1: enqueued %d bytes to queue\n", ep.bEndpointAddress,
							transfer_type.c_str(), dir.c_str(), nbytes);
				//
				//printf("EP%x(%s_%s): read %d bytes from device\n", ep.bEndpointAddress,
				//		transfer_type.c_str(), dir.c_str(), nbytes);
				//store for network processing
				cpkt.header.type = 6; //PKTT_IN //WHL data
				cpkt.header.length = nbytes + 1; //with endpoint info
				cpkt.value[0] = ep.bEndpointAddress; //save endpoint info
				memcpy(cpkt.value + 1, data, nbytes);
				//send to network for processing
				client_send(&cpkt, 0);
				//client_send2(6, ep.bEndpointAddress, data, nbytes);
			}
			delete[] data;
		}
		else {
			temp_data.io.inner.ep = ep_num;
			temp_data.io.inner.flags = 0;
			temp_data.io.inner.length = ep.wMaxPacketSize; //sizeof(temp_data.io.data);

			int rv = usb_raw_ep_read(fd, (struct usb_raw_ep_io *)&temp_data.io);
			if (rv >= 0) {
				//printf("EP%x(%s_%s): read %d bytes from host\n", ep.bEndpointAddress,
					//	transfer_type.c_str(), dir.c_str(), rv);
				temp_data.length = rv;

				data_mutex->lock();
				data_queue->push_back(temp_data);
				data_mutex->unlock();
				if (verbose_level)
					printf("EP%x(%s_%s):R2: enqueued %d bytes to queue\n", ep.bEndpointAddress,
							transfer_type.c_str(), dir.c_str(), rv);
				if (1|| rv < 1048)
				{
					//store for network processing
					cpkto.header.type = 7; //PKTT_OUT //FFB data
					cpkto.header.length = rv + 1; //with endpoint info
					cpkto.value[0] = ep.bEndpointAddress; //save endpoint info
					memcpy(cpkto.value + 1, temp_data.io.data, rv);
					//send to network for processing
					client_send(&cpkto, 1);
					//client_send2(7, ep.bEndpointAddress, (unsigned char *)temp_data.io.data, rv);
				}
			}
		}
	}

	printf("End reading thread for EP%02x, thread id(%ld)\n",
		ep.bEndpointAddress, gettid());
	return NULL;
}

void process_eps(int fd, int desired_interface) {
	struct usb_raw_eps_info info;
	memset(&info, 0, sizeof(info));

	int num = usb_raw_eps_info(fd, &info);
	if (0||verbose_level) {
		for (int i = 0; i < num; i++) {
			printf("ep #%d:\n", i);
			printf("  name: %s\n", &info.eps[i].name[0]);
			printf("  addr: %u\n", info.eps[i].addr);
			printf("  type: %s %s %s\n",
				info.eps[i].caps.type_iso ? "iso" : "___",
				info.eps[i].caps.type_bulk ? "blk" : "___",
				info.eps[i].caps.type_int ? "int" : "___");
			printf("  dir : %s %s\n",
				info.eps[i].caps.dir_in ? "in " : "___",
				info.eps[i].caps.dir_out ? "out" : "___");
			printf("  maxpacket_limit: %u\n",
				info.eps[i].limits.maxpacket_limit);
			printf("  max_streams: %u\n", info.eps[i].limits.max_streams);
		}
	}

	struct raw_gadget_interface_descriptor temp_interface =
		host_config_desc[desired_configuration].interfaces[desired_interface].altsetting[0];
	ep_thread_list = new struct endpoint_thread[temp_interface.interface.bNumEndpoints];
	printf("bNumEndpoints is %d\n", static_cast<int>(temp_interface.interface.bNumEndpoints));

	for (int i = 0; i < temp_interface.interface.bNumEndpoints; i++) {
		int addr = usb_endpoint_num(&temp_interface.endpoints[i]);
		assert(addr != 0);

		ep_thread_list[i].ep_thread_info.fd = fd;
		ep_thread_list[i].ep_thread_info.endpoint = temp_interface.endpoints[i];
		ep_thread_list[i].ep_thread_info.data_queue = new std::deque<data_queue_info>;
		ep_thread_list[i].ep_thread_info.data_mutex = new std::mutex;

		switch (usb_endpoint_type(&temp_interface.endpoints[i])) {
		case USB_ENDPOINT_XFER_BULK:
			ep_thread_list[i].ep_thread_info.transfer_type = "bulk";
			break;
		case USB_ENDPOINT_XFER_INT:
			ep_thread_list[i].ep_thread_info.transfer_type = "int";
			break;
		default:
			printf("transfer_type is: %d\n", usb_endpoint_type(&temp_interface.endpoints[i]));
			assert(false);
		}

		if (usb_endpoint_dir_in(&temp_interface.endpoints[i]))
			ep_thread_list[i].ep_thread_info.dir = "in";
		else
			ep_thread_list[i].ep_thread_info.dir = "out";

		ep_thread_list[i].ep_thread_info.ep_num = usb_raw_ep_enable(fd,
					&ep_thread_list[i].ep_thread_info.endpoint);
		printf("%s_%s: addr = %u, ep = #%d, pktsize = %d, poll = %dms\n",
			ep_thread_list[i].ep_thread_info.transfer_type.c_str(),
			ep_thread_list[i].ep_thread_info.dir.c_str(),
			addr, ep_thread_list[i].ep_thread_info.ep_num,
			ep_thread_list[i].ep_thread_info.endpoint.wMaxPacketSize,
			ep_thread_list[i].ep_thread_info.endpoint.bInterval);

		if (verbose_level)
			printf("Creating thread for EP%02x\n",
				ep_thread_list[i].ep_thread_info.endpoint.bEndpointAddress);
		pthread_create(&ep_thread_list[i].ep_thread_read, 0,
			ep_loop_read, (void *) &ep_thread_list[i].ep_thread_info);
		pthread_create(&ep_thread_list[i].ep_thread_write, 0,
			ep_loop_write, (void *) &ep_thread_list[i].ep_thread_info);
	}

	printf("process_eps done\n");
}

void ep0_loop(int fd) {
	bool set_configuration_done_once = false;
	int previous_bConfigurationValue = -1;

	printf("Start for EP0, thread id(%ld)\n", gettid());
	while (!please_stop_ep0) {
		struct usb_raw_control_event event;
		event.inner.type = 0;
		event.inner.length = sizeof(event.ctrl);

		usb_raw_event_fetch(fd, (struct usb_raw_event *)&event);
		//log_event((struct usb_raw_event *)&event);

		if (event.inner.length == 4294967295) {
			printf("End for EP0, thread id(%ld)\n", gettid());
			return;
		}

		if (event.inner.type != USB_RAW_EVENT_CONTROL)
			continue;

		struct usb_raw_control_io io;
		io.inner.ep = 0;
		io.inner.flags = 0;
		io.inner.length = event.ctrl.wLength;

		int nbytes = 0;
		int result = 0;
		unsigned char *control_data = new unsigned char[event.ctrl.wLength];

		int rv = -1;
		if (event.ctrl.bRequestType & USB_DIR_IN) {
			result = control_request(&event.ctrl, &nbytes, &control_data, 1000);
			if (result == 0) {
				memcpy(&io.data[0], control_data, nbytes);
				io.inner.length = nbytes;
				rv = usb_raw_ep0_write(fd, (struct usb_raw_ep_io *)&io);
				//printf("ep0: transferred %d bytes (in)\n", rv);
			}
			else {
				printf("ep0: stalling\n");
				usb_raw_ep0_stall(fd);
			}
		}
		else {
			rv = usb_raw_ep0_read(fd, (struct usb_raw_ep_io *)&io);

			if (event.ctrl.bRequestType == 0x00 && event.ctrl.bRequest == 0x09) {
				// Set configuration
				if (previous_bConfigurationValue == event.ctrl.wValue) {
					printf("Skip changing configuration, wValue is same as previous\n");
					continue;
				}

				if (set_configuration_done_once) {
					// Need to stop all threads for eps and cleanup
					printf("Changing configuration\n");

					please_stop_eps = true;
					release_interface(0);

					int thread_num =
						host_config_desc[desired_configuration].interfaces[0].altsetting[0].interface.bNumEndpoints;
					for (int i = 0; i < thread_num; i++) {
						if (ep_thread_list[i].ep_thread_read &&
							pthread_join(ep_thread_list[i].ep_thread_read, NULL)) {
							fprintf(stderr, "Error join ep_thread_read\n");
						}
						if (ep_thread_list[i].ep_thread_write &&
							pthread_join(ep_thread_list[i].ep_thread_write, NULL)) {
							fprintf(stderr, "Error join ep_thread_write\n");
						}

						usb_raw_ep_disable(fd, ep_thread_list[i].ep_thread_info.ep_num);
					}

					set_configuration(event.ctrl.wValue);

					please_stop_eps = false;
				}

				for (int i = 0; i < host_device_desc.bNumConfigurations; i++) {
					if (host_config_desc[i].config.bConfigurationValue == event.ctrl.wValue) {
						desired_configuration = i;
						printf("Found desired configuration at index: %d\n", i);
					}
				}
				claim_interface(0);
				process_eps(fd, 0);

				set_configuration_done_once = true;
				previous_bConfigurationValue = event.ctrl.wValue;
			}
			else {
				memcpy(control_data, io.data, event.ctrl.wLength);
				result = control_request(&event.ctrl, &nbytes, &control_data, 1000);
				if (result == 0) {
					//printf("ep0: transferred %d bytes (out)\n", rv);
				}
				else {
					printf("ep0: stalling\n");
					usb_raw_ep0_stall(fd);
				}
			}
		}

		delete[] control_data;
	}

	printf("End for EP0, thread id(%ld)\n", gettid());
}
