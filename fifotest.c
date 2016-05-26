/*
 *  Serial FIFO Test Program
 *
 *  (C) Copyright 2016 Geert Uytterhoeven
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.
 */

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <bsd/stdlib.h>

#include <libbrahe/prng.h>

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <linux/serial.h>


#define DEFAULT_MAX_MSG_LEN	1024
#define MAX_MAX_MSG_LEN		4096

#define MAX_LIST_SIZE		64

#define TX_TIMEOUT		5
#define RX_TIMEOUT		5
#define RX_TIMEOUT_INIT		60

#define max(x, y) ({ \
	typeof(x) _x = (x);     \
	typeof(y) _y = (y);     \
	(void) (&_x == &_y);	\
	_x > _y ? _x : _y; })

#define min(x, y) ({ \
	typeof(x) _x = (x);     \
	typeof(y) _y = (y);     \
	(void) (&_x == &_y);	\
	_x < _y ? _x : _y; })

static const char *opt_txdev, *opt_rxdev;
static uint32_t opt_seed = 42;
static uint32_t opt_msglen = DEFAULT_MAX_MSG_LEN;
static uint32_t opt_nmsgs;
static uint32_t opt_speed;
static int opt_verbose;

#define ESC_BLACK	"\e[30m"
#define ESC_RED		"\e[31m"
#define ESC_GREEN	"\e[32m"
#define ESC_YELLOW	"\e[33m"
#define ESC_BLUE	"\e[34m"
#define ESC_PURPLE	"\e[35m"
#define ESC_CYAN	"\e[36m"
#define ESC_WHITE	"\e[37m"
#define ESC_RM		"\e[0m"

#define TAG_TX		ESC_BLUE "[tx] "
#define TAG_RX		ESC_PURPLE "[rx] "

static brahe_prng_state_t prng;

struct msg {
	struct msg *next;
	unsigned int len;
	unsigned char buf[0];
};

static pthread_t rx_thread, tx_thread;
static unsigned long long rx_bytes, tx_bytes;
static unsigned int msgs;

static const struct speed {
	speed_t sym;
	unsigned int val;
} speeds[] = {
	{ B0,		0 },
	{ B50,		50 },
	{ B75,		75 },
	{ B110,		110 },
	{ B134,		134 },
	{ B150,		150 },
	{ B200,		200 },
	{ B300,		300 },
	{ B600,		600 },
	{ B1200,	1200 },
	{ B1800,	1800 },
	{ B2400,	2400 },
	{ B4800,	4800 },
	{ B9600,	9600 },
	{ B19200,	19200 },
	{ B38400,	38400 },
#ifdef B57600
	{ B57600,	57600 },
#endif
#ifdef B115200
	{ B115200,	115200 },
#endif
#ifdef B230400
	{ B230400,	230400 },
#endif
#ifdef B460800
	{ B460800,	460800 },
#endif
#ifdef B500000
	{ B500000,	500000 },
#endif
#ifdef B576000
	{ B576000,	576000 },
#endif
#ifdef B921600
	{ B921600,	921600 },
#endif
#ifdef B1000000
	{ B1000000,	1000000 },
#endif
#ifdef B1152000
	{ B1152000,	1152000 },
#endif
#ifdef B1500000
	{ B1500000,	1500000 },
#endif
#ifdef B2000000
	{ B2000000,	2000000 },
#endif
#ifdef B2500000
	{ B2500000,	2500000 },
#endif
#ifdef B3000000
	{ B3000000,	3000000 },
#endif
#ifdef B3500000
	{ B3500000,	3500000 },
#endif
#ifdef B4000000
	{ B4000000,	4000000 },
#endif
};

static int get_speed_val(speed_t speed)
{
	unsigned int i;

	for (i = 0; i < sizeof(speeds)/sizeof(*speeds); i++)
		if (speeds[i].sym == speed)
			return speeds[i].val;
	return -1;
}

static speed_t get_speed_sym(unsigned speed)
{
	unsigned int i;

	for (i = 0; i < sizeof(speeds)/sizeof(*speeds); i++)
		if (speeds[i].val == speed)
			return speeds[i].sym;
	return -1;
}

static const char *thread_prefix(void)
{
	pthread_t self = pthread_self();

	if (self == rx_thread)
		return TAG_RX;
	if (self == tx_thread)
		return TAG_TX;

	return "";
}

#define pr_debug(fmt, ...) \
{ \
	if (opt_verbose) \
		printf("%s" fmt ESC_RM, thread_prefix(), ##__VA_ARGS__); \
}

#define pr_info(fmt, ...) \
	printf("%s" fmt ESC_RM, thread_prefix(), ##__VA_ARGS__)

#define pr_warn(fmt, ...) \
	printf("%s" ESC_YELLOW fmt ESC_RM, thread_prefix(), ##__VA_ARGS__)

#define pr_error(fmt, ...) \
	fprintf(stderr, "%s" ESC_RED fmt ESC_RM, thread_prefix(), ##__VA_ARGS__)

static struct msg *msg_gen(int len)
{
	struct msg *msg;
	unsigned int i;

	if (len < 0)
		len = brahe_prng_range(&prng, 1, -len);

	msg = malloc(sizeof(*msg) + len);
	memset(msg, 0, sizeof(*msg));

	msg->len = len;
	for (i = 0; i < len; i++)
		msg->buf[i] = brahe_prng_next(&prng);

	return msg;
}

static void print_stats(void)
{
	pr_warn("MSG: %u, TX: %llu bytes, RX: %llu bytes\n", msgs, tx_bytes,
		rx_bytes);
}

static void print_line(unsigned int index, const unsigned char *buf,
		       unsigned int len)
{
	unsigned int i;

	pr_info("%04x:", index);

	for (i = 0; i < len; i++)
		printf(" %02x", buf[i]);
	for (i = len; i < 16; i++)
		printf("   ");

	printf(" |");
	for (i = 0; i < len; i++)
		putchar(buf[i] >= 32 && buf[i] < 127 ? buf[i] : '.');
	puts("|");
}

static void print_buffer(const void *buf, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i += 16)
		print_line(i, buf + i, min(len - i, 16u));
}

static int cmp_line(unsigned int address, const unsigned char *buf1,
		    const unsigned char *buf2, unsigned int len)
{
	unsigned int i;
	unsigned char c;
	int res = 0;

	pr_info("%04x:", address);

	for (i = 0; i < len; i++)
		if (buf1[i] == buf2[i]) {
			printf(" %02x", buf1[i]);
		} else {
			printf(" " ESC_RED "%02x" ESC_RM, buf1[i]);
			res++;
		}
	for (i = len; i < 16; i++)
		printf("   ");

	printf(" |");
	for (i = 0; i < len; i++) {
		c = buf1[i] >= 32 && buf1[i] < 127 ? buf1[i] : '.';
		if (buf1[i] == buf2[i])
			putchar(c);
		else
			printf(ESC_RED "%c" ESC_RM, c);
	}
	puts("|");
	return res;
}

static void cmp_buffer(const void *buf1, const void *buf2, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i += 16) {
		if (!cmp_line(i, buf1 + i, buf2 + i, min(len - i, 16u)))
			continue;
		pr_info("Expected:\n");
		print_line(i, buf2 + i, min(len - i, 16u));

	}
}

static void msg_dump(const struct msg *msg)
{
	pr_info("Message with %u bytes of data\n", msg->len);
	print_buffer(msg->buf, msg->len);
}

static void signal_handler(int signum)
{
	print_stats();
	exit(-1);
}

static struct sigaction signal_action = {
	.sa_handler = signal_handler,
};

static void __attribute__ ((noreturn)) usage(void)
{
	fprintf(stderr,
		"\n"
		"%s: [options] <txdev> <rxdev>\n\n"
		"Valid options are:\n"
		"    -h, --help       Display this usage information\n"
		"    -i, --seed       Initial seed (zero is pseudorandom)\n"
		"    -l, --len        Maximum message length (default %u, must be <= %u)\n"
		"    -n               Number of messages to send (default zero is unlimited)\n"
		"    -s, --speed      Serial speed\n"
		"    -v, --verbose    Enable verbose mode\n"
		"\n",
		getprogname(), DEFAULT_MAX_MSG_LEN, MAX_MAX_MSG_LEN);
	exit(1);
}

static int device_open(const char *pathname, int flags, int makeraw)
{
	struct termios termios;
	int fd;

	pr_debug("Trying to open %s...\n", pathname);
	fd = open(pathname, flags);
	if (fd < 0) {
		pr_error("Failed to open %s%s: %s\n", pathname,
			 flags == O_WRONLY ? " for writing" :
			 flags == O_RDONLY ? " for reading" : "",
			 strerror(errno));
		exit(-1);
	}

	if (!makeraw)
		return fd;

	if (tcgetattr(fd, &termios)) {
		if (errno == ENOTTY) {
			pr_info("%s is not a tty, skipping tty config\n",
				pathname);
			return fd;
		}
		pr_error("Failed to get terminal attributes: %s\n",
			 strerror(errno));
		exit(-1);
	}
	pr_debug("termios.c_iflag = 0%o\n", termios.c_iflag);
	pr_debug("termios.c_oflag = 0%o\n", termios.c_oflag);
	pr_debug("termios.c_cflag = 0%o\n", termios.c_cflag);
	pr_debug("termios.c_lflag = 0%o\n", termios.c_lflag);

	cfmakeraw(&termios);
	if (tcsetattr(fd, TCSANOW, &termios)) {
		pr_error("Failed to enable raw mode: %s\n", strerror(errno));
		exit(-1);
	}

	if (opt_speed) {
		int sym = get_speed_sym(opt_speed);

		if (sym == -1) {
			pr_error("Unknown serial speed %u\n", opt_speed);
			exit(-1);
		}
		if (cfsetspeed(&termios, sym)) {
			pr_error("Failed to set terminal speed: %s\n",
				 strerror(errno));
			exit(-1);
		}
		if (tcsetattr(fd, TCSANOW, &termios)) {
			pr_error("Failed to set speed attribute: %s\n",
				 strerror(errno));
			exit(-1);
		}
	} else {
		pr_debug("Serial speed is %u/%u\n",
			 get_speed_val(cfgetispeed(&termios)),
			 get_speed_val(cfgetospeed(&termios)));
	}

	if (tcflush(fd, TCIOFLUSH)) {
		pr_error("Failed to flush: %s\n", strerror(errno));
		exit(-1);
	}

	return fd;
}

static void *transmit_start(void *arg)
{
	int fd = device_open(opt_txdev, O_WRONLY, 1);
	struct msg *msg = arg;
	ssize_t res;

	if (opt_verbose)
		msg_dump(msg);

	res = write(fd, msg->buf, msg->len);
	if (res < 0) {
		pr_error("Write error %d\n", errno);
		exit(-1);
	}
	tx_bytes += res;

	if (res < msg->len) {
		pr_error("Short write %zd < %u\n", res, msg->len);
		exit(-1);
	}

	close(fd);

	return NULL;
}

static void *receive_start(void *arg)
{
	int fd = device_open(opt_rxdev, O_RDONLY, 1);
	static unsigned char buf[MAX_MAX_MSG_LEN];
	unsigned int avail = 0, len;
	struct msg *msg = arg;
	ssize_t res;

	len = brahe_prng_range(&prng, 1, msg->len);
	pr_debug(ESC_GREEN "Receiving first %u bytes of message of size %u\n",
		 len, msg->len);

	while (avail < len) {
		res = read(fd, buf + avail, len - avail);
		if (res < 0) {
			pr_error("Read error %d\n", errno);
			exit(-1);
		}
		avail += res;
		rx_bytes += res;
	}

	if (memcmp(buf, msg->buf, len)) {
		pr_error("Data mismatch\n");
		cmp_buffer(buf, msg->buf, len);
		print_stats();
		exit(-1);
	}

	pr_debug(ESC_GREEN "OK\n");

	close(fd);

	return NULL;
}

int main(int argc, char *argv[])
{
	while (argc > 1) {
		if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
			usage();
		} else if (!strcmp(argv[1], "-i") ||
			   !strcmp(argv[1], "--seed")) {
			if (argc <= 2)
				usage();
			opt_seed = strtoul(argv[2], NULL, 0);
			argv++;
			argc--;
		} else if (!strcmp(argv[1], "-l") ||
			   !strcmp(argv[1], "--len")) {
			if (argc <= 2)
				usage();
			opt_msglen = strtoul(argv[2], NULL, 0);
			if (!opt_msglen || opt_msglen > MAX_MAX_MSG_LEN)
				usage();
			argv++;
			argc--;
		} else if (!strcmp(argv[1], "-n")) {
			if (argc <= 2)
				usage();
			opt_nmsgs = strtoul(argv[2], NULL, 0);
			argv++;
			argc--;
		} else if (!strcmp(argv[1], "-s") ||
			   !strcmp(argv[1], "--speed")) {
			if (argc <= 2)
				usage();
			opt_speed = strtoul(argv[2], NULL, 0);
			argv++;
			argc--;
		} else if (!strcmp(argv[1], "-v") ||
			   !strcmp(argv[1], "--verbose")) {
			opt_verbose = 1;
		} else if (!opt_txdev) {
			opt_txdev = argv[1];
		} else if (!opt_rxdev) {
			opt_rxdev = argv[1];
		} else {
			usage();
		}
		argv++;
		argc--;
	}

	if (!opt_rxdev)
		usage();

	brahe_prng_init(&prng, BRAHE_PRNG_MARSENNE_TWISTER, opt_seed);

	sigaction(SIGINT, &signal_action, NULL);

	for (msgs = 0; !opt_nmsgs || msgs < opt_nmsgs; msgs++) {
		struct timespec delay = { .tv_nsec = 100 * 1000 * 1000 };
		struct msg *msg = msg_gen(-opt_msglen);

		pthread_create(&rx_thread, NULL, receive_start, msg);

		/* Wait a bit to make sure the receiver thread has started */
		nanosleep(&delay, NULL);

		pthread_create(&tx_thread, NULL, transmit_start, msg);

		pthread_join(rx_thread, NULL);
		pthread_join(tx_thread, NULL);

		free(msg);
	}

	print_stats();

	exit(0);
}

