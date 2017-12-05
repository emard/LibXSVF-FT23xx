/*
 *  XSVF_BANG - A software for bitbanging XSVF to JTAG through FTDI 2xx
 * 
 *  Copyright (C) 2014  Adam Li <adamli@hyervision.com>
 *  Copyright (C) 2009  RIEGL Research ForschungsGmbH
 *  Copyright (C) 2009  Clifford Wolf <clifford@clifford.at>
 *  
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/*
 *  This code is modified based on xsvftool-gpio.c from Lib(X)SVF.
 *  A typical computer does not have GPIO. In order to send bits in and
 *  out, a FTDI 2xx chip is used at its bitbang mode as specified in
 *  FTDI application notes AN_232R-01. For simplicity, synchronous 
 *  bitbang mode is used.
 * 
 *  The bitbang bits are mapped as following on FT232RL/FT245RL chips:
 * 
 *  	Bit 0:		Pin  1		TXD
 *  	Bit 1:		Pin  5		RXD
 *  	Bit 2:		Pin  3		RTS
 *  	Bit 3:		Pin 11		CTS
 *  	Bit 4:		Pin  2		DTR
 *  	Bit 5:		Pin  9		DSR
 *  	Bit 6:		Pin 10		DCD
 *  	Bit 7:		Pin  6		RI
 */

#define LINUX 1
#define WINDOWS 0

#include "libxsvf.h"
#if WINDOWS
#include "ftd2xx.h"
#endif


#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#if LINUX
#include <stdint.h>
#include <ftdi.h>
#endif

/** BEGIN: Low-Level I/O BitBang (bb) Implementation **/
#define DWORD uint32_t

// Define the pins used.
#define MASK_TMS (1<<7)	// Bit5, DSR, Output
#define MASK_TDI (1<<3)	// Bit0, TXD, Output
#define MASK_TDO (1<<6)	// Bit1, RXD, Input
#define MASK_TCK (1<<5)	// Bit6, DCD, Output

// Pin direction mask, 0 for input and 1 for output.
// TMS, TDI, and TCK are output pins. All others are input pins.
#define MASK_IO (MASK_TMS | MASK_TDI | MASK_TCK)

// The clock rate for bitbang is 16 times baudrate.
// It takes about 6 clock cycles for a read/write cycle.
#define BAUDRATE 62500

// The latency timer is how long the timer is to flush
// data in buffer into USB bus. The minimum number is 2ms.
#define LATENCY 1

// For simplicity, these variables are put here as static variables.
static unsigned char bb_reg = 0;
// static FT_HANDLE bb_handle = NULL;
struct ftdi_context bb_handle;

static void bb_setup(void)
{
	DWORD nDevs;

        #if WINDOWS
	// Check number of devices
	if (FT_OK != FT_CreateDeviceInfoList(&nDevs)) {
		fprintf(stderr, "FT_CreateDeviceInfoList error\n");
	}
	if (nDevs != 1) {
		printf("%ld FTDI 2xx devices found. This first devices will be used.\n", nDevs);
	}
	#endif

	// Open the port
	#if WINDOWS
	if (FT_OK != FT_Open(0, &bb_handle)) {
		fprintf(stderr, "FT_Open error\n");
	}
	#endif
	#if LINUX
	ftdi_init(&bb_handle);

	/* Open FTDI device based on FT232R vendor & product IDs */
	if(ftdi_usb_open(&bb_handle, 0x0403, 0x6001) < 0)
	{
		puts("Can't open device");
		return;
    	}
	#endif

	// Set baudrate
	#if WINDOWS
	if (FT_OK != FT_SetBaudRate(bb_handle, BAUDRATE)) {
		fprintf(stderr, "FT_SetBaudRate error\n");
	}
	#endif
	#if LINUX
	ftdi_set_baudrate(&bb_handle, BAUDRATE); /* Actually n * 16 */
	#endif

	
	// Set latency timer
	#if WINDOWS
	if (FT_OK != FT_SetLatencyTimer(bb_handle, LATENCY)) {
		fprintf(stderr, "FT_SetLatencyTimer error\n");
	}
	#endif
	#if LINUX
	ftdi_set_latency_timer(&bb_handle, LATENCY);
	#endif
	
	// Set bitbang mode (Synchronous BitBang 0x4)
	#if WINDOWS
	if (FT_OK != FT_SetBitMode(bb_handle, MASK_IO, 0x4)) {
		fprintf(stderr, "FT_SetBitMode error\n");
	}
	#endif
	#if LINUX
        ftdi_set_bitmode(&bb_handle, MASK_IO, 0x4);
	#endif
	
	// Reset register
	bb_reg = 0;
}

static void bb_shutdown(void)
{
	// Close the port
	// if (bb_handle)
	{
		// FT_SetBitMode(bb_handle, 0, 0);	// Reset the module
		ftdi_set_bitmode(&bb_handle, 0, 0);
		// FT_Close(bb_handle);
	}
	// bb_handle = NULL;
}

static void bb_tms(int val)
{
	// Set TMS in BB register
	if (val) bb_reg |= MASK_TMS;
	else bb_reg &= ~MASK_TMS;
}

static void bb_tdi(int val)
{
	// Set tdi in BB register
	if (val) bb_reg |= MASK_TDI;
	else bb_reg &= ~MASK_TDI;
}

static int bb_pulse_tck()
{
	// Pulse TCK and return the TDO value
	/* 
	 *  In FT2xx bitbang mode, the read and write operations are performed
	 *  in sequence. In other words, the lines are read first before the 
	 *  write operation. So for each TCK pulse, three writes are performed.
	 *  The first one is to set the output lines (TMS and TDI). The second
	 *  is to pulse the TCK high, and the third one to pulse the TCK low.
	 *  The third read bytes contains the TDO value after the raising edge
	 *  of TCK (before before the falling edge of TCK). 
	 */

	unsigned char buf_out[3], buf_in[3];
	DWORD lenWritten, lenRead;
	DWORD lenRX, lenTX, devStatus;
	
	buf_out[0] = bb_reg & ~MASK_TCK;
	buf_out[1] = bb_reg | MASK_TCK;
	buf_out[2] = buf_out[0];
	
	// Check for the queue length to be zero
	#if WINDOWS
	if (FT_OK != FT_GetStatus(bb_handle, &lenRX, &lenTX, &devStatus)) {
		fprintf(stderr, "FT_GetStatus error\n");
	}
	if (lenRX != 0 || lenTX != 0) {
		fprintf(stderr, "FT device queue is not zero (%ld, %ld)\n", lenTX, lenRX);
	}
	#endif
	
	// Write and read
	#if WINDOWS
	if (FT_OK != FT_Write(bb_handle, buf_out, 3, &lenWritten)) {
		fprintf(stderr, "FT_Write error\n");
	}
	if (FT_OK != FT_Read(bb_handle, buf_in, 3, &lenRead)) {
		fprintf(stderr, "FT_Read error\n");
	}
	if (lenWritten != 3 || lenRead != 3) {
		fprintf(stderr, "FT write read mismatch (%ld, %ld)\n", lenWritten, lenRead);
	}
	#endif
	#if LINUX
	ftdi_write_data(&bb_handle, buf_out, 3);
	ftdi_read_data(&bb_handle, buf_in, 3);
	#endif

	// Return the TDO value after the pulse
	return ((buf_in[2] & MASK_TDO) ? 1 : 0);
}

/** END: Low-Level I/O BitBang (bb) Implementation **/

struct udata_s {
	FILE *f;
	int verbose;
	int clockcount;
	int bitcount_tdi;
	int bitcount_tdo;
	int retval_i;
	int retval[256];
};

static int h_setup(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 2) {
		fprintf(stderr, "[SETUP]\n");
		fflush(stderr);
	}
	bb_setup();
	return 0;
}

static int h_shutdown(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 2) {
		fprintf(stderr, "[SHUTDOWN]\n");
		fflush(stderr);
	}
	bb_shutdown();
	return 0;
}

static void h_udelay(struct libxsvf_host *h, long usecs, int tms, long num_tck)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 3) {
		fprintf(stderr, "[DELAY:%ld, TMS:%d, NUM_TCK:%ld]\n", usecs, tms, num_tck);
		fflush(stderr);
	}
	if (num_tck > 0) {
		struct timeval tv1, tv2;
		gettimeofday(&tv1, NULL);
		bb_tms(tms);
		while (num_tck > 0) {
			//io_tck(0);
			//io_tck(1);
			bb_pulse_tck();
			num_tck--;
		}
		gettimeofday(&tv2, NULL);
		if (tv2.tv_sec > tv1.tv_sec) {
			usecs -= (1000000 - tv1.tv_usec) + (tv2.tv_sec - tv1.tv_sec - 1) * 1000000;
			tv1.tv_usec = 0;
		}
		usecs -= tv2.tv_usec - tv1.tv_usec;
		if (u->verbose >= 3) {
			fprintf(stderr, "[DELAY_AFTER_TCK:%ld]\n", usecs > 0 ? usecs : 0);
			fflush(stderr);
		}
	}
	if (usecs > 0) {
		usleep(usecs);
	}
}

static int h_getbyte(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	return fgetc(u->f);
}

static int h_pulse_tck(struct libxsvf_host *h, int tms, int tdi, int tdo, int rmask, int sync)
{
	struct udata_s *u = h->user_data;

	bb_tms(tms);

	if (tdi >= 0) {
		u->bitcount_tdi++;
		bb_tdi(tdi);
	}

	//io_tck(0);
	//io_tck(1);
	//int line_tdo = io_tdo();
	int line_tdo = bb_pulse_tck();
	int rc = line_tdo >= 0 ? line_tdo : 0;

	if (rmask == 1 && u->retval_i < 256)
		u->retval[u->retval_i++] = line_tdo;

	if (tdo >= 0 && line_tdo >= 0) {
		u->bitcount_tdo++;
		if (tdo != line_tdo)
			rc = -1;
	}

	if (u->verbose >= 4) {
		fprintf(stderr, "[TMS:%d, TDI:%d, TDO_ARG:%d, TDO_LINE:%d, RMASK:%d, RC:%d]\n", tms, tdi, tdo, line_tdo, rmask, rc);
	}

	u->clockcount++;
	return rc;
}

static void h_pulse_sck(struct libxsvf_host *h)
{
	fprintf(stderr, "WARNING: Pulsing SCK ignored!\n");
	/*
	struct udata_s *u = h->user_data;
	if (u->verbose >= 4) {
		fprintf(stderr, "[SCK]\n");
	}
	io_sck(0);
	io_sck(1);
	 */
}

static void h_set_trst(struct libxsvf_host *h, int v)
{
	fprintf(stderr, "WARNING: Setting TRST to %d ignored!\n", v);
	/*
	struct udata_s *u = h->user_data;
	if (u->verbose >= 4) {
		fprintf(stderr, "[TRST:%d]\n", v);
	}
	io_trst(v);
	 */
}

static int h_set_frequency(struct libxsvf_host *h, int v)
{
	fprintf(stderr, "WARNING: Setting JTAG clock frequency to %d ignored!\n", v);
	return 0;
}

static void h_report_tapstate(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 3) {
		fprintf(stderr, "[%s]\n", libxsvf_state2str(h->tap_state));
	}
}

static void h_report_device(struct libxsvf_host *h, unsigned long idcode)
{
	// struct udata_s *u = h->user_data;
	printf("idcode=0x%08lx, revision=0x%01lx, part=0x%04lx, manufactor=0x%03lx\n", idcode,
			(idcode >> 28) & 0xf, (idcode >> 12) & 0xffff, (idcode >> 1) & 0x7ff);
}

static void h_report_status(struct libxsvf_host *h, const char *message)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 2) {
		fprintf(stderr, "[STATUS] %s\n", message);
	}
}

static void h_report_error(struct libxsvf_host *h, const char *file, int line, const char *message)
{
	fprintf(stderr, "[%s:%d] %s\n", file, line, message);
}

static int realloc_maxsize[LIBXSVF_MEM_NUM];

static void *h_realloc(struct libxsvf_host *h, void *ptr, int size, enum libxsvf_mem which)
{
	struct udata_s *u = h->user_data;
	if (size > realloc_maxsize[which])
		realloc_maxsize[which] = size;
	if (u->verbose >= 3) {
		fprintf(stderr, "[REALLOC:%s:%d]\n", libxsvf_mem2str(which), size);
	}
	return realloc(ptr, size);
}

static struct udata_s u;

static struct libxsvf_host h = {
	.udelay = h_udelay,
	.setup = h_setup,
	.shutdown = h_shutdown,
	.getbyte = h_getbyte,
	.pulse_tck = h_pulse_tck,
	.pulse_sck = h_pulse_sck,
	.set_trst = h_set_trst,
	.set_frequency = h_set_frequency,
	.report_tapstate = h_report_tapstate,
	.report_device = h_report_device,
	.report_status = h_report_status,
	.report_error = h_report_error,
	.realloc = h_realloc,
	.user_data = &u
};

const char *progname;

static void copyleft()
{
	static int already_printed = 0;
	if (already_printed)
		return;
	fprintf(stderr, "xsvf_bang, XSVF bitbanging JTAG through FTDI 2xx.\n");
	fprintf(stderr, "Copyright (C) 2014  Adam Li <adamli@hyervision.com>\n");
	fprintf(stderr, "Copyright (C) 2009  RIEGL Research ForschungsGmbH\n");
	fprintf(stderr, "Copyright (C) 2009  Clifford Wolf <clifford@clifford.at>\n");
	already_printed = 1;
}

static void help()
{
	copyleft();
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: %s [ -r funcname ] [ -v ... ] [ -L | -B ] { -s svf-file | -x xsvf-file | -c } ...\n", progname);
	fprintf(stderr, "\n");
	fprintf(stderr, "   -r funcname\n");
	fprintf(stderr, "          Dump C-code for pseudo-allocator based on example files\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -v, -vv, -vvv, -vvvv\n");
	fprintf(stderr, "          Verbose, more verbose and even more verbose\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -L, -B\n");
	fprintf(stderr, "          Print RMASK bits as hex value (little or big endian)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -s svf-file\n");
	fprintf(stderr, "          Play the specified SVF file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -x xsvf-file\n");
	fprintf(stderr, "          Play the specified XSVF file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -c\n");
	fprintf(stderr, "          List devices in JTAG chain\n");
	fprintf(stderr, "\n");
	exit(1);
}

int main(int argc, char **argv)
{
	int rc = 0;
	int gotaction = 0;
	int hex_mode = 0;
	const char *realloc_name = NULL;
	int opt, i, j;

	progname = argc >= 1 ? argv[0] : "xvsf_bang";
	while ((opt = getopt(argc, argv, "r:vLBx:s:c")) != -1)
	{
		switch (opt)
		{
		case 'r':
			realloc_name = optarg;
			break;
		case 'v':
			copyleft();
			u.verbose++;
			break;
		case 'x':
		case 's':
			gotaction = 1;
			if (u.verbose)
				fprintf(stderr, "Playing %s file `%s'.\n", opt == 's' ? "SVF" : "XSVF", optarg);
			if (!strcmp(optarg, "-"))
				u.f = stdin;
			else
				u.f = fopen(optarg, "rb");
			if (u.f == NULL) {
				fprintf(stderr, "Can't open %s file `%s': %s\n", opt == 's' ? "SVF" : "XSVF", optarg, strerror(errno));
				rc = 1;
				break;
			}
			if (libxsvf_play(&h, opt == 's' ? LIBXSVF_MODE_SVF : LIBXSVF_MODE_XSVF) < 0) {
				fprintf(stderr, "Error while playing %s file `%s'.\n", opt == 's' ? "SVF" : "XSVF", optarg);
				rc = 1;
			}
			if (strcmp(optarg, "-"))
				fclose(u.f);
			break;
		case 'c':
			gotaction = 1;
			if (libxsvf_play(&h, LIBXSVF_MODE_SCAN) < 0) {
				fprintf(stderr, "Error while scanning JTAG chain.\n");
				rc = 1;
			}
			break;
		case 'L':
			hex_mode = 1;
			break;
		case 'B':
			hex_mode = 2;
			break;
		default:
			help();
			break;
		}
	}

	if (!gotaction)
		help();

	if (u.verbose) {
		fprintf(stderr, "Total number of clock cycles: %d\n", u.clockcount);
		fprintf(stderr, "Number of significant TDI bits: %d\n", u.bitcount_tdi);
		fprintf(stderr, "Number of significant TDO bits: %d\n", u.bitcount_tdo);
		if (rc == 0) {
			fprintf(stderr, "Finished without errors.\n");
		} else {
			fprintf(stderr, "Finished with errors!\n");
		}
	}

	if (u.retval_i) {
		if (hex_mode) {
			printf("0x");
			for (i=0; i < u.retval_i; i+=4) {
				int val = 0;
				for (j=i; j<i+4; j++)
					val = val << 1 | u.retval[hex_mode > 1 ? j : u.retval_i - j - 1];
				printf("%x", val);
			}
		} else {
			printf("%d rmask bits:", u.retval_i);
			for (i=0; i < u.retval_i; i++)
				printf(" %d", u.retval[i]);
		}
		printf("\n");
	}

	if (realloc_name) {
		int num = 0;
		for (i = 0; i < LIBXSVF_MEM_NUM; i++) {
			if (realloc_maxsize[i] > 0)
				num = i+1;
		}
		printf("void *%s(void *h, void *ptr, int size, int which) {\n", realloc_name);
		for (i = 0; i < num; i++) {
			if (realloc_maxsize[i] > 0)
				printf("\tstatic unsigned char buf_%s[%d];\n", libxsvf_mem2str(i), realloc_maxsize[i]);
		}
		printf("\tstatic unsigned char *buflist[%d] = {", num);
		for (i = 0; i < num; i++) {
			if (realloc_maxsize[i] > 0)
				printf("%sbuf_%s", i ? ", " : " ", libxsvf_mem2str(i));
			else
				printf("%s(void*)0", i ? ", " : " ");
		}
		printf(" };\n\tstatic int sizelist[%d] = {", num);
		for (i = 0; i < num; i++) {
			if (realloc_maxsize[i] > 0)
				printf("%ssizeof(buf_%s)", i ? ", " : " ", libxsvf_mem2str(i));
			else
				printf("%s0", i ? ", " : " ");
		}
		printf(" };\n");
		printf("\treturn which < %d && size <= sizelist[which] ? buflist[which] : (void*)0;\n", num);
		printf("};\n");
	}

	return rc;
}

