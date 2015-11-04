/*
 * ux2cand.c - userspace daemon for usb CAN device
 *
 * Author: 2010 sygi <sygi@canbus.pl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This code is derived from an example daemon code from
 *
 * http://en.wikipedia.org/wiki/Daemon_(computer_software)#Sample_Program_in_C_on_Linux
 * (accessed 2009-05-05)
 *
 * So it is additionally licensed under the GNU Free Documentation License:
 *
 * Permission is granted to copy, distribute and/or modify this document
 * under the terms of the GNU Free Documentation License, Version 1.2
 * or any later version published by the Free Software Foundation;
 * with no Invariant Sections, no Front-Cover Texts, and no Back-Cover Texts.
 * A copy of the license is included in the section entitled "GNU
 * Free Documentation License".
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>
#include <pwd.h>
#include <signal.h>

#define WINTYPES
#include "usb2can.h"

/* Change this to whatever your daemon is called */
#define DAEMON_NAME "ux2cand"

/* Change this to the user under which to run */
#define RUN_AS_USER "root"

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

void print_usage(char *prg)
{
	fprintf(stderr, "\nUsage: %s [dev number] [bitrate-kbps]\n\n", prg);
	fprintf(stderr, "Example:\n");
	fprintf(stderr, "%s 1 1000\n", prg);
	fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

static void child_handler (int signum)
{
	switch (signum)
	{
	case SIGALRM:
		exit (EXIT_FAILURE);
		break;
	case SIGUSR1:
		exit (EXIT_SUCCESS);
		break;
	case SIGCHLD:
		exit (EXIT_FAILURE);
		break;
	}
}

static void daemonize (const char *lockfile, char *dev)
{
	pid_t pid, sid, parent;
	int lfp = -1;
	FILE * pFile;
	char const *pidprefix = "/var/run/";
	char const *pidsuffix = ".pid";
	char *pidfile;

	pidfile = malloc((strlen(pidprefix) + strlen(DAEMON_NAME) +
			strlen(dev) + strlen(pidsuffix) + 1) * sizeof(char));
	sprintf(pidfile, "%s%s-%s%s", pidprefix, DAEMON_NAME, dev, pidsuffix);

	/* already a daemon */
	if (getppid () == 1)
		return;

	/* Create the lock file as the current user */
	if (lockfile && lockfile[0])
	{
		lfp = open (lockfile, O_RDWR | O_CREAT, 0640);
		if (lfp < 0)
		{
			syslog (LOG_ERR, "unable to create lock file %s, code=%d (%s)",
				lockfile, errno, strerror (errno));
			exit (EXIT_FAILURE);
		}
	}

	/* Drop user if there is one, and we were run as root */
	if (getuid () == 0 || geteuid () == 0)
	{
		struct passwd *pw = getpwnam (RUN_AS_USER);
		if (pw)
		{
			syslog (LOG_NOTICE, "setting user to " RUN_AS_USER);
			setuid (pw->pw_uid);
		}
	}

	/* Trap signals that we expect to receive */
	signal (SIGCHLD, child_handler);
	signal (SIGUSR1, child_handler);
	signal (SIGALRM, child_handler);

	/* Fork off the parent process */
	pid = fork ();
	if (pid < 0)
	{
		syslog (LOG_ERR, "unable to fork daemon, code=%d (%s)",
			errno, strerror (errno));
		exit (EXIT_FAILURE);
	}
	/* If we got a good PID, then we can exit the parent process. */
	if (pid > 0)
	{

		/* Wait for confirmation from the child via SIGTERM or SIGCHLD, or
		   for two seconds to elapse (SIGALRM).  pause() should not return. */
		alarm (2);
		pause ();

		exit (EXIT_FAILURE);
	}

	/* At this point we are executing as the child process */
	parent = getppid ();

	/* Cancel certain signals */
	signal (SIGCHLD, SIG_DFL);	/* A child process dies */
	signal (SIGTSTP, SIG_IGN);	/* Various TTY signals */
	signal (SIGTTOU, SIG_IGN);
	signal (SIGTTIN, SIG_IGN);
	signal (SIGHUP, SIG_IGN);	/* Ignore hangup signal */
	signal (SIGTERM, SIG_DFL);	/* Die on SIGTERM */

	/* Change the file mode mask */
	umask (0);

	/* Create a new SID for the child process */
	sid = setsid ();
	if (sid < 0)
	{
		syslog (LOG_ERR, "unable to create a new session, code %d (%s)",
			errno, strerror (errno));
		exit (EXIT_FAILURE);
	}

	pFile = fopen (pidfile,"w");
	if (pFile < 0)
	{
		syslog (LOG_ERR, "unable to create pid file %s, code=%d (%s)",
			pidfile, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}
	fprintf (pFile, "%d\n", sid);
	fclose (pFile);

	/* Change the current working directory.  This prevents the current
	   directory from being locked; hence not being able to remove it. */
	if ((chdir ("/")) < 0)
	{
		syslog (LOG_ERR, "unable to change directory to %s, code %d (%s)",
			"/", errno, strerror (errno));
		exit (EXIT_FAILURE);
	}

	/* Redirect standard files to /dev/null */
	freopen ("/dev/null", "r", stdin);
	freopen ("/dev/null", "w", stdout);
	freopen ("/dev/null", "w", stderr);

	/* Tell the parent process that we are A-okay */
	kill (parent, SIGUSR1);
}

static void hexstring2array(char *hexs, int slen, unsigned char *tab)
{
	int i, j = 0;
	unsigned char val;

	for (i = 0; i < slen; i++) {
		val = hexs[j] > '9' ? hexs[j] - 0x37 : hexs[j] - 0x30;
		tab[i] = val << 4;
		val = hexs[j+1] > '9' ? hexs[j+1] - 0x37 : hexs[j+1] - 0x30;
		tab[i] |= val & 0x0F;
		j += 2;
	}
}

int main (int argc, char *argv[])
{
	char const *devprefix = "/sys/devices/platform/ux2can.";
	char *dev;
	char *path;
	CANCTRLMSG f;
	CANCTRLSTATUS s;
	USB2CAN_LIBSTATUS retval;
	unsigned char dlc;
	char out[30];
	char buf[10];
	int i, ret;
	unsigned char in[40];
	int fd_fifo;
	int fd_status;
	int fd_conf;
	char status = -1;
	char *rate;
	int sleeper = 0;
	char fifolink[100];
	char statuslink[100];
	char conflink[100];

	/* Initialize the logging interface */
	openlog (DAEMON_NAME, LOG_PID, LOG_LOCAL5);

	/* See how we're called */
	if (argc != 3)
		print_usage(argv[0]);
	dev = argv[1];
	rate = argv[2];

	path = malloc((strlen(devprefix) + strlen(dev)) * sizeof(char));
	sprintf (path, "%s%s", devprefix, dev);
	syslog (LOG_INFO, "starting on ux2can.%s device at %d[kbps]", dev, atoi(rate));

	/* Daemonize */
	daemonize ("/var/lock/" DAEMON_NAME, dev);

	sprintf(fifolink, "%s%s", path, "/fifo");
	// open descriptors
	if ((fd_fifo = open (fifolink, O_RDONLY)) < 0) {
		perror(path);
		syslog (LOG_INFO, "unable to open ux2can.%s fifo", dev);
		exit(1);
	}

	close(fd_fifo);
	sprintf(statuslink, "%s%s", path, "/status");
	if ((fd_status = open (statuslink, O_WRONLY)) < 0) {
		perror(path);
		syslog (LOG_INFO, "unable to open ux2can.%s status", dev);
		exit(1);
	}

	// init lib and can device
	Usb2Can_InitLib(NULL);
	retval = Usb2Can_Open(atoi(rate), CANCTRL_MODE_LOOPBACK, CANCTRL_PRIORITY_NORMAL, 256);
	if (retval != USB2CAN_OK) {
		syslog (LOG_INFO, "unable to open USB2CAN device");
		exit(1);
	}

	sprintf(conflink, "%s%s", path, "/conf");
	if ((fd_conf = open (conflink, O_WRONLY)) < 0) {
		perror(path);
		syslog (LOG_INFO, "unable to open ux2can.%s conf", dev);
		exit(1);
	}

	sprintf(buf, "%d", atoi(rate));
	write(fd_conf, buf, strlen(buf));
	close(fd_conf);

	/* Main loop */
	while (1) {
		sleeper = 0;

		// pull frames from the can device
		// and write them to fifo pipe in correct format - see ux2can kernel driver
		if (Usb2Can_Pull(&f) == USB2CAN_OK) {
			dlc = f.frameInfo & CANCTRL_FRAME_DLC;
			if (f.frameInfo & CANCTRL_FRAME_EXT)
				f.id |= 1<<31;
			if (f.frameInfo & CANCTRL_FRAME_RTR)
				f.id |= 1<<30;
			sprintf(out, "%08X%02X", f.id, dlc);
			for (i = 0; i < dlc; i++) {
				sprintf(buf, "%X", f.data[i]);
				strcat(out, buf);
			}
			fd_fifo = open (fifolink, O_WRONLY);
			write(fd_fifo, out, strlen(out));
			close(fd_fifo);
			syslog (LOG_INFO, "frame received by device");
			sleeper++;
		}

		// read fifo pipe
		// and push them to can device in correct format
		fd_fifo = open (fifolink, O_RDONLY);
		ret = read(fd_fifo, out, 30); // try to read max
		close(fd_fifo);
		if (ret > 0) {
			hexstring2array(out, strlen(out), in);
			syslog (LOG_INFO, "frame received: %s len=%d", out, strlen(out));

			f.id = (in[0] << 24) & 0x1FFFFFFF;
			f.id |= in[1] << 16;
			f.id |= in[2] << 8;
			f.id |= in[3];
			f.frameInfo = in[4];
			dlc = f.frameInfo;
			if (in[0] & (1<<31))
				f.frameInfo |= CANCTRL_FRAME_EXT;
			if (in[0] & (1<<30))
				f.frameInfo |= CANCTRL_FRAME_RTR;
			for (i = 0; i < dlc; i++)
				f.data[i] = in[5 + i];
			Usb2Can_Push(f);
			//fd_fifo = open (fifolink, O_WRONLY);
			//write(fd_fifo, out, strlen(out));
			//close(fd_fifo);
			syslog (LOG_INFO, "frame pushed to device ID=0x%x dlc=%d ", f.id, dlc);
			sleeper++;
		}

		// read can device status
		// and write it to status pipe in correct format
		Usb2Can_BusStatus(&s);
		if (s.state != status) {
			status = s.state;
			sprintf(out, "%X", status);
			write(fd_status, out, strlen(out));
			sleeper++;
		}

		// if there is no activity sleep 100ms otherwise sleep 10ms
		if (sleeper)
			usleep(1000);
		else
			usleep(100000);
	}

	// close can device and release lib
	Usb2Can_Close();
	Usb2Can_ReleaseLib();

	/* Finish up */
	close(fd_fifo);
	close(fd_status);
	syslog (LOG_NOTICE, "terminated on %s", path);
	closelog ();
	return 0;
}
