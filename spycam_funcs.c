/**
 * @file spycam_funcs.c
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Raspberry Pi spy camera functions file.
 *
 * This file contains functions to start and stop camera recording with particular
 * settings. The code is taken directly from the very kindly provided code by
 * ceptimus <a href="http://ceptimus.co.uk/?p=91">here</a>.
 *
 * No license file nor text was provided with this source code, so none is included
 * here.
 */

# include <signal.h>
# include <unistd.h>
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include "spycam_header.h"

pid_t pid=0;

/**
 * @fn void startVideo(char *filename, char *options)
 * This function starts the Raspberry Pi camera recording with specific recording
 * options *options stored at the adress pointed to by options. If you want to enable
 * preview/monitoring then make the obvious change to remove the -n (no preview) option
 * from the source code of this function.
 *
 * @param filename A string specifying the name of the resulting video file (must have .h264 ending).
 * @param options Normal raspivid options. Avoid -t, -n, -o, and -s as the code fills those in for you.
 */
void startVideo(char *filename, char *options) {
	/* Description:
	 *
	 */
	if ((pid = fork()) == 0) {
		char **cmd;

		// count tokens in options string
		int count = 0;
		char *copy;
		copy = strdup(options);
		if (strtok(copy, " \t") != NULL) {
			count = 1;
			while (strtok(NULL, " \t") != NULL)
			count++;
		}

		cmd = malloc((count + 8) * sizeof(char **));
		free(copy);

		// if any tokens in options,
		// copy them to cmd starting at positon[1]
		if (count) {
			int kk;
			copy = strdup(options);
			cmd[1] = strtok(copy, " \t");
			for (kk = 2; kk <= count; kk++)
			cmd[kk] = strtok(NULL, " \t");
		}

		// add default options
		cmd[0] = "raspivid"; // executable name
		cmd[count + 1] = "-n"; // no preview
		cmd[count + 2] = "-t"; // default time (overridden by -s)
		   // but needed for clean exit
		cmd[count + 3] = "10"; // 10 millisecond (minimum) time for -t
		cmd[count + 4] = "-s"; // enable USR1 signal to stop recording
		cmd[count + 5] = "-o"; // output file specifer
		cmd[count + 6] = filename;
		cmd[count + 7] = (char *)0; // terminator
		execv("/usr/bin/raspivid", cmd);
	}
}

/**
 * @fn void stopVideo(void)
 * This functions stops the camera recording process.
 */
void stopVideo(void) {
	if (pid) {
		kill(pid, 10); // seems to stop with two signals separated
			   	   	   // by 1 second if started with -t 10 parameter
		sleep(1);
		kill(pid, 10);
	}
}
