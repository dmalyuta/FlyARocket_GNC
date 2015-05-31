/**
 * @file spycam_header.h
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief Raspberry Pi spy camera header file.
 *
 * This is the header to spycam_funcs.c containing necessary function and variable
 * declarations.
 */

#ifndef SPYCAM_HEADER_H_
#define SPYCAM_HEADER_H_

extern pid_t pid; ///< Process ID variable for the Raspberry Pi spy camera video recording

/** @cond INCLUDE_WITH_DOXYGEN */
void startVideo(char *filename, char *options);
void stopVideo(void);
/** @endcond */

#endif /* SPYCAM_HEADER_H_ */
