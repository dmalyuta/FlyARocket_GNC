/**
 * @file rpi_gpio_funcs.c
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief GPIO functions file
 *
 * This file contains GPIO low-level access functions. This file is a slightly modified
 * version of the code provided by Pieter-Jan Van de Maele <a href="http://www.pieter-jan.com/node/15">here</a>.
 */

# include <fcntl.h>
# include "rpi_gpio_header.h"

/**
 * @fn int map_peripheral(struct bcm2835_peripheral *p)
 * This function maps a GPIO port for access in the software.
 * Exposes the physical address defined in the passed structure using mmap on /dev/mem.
 *
 * @param p Pointer to the GPIO port structure.
 */
int map_peripheral(struct bcm2835_peripheral *p) {
   // Open /dev/mem
   if ((p->mem_fd = open("/dev/mem", O_RDWR | O_SYNC) ) < 0) { // NB : p->mem_fd is a shortcut commad for (*p).mem_fd
      printf("Failed to open /dev/mem, try checking permissions.\n");
      return -1;
   }

   p->map = mmap(
      NULL,
      BLOCK_SIZE,
      PROT_READ|PROT_WRITE,
      MAP_SHARED,
      p->mem_fd,      // File descriptor to physical memory virtual file '/dev/mem'
      p->addr_p       // Address in physical map that we want this memory block to expose
   );

   if (p->map == MAP_FAILED) {
        perror("mmap");
        return -1;
   }

   p->addr = (volatile unsigned int *)p->map;

   return 0;
}

/**
 * @fn void unmap_peripheral(struct bcm2835_peripheral *p)
 * This function unmaps a GPIO port.
 *
 * @param p Pointer to the GPIO port structure.
 */
void unmap_peripheral(struct bcm2835_peripheral *p) {
    munmap(p->map, BLOCK_SIZE);
    close(p->mem_fd);
}
