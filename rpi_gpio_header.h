/**
 * @file rpi_gpio_header.h
 * @author Danylo Malyuta <danylo.malyuta@gmail.com>
 * @version 1.0
 *
 * @brief MSP430 header file
 *
 * This is the header to rpi_gpio_funcs.c containing necessary definitions and
 * initializations. This header is a slightly reduced version (to the bare bones that
 * the GNC program needs) of the header provided by Pieter-Jan Van de Maele
 * <a href="http://www.pieter-jan.com/node/15">here</a>.
 */

#ifndef RPI_GPIO_HEADER_H_
#define RPI_GPIO_HEADER_H_

# include <stdio.h>
# include <sys/mman.h>
# include <sys/types.h>
# include <sys/stat.h>
# include <unistd.h>

# define BCM2708_PERI_BASE       0x20000000 ///< Physical address at which the peripheral registers start
# define GPIO_BASE               (BCM2708_PERI_BASE + 0x200000)	///< Address of the GPIO controller (expresset as an offset with respect to #BCM2708_PERI_BASE)

# define BLOCK_SIZE 		(4*1024)

/**
 * @struct bcm2835_peripheral
 * Holds what's needed to access the Raspberry Pi GPIO port.
 */
struct bcm2835_peripheral {
    unsigned long addr_p; ///< Address in physical map that we want this memory block to expose
    int mem_fd; ///< File descriptor to physical memory virtual file '/dev/mem'
    void *map;
    volatile unsigned int *addr; ///< Refers to register address
};

extern struct bcm2835_peripheral gpio; ///< The GPIO port access variable

// Always do an INP_GPIO(g) before doing an OUT_GPIO(g)
# define INP_GPIO(g)   *(gpio.addr + ((g)/10)) &= ~(7<<(((g)%10)*3)) ///< Set pin as input
# define GPIO_READ(g)  *(gpio.addr + 13) &= (1<<(g)) ///< Read an input pin's state

/** @cond INCLUDE_WITH_DOXYGEN */
int map_peripheral(struct bcm2835_peripheral *p);
void unmap_peripheral(struct bcm2835_peripheral *p);
/** @endcond */

#endif /* RPI_GPIO_HEADER_H_ */
