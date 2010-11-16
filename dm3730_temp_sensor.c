#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
__LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

#define DM3730_TEMP_CLOCK_ADDR 0x48004A08
#define DM3730_TEMP_REG_ADDR 0x48002524

static const float dm3730_temp_adc_table[128] = {
   -40,    -40,    -40,    -40,   -40,   -40,    -40,    -40, //7
   -40,    -40,    -40,    -40,   -40,   -39,  -36.5,  -34.5, //15
   -33,    -31,    -29,    -27,   -25,   -23,    -21, -19.25, //23
-17.75,    -16, -14.25, -12.75,   -11,    -9,  -7.25,  -5.75, //31
 -4.25,   -2.5,  -0.75,      1,  2.75,  4.25,   5.75,    7.5, //39
  9.25,     11,  12.75,  14.25,    16,    18,     20,     22, //47
    24,     26,  27.75,  29.25,    31, 32.75,  34.25,     36, //55
 37.75,  39.25,     41,  42.75, 44.25,    46,  47.75,  49.25, //63
    51,  52.75,  54.25,     56, 57.75, 59.25,     61,     63, //71
    65,     67,     69,  70.75,  72.5, 74.25,     76,  77.75, //79
 79.25,     81,  82.75,  84.25,    86, 87.75,  89.25,     91, //87
 92.75,  94.25,     96,  97.75, 99.25,   101, 102.75, 104.25, //95
   106,    108,    110,    112,   114,   116, 117.75, 119.25, //103
   121, 122.75, 124.25,    125,   125,   125,    125,    125, //111
   125,    125,    125,    125,   125,   125,    125,    125, //119
   125,    125,    125,    125,   125,   125,    125,    125  //127
}; 

static int dm3730_temp_initialized = 0;

//Caller should avoid calling this if dm3730_temp_initialized is 1.
static void dm3730_temp_init() {
  int dm3730_temp_devmem_fd;
  void *dm3730_temp_clock_mapbase, *dm3730_temp_reg_mapbase;

  if((dm3730_temp_devmem_fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;

  dm3730_temp_clock_mapbase = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, dm3730_temp_devmem_fd, DM3730_TEMP_CLOCK_ADDR & ~MAP_MASK);
  if(dm3730_temp_clock_mapbase == (void *) -1) FATAL;
  volatile void *clock_vaddr = dm3730_temp_clock_mapbase + (DM3730_TEMP_CLOCK_ADDR & MAP_MASK);

  dm3730_temp_reg_mapbase = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, dm3730_temp_devmem_fd, DM3730_TEMP_REG_ADDR & ~MAP_MASK);
  if(dm3730_temp_reg_mapbase == (void *) -1) FATAL;
  volatile void *reg_vaddr = dm3730_temp_reg_mapbase + (DM3730_TEMP_REG_ADDR & MAP_MASK);

  *((volatile unsigned long *)reg_vaddr) = 0x00000000UL;

  volatile unsigned long clock_val = *((volatile unsigned long *) clock_vaddr);
  *((volatile unsigned long *) clock_vaddr) = clock_val | 0x00000002UL;

  if(munmap(dm3730_temp_reg_mapbase, MAP_SIZE) == -1) FATAL;
  if(munmap(dm3730_temp_clock_mapbase, MAP_SIZE) == -1) FATAL;
  close(dm3730_temp_devmem_fd);
  dm3730_temp_initialized = 1;
}

void dm3730_get_temp_start() {
  int dm3730_temp_devmem_fd;
  void *dm3730_temp_reg_mapbase;
  if(!dm3730_temp_initialized)
    dm3730_temp_init();

  if((dm3730_temp_devmem_fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;

  dm3730_temp_reg_mapbase = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, dm3730_temp_devmem_fd, DM3730_TEMP_REG_ADDR & ~MAP_MASK);
  if(dm3730_temp_reg_mapbase == (void *) -1) FATAL;
  volatile void *reg_vaddr = dm3730_temp_reg_mapbase + (DM3730_TEMP_REG_ADDR & MAP_MASK);

  *((volatile unsigned long *)reg_vaddr) = 0x00000000UL; //Should already be at 0, but we'll make sure.

  *((volatile unsigned long *)reg_vaddr) = 0x00000200UL; //Set bit 9 to start the conversion

  //Spin until bit 8 is set.  Otherwise, dm3730_get_temp_finish() won't know whether bit 8 clear means pre- or post-conversion.
  while(!((*((volatile unsigned long *)reg_vaddr)) & 0x00000100)) { }

  *((volatile unsigned long *)reg_vaddr) = 0x00000000UL; //Clear bit 9 to enable the next posedge to start the next conversion.
  if(munmap(dm3730_temp_reg_mapbase, MAP_SIZE) == -1) FATAL;
  close(dm3730_temp_devmem_fd);
}

//If conversion is done, returns 0 and places temperature value in *temp_val.
//If conversion is not done, returns -1.
int dm3730_get_temp_check(float *temp_val) {
  int dm3730_temp_devmem_fd;
  void *dm3730_temp_reg_mapbase;
  int retval;
  if((dm3730_temp_devmem_fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;

  dm3730_temp_reg_mapbase = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, dm3730_temp_devmem_fd, DM3730_TEMP_REG_ADDR & ~MAP_MASK);
  if(dm3730_temp_reg_mapbase == (void *) -1) FATAL;
  volatile void *reg_vaddr = dm3730_temp_reg_mapbase + (DM3730_TEMP_REG_ADDR & MAP_MASK);
  volatile unsigned long reg_val;
  if((reg_val = (*((volatile unsigned long *)reg_vaddr))) & 0x00000100UL) {
    retval = -1;
    goto end;
  }
  else {
    *temp_val = dm3730_temp_adc_table[reg_val & 0x7FU];
    retval = 0;
    goto end;
  }
  end:
  if(munmap(dm3730_temp_reg_mapbase, MAP_SIZE) == -1) FATAL;
  close(dm3730_temp_devmem_fd);
  return retval;
}

//Will block until temperature conversion is finished and return the result.
float dm3730_get_temp_finish() {
  int dm3730_temp_devmem_fd;
  void *dm3730_temp_reg_mapbase;

  if((dm3730_temp_devmem_fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;

  dm3730_temp_reg_mapbase = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, dm3730_temp_devmem_fd, DM3730_TEMP_REG_ADDR & ~MAP_MASK);
  if(dm3730_temp_reg_mapbase == (void *) -1) FATAL;
  volatile void *reg_vaddr = dm3730_temp_reg_mapbase + (DM3730_TEMP_REG_ADDR & MAP_MASK);

  unsigned long reg_val;
  while((reg_val = (*((unsigned long *)reg_vaddr))) & 0x00000100UL) { } //Spin until bit 8 is cleared, signalling the end of the conversion
  if(munmap(dm3730_temp_reg_mapbase, MAP_SIZE) == -1) FATAL;
  close(dm3730_temp_devmem_fd);
  return dm3730_temp_adc_table[reg_val & 0x7FU];
}

float get_dm3730_temp_block() {
  dm3730_get_temp_start();
  return dm3730_get_temp_finish();
}
