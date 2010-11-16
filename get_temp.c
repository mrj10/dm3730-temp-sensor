#include <stdio.h>
#include <unistd.h>

void dm3730_get_temp_start();
int dm3730_get_temp_check(float *temp_val);
float dm3730_get_temp_finish();

int main(int argc, char *argv[]) {
  float this_temp;
  int waiting_on_conversion = 0;
  while(1) {
    if(!waiting_on_conversion) {
      dm3730_get_temp_start();
      waiting_on_conversion = 1;
    }
    else {
      if(dm3730_get_temp_check(&this_temp)) { //Still waiting
      }
      else { //Conversion done
        printf("%fC\n", this_temp);
        waiting_on_conversion = 0;
      }
    }
    usleep(500000);
  }
  return 0;
}

      
