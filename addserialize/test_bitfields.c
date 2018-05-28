#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <inttypes.h>

struct test_bitfield{
    uint8_t x:4;
    int y:4;
    int m:1;
    uint8_t n:1;
};


int main () {
  struct  test_bitfield myStruct;
  myStruct.x = 15;
  myStruct.y = 1;
  myStruct.m = 0;
  myStruct.n = 1;

  {
SERIALIZE:
  }
  {
DESERIALIZE:
  }


  return 0; 

} 
