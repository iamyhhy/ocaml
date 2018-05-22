#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
struct yu{

    int x;
    int y;
    int z;
    int m;
    int n;
};

struct union_test {
    int x;
    int y;
  union Content {
      int v1;
      int v2;
      float v3;
      long v4;
      struct yu v5; /* &v5 == 0x1000 */
  }Content;
};

int main () {
  struct union_test myStruct;
  {
    SERIALIZE:
  }
  {
    DESERIALIZE:
  }
  /*
   
     union Content u;

     serailize on & (u.v5) once you find that v5 is the biggest
     in LVal 
  
    */

  return 0;
} 
