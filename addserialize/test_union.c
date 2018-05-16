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
      struct yu v5;
  };
};

int main () {
  struct union_test my_union_test;
/*  {
    SERIALIZE:
  }
  {
    DESERIALIZE:
  }
*/
  return 0;
} 
