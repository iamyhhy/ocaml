#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
struct Copter{
       float gps;
        int x,y;

};

int main() {
    int x;
    struct Copter myCopter;
    myCopter.gps = 2.5;
    myCopter.x =1;
    myCopter.y = 2;
    {
SERIALIZE:
    }
    {
DESERIALIZE:
    }

    return 0;
} 

