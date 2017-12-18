#include <iostream>
#include <unistd.h>
using namespace std;

class Copter{
    public:
       float gps;
        float x,y;

};

extern "C" {
    #include <fcntl.h>
}
extern "C" void *_memoize[4096] ;
//extern "C" int  _memoizeMax ;
int _memoizeMax;

extern "C" void __serialize_Copter(struct Copter *ptr , int fd ) ;
extern "C" void __serialize_scalar32(float *ptr , int fd ) ;
extern "C" void __deserialize_Copter(struct Copter *ptr , int fd ) ;
extern "C" void __deserialize_scalar32(float *ptr , int fd ) ;
extern "C" void __serialize_Copter(struct Copter *ptr , int fd ) 
{ 


  {
  __serialize_scalar32(& ptr->gps, fd);
  __serialize_scalar32(& ptr->x, fd);
  __serialize_scalar32(& ptr->y, fd);
}
}
extern "C" void __serialize_scalar32(float *ptr , int fd ) 
{ 


  {
  write(fd, ptr, 4);
}
}
extern "C" void __deserialize_Copter(struct Copter *ptr , int fd ) 
{ 


  {
  __deserialize_scalar32(& ptr->gps, fd);
  __deserialize_scalar32(& ptr->x, fd);
  __deserialize_scalar32(& ptr->y, fd);
}
}
extern "C" void __deserialize_scalar32(float *ptr , int fd ) 
{ 


  {
  read(fd, ptr, 4);
}
}


int  main(){
    Copter myCopter;
    myCopter.x = 1;
    myCopter.y = 2;
    myCopter.gps = 3.5;
 int fd3 ;
  int fd4 ;


  {
  SERIALIZE: ;
  {
  fd3 = open("serialized.data", 514, 504);
  _memoizeMax = 0;
  __serialize_Copter(& myCopter, fd3);
  close(fd3);
  }
  }
  {
  DESERIALIZE: ;
  {
  fd4 = open("serialized.data", 514, 504);
  _memoizeMax = 0;
  __deserialize_Copter(& myCopter, fd4);
  close(fd4);
  }
  }


    while(true){
        sleep(2);
        cout << "myCopter.x = "<< myCopter.x << endl; 
        cout << "myCopter.y = "<< myCopter.y << endl; 
        cout << "myCopter.gps = "<< myCopter.gps << endl; 
    }
    return 0;
}

