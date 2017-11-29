#include <iostream>
#include <unistd.h>
using namespace std;

class Copter{
    public:
       float gps;
        int x,y;

};

int  main(){
    Copter myCopter;
    myCopter.x = 1;
    myCopter.y = 2;
    myCopter.gps = 3.5;

    while(true){
        sleep(2);
        cout << "myCopter.x = "<< myCopter.x << endl; 
        cout << "myCopter.y = "<< myCopter.y << endl; 
        cout << "myCopter.gps = "<< myCopter.gps << endl; 
    }
    return 0;
}
