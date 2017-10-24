#include <stdio.h>
#include <stdlib.h>
long l1=0x111111111;
long l2=0x222222222;

typedef struct bar{
    int bx;
    int by;
}bar;

typedef struct foo{
    int x;
    float y;
    bar z;
    char arr[8];
    struct bar * pbar;
   // struct bar ** ppbar;
}foo;

typedef struct wes {
        int w;
        int e;
        char s[]; 
} wes;


int main(){
    foo f;
    f.x = 1;
    f.y = 2.5;
    bar b;
    f.pbar = malloc(sizeof(bar));
    f.pbar->bx =1;
    f.pbar->by =2;
    b.bx = 3;
    b.by = 4;
    //f.ppbar = malloc(sizeof(struct bar *));
    
    printf("foo.x = %d\n",f.x);
HERE:
    f.z = b;

    {
        wes * wes_ptr ;
        wes_ptr = malloc(sizeof(struct wes) + 8); 
        wes_ptr->s[0] = 'a'; 
        wes_ptr->s[1] = 'b'; 
        wes_ptr->s[2] = 'c'; 
        wes_ptr->s[3] = 'd'; 
        wes_ptr->s[4] = 'e'; 
        wes_ptr->s[5] = 0; 


    } 
    return 0;
}
