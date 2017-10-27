#include <unistd.h>
#include <fcntl.h>

int main() {
    int fd;
    fd = open("foo.txt", O_RDWR|O_CREAT, 0770); 
    write(fd, "hello", 5);
    close(fd); 
    return 0; 
} 
