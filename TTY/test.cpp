#include "TTY.h"

int main(){
    TTY *my_tty = new TTY("/dev/USB0",B115200);
    char * message = "Hello,tty!";
    my_tty->SendData(message,sizeof(message));
    my_tty->ReceiveData(message,sizeof(message));
    printf("receive data:%s",message);
    return 0;
}