#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

//Default settings

#define  B57600    0010001
#define  B115200   0010002
#define  B230400   0010003
#define  B460800   0010004
#define  B500000   0010005
#define  B576000   0010006
#define  B921600   0010007
#define  B1000000  0010010
#define  B1152000  0010011
#define  B1500000  0010012
#define  B2000000  0010013
#define  B2500000  0010014
#define  B3000000  0010015
#define  B3500000  0010016
#define  B4000000  0010017
#define __MAX_BAUD B4000000

/*
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 数据位
    tty.c_iflag &= ~IGNBRK;                     // 禁用 break 处理
    tty.c_lflag = 0;                            // 非规范模式
    tty.c_oflag = 0;                            // 禁用输出处理
    tty.c_cflag |= (CLOCAL | CREAD);            // 启用接收
    tty.c_cflag &= ~(PARENB | PARODD);          // 无奇偶校验
    tty.c_cflag &= ~CSTOPB;                     // 1 个停止位
    tty.c_cflag &= ~CRTSCTS;                    // 禁用硬件流控
*/


class TTY {
    public:
        TTY(char * device , int IoSpeed){
            this->fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);

            if (this->fd == -1) {
                perror("Error opening serial port");
                exit(114514);
            }
            if (tcgetattr(this->fd, &this->tty) != 0) {
                perror("Error getting terminal attributes");
                close(this->fd);
                exit(114514);
            }

            cfsetospeed(&this->tty, IoSpeed); // 设置输入波特率
            cfsetispeed(&this->tty, IoSpeed); // 设置输出波特率

            this->tty.c_cflag = (this->tty.c_cflag & ~CSIZE) | CS8; // 8 数据位
            this->tty.c_iflag &= ~IGNBRK;                     // 禁用 break 处理
            this->tty.c_lflag = 0;                            // 非规范模式
            this->tty.c_oflag = 0;                            // 禁用输出处理
            this->tty.c_cflag |= (CLOCAL | CREAD);            // 启用接收
            this->tty.c_cflag &= ~(PARENB | PARODD);          // 无奇偶校验
            this->tty.c_cflag &= ~CSTOPB;                     // 1 个停止位
            this->tty.c_cflag &= ~CRTSCTS;                    // 禁用硬件流控

            // 提交配置
            if (tcsetattr(this->fd, TCSANOW, &this->tty) != 0) {
                perror("Error setting terminal attributes");
                close(this->fd);
                exit(114514);
            }
        }
        TTY(char * device , int IoSpeed , struct termios default_tty){
            this->fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);

            if (this->fd == -1) {
                perror("Error opening serial port");
                exit(114514);
            }
            if (tcgetattr(this->fd, &default_tty) != 0) {
                perror("Error getting terminal attributes");
                close(this->fd);
                exit(114514);
            }

            cfsetospeed(&default_tty, IoSpeed); // 设置输入波特率
            cfsetispeed(&default_tty, IoSpeed); // 设置输出波特率
            // 提交配置
            if (tcsetattr(this->fd, TCSANOW, &default_tty) != 0) {
                perror("Error setting terminal attributes");
                close(this->fd);
                exit(114514);
            }
        }
       ~TTY(){
            close(this->fd);
        }
        template <typename T>
        bool SendData(T *data, size_t length){
            this->crc = CalculateCrc8(data,length);
            memcpy(data,&crc,sizeof(crc));
            int n = write(this->fd, data, length);
            if (n > 0) {
                printf("CRC valid. Data send correctly.\n");
                for (uint32_t i = 0; i < length; i++) {
                    printf("0x%02X ", data[i]);
                }
            }else{
                perror("Write Error:");
            }
        }
        template <typename T>
        bool ReceiveData(T *data, size_t length){
            int n = read(this->fd, data, length);
            if (n > 0) {
                if (this->ValidateCrc8(data, length)) {
                    printf("CRC valid. Data received correctly.\n");
                } else {
                    printf("CRC invalid. Data corrupted.\n");
                }
                for (uint32_t i = 0; i < length; i++) {
                    printf("0x%02X ", data[i]);
                }
            }else{
                perror("Read Error:");
            }
        }


    private:
        int fd;
        struct termios tty;
        uint8_t crc;
    protected:
        template <typename T>
        bool ValidateCrc8(T *data, size_t length){
            if (length < 2) {
                // the mini length is 2
                return false;
            }

            size_t dataLength = length - 1; // delete the last byte
            uint8_t calculated_crc = this->CalculateCrc8(data, dataLength); // calculate
            uint8_t received_crc = data[dataLength]; // the crc which we received

            return calculated_crc == received_crc; // compare them
        }
                template <typename T>
        bool CalculateCrc8(T *data, size_t length){
            uint8_t crc = 0x00; // 初始化 CRC 值
            uint8_t polynomial = 0x07; // 生成多项式 x^8 + x^2 + x + 1

            for (size_t i = 0; i < length; i++) {
                crc ^= data[i]; // 与输入数据异或
                for (uint8_t bit = 0; bit < 8; bit++) {
                    if (crc & 0x80) {
                        crc = (crc << 1) ^ polynomial; // 左移并与多项式异或
                    } else {
                        crc <<= 1; // 仅左移
                    }
                }
            }
            return crc;
        }
};