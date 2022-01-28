#define TERMINAL    "/dev/ttyUSB0"

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

char* buffTest = "$100,12,321,54,";

struct motorSpeed {
    unsigned int motorSpeed1;
    unsigned int motorSpeed2;
    unsigned int motorSpeed3;
    unsigned int motorSpeed4;
};

struct speedBuff {
    char motor1Buff[10];
    char motor2Buff[10];
    char motor3Buff[10];
    char motor4Buff[10];
};

void speed_parse(char* rxBuffer, struct motorSpeed* motorSpeedOut, struct speedBuff* speedBuff) 
{
    int j = 0;
    int i = 0;
    char motor1_buff[10];
    char motor2_buff[10];
    char motor3_buff[10];
    char motor4_buff[10];
    int index_of_comma[10];

    if(rxBuffer[0] != '$' && rxBuffer[strlen(rxBuffer)] != ',') {
        printf("Error received string!!\n");
        goto exit;
    }

    for (j = 0; j < strlen(rxBuffer); j++) {
        if (rxBuffer[j] == ',')
            index_of_comma[i++] = j;
    }

    strncpy(motor1_buff, rxBuffer + 1, index_of_comma[0] - 1);
    motor1_buff[index_of_comma[0] - 1] = '\0';
    motorSpeedOut->motorSpeed1 = atoi(motor1_buff);
    strcpy(speedBuff->motor1Buff, motor1_buff);

    strncpy(motor2_buff, rxBuffer + index_of_comma[0] + 1, index_of_comma[1] - index_of_comma[0] - 1);
    motor2_buff[index_of_comma[1] - index_of_comma[0] - 1] = '\0';
    motorSpeedOut->motorSpeed2 = atoi(motor2_buff);
    strcpy(speedBuff->motor2Buff, motor2_buff);

    strncpy(motor3_buff, rxBuffer + index_of_comma[1] + 1, index_of_comma[2] - index_of_comma[1] - 1);
    motor3_buff[index_of_comma[2] - index_of_comma[1] - 1] = '\0';
    motorSpeedOut->motorSpeed3 = atoi(motor3_buff);
    strcpy(speedBuff->motor3Buff, motor3_buff);

    strncpy(motor4_buff, rxBuffer + index_of_comma[2] + 1, index_of_comma[3] - index_of_comma[2] - 1);
    motor4_buff[index_of_comma[3] - index_of_comma[2] - 1] = '\0';
    motorSpeedOut->motorSpeed4 = atoi(motor4_buff);
    strcpy(speedBuff->motor4Buff, motor4_buff);
    
    exit: ;
}

int set_interface_attr(int fd, int speed)
{
    struct termios tty;


    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);


    tty.c_cflag |= (CLOCAL | CREAD);        /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8 bits character */
    tty.c_cflag &= ~PARENB;     /* No parity bits */
    tty.c_cflag &= ~CSTOPB;     /* 1 stop bit */
    //tty.c_cflag &= ~CRTSCTS;    /* No hardware flow control */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;


    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

int main()
{
    char *portname = TERMINAL;
    int fd, fd1, fd2, fdm1, fdm2, fdm3, fdm4;
    int ret;
    char *xstr = "Hello world from Tuan Trung!\n";
    int xlen = strlen(xstr);
    unsigned char rx_buffer[80];
    unsigned char run_buffer[80];
    int rdlen;
    unsigned char outputSpeedBuff[80];
    int write_count = 0;
    struct motorSpeed motorSpeedOut;
    struct speedBuff speedBuff;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    /* Config baud rate for UART communication */
    set_interface_attr(fd, B115200);

    /* Open file speedToRun */
    fd1 = open("speedToRun.txt", O_RDWR | O_CREAT);
    if (fd1 < 0) {
        printf("Error opening speedToRun.txt\n");
        return -1;
    }

    /* Read file speed to run */
    ret = read(fd1, outputSpeedBuff, sizeof(outputSpeedBuff));
    outputSpeedBuff[ret] = '\0';
    printf("SpeedToRead buffer: %s\n", outputSpeedBuff);
    lseek(fd1, 0, SEEK_SET);

    fdm1 = open("motor1.txt", O_RDWR | O_CREAT);
    if (fdm1 < 0) {
        printf("Error opening motor1.txt\n");
        return -1;
    }

    fdm2 = open("motor2.txt", O_RDWR | O_CREAT);
    if (fdm2 < 0) {
        printf("Error opening motor1.txt\n");
        return -1;
    }

    fdm3 = open("motor3.txt", O_RDWR | O_CREAT);
    if (fdm3 < 0) {
        printf("Error opening motor3.txt\n");
        return -1;
    }

    fdm4 = open("motor4.txt", O_RDWR | O_CREAT);
    if (fdm4 < 0) {
        printf("Error opening motor4.txt\n");
        return -1;
    }

    /* Send speed to UART */
    ret = write(fd, outputSpeedBuff, strlen(outputSpeedBuff) + 1);
    if (ret != (strlen(outputSpeedBuff) + 1)) {
        printf("Error sending speed to run\n");
        return -1;
    }

    ret = write(fd, xstr, xlen);

    if (ret != xlen) {
        printf("Error from write %d bytes with errno %d\n", ret, errno);
        return -1;
    }
    tcdrain(fd);

    fd2 = open("speedToRead.txt", O_RDWR | O_CREAT);
    if (fd2 < 0) {
        printf("Error opening speedToRead.txt\n");
        return -1;
    }
    lseek(fd2, 0, SEEK_SET);

    write(fd2, outputSpeedBuff, strlen(outputSpeedBuff));
    lseek(fd2, 0, SEEK_SET);

    do {
        rdlen = read(fd, rx_buffer, sizeof(rx_buffer) - 1);
        lseek(fd, 0, SEEK_SET);
        speed_parse(rx_buffer, &motorSpeedOut, &speedBuff);

        rdlen = read(fd1, run_buffer, sizeof(run_buffer) - 1);

        write(fd, run_buffer, strlen(run_buffer) + 1);
        if (rdlen >= 0) {
            rx_buffer[rdlen] = 0;
            printf("Read %d: %s \n", rdlen, rx_buffer);
            ret = write(fd2, rx_buffer, strlen(rx_buffer) - 1);
            write(fd2, "\n", 1);

            write(fdm1, speedBuff.motor1Buff, strlen(speedBuff.motor1Buff) + 1);
            write(fdm1, "\n", 1);

            write(fdm2, speedBuff.motor2Buff, strlen(speedBuff.motor2Buff) + 1);
            write(fdm2, "\n", 1);

            write(fdm3, speedBuff.motor3Buff, strlen(speedBuff.motor3Buff) + 1);
            write(fdm3, "\n", 1);

            write(fdm4, speedBuff.motor4Buff, strlen(speedBuff.motor4Buff) + 1);
            write(fdm4, "\n", 1);



            printf("Write %d bytes successfully\n", ret);
            if (ret < 0) {
                printf("Write error with %s\n", strerror(errno));
            }
            write_count++;
            printf("count: %d\n", write_count);
            if (write_count >= 60) {
                write_count = 0; 
                close(fd2);

                close(fdm1);
                close(fdm2);
                close(fdm3);
                close(fdm4);

                fdm1 = open("motor1.txt", O_RDWR | O_TRUNC);
                if (fdm1 < 0) {
                    printf("Error opening motor1.txt\n");
                    return -1;
                }
                lseek(fdm1, 0, SEEK_SET);

                fdm2 = open("motor2.txt", O_RDWR | O_TRUNC);
                if (fdm2 < 0) {
                    printf("Error opening motor1.txt\n");
                    return -1;
                }
                lseek(fdm2, 0, SEEK_SET);

                fdm3 = open("motor3.txt", O_RDWR | O_TRUNC);
                if (fdm3 < 0) {
                    printf("Error opening motor3.txt\n");
                    return -1;
                }
                lseek(fdm3, 0, SEEK_SET);

                fdm4 = open("motor4.txt", O_RDWR | O_TRUNC);
                if (fdm4 < 0) {
                    printf("Error opening motor4.txt\n");
                    return -1;
                }
                lseek(fdm4, 0, SEEK_SET);
                
                fd2 = open("speedToRead.txt", O_RDWR | O_TRUNC);
                if (fd2 < 0) {
                    printf("Error opening speedToRead.txt\n");
                    return -1;
                }
                lseek(fd2, 0, SEEK_SET);
            }
        } else if (rdlen < 0) {
            printf("Error from read %d: %s\n", rdlen, strerror(errno));
            return -1;
        }
    } while (1);

    close(fd2);
    return 0;
}