//#include "Linux_orangepi.h"
//#include <termios.h>
//#include <fcntl.h>
//#include <unistd.h>
//#include <string.h>
//#include <time.h>
//#include <errno.h>
//#include <stdio.h>
//#include <stdlib.h>

//#define FRAME_HEADER1 0xFE
//#define FRAME_HEADER2 0xEE
//#define TIMEOUT_MS    1000

//typedef enum {
//    STATE_IDLE,
//    STATE_HEADER2,
//    STATE_PAYLOAD,
//    STATE_CRC
//} RecvState;

//static RecvState recv_state = STATE_IDLE;
//static uint8_t recv_buf[sizeof(MotorFrame)];
//static uint16_t recv_idx = 0;
//static struct timespec last_recv_time;


///**
//    @name: calculate_crc
//    @brief:CRC32计算
//**/
//static uint32_t calculate_crc(const uint8_t* data, size_t len) {
//    uint32_t crc = 0xFFFFFFFF;
//    for(size_t i = 0; i < len; ++i) {
//        crc ^= data[i];
//        for(int j = 0; j < 8; ++j) {
//            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
//        }
//    }
//    return ~crc;
//}


///**
//    @name: check_timeout
//    @brief:超时检测
//**/
//static int check_timeout() {
//    struct timespec now;
//    clock_gettime(CLOCK_MONOTONIC, &now);
//    long ms = (now.tv_sec - last_recv_time.tv_sec) * 1000;
//    ms += (now.tv_nsec - last_recv_time.tv_nsec) / 1000000;
//    return ms > TIMEOUT_MS;
//}


///**
//    @name: Linux_connet_init
//    @brief:Linux连接初始化
//**/
//int Linux_connet_init(const char* device, int baudrate) {
//    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
//    if (fd < 0) {
//        perror("open");
//        return -1;
//    }

//    struct termios tty;
//    memset(&tty, 0, sizeof(tty));
//    
//    // 波特率转换（支持更高波特率）
//    speed_t speed;
//    switch(baudrate) {
//        case 4800000:  speed = B4000000; break;
//        case 2000000:  speed = B2000000; break;
//        case 115200:   speed = B115200;  break;
//        default: 
//            fprintf(stderr, "Unsupported baudrate: %d\n", baudrate);
//            close(fd);
//            return -1;
//    }

//    cfsetispeed(&tty, speed);
//    cfsetospeed(&tty, speed);

//    tty.c_cflag &= ~PARENB;
//    tty.c_cflag &= ~CSTOPB;
//    tty.c_cflag |= CS8 | CREAD | CLOCAL;
//    tty.c_lflag = 0;
//    tty.c_oflag = 0;
//    tty.c_cc[VMIN]  = 0;  // 非阻塞模式
//    tty.c_cc[VTIME] = 5;  // 0.5秒超时

//    if(tcsetattr(fd, TCSANOW, &tty) != 0) {
//        perror("tcsetattr");
//        close(fd);
//        return -1;
//    }
//    tcflush(fd, TCIOFLUSH);
//    return fd;
//}


///**
//    @name: Linux_send
//    @brief:Linux发送数据
//**/
//int Linux_send(int fd, uint8_t id, uint8_t mode, 
//              float position, float torque,
//              float kp, float kd) 
//{
//    if (fd < 0) return -1;

//    MotorFrame frame = {
//        .header = {FRAME_HEADER1, FRAME_HEADER2},
//        .motor_id = id,
//        .mode = mode,
//        .modify_bit = 0xFF,
//        .position = (int32_t)(position * 131072.0f / (2*M_PI)),  // Q17格式
//        .torque = (int16_t)(torque * 256.0f),                    // Q8格式
//        .kp = (int16_t)(kp * 1024.0f),                          // Q10格式
//        .kd = (int16_t)(kd * 512.0f)                            // Q9格式
//    };

//    frame.crc32 = calculate_crc((uint8_t*)&frame, sizeof(frame)-4);

//    ssize_t sent = write(fd, &frame, sizeof(frame));
//    if (sent != sizeof(frame)) {
//        perror("Send failed");
//        return -1;
//    }

//    tcdrain(fd); // 确保数据发送完成
//    return 0;
//}


///**
//    @name: Linux_recv
//    @brief:Linux接收数据
//**/
//int Linux_recv(int fd, MotorData* data) {
//    if(fd < 0 || !data) return -1;

//    uint8_t byte;
//    while(read(fd, &byte, 1) > 0) {
//        clock_gettime(CLOCK_MONOTONIC, &last_recv_time);

//        switch(recv_state) {
//        case STATE_IDLE:
//            if(byte == FRAME_HEADER1) {
//                recv_buf[0] = byte;
//                recv_idx = 1;
//                recv_state = STATE_HEADER2;
//            }
//            break;

//        case STATE_HEADER2:
//            if(byte == FRAME_HEADER2) {
//                recv_buf[1] = byte;
//                recv_idx = 2;
//                recv_state = STATE_PAYLOAD;
//            } else {
//                recv_state = STATE_IDLE;
//            }
//            break;

//        case STATE_PAYLOAD:
//            recv_buf[recv_idx++] = byte;
//            if(recv_idx >= sizeof(MotorFrame)-4) {
//                recv_state = STATE_CRC;
//            }
//            break;

//        case STATE_CRC:
//            recv_buf[recv_idx++] = byte;
//            if(recv_idx == sizeof(MotorFrame)) {
//                uint32_t calc_crc = calculate_crc(recv_buf, sizeof(MotorFrame)-4);
//                uint32_t recv_crc = *(uint32_t*)(recv_buf + sizeof(MotorFrame)-4);

//                if(calc_crc == recv_crc) {
//                    MotorFrame* frame = (MotorFrame*)recv_buf;
//                    data->motor_id = frame->motor_id;
//                    data->q = (frame->position * 2 * M_PI) / 131072.0f;
//                    data->tau = frame->torque / 256.0f;
//                    data->Temp = frame->Temp;
//                    data->merror = frame->Merror;
//                    data->footForce = (frame->Force16 << 8) | frame->Force8;
//                    data->Acc = frame->Acc;
//                    data->correct = frame->correct;
//                    data->LW = (float)frame->LW;
//                    for (int i = 0; i < 3; i++) {
//                        uint8_t Fgyro_bytes[2];
//                        Fgyro_bytes[0] = (frame->Fgyro[i] >> 8) & 0xFF;
//                        Fgyro_bytes[1] = frame->Fgyro[i] & 0xFF;
//                        data->gyro[i] = *(float*)Fgyro_bytes;
//                        uint8_t Facc_bytes[2];
//                        Facc_bytes[0] = (frame->Facc[i] >> 8) & 0xFF;
//                        Facc_bytes[1] = frame->Facc[i] & 0xFF;
//                        data->acc[i] = *(float*)Facc_bytes;
//                    }
//                    recv_state = STATE_IDLE;
//                    return 1;
//                }
//                recv_state = STATE_IDLE;
//            }
//            break;
//        }

//        if(check_timeout()) {
//            recv_state = STATE_IDLE;
//            recv_idx = 0;
//        }
//    }
//    return 0;
//}


///**
//    @name: Linux_connet_close
//    @brief:Linux关闭接口
//**/
//void Linux_connet_close(int fd) {
//    if(fd >= 0) {
//        tcflush(fd, TCIOFLUSH);
//        close(fd);
//    }
//}    