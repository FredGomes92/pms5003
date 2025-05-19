#include "pms5003.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

static int pms_fd = -1;


void convert_pms_data_to_le(const PMS5003_DATA *data, PMS5003_DATA_LE *data_le){
    if (data == NULL || data_le == NULL) {
        fprintf(stderr, "Null pointer passed to convert_pms_data__to_le\n");
        return;
    }
    data_le->f_length = (*(data->raw_data + 2) << 8) | *(data->raw_data + 3);
    data_le->pm1cf = (*(data->raw_data + 4) << 8) | *(data->raw_data + 5);
    data_le->pm2_5cf = (*(data->raw_data + 6) << 8) | *(data->raw_data + 7);
    data_le->pm10cf = (*(data->raw_data + 8) << 8) | *(data->raw_data + 9);
    data_le->pm1at = (*(data->raw_data + 10) << 8) | *(data->raw_data + 11);
    data_le->pm2_5at = (*(data->raw_data + 12) << 8) | *(data->raw_data + 13);
    data_le->pm10at = (*(data->raw_data + 14) << 8) | *(data->raw_data + 15);
    data_le->gt0_3 = (*(data->raw_data + 16) << 8) | *(data->raw_data + 17);
    data_le->gt0_5 = (*(data->raw_data + 18) << 8) | *(data->raw_data + 19);
    data_le->gt1 = (*(data->raw_data + 20) << 8) | *(data->raw_data + 21);
    data_le->gt2_5 = (*(data->raw_data + 22) << 8) | *(data->raw_data + 23);
    data_le->gt5 = (*(data->raw_data + 24) << 8) | *(data->raw_data + 25);
    data_le->gt10 = (*(data->raw_data + 26) << 8) | *(data->raw_data + 27);

    data_le->reserved = data->d.reserved;
    data_le->cksum = data->d.cksum;
}
int pms_init(const char *device, int baud) {
    struct termios options;

    pms_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (pms_fd < 0) {
        perror("Failed to open serial port");
        return -1;
    }

    tcgetattr(pms_fd, &options);
    cfmakeraw(&options);

    speed_t speed;
    switch (baud) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default:
            fprintf(stderr, "Unsupported baud rate\n");
            close(pms_fd);
            return -1;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(pms_fd, TCSANOW, &options);

    return 0;
}

void pms_close() {
    if (pms_fd >= 0) {
        close(pms_fd);
        pms_fd = -1;
    }
}

int read_pms_frame(PMS5003_DATA *data, double timeout_sec) {
    if (pms_fd < 0 || data == NULL) {
        return -1;
    }

    memset(data->raw_data, 0, PMS5003_EXPECTED_BYTES);

    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    int idx = 0;
    int state = 0;  // 0: looking for 0x42, 1: looking for 0x4D, 2: reading frame
    uint8_t byte;
    double elapsed = 0;

    printf("Waiting for frame...\n");
    int req_attempts = 0;

    while (elapsed < timeout_sec) {

        int n = read(pms_fd, &byte, 1);
        if (n == 1) {
            switch (state) {
                case 0:
                    if (byte == 0x42) {
                        data->raw_data[0] = byte;
                        idx = 1;
                        state = 1;
                    }
                    break;

                case 1:
                    if (byte == 0x4D) {
                        data->raw_data[1] = byte;
                        idx = 2;
                        state = 2;
                    } else {
                        // Reset if second byte isn't 0x4D
                        state = 0;
                        idx = 0;
                    }
                    break;

                case 2:
                    data->raw_data[idx++] = byte;
                    if (idx == PMS5003_EXPECTED_BYTES) {
                        // Step 1: Validate frame length
                        uint16_t frame_length = ((uint16_t)data->raw_data[2] << 8) | data->raw_data[3];
                        if (frame_length != 28) {
                            fprintf(stderr, "Invalid frame length: %u\n", frame_length);
                            return -1;
                        }

                        // Step 2: Validate checksum
                        uint16_t calc_cksum = 0;
                        for (int i = 0; i < PMS5003_EXPECTED_BYTES - 2; ++i) {
                            calc_cksum += data->raw_data[i];
                        }
                        uint16_t recv_cksum = ((uint16_t)data->raw_data[30] << 8) | data->raw_data[31];

                        if (calc_cksum != recv_cksum) {
                            fprintf(stderr, "Checksum mismatch: calc=0x%04X, recv=0x%04X\n", calc_cksum, recv_cksum);
                            return -1;
                        }

                        // Success
                        printf("Frame received and validated!\n");
                        print_buffer(data->raw_data, PMS5003_EXPECTED_BYTES);
                        return 0;
                    }
                    break;
            }
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("read");
            return -1;
        }

        usleep(1000);  // Sleep 1ms to reduce CPU load

        // Update elapsed time
        clock_gettime(CLOCK_MONOTONIC, &now);
        elapsed = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
    }

    fprintf(stderr, "Timeout reading full frame (%d bytes collected)\n", idx);
    return -1;
}



int read_pms_data(PMS5003_DATA *data) {
    if (pms_fd < 0 || data == NULL) {
        return -1;
    }

    memset(data->raw_data, 0, PMS5003_EXPECTED_BYTES);

    int8_t header[2];

    // -- Step 1: Find the header 0x42 0x4D --
    uint8_t byte;
    int sync = 0;
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    double elapse_time = 0;

    printf("Waiting for frame...\n");

    while (1) {
        // Timeout check
        clock_gettime(CLOCK_MONOTONIC, &now);
        elapse_time = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
        if (elapse_time >= 10.0) {
            fprintf(stderr, "Timeout waiting for header\n");
            flush_uart();
            reset_uart("/dev/ttyAMA0", 9600);

            return -1;
        }

        // req_frame_passive_mode();

        int n = read(pms_fd, &byte, 1);
        if (n != 1) {
            if (n < 0) {
                perror("read");
            } else {
                fprintf(stderr, "Unexpected read size: %d bytes\n", n);
                flush_uart();
                usleep(100000);
            }
            continue;
        }

        if (sync == 0 && byte == 0x42) {
            sync = 1;
            data->raw_data[0] = byte;
        } else if (sync == 1 && byte == 0x4D) {
            data->raw_data[1] = byte;
            break;
        } else {
            sync = 0;
        }
    }

    printf("Header: 0x%02X 0x%02X\n", data->raw_data[0], data->raw_data[1]);

    // -- Step 2: Read the rest of the frame --
    int bytes_read = 2;
    while (bytes_read < PMS5003_EXPECTED_BYTES) {
        int r = read(pms_fd, data->raw_data + bytes_read, PMS5003_EXPECTED_BYTES - bytes_read);
        if (r <= 0) {
            perror("read");
            return -1;
        }
        bytes_read += r;
    }
    printf("Read %d bytes\n", bytes_read);
    print_buffer(data->raw_data, PMS5003_EXPECTED_BYTES);

    // -- Step 3: Validate frame length --
    uint16_t frame_length = ((uint16_t)data->raw_data[2] << 8) | data->raw_data[3];
    if (frame_length != 28) {
        fprintf(stderr, "Invalid frame length: %u\n", frame_length);
        // return -1;
    }

    // -- Step 4: Validate checksum --
    uint16_t calculated_cksum = 0;
    for (int i = 0; i < PMS5003_EXPECTED_BYTES - 2; ++i) {
        calculated_cksum += data->raw_data[i];
    }

    uint16_t received_cksum = ((uint16_t)data->raw_data[30] << 8) | data->raw_data[31];
    if (calculated_cksum != received_cksum) {
        fprintf(stderr, "Checksum mismatch: calculated 0x%04X, received 0x%04X\n",
                calculated_cksum, received_cksum);
        // return -1;
    }
    flush_uart();

    // Valid frame
    return 0;
}


int send_pms5003_command(int fd, uint8_t command, uint8_t dataH, uint8_t dataL) {

    // int flags = fcntl(pms_fd, F_GETFL); // Get current flags
    // fcntl(pms_fd, F_SETFL, flags | O_NDELAY);

    uint8_t buffer[7];
    uint16_t checksum;

    // Construct frame
    buffer[0] = 0x42;
    buffer[1] = 0x4D;
    buffer[2] = command;
    buffer[3] = dataH;
    buffer[4] = dataL;

    // Calculate checksum
    checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4];
    buffer[5] = (checksum >> 8) & 0xFF;
    buffer[6] = checksum & 0xFF;

    // Send frame
    ssize_t written = write(fd, buffer, 7);
    if (written < 0) {
        perror("write");
        sleep(1); // Give it time to wake or recover
    }
    else if (written != 7) {
        perror("Failed to write command to PMS5003.");
        return -1;
    }

    printf("Sent command: 0x%02X with data: 0x%02X 0x%02X (checksum: 0x%02X 0x%02X)\n",
           command, dataH, dataL, buffer[5], buffer[6]);
    return 0;
}

void print_buffer(uint8_t *data, int size) {
    for (int i = 0; i < size; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}
void flush_uart()
{
    if (pms_fd < 0) {
        fprintf(stderr, "UART not initialized\n");
        return;
    }
    printf("Flushing UART\n");
    tcflush(pms_fd, TCIFLUSH);
}
void reset_uart(const char *device, int baud)
{
    if (pms_fd >= 0) {
        close(pms_fd);
    }
    pms_init(device, baud);
}

void set_active_mode()
{
    printf("Active command\n");
    send_pms5003_command(pms_fd, 0xE1, 0x00, 0x01);
}
void set_passive_mode()
{
    printf("Passive command\n");
    send_pms5003_command(pms_fd, 0xE1, 0x00, 0x00);
}
void req_frame_passive_mode()
{
    printf("Request frame command\n");
    send_pms5003_command(pms_fd, 0xE2, 0x00, 0x00);
}
void set_sleep_mode()
{
    printf("Sleep command\n");
    send_pms5003_command(pms_fd, 0xE4, 0x00, 0x00);
}
void set_wake_mode()
{
    printf("Wake-up command\n");
    send_pms5003_command(pms_fd, 0xE4, 0x00, 0x01);
}
