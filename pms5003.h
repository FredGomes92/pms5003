#ifndef PMS5003_H
#define PMS5003_H

#include <stdint.h>

#define PMS5003_EXPECTED_BYTES 32

typedef union pms5003_data_block {
    struct {
        uint8_t start_ch0;
        uint8_t start_ch1;
        uint16_t f_length;
        uint16_t pm1cf;
        uint16_t pm2_5cf;
        uint16_t pm10cf;
        uint16_t pm1at;
        uint16_t pm2_5at;
        uint16_t pm10at;
        uint16_t gt0_3;
        uint16_t gt0_5;
        uint16_t gt1;
        uint16_t gt2_5;
        uint16_t gt5;
        uint16_t gt10;
        uint16_t reserved;
        uint16_t cksum;
    } d;
    uint8_t raw_data[PMS5003_EXPECTED_BYTES];
} PMS5003_DATA;

typedef struct pms5003_data_block_le {
    uint16_t f_length; // Frame length
    uint16_t pm1cf;   // PM1.0 CF=1
    uint16_t pm2_5cf; // PM2.5 CF=1
    uint16_t pm10cf;  // PM10 CF=1
    uint16_t pm1at;   // PM1.0 ATM
    uint16_t pm2_5at; // PM2.5 ATM
    uint16_t pm10at;  // PM10 ATM
    uint16_t gt0_3;   // >0.3um
    uint16_t gt0_5;   // >0.5um
    uint16_t gt1;     // >1.0um
    uint16_t gt2_5;   // >2.5um
    uint16_t gt5;     // >5.0um
    uint16_t gt10;    // >10um
    uint16_t reserved;
    uint16_t cksum;
} PMS5003_DATA_LE;

int pms_init(const char *device, int baud);
void pms_close();
int read_pms_data(PMS5003_DATA *data);
int read_pms_frame(PMS5003_DATA *data, double timeout_sec);
void convert_pms_data_to_le(const PMS5003_DATA *data, PMS5003_DATA_LE *data_le);
int send_pms5003_command(int fd, uint8_t command, uint8_t dataH, uint8_t dataL);
void print_buffer(uint8_t *data, int size);

void set_active_mode();
void set_passive_mode();
void req_frame_passive_mode();
void set_sleep_mode();
void set_wake_mode();
void flush_uart();
void reset_uart(const char *device, int baud);


#endif // PMS5003_H
