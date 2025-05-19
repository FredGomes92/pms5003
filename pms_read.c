#include "pms5003.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>

void print_data(const PMS5003_DATA *data) {
    printf("Parsed PMS5003 Data:\n");
    printf("Frame length: %u\n", (data->d.f_length));
    printf("PM1.0 CF=1:  %u µg/m³\n", data->d.pm1cf);
    printf("PM2.5 CF=1:  %u µg/m³\n", data->d.pm2_5cf);
    printf("PM10  CF=1:  %u µg/m³\n", data->d.pm10cf);
    printf("PM1.0 ATM:   %u µg/m³\n", data->d.pm1at);
    printf("PM2.5 ATM:   %u µg/m³\n", data->d.pm2_5at);
    printf("PM10  ATM:   %u µg/m³\n", data->d.pm10at);
    printf(">0.3µm:      %u\n", data->d.gt0_3);
    printf(">0.5µm:      %u\n", data->d.gt0_5);
    printf(">1.0µm:      %u\n", data->d.gt1);
    printf(">2.5µm:      %u\n", data->d.gt2_5);
    printf(">5.0µm:      %u\n", data->d.gt5);
    printf(">10µm:       %u\n", data->d.gt10);
    printf("Checksum:    0x%04X\n", data->d.cksum);

    printf("\nRaw Data:\n");
    for (int i = 0; i < PMS5003_EXPECTED_BYTES; i++) {
        printf("%02X ", data->raw_data[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
}
void print_data_le(const PMS5003_DATA_LE *data) {
    printf("Parsed PMS5003 Data:\n");
    printf("Frame length: %u\n", (data->f_length));
    printf("PM1.0 CF=1:  %u µg/m³\n", data->pm1cf);
    printf("PM2.5 CF=1:  %u µg/m³\n", data->pm2_5cf);
    printf("PM10  CF=1:  %u µg/m³\n", data->pm10cf);
    printf("PM1.0 ATM:   %u µg/m³\n", data->pm1at);
    printf("PM2.5 ATM:   %u µg/m³\n", data->pm2_5at);
    printf("PM10  ATM:   %u µg/m³\n", data->pm10at);
    printf(">0.3µm:      %u\n", data->gt0_3);
    printf(">0.5µm:      %u\n", data->gt0_5);
    printf(">1.0µm:      %u\n", data->gt1);
    printf(">2.5µm:      %u\n", data->gt2_5);
    printf(">5.0µm:      %u\n", data->gt5);
    printf(">10µm:       %u\n", data->gt10);
    printf("Checksum:    0x%04X\n", data->cksum);

    printf("\n");
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s /dev/ttyUSB0\n", argv[0]);
        return 1;
    }

    if (pms_init(argv[1], 9600) != 0) {
        fprintf(stderr, "Failed to initialize PMS5003\n");
        return 1;
    }

    PMS5003_DATA data;
    PMS5003_DATA_LE data_le;

    set_wake_mode();
    usleep(1000000); // wait 1s

    // set_passive_mode();
    set_active_mode();
    usleep(1000000); // wait for mode to take effect
    flush_uart();

    int max_retries = 5;

    bool suceed = false;

    while(max_retries-->0 && !suceed) {
        if (read_pms_frame(&data, 5) == 0) {
            convert_pms_data_to_le(&data, &data_le);
            print_data(&data);
            print_data_le(&data_le);

            printf("Valid frame received!\n");
            suceed = true;
        } else {
            fprintf(stderr, "Failed to read data from PMS5003, retrying... (attempt = %d)\n", 5 - max_retries);
            usleep(1000000); // wait 1s
            reset_uart(argv[1], 9600);
            set_wake_mode();
            usleep(1000000); // wait 1s
            // set_passive_mode();
            // usleep(1000000); // wait for mode to take effect
            flush_uart();
        }
    }

    set_sleep_mode();

    usleep(1000000); // wait 1s
    pms_close();
    return 0;
}