#ifndef RF_PROTOCOL_H
#define RF_PROTOCOL_H

#include <stdint.h>
#include "nrf24l01.h"

// ******************* RF communication commands********************//

#define READ_ALL	'*'	//command to read all regular registers 
#define READ_REGISTER '<' //command to read specific register 
#define WRITE_REGISTER '>' //command to write specific register
#define STOP_CMD	';' // command to stop communication
#define WAIT   '?'

#define BUFFER_SIZE 512
#define MAX_CHUNK_LEN 24
#define MAX_EMPTY_ACK 20
#define MAX_SOFT_COUNT_RETRY 5

#define RESP_TIMEOUT  60000

// ******************** packet structure **************************//
typedef struct  __attribute__((__packed__))
{
    uint8_t packet_no;
    uint16_t total_length;
    uint8_t packet_data_length;
    uint8_t dev_type;
    uint8_t data[24];
    uint8_t unused1;
    uint8_t unused2;
    uint8_t unused3;
}received_packet;

// ******************** cmd structure ****************************//
typedef struct  __attribute__((__packed__))
{
	uint8_t cmd;
	uint8_t length;
	uint8_t data[30];
}command;

//********************** enums *********************************//
typedef enum {READ=0,WRITE=!READ}Cmd_type;

typedef enum {ALL_READ = 100,REGISTER_READ,REGISTER_WRITE,STOP, DATA_EXCHANGE}Req_type;

//*********************global variables************************//

//extern uint8_t transmit;
extern volatile uint8_t received, transmit;
extern uint8_t itr,interrupt;
//extern uint16_t total_len,reg_address,sent_size;
extern uint8_t slave_reg_data[4];
extern uint8_t previous_request;
extern uint8_t buffer[BUFFER_SIZE];
extern char send_buffer[BUFFER_SIZE];
extern received_packet rfdata;


void cmd_delay(void);
void nrf24l01_master_Init(void);
void nrf24l01_slave_Init(void);
void reinit_slave(void);
int get_all_slave_data(uint8_t *to_address);
int control_slave(uint8_t *to_address,uint16_t *reg_addr,Cmd_type commd,uint8_t *data);
void slave_start_listen(uint8_t *to_addr);
void slave_read_write(uint16_t *reg_addr,Cmd_type commd,uint8_t *data);
void execute_master_cmd(void);
void read_all_slave_data(void);
void clear_all(void);
void send_modbus_data_rf(void);
void send_rfmodbus_data(void);

#endif

