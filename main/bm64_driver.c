#include "driver/gpio.h"
#include "driver/uart.h"
#include "bm64_driver.h"
#include <stdio.h>
#include <string.h>
#include "endian.h"

#define P2_0        18
#define EAN         19
#define MFB         13
#define RST         12
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<P2_0) | (1ULL<<EAN) | (1ULL<<MFB) | (1ULL<<RST))

// uart config
#define UART_PORT_NUMBER 2
#define UART_SPEED       115200
#define UART_RXD_PIN     5
#define UART_TXD_PIN     4
#define UART_BUF_SIZE    1024

/* Static definitions */
static void bm64_uart_init();
static void bm64_reset(void);
static void bm64_gpio_init();

uart_config_t uart_config = {
    .baud_rate = UART_SPEED,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};

static void bm64_uart_init()
{
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUMBER, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUMBER, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUMBER, UART_TXD_PIN, UART_RXD_PIN, 0, 0));

}

static void bm64_reset(void)
{
    gpio_set_level(MFB, 0);
    gpio_set_level(RST, 0);
    vTaskDelay(499 / portTICK_RATE_MS);

    gpio_set_level(MFB, 1);

    vTaskDelay(1 / portTICK_RATE_MS);

    gpio_set_level(RST, 1);
}

static void bm64_gpio_init()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(P2_0, 1);
    gpio_set_level(EAN, 0);
    gpio_set_level(MFB, 1);
    gpio_set_level(RST, 1);
}

static uint8_t compute_checksum(uint8_t *data, int length)
{
    int i=0;
    uint8_t res = 0;
    for(i=0;i<length;i++)
    {
        res += data[i];
    }

    return (~res) + 1;
}

typedef struct EventHeader {
    //0x00 0xAA 0x00 0x03 0x00 0x05 0x00 0xF8
    uint16_t preamble;
    uint16_t length;
    uint8_t event_code;
} __attribute__((packed)) EventHeader ;

typedef enum EventType {
    ACK=0,
    BTM_STATUS=1,
} EventType;

typedef struct Ack {
    uint8_t opcode;
    uint8_t status;
} __attribute__((packed)) Ack;

typedef struct BTMStatus {
    uint8_t status;
} __attribute__((packed)) BTMStatus;

typedef struct Event {
    EventType type;
    union data {
        struct Ack ack;
    };
} Event;

static int bm64_wait_event_buffer(uint8_t * buf, int * read_len)
{
    int len = uart_read_bytes(UART_PORT_NUMBER, buf, sizeof(EventHeader), 200 / portTICK_RATE_MS);
    if(len < sizeof(EventHeader))
    {
        return BM64_DATA_TOOSHORT;
    }
    EventHeader header;
    memcpy(&header, buf, sizeof(header));

    header.length = bswap16(header.length);

    // read length more bytes+checksum
    len = len + uart_read_bytes(UART_PORT_NUMBER, 
            buf+len, // offset
            header.length, // length of data + checksum
            200/portTICK_RATE_MS);
    if(len < sizeof(EventHeader) + header.length)
    {
        return BM64_DATA_TOOSHORT;
    }

    return BM64_NOERROR;
}

static const char * EVENT_NAMES[] = {
"Command_ACK", 
"BTM_Status", 
"Call_Status", 
"Caller_ID", 
"SMS_Received_Indication", 
"Missed_Call_Indication", 
"Phone_Max_Battery_Level", 
"Phone_Current_Battery_Level", 
"Roaming_Status", 
"Phone_Max_Signal_Strength_Level", 
"Phone_Current_Signal_Strength_Level", 
"Phone_Service_Status", 
"BTM_Battery_Status", 
"BTM_Charging_Status", 
"Reset_To_Default", 
"Report_HF_Gain_Level", 
"EQ_Mode_Indication", 
"PBAP_Missed_Call_History",
"PBAP_Received_Call_History",
"PBAP_Dialed_Call_History",
"PBAP_Combine_Call_History",
"Phonebook_Contacts",
"PBAP_Access_Finish",
"Read_Linked_Device_Information_Reply", 
"Read_BTM_Version_Reply", 
"Call_List_Report", 
"AVC_Specific_Rsp", 
"BTM_Utility_Req", 
"Vendor_AT_Cmd_Reply", 
"Report_Vendor_AT_Event", 
"Read_Link_Status_Reply", 
"Read_Paired_Device_Record_Reply", 
"Read_Local_BD_Address_Reply", 
"Read_Local_Device_Name_Reply", 
"Report_SPP/iAP_Data", 
"Report_Link_Back_Status", 
"REPORT_RING_TONE_STATUS", 
"User_Confrim_SSP_Req", 
"Report_AVRCP_Vol_Ctrl", 
"Report_Input_Signal_Level", 
"Report_iAP_Info", 
"REPORT_AVRCP_ABS_VOL_CTRL", 
"Report_Voice_Prompt_Status", 
"Report_MAP_Data",
"Security_Bonding_Res", 
"Report_Type_Codec", 
"Report_Type_BTM_Setting", 
"Report_MCU_Update_Reply", 
"Report_BTM_Initial_Status", 
"LE_ANCS_Service_Event", 
"LE_Signaling_Event", 
"Report_nSPK_Link_Status", 
"Report_nSPK_Vendor_Event", 
"Report_nSPK_Audio_Setting", 
"Report_Sound_Effect_Status", 
"Report_Vendor_EEPROM_Data", 
"REPORT_IC_VERSION_INFO", 
"REPORT_LE_GATT_EVENT"};


static const char * BTM_STATUSES[] = {
"Power OFF state"
,"Pairing state (discoverable mode)"
,"Power ON state"
,"Pairing successful"
,"Pairing failed"
,"HF/HS link established"
,"A2DP link established"
,"HF link disconnected"
,"A2DP link disconnected"
,"SCO link connected"
,"SCO link disconnected"
,"AVRCP link established"
,"AVRCP link disconnected"
,"Standard SPP connected"
,"Standard_SPP / iAP disconnected"
,"Standby state"
,"iAP connected"
,"ACL disconnected"
,"MAP connected"
,"MAP operation forbidden"
,"MAP disconnected"
,"ACL connected"};

static int bm64_wait_event(Event * evt)
{
    uint8_t buf[UART_BUF_SIZE];
    int read_len = 0;
    int err = bm64_wait_event_buffer(buf, &read_len);
    if(err != BM64_NOERROR)
    {
        return err;
    }

    EventHeader * header = (EventHeader*) buf;     
    printf("Event : %s\n", EVENT_NAMES[header->event_code]);
    switch(header->event_code)
    {
        case ACK:
            printf("ACK\n");
            break;

        case BTM_STATUS:
            {
                BTMStatus * status = (BTMStatus*) (buf+sizeof(header)); 
                printf("BTM Status : %s\n", BTM_STATUSES[status->status]);
                break;
            };
        default:
            break;
    }
    return BM64_NOERROR;
}

static int bm64_make_command(uint8_t opcode, uint8_t * data, int data_length, uint8_t *destination, int *dest_len)
{
    uint8_t preamble[2] = {0x00, 0xAA};
    uint16_t length = 0x00;
    
    memcpy(destination, preamble, sizeof(preamble)); 
    length = sizeof(opcode) + data_length;
    destination[2] = length >> 8;
    destination[3] = length & 0xFF;
    destination[4] = opcode;
    memcpy(destination+5, data, data_length); 
   
    destination[data_length+5] = compute_checksum(destination+2, data_length+3);

    *dest_len = sizeof(preamble) + sizeof(length) + sizeof(opcode) + data_length + 1; // checksum

    return BM64_NOERROR;
}

static void bm64_change_device_name(char* new_name)
{
    printf("Changing device name...\n");
    uint8_t parameter[32] = {0x00};
    memcpy((char*) parameter, new_name, strlen(new_name) > 32 ? 32 : strlen(new_name));

    uint8_t buf[1024];   
    int dest_len = 0;

    bm64_make_command(0x05, parameter, 32, buf, &dest_len);

    uart_write_bytes(UART_PORT_NUMBER, (const char *) buf, dest_len);

    while(1)
    {
        Event evt;
        bm64_wait_event(&evt);
    }
}

static void bm64_make_call(char* number)
{
    uint8_t parameter[20] = {0x00};
    sprintf((char*)parameter+1, "ATD+%s", number);

    uint8_t buf[1024];   
    int dest_len = 0;

    bm64_make_command(0x00, parameter, 20, buf, &dest_len);

    uart_write_bytes(UART_PORT_NUMBER, (const char *) buf, dest_len);

    int len = uart_read_bytes(UART_PORT_NUMBER, buf, 1024, 10000 / portTICK_RATE_MS);
    int i;
    for(i=0;i<len;i++)
    {
        printf("0x%02X ", buf[i]);
    }
}

/* Externally defined functions */
int bm64_init()
{
    bm64_gpio_init();
    bm64_uart_init();

    bm64_reset();

    vTaskDelay(1000 / portTICK_RATE_MS);
    uart_flush(UART_PORT_NUMBER);

    bm64_change_device_name("Marty 1910");

    setvbuf(stdout, NULL, _IONBF, 0);
    while(1)
    {
        uint8_t buf;
        int len = uart_read_bytes(UART_PORT_NUMBER, &buf, 1, 10000 / portTICK_RATE_MS);
        if(len>0)
        {
            //printf("%02X ", buf);
        }
    }

    return BM64_NOERROR; 
}

void bm64_test()
{
    uint8_t parameter[32] = {0x00};
    sprintf((char*)parameter, "Marty 1910");
    uint8_t buf[1024];
    int dest_len = 0;

    bm64_make_command(0x5, parameter, 32, buf, &dest_len);

    int i;
    for(i=0;i<dest_len;i++)
    {
        printf("0x%02X ", buf[i]);
    }
    printf("\r\n");
}
