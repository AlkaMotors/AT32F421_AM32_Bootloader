#include "at32f421.h"
#include <stdio.h>
//#include "at32_board.h"
#include "at32f421_dma.h"
#include "at32f421_tmr.h"
#include "at32f421_crm.h"
#include "at32f421_gpio.h"




//TMR_ICInitType  TMR_ICInitStructure;
//DMA_InitType DMA_InitStructure = {0};
//TMR_TimerBaseInitType  TMR_TMReBaseStructure;
//TMR_OCInitType  TMR_OCInitStructure;

 
 /* Bootloader */

#define BOOTLOADER_VERSION 4

//#define USE_PB4
#define USE_PA2

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>

#include "main.h"
#include "eeprom.h"

//#include "systick.h"
//#define USE_ADC_INPUT      // will go right to application and ignore eeprom

#include <string.h>
//#include "bootloader.h"

#define STM32_FLASH_START 0x08000000
#define FIRMWARE_RELATIVE_START 0x1000
#define EEPROM_RELATIVE_START 0x7c00

//uint8_t __attribute__ ((section(".bootloader_info"))) bootloader_version = BOOTLOADER_VERSION;

typedef void (*pFunction)(void);

#define APPLICATION_ADDRESS     (uint32_t)(STM32_FLASH_START + FIRMWARE_RELATIVE_START) // 4k

#define EEPROM_START_ADD         (uint32_t)(STM32_FLASH_START + EEPROM_RELATIVE_START)
#define FLASH_END_ADD           (uint32_t)(STM32_FLASH_START + 0x7FFF)               // 32 k


#define CMD_RUN             0x00
#define CMD_PROG_FLASH      0x01
#define CMD_ERASE_FLASH     0x02
#define CMD_READ_FLASH_SIL  0x03
#define CMD_VERIFY_FLASH    0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM     0x04
#define CMD_PROG_EEPROM     0x05
#define CMD_READ_SRAM       0x06
#define CMD_READ_FLASH_ATM  0x07
#define CMD_KEEP_ALIVE      0xFD
#define CMD_SET_ADDRESS     0xFF
#define CMD_SET_BUFFER      0xFE


#ifdef USE_PA2
#define input_pin     GPIO_PINS_2
#define input_port       GPIOA
#define PIN_NUMBER       2
#define PORT_LETTER      0
#endif

#ifdef USE_PB4
#define input_pin       GPIO_PINS_4
#define input_port        GPIOB
#define PIN_NUMBER        4
#define PORT_LETTER       1
#endif

uint16_t jumpup = 0;

uint16_t low_pin_count = 0;
char receviedByte;
int receivedCount;
int count = 0;
char messagereceived = 0;
uint16_t invalid_command = 0;
uint16_t address_expected_increment;
int cmd = 0;
char eeprom_req = 0;
int received;
uint8_t port_letter;
volatile int newcount = 1;



uint8_t pin_code = PORT_LETTER << 4 | PIN_NUMBER;
uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x00,0x1f,0x06,0x06,0x01,0x30};      // stm32 device info

//uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0xf3,0x90,0x06,0x01, 0x30};       // silabs device id
//uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0xe8,0xb2,0x06,0x01, 0x30};     // blheli_s identifier


size_t str_len;
char connected = 0;
uint8_t rxBuffer[258];
uint8_t payLoadBuffer[256];
char rxbyte=0;
uint32_t address;
int tick = 0;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;
uint16_t len;
uint8_t received_crc_low_byte;
uint8_t received_crc_high_byte;
uint8_t calculated_crc_low_byte;
uint8_t calculated_crc_high_byte;
uint16_t payload_buffer_size;
char incoming_payload_no_command = 0;

char bootloaderactive = 1;

uint32_t JumpAddress;
pFunction JumpToApplication;


static void TIM3_Init(void);
static void system_clock_config(void);


/* USER CODE BEGIN PFP */
static void MX_GPIO_INPUT_INIT(void);
void processmessage(void);
void serialwriteChar(char data);
void sendString(uint8_t data[], int len);
void recieveBuffer();

#define BAUDRATE              19200
#define BITTIME          1000000/BAUDRATE
#define HALFBITTIME       500000/BAUDRATE

void gpio_mode_set(uint32_t mode, uint32_t pull_up_down, uint32_t pin)
{
 input_port->cfgr = (((((input_port->cfgr))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * mode)));
 input_port->pull = ((((((input_port->pull))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * pull_up_down))));
}

void delayMicroseconds(uint32_t micros){
	TMR3->cval = 0;
	while (TMR3->cval< micros){

	}
}

void jump(){
	TMR3->c3dt = 1;
	__disable_irq();
	JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
	uint8_t value = *(uint8_t*)(EEPROM_START_ADD);
	if (value != 0x01){      // check first byte of eeprom to see if its programmed, if not do not jump
		invalid_command = 0;
		return;
	}
TMR3->ctrl1_bit.tmren = FALSE;
    JumpToApplication = (pFunction) JumpAddress;

    __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
   JumpToApplication();
}



void makeCrc(uint8_t* pBuff, uint16_t length){
	static uint8_16_u CRC_16;
		CRC_16.word=0;

		for(int i = 0; i < length; i++) {


		     uint8_t xb = pBuff[i];
		     for (uint8_t j = 0; j < 8; j++)
		     {
		         if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) !=0 ) {
		             CRC_16.word = CRC_16.word >> 1;
		             CRC_16.word = CRC_16.word ^ 0xA001;
		         } else {
		             CRC_16.word = CRC_16.word >> 1;
		         }
		         xb = xb >> 1;
		     }
		 }
		calculated_crc_low_byte = CRC_16.bytes[0];
		calculated_crc_high_byte = CRC_16.bytes[1];

}

char checkCrc(uint8_t* pBuff, uint16_t length){

		char received_crc_low_byte2 = pBuff[length];          // one higher than len in buffer
		char received_crc_high_byte2 = pBuff[length+1];
		makeCrc(pBuff,length);

		if((calculated_crc_low_byte==received_crc_low_byte2)   && (calculated_crc_high_byte==received_crc_high_byte2)){
			return 1;
		}else{
			return 0;
		}
}


void setReceive(){
	
input_port->cfgr = (((((input_port->cfgr))) & (~(((input_pin * input_pin) * (0x3UL << (0U)))))) | (((input_pin * input_pin) * GPIO_MODE_INPUT)));
received = 0;

}

void setTransmit(){
//gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
input_port->scr = input_pin;
input_port->cfgr = (((((input_port->cfgr))) & (~(((input_pin * input_pin) * (0x3UL << (0U)))))) | (((input_pin * input_pin) * GPIO_MODE_OUTPUT)));
//gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);	// set as reciever // clear bits and set receive bits..
}


void send_ACK(){
    setTransmit();
    serialwriteChar(0x30);             // good ack!
	setReceive();
}

void send_BAD_ACK(){
    setTransmit();
 		serialwriteChar(0xC1);                // bad command message.
 		setReceive();
}

void send_BAD_CRC_ACK(){
    setTransmit();
 		serialwriteChar(0xC2);                // bad command message.
 		setReceive();
}

void sendDeviceInfo(){
	
	setTransmit();
	sendString(deviceInfo,9);
	setReceive();

}

bool checkAddressWritable(uint32_t address) {
	return address >= APPLICATION_ADDRESS;
}

void decodeInput(){
	if(incoming_payload_no_command){
		len = payload_buffer_size;
	//	received_crc_low_byte = rxBuffer[len];          // one higher than len in buffer
	//	received_crc_high_byte = rxBuffer[len+1];
		if(checkCrc(rxBuffer,len)){
			memset(payLoadBuffer, 0, sizeof(payLoadBuffer));             // reset buffer

			for(int i = 0; i < len; i++){
				payLoadBuffer[i]= rxBuffer[i];
			}
			send_ACK();
			incoming_payload_no_command = 0;
			return;
		}else{
			send_BAD_CRC_ACK();
			return;
		}
	}

	cmd = rxBuffer[0];

	if(rxBuffer[16] == 0x7d){
		if(rxBuffer[8] == 13 && rxBuffer[9] == 66){
			sendDeviceInfo();
			rxBuffer[20]= 0;

		}
		return;
	}

	if(rxBuffer[20] == 0x7d){
			if(rxBuffer[12] == 13 && rxBuffer[13] == 66){
				sendDeviceInfo();
				rxBuffer[20]= 0;
				return;
			}

	}
	if(rxBuffer[40] == 0x7d){
				if(rxBuffer[32] == 13 && rxBuffer[33] == 66){
					sendDeviceInfo();
					rxBuffer[20]= 0;
					return;
				}
		}

	if(cmd == CMD_RUN){         // starts the main app
		if((rxBuffer[1] == 0) && (rxBuffer[2] == 0) && (rxBuffer[3] == 0)){
			invalid_command = 101;
		}
	}

	if(cmd == CMD_PROG_FLASH){
		len = 2;
		if (!checkCrc((uint8_t*)rxBuffer, len)) {
			send_BAD_CRC_ACK();

			return;
		}

		if (!checkAddressWritable(address)) {
			send_BAD_ACK();

			return;
		}

		save_flash_nolib((uint8_t*)payLoadBuffer, payload_buffer_size,address);
		send_ACK();

	 	return;
	}

	if(cmd == CMD_SET_ADDRESS){             //  command set addressinput format is: CMD, 00 , High byte address, Low byte address, crclb ,crchb
		len = 4;  // package without 2 byte crc
		if (!checkCrc((uint8_t*)rxBuffer, len)) {
			send_BAD_CRC_ACK();

			return;
		}


	    // will send Ack 0x30 and read input after transfer out callback
		invalid_command = 0;
		address = STM32_FLASH_START + (rxBuffer[2] << 8 | rxBuffer[3]);
		send_ACK();

		return;
	}

	if(cmd == CMD_SET_BUFFER){        // for writing buffer rx buffer 0 = command byte.  command set address, input , format is CMD, 00 , 00 or 01 (if buffer is 256), buffer_size,
		len = 4;  // package without 2 byte crc
		if (!checkCrc((uint8_t*)rxBuffer, len)) {
			send_BAD_CRC_ACK();

			return;
		}

        // no ack with command set buffer;
       	if(rxBuffer[2] == 0x01){
       		payload_buffer_size = 256;                          // if nothing in this buffer
       	}else{
	        payload_buffer_size = rxBuffer[3];
        }
	    incoming_payload_no_command = 1;
	    address_expected_increment = 256;
        setReceive();

        return;
	}

	if(cmd == CMD_KEEP_ALIVE){
		len = 2;
		if (!checkCrc((uint8_t*)rxBuffer, len)) {
			send_BAD_CRC_ACK();

			return;
		}

	   	setTransmit();
	 	serialwriteChar(0xC1);                // bad command message.
		setReceive();

		return;
	}

	if(cmd == CMD_ERASE_FLASH){
		len = 2;
		if (!checkCrc((uint8_t*)rxBuffer, len)) {
			send_BAD_CRC_ACK();

			return;
		}

		if (!checkAddressWritable(address)) {
			send_BAD_ACK();

			return;
		}

		send_ACK();
		return;
	}

	if(cmd == CMD_READ_EEPROM){
		eeprom_req = 1;
	}

	if(cmd == CMD_READ_FLASH_SIL){     // for sending contents of flash memory at the memory location set in bootloader.c need to still set memory with data from set mem command
		len = 2;
		if (!checkCrc((uint8_t*)rxBuffer, len)) {
			send_BAD_CRC_ACK();

			return;
		}

		count++;
		uint16_t out_buffer_size = rxBuffer[1];//
		if(out_buffer_size == 0){
			out_buffer_size = 256;
		}
		address_expected_increment = 128;

		setTransmit();
		uint8_t read_data[out_buffer_size + 3];        // make buffer 3 larger to fit CRC and ACK
		memset(read_data, 0, sizeof(read_data));
        //    read_flash((uint8_t*)read_data , address);                 // make sure read_flash reads two less than buffer.
		read_flash_bin((uint8_t*)read_data , address, out_buffer_size);

        makeCrc(read_data,out_buffer_size);
        read_data[out_buffer_size] = calculated_crc_low_byte;
        read_data[out_buffer_size + 1] = calculated_crc_high_byte;
        read_data[out_buffer_size + 2] = 0x30;
        sendString(read_data, out_buffer_size+3);

		setReceive();

		return;
	}

    setTransmit();

	serialwriteChar(0xC1);                // bad command message.
	invalid_command++;
 	setReceive();
}


void serialreadChar()
{
	
rxbyte=0;
while(!(input_port->idt & input_pin)){ // wait for rx to go high
	if(TMR3->cval > 20000){
			invalid_command = 101;
		
			return;
	}
}


while((input_port->idt & input_pin)){   // wait for it go go low
	if(TMR3->cval > 250 && messagereceived){

		return;
	}
}

delayMicroseconds(HALFBITTIME);//wait to get the center of bit time

int bits_to_read = 0;
while (bits_to_read < 8) {
	delayMicroseconds(BITTIME);
	rxbyte = rxbyte | ((input_port->idt & input_pin) >> PIN_NUMBER) << bits_to_read;
  bits_to_read++;
}

delayMicroseconds(HALFBITTIME); //wait till the stop bit time begins
messagereceived = 1;
receviedByte = rxbyte;
TMR3->c3dt = rxbyte;
}


void serialwriteChar(char data)
{
//GPIO_BC(input_port) = input_pin; //initiate start bit HIGH
input_port->clr = input_pin;	
char bits_to_read = 0;
while (bits_to_read < 8) {
  delayMicroseconds(BITTIME);
  if (data & 0x01) {
	  //GPIO_BOP(input_port) = input_pin;
		input_port->scr = input_pin;
  }else{
	 // GPIO_BC(input_port) = input_pin;
		input_port->clr = input_pin;
  }
  bits_to_read++;
  data = data >> 1;
}

delayMicroseconds(BITTIME);
input_port->scr = input_pin;

// if more than one byte a delay is needed after stop bit,
//if its the only one no delay, the sendstring function adds delay after each bit

//if(cmd == 255 || cmd == 254 || cmd == 1  || incoming_payload_no_command){
//
//}else{
//	delayMicroseconds(BITTIME);
//}


}


void sendString(uint8_t *data, int len){

	for(int i = 0; i < len; i++){
		serialwriteChar(data[i]);
		delayMicroseconds(BITTIME);

	}
}

void recieveBuffer(){

	//int i = 0;
	count = 0;
	messagereceived = 0;
	memset(rxBuffer, 0, sizeof(rxBuffer));

	for(int i = 0; i < sizeof(rxBuffer); i++){

		serialreadChar();
		newcount++;


	if(incoming_payload_no_command){
		if(count == payload_buffer_size+2){

			break;
		}
		rxBuffer[i] = rxbyte;
		count++;
		
	}else{
		if(TMR3->cval > 250){
	
		count = 0;
			
		break;
	    }else{
		rxBuffer[i] = rxbyte;
		if(i == 257){
			invalid_command+=20;       // needs one hundred to trigger a jump but will be reset on next set address commmand

		}
	}
	}
	}
	
		decodeInput();
	
}

void update_EEPROM(){
read_flash_bin(rxBuffer , EEPROM_START_ADD , 48);
if(BOOTLOADER_VERSION != rxBuffer[2]){
	if (rxBuffer[2] == 0xFF || rxBuffer[2] == 0x00){
		return;
	}
	rxBuffer[2] = BOOTLOADER_VERSION;
save_flash_nolib(rxBuffer, 48, EEPROM_START_ADD);
}
}

void checkForSignal(){
	
		 gpio_mode_set(GPIO_MODE_INPUT, GPIO_PULL_DOWN, input_pin); 
	
	  delayMicroseconds(500);

	  for(int i = 0 ; i < 500; i ++){
		 if(!(input_port->idt & input_pin)){
			 low_pin_count++;
		 }else{
		
		 }

		  delayMicroseconds(10);
	  }
	if(low_pin_count > 450){
				if(crm_flag_get(CRM_SW_RESET_FLAG) != SET){
		jump();
	}
	}
	
	
	  gpio_mode_set(GPIO_MODE_INPUT, GPIO_PULL_UP, input_pin);
	
	  delayMicroseconds(500);

	  for(int i = 0 ; i < 500; i ++){
		 if( !((input_port->idt & input_pin))){
			 low_pin_count++;
		 }else{
		
		 }

		  delayMicroseconds(10);
	  }
			 if(low_pin_count == 0){
				 return;           // all high while pin is pulled low, bootloader signal
			 }

		 low_pin_count = 0;

		 gpio_mode_set(GPIO_MODE_INPUT, GPIO_PULL_NONE, input_pin);
		 delayMicroseconds(500);

		 for(int i = 0 ; i < 500; i ++){
		 if( !((input_port->idt & input_pin))){
			 low_pin_count++;
		 }

		  delayMicroseconds(10);
	  }
		 if(low_pin_count == 0){
			 return;            // when floated all
		 }

		 if(low_pin_count > 0){
			 jump();
		 }



}



int main(void)
{
//	fmc_wscnt_set(2);
//  fmc_prefetch_enable();
system_clock_config();
	
	
	newcount= 2;
//Prevent warnings
//(void)bootloader_version;
  newcount= 3;
  TIM3_Init();

   MX_GPIO_INPUT_INIT();     // init the pin with a pulldown

   checkForSignal();
	 

	gpio_mode_set( GPIO_MODE_INPUT, GPIO_PULL_NONE, input_pin);
		
  #ifdef USE_ADC_INPUT  // go right to application
  jump();

#endif
  deviceInfo[3] = pin_code;
//  update_EEPROM();
//gpio_mode_set( GPIO_Mode_OUT, GPIO_Pull_NOPULL, input_pin);
//  sendDeviceInfo();
  while (1)
  {

	  recieveBuffer();
//	sendDeviceInfo();
//	delayMicroseconds(10000);
	  if (invalid_command > 100){
		  jump();
	  }

  }

}


void TIM3_Init(void)
{
	crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
	
	
	TMR3->div = 119;
	TMR3->pr = 0xFFFF;
	TMR3->swevt_bit.ovfswtr = TRUE;
	
	TMR3->ctrl1_bit.tmren = TRUE;
	
}


static void MX_GPIO_INPUT_INIT(void)
{

	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	
	  gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
	
  gpio_init_struct.gpio_pins  = input_pin;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  gpio_init(GPIOB, &gpio_init_struct);
}

void system_clock_config(void)
{
  flash_psr_set(FLASH_WAIT_CYCLE_3);
  crm_reset();
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
  {
  }
  crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_30);
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }
  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb2_div_set(CRM_APB2_DIV_1);
  crm_apb1_div_set(CRM_APB1_DIV_1);
	crm_auto_step_mode_enable(TRUE);
  crm_sysclk_switch(CRM_SCLK_PLL);
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }
  crm_auto_step_mode_enable(FALSE);
  system_core_clock_update();
}