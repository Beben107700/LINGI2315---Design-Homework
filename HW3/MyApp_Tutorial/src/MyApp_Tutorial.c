/* ------------------------------------------------------------------------------------------------ */

#define MYAPP

#include "MyApp_mAbassi.h"

#include "mAbassi.h"          /* MUST include "SAL.H" and not uAbassi.h        */
#include "Platform.h"         /* Everything about the target platform is here  */
#include "HWinfo.h"           /* Everything about the target hardware is here  */

#include "Syscall.h"
#include "arm_pl330.h"
#include "dw_i2c.h"
#include "dw_uart.h"


/* ------------------------------------------------------------------------------------------------ */


void Task_HPS_Led(void)
{
    MBX_t    *PrtMbx;
    intptr_t PtrMsg;
    
    setup_hps_gpio();               // This is the Adam&Eve Task and we have first to setup everything
    setup_Interrupt();
    
    MTXLOCK_STDIO();
    printf("\n\nDE10-Nano - DJ booba \n\n");
    printf("Task_HPS_Led running on core #%d\n\n", COREgetID());
    MTXUNLOCK_STDIO();
    
    PrtMbx = MBXopen("MyMailbox", 128);

	for( ;; )
	{
        if (MBXget(PrtMbx, &PtrMsg, 0) == 0) {  // 0 = Never blocks
            MTXLOCK_STDIO();
            printf("Receive message (Core = %d)\n", COREgetID());
            MTXUNLOCK_STDIO();
        }
        toogle_hps_led();
        
        TSKsleep(OS_MS_TO_TICK(500));
	}
}

/* ------------------------------------------------------------------------------------------------ */
void Task_Tutorial(void){
	for(;;){
        TSKsleep(OS_MS_TO_TICK(250));
	}
}


void Task_FPGA_Led(void)
{
    uint32_t leds_mask;
    MTXLOCK_STDIO();
    printf("Coucou je suis le roi %d \n \n", COREgetID());
    MTXUNLOCK_STDIO();

    alt_write_word(fpga_leds, 0x01);

	for( ;; )
	{
        leds_mask = alt_read_word(fpga_leds);
        if (leds_mask != (0x01 << (LED_PIO_DATA_WIDTH - 1))) {
            // rotate leds
            leds_mask <<= 1;
        } else {
            // reset leds
            leds_mask = 0x1;
        }
        alt_write_word(fpga_leds, leds_mask);
        
        TSKsleep(OS_MS_TO_TICK(250));
	}
}

/* ------------------------------------------------------------------------------------------------ */

void Task_FPGA_Button(void)
{
    MBX_t    *PrtMbx;
    intptr_t  PtrMsg = (intptr_t) NULL;
    SEM_t    *PtrSem;
    
    PrtMbx = MBXopen("MyMailbox", 128);
    PtrSem = SEMopen("MySemaphore");
    
    for( ;; )
    {
        SEMwait(PtrSem, -1);            // -1 = Infinite blocking
        SEMreset(PtrSem);
        MTXLOCK_STDIO();
        printf("Receive IRQ from Button %d and send message (Core = %d)\n", (int) alt_read_word(fpga_buttons) - 1, COREgetID());  // The Keys O and 1 seem to be inverted somewhere...
        //I will now change the blinking frequency:

    	uint32_t button = alt_read_word(fpga_buttons);
    	uint32_t frequency = alt_read_word(fpga_pio);

    	if (button == 1){
    		frequency  += PIO_0_FREQ/10;
    		alt_write_word(fpga_pio, frequency);
    	}
    	if (button == 2){
    		frequency -= PIO_0_FREQ/10;
    		alt_write_word(fpga_pio, frequency);
    	}

        MTXUNLOCK_STDIO();
        
        MBXput(PrtMbx, PtrMsg, -1);     // -1 = Infinite blocking
    }
}

/* ------------------------------------------------------------------------------------------------ */

void spi_CallbackInterrupt (uint32_t icciar, void *context)
{
    // Do something
    MTXLOCK_STDIO();
    printf("INFO: IRQ from SPI : %08x (status = %x)\r\n",
        (unsigned int) alt_read_word(fpga_spi + SPI_RXDATA),
        (unsigned int) alt_read_word(fpga_spi + SPI_STATUS));
    MTXUNLOCK_STDIO();
    alt_write_word(fpga_spi + SPI_TXDATA, 0x113377FF);
    
    // Clear the status of SPI core
    alt_write_word(fpga_spi + SPI_STATUS, 0x00);
}

/* ------------------------------------------------------------------------------------------------ */

void button_CallbackInterrupt (uint32_t icciar, void *context)
{
    SEM_t    *PtrSem;
    
    // Clear the interruptmask of PIO core
    alt_write_word(fpga_buttons + PIOinterruptmask, 0x0);
    
    // Enable the interruptmask and edge register of PIO core for new interrupt
    alt_write_word(fpga_buttons + PIOinterruptmask, 0x3);
    alt_write_word(fpga_buttons + PIOedgecapture, 0x3);
    
    PtrSem = SEMopen("MySemaphore");
    SEMpost(PtrSem);
}

/* ------------------------------------------------------------------------------------------------ */

void setup_Interrupt( void )
{
    // IRQ from Key0 and Key1
    OSisrInstall(GPT_BUTTON_IRQ, (void *) &button_CallbackInterrupt);
    GICenable(GPT_BUTTON_IRQ, 128, 1);
    
    // Enable interruptmask and edgecapture of PIO core for buttons 0 and 1
    alt_write_word(fpga_buttons + PIOinterruptmask, 0x3);
    alt_write_word(fpga_buttons + PIOedgecapture, 0x3);
    
    // IRQ from SPI slave connected to the RaspberryPI
    OSisrInstall(GPT_SPI_IRQ, (void *) &spi_CallbackInterrupt);
    GICenable(GPT_SPI_IRQ, 128, 1);
    
    // Initialize TXDATA to something (for testing purpose)
    alt_write_word(fpga_spi + SPI_TXDATA, 0x0103070F);
    alt_write_word(fpga_spi + SPI_EOP_VALUE, 0x55AA55AA);
    // Enable interrupt
    alt_write_word(fpga_spi + SPI_CONTROL, SPI_CONTROL_IRRDY + SPI_CONTROL_IE);
}

/* ------------------------------------------------------------------------------------------------ */

void setup_hps_gpio()
{
    uint32_t hps_gpio_config_len = 2;
    ALT_GPIO_CONFIG_RECORD_t hps_gpio_config[] = {
        {HPS_LED_IDX  , ALT_GPIO_PIN_OUTPUT, 0, 0, ALT_GPIO_PIN_DEBOUNCE, ALT_GPIO_PIN_DATAZERO},
        {HPS_KEY_N_IDX, ALT_GPIO_PIN_INPUT , 0, 0, ALT_GPIO_PIN_DEBOUNCE, ALT_GPIO_PIN_DATAZERO}
    };
    
    assert(ALT_E_SUCCESS == alt_gpio_init());
    assert(ALT_E_SUCCESS == alt_gpio_group_config(hps_gpio_config, hps_gpio_config_len));

    //I setup the hps let at the eginning
    alt_write_word(fpga_pio, 24999999);
}

void toogle_hps_led()
{
    uint32_t hps_led_value = alt_read_word(ALT_GPIO1_SWPORTA_DR_ADDR);
    hps_led_value >>= HPS_LED_PORT_BIT;
    hps_led_value = !hps_led_value;
    hps_led_value <<= HPS_LED_PORT_BIT;
    alt_gpio_port_data_write(HPS_LED_PORT, HPS_LED_MASK, hps_led_value);
}

/* ------------------------------------------------------------------------------------------------ */


void Task_ReadSensor(){
	//*************************************

	/* DMA set-up										*/
	int ii;
	  #if ((((OS_PLATFORM) & 0x00FFFFFF) == 0x0000AA10)													\
	   ||  (((OS_PLATFORM) & 0x00FFFFFF) == 0x0000AAC5))
		ii = dma_init(0);
		if (ii != 0) {
			printf("ERROR : dma_init() returned %d\n", ii);
		}

	   #ifdef DMA0_INT
		OSisrInstall(DMA0_INT, &DMAintHndl_0);
		GICenable(DMA0_INT, 128, 0);
	   #endif

	   #ifdef DMA1_INT
		OSisrInstall(DMA1_INT, &DMAintHndl_1);
		GICenable(DMA1_INT, 128, 0);
	   #endif

	   #ifdef DMA2_INT
		OSisrInstall(DMA2_INT, &DMAintHndl_2);
		GICenable(DMA2_INT, 128, 0);
	   #endif

	   #ifdef DMA3_INT
		OSisrInstall(DMA3_INT, &DMAintHndl_3);
		GICenable(DMA3_INT, 128, 0);
	   #endif

	   #ifdef DMA4_INT
		OSisrInstall(DMA4_INT, &DMAintHndl_4);
		GICenable(DMA4_INT, 128, 0);
	   #endif

	   #ifdef DMA5_INT
		OSisrInstall(DMA5_INT, &DMAintHndl_5);
		GICenable(DMA5_INT, 128, 0);
	   #endif

	   #ifdef DMA6_INT
		OSisrInstall(DMA6_INT, &DMAintHndl_6);
		GICenable(DMA6_INT, 128, 0);
	   #endif

	   #ifdef DMA7_INT
		OSisrInstall(DMA7_INT, &DMAintHndl_7);
		GICenable(DMA7_INT, 128, 0);
	   #endif

	   #ifdef DMA8_INT
		OSisrInstall(DMA8_INT, &DMAintHndl_8);
		GICenable(DMA8_INT, 128, 0);
	   #endif

	   #ifdef DMA9_INT
		OSisrInstall(DMA9_INT, &DMAintHndl_9);
		GICenable(DMA9_INT, 128, 0);
	   #endif

	   #ifdef DMA10_INT
		OSisrInstall(DMA10_INT, &DMAintHndl_10);
		GICenable(DMA10_INT, 128, 0);
	   #endif

	   #ifdef DMA11_INT
		OSisrInstall(DMA11_INT, &DMAintHndl_11);
		GICenable(DMA11_INT, 128, 0);
	   #endif

	   #ifdef DMA12_INT
		OSisrInstall(DMA12_INT, &DMAintHndl_12);
		GICenable(DMA12_INT, 128, 0);
	   #endif

	   #ifdef DMA13_INT
		OSisrInstall(DMA13_INT, &DMAintHndl_13);
		GICenable(DMA13_INT, 128, 0);
	   #endif

	   #ifdef DMA14_INT
		OSisrInstall(DMA14_INT, &DMAintHndl_14);
		GICenable(DMA14_INT, 128, 0);
	   #endif

	   #ifdef DMA15_INT
		OSisrInstall(DMA15_INT, &DMAintHndl_15);
		GICenable(DMA15_INT, 128, 0);
	   #endif
	 #endif

	/* ------------------------------------------------ */


	//*************************************
	i2c_init(0, 10, 400000);
	int init = XL345init();

	MBX_t *Accelbox;
	Accelbox = MBXopen("MailboxAccel",128);

	if(init !=0){
		printf("Init of 345 didn't work");
	}
	int x,y,z;
	x=0;
	y=0;
	z=0;

	for(;;){

		int err = XL345read(&x,&y,&z);
		if(err!=0){
			//printf("I've had trouble reading the XL345");
		}
		else{
			int out[3];
			out[0]=x;
			out[1]=y;
			out[2]=z;

			MBXput(Accelbox, (intptr_t)(out),-1);

		}
        TSKsleep(OS_MS_TO_TICK(250));
	}
}


//*************************READ THE SENSOR***************************
void Task_DisplaySensor(void){

	MBX_t *Accelbox;
	Accelbox = MBXopen("MailboxAccel",128);

	int *p; //creates an array

	intptr_t Message;


	for(;;){
		int err = MBXget(Accelbox, &Message, -1);

		if (err!=0){
			printf("Error recieving from mailbox");
		}
		else{
			MTXLOCK_STDIO();
			p = (int*)Message;

			uint32_t switches = alt_read_word(fpga_sw);

			switches = switches%3;



			printf("\nReading from axis %d : %d \n", switches, p[switches]);
			MTXUNLOCK_STDIO();

		}
	}
}

/***********************INIT GSENSOR********************/

int XL345init(void)
{
char Buffer[2];
int  RetVal;

	Buffer[0]  = XL345_REG_DATA_FORMAT;
	Buffer[1]  = (XL345_RANGE_2G)					/* +- 2G range, 10 bit resolution				*/
	           | (XL345_FULL_RESOLUTION);			/* This delivers results of 4mg per bit			*/
	RetVal     = i2c_send(I2C_XL345_DEVICE, I2C_XL345_ADDR, &Buffer[0], 2);

	Buffer[0]  = XL345_REG_BW_RATE;
	Buffer[1]  = XL345_RATE_12_5;					/* Conversion rate is 12.5 Hz (once every 80ms)	*/
	RetVal    |= i2c_send(I2C_XL345_DEVICE, I2C_XL345_ADDR, &Buffer[0], 2);

	Buffer[0]  = XL345_REG_INT_ENABLE;
	Buffer[1]  = XL345_DATAREADY;					/* Enable the interrupt for data ready			*/
	RetVal    |= i2c_send(I2C_XL345_DEVICE, I2C_XL345_ADDR, &Buffer[0], 2);

	Buffer[0]  = XL345_REG_POWER_CTL;
	Buffer[1]  = XL345_STANDBY;						/* Stop the measurements						*/
	RetVal    |= i2c_send(I2C_XL345_DEVICE, I2C_XL345_ADDR, &Buffer[0], 2);

 	return(RetVal);
}

/* ------------------------------------------------------------------------------------------------ */

int XL345read(int *Xaxis, int *Yaxis, int *Zaxis)
{
char Buffer[7];
int  RetVal;

	Buffer[0]  = XL345_REG_POWER_CTL;
	Buffer[1]  = XL345_MEASURE;						/* Start the measurements						*/
	RetVal     = i2c_send(I2C_XL345_DEVICE, I2C_XL345_ADDR, &Buffer[0], 2);

	TSKsleep(OS_MS_TO_TICK(100));					/* Wait for the measurement to be done			*/

	do {											/* Check to make sure is done					*/
		Buffer[0] = XL345_REG_INT_SOURCE;
		RetVal = i2c_send_recv(I2C_XL345_DEVICE, I2C_XL345_ADDR, &Buffer[0], 1, &Buffer[0], 1);
	} while (((XL345_DATAREADY & (int)Buffer[0]) == 0)
	  &&      (RetVal == 0));

	Buffer[0] = 0x32;								/* Read the data; *4 because 0.004g/bit			*/
	RetVal |= i2c_send_recv(I2C_XL345_DEVICE, I2C_XL345_ADDR, &Buffer[0], 1, &Buffer[0], 6);

	if (RetVal == 0) {
		*Xaxis = 4*((((int)(signed char)Buffer[1])<<8) + (0xFF & ((int)Buffer[0])));
		*Yaxis = 4*((((int)(signed char)Buffer[3])<<8) + (0xFF & ((int)Buffer[2])));
		*Zaxis = 4*((((int)(signed char)Buffer[5])<<8) + (0xFF & ((int)Buffer[4])));
	}

	Buffer[0]  = XL345_REG_POWER_CTL;
	Buffer[1]  = XL345_STANDBY;						/* Stop the measurements						*/
	RetVal |= i2c_send(I2C_XL345_DEVICE, I2C_XL345_ADDR, &Buffer[0], 2);

	return(RetVal);
}



