/*
             LUFA Library
     Copyright (C) Dean Camera, 2013.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the VirtualSerial demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
*/

#include "main.h"
#include <inttypes.h>
#include <util/delay.h>

#define LED_DDR DDRE
#define LED_PORT PORTE
#define LED_PIN (1<<PE6)

#define LHI LED_PORT |= LED_PIN
#define LLO LED_PORT &= ~LED_PIN
#define LTG LED_PORT ^= LED_PIN

// note: touchpanel has to be connected to ADC pins on port F
#define TOUCH_DDR DDRF
#define TOUCH_PORT PORTF
#define TOUCH_X1_PIN	(1<<PF5)
#define TOUCH_X2_PIN	(1<<PF1)
#define TOUCH_Y1_PIN	(1<<PF0)
#define TOUCH_Y2_PIN	(1<<PF4)
#define TOUCH_ALL_PINS (TOUCH_X1_PIN|TOUCH_X2_PIN|TOUCH_Y1_PIN|TOUCH_Y2_PIN)
#define TOUCH_X_PINS (TOUCH_X1_PIN|TOUCH_X2_PIN)
#define TOUCH_Y_PINS (TOUCH_Y1_PIN|TOUCH_Y2_PIN)
#define TOUCH_X1_ADCMUX	((1<<MUX0)|(1<<MUX2))//(5)
#define TOUCH_X2_ADCMUX	((1<<MUX0))//(1)
#define TOUCH_Y1_ADCMUX	((0))//(0)
#define TOUCH_Y2_ADCMUX ((1<<MUX2))//(4)
#define TOUCH_ADCREFV ((0<<REFS1)|(1<<REFS0))	// AVcc w/ cap on AREF

#define SET_INPUT_HIZ(x) TOUCH_DDR &= ~x; TOUCH_PORT &= ~x;
#define SET_ALL_INPUT_HIZ() SET_INPUT_HIZ(TOUCH_ALL_PINS)
#define SET_OUTPUT_LOW(x) TOUCH_DDR |= x; TOUCH_PORT &= ~x;
#define SET_OUTPUT_HIGH(x) TOUCH_DDR |= x; TOUCH_PORT |= x;
#define SEL_ADC(x) ADMUX = (TOUCH_ADCREFV|x);

#define SAMPLE_ACCU_COORD (32)
#define SAMPLE_ACCU_PRESSURE (16)

//#define SET_OUTPUT(x) TOUCH_DDR &= (~TOUCH_ALL_PINS | x)
//#define SET_DRIVER(x) TOUCH_PORT &= (~TOUCH_ALL_PINS | x)


struct STouchPosition {
	uint16_t x;
	uint16_t y;
	uint16_t p;
};

static struct STouchPosition touchPosition;

/** Contains the current baud rate and other settings of the virtual serial port. While this demo does not use
 *  the physical USART and thus does not use these settings, they must still be retained and returned to the host
 *  upon request or the host will assume the device is non-functional.
 *
 *  These values are set by the host via a class-specific request, however they are not required to be used accurately.
 *  It is possible to completely ignore these value or use other settings as the host is completely unaware of the physical
 *  serial link characteristics and instead sends and receives data in endpoint streams.
 */
static CDC_LineEncoding_t LineEncoding = { .BaudRateBPS = 0,
                                           .CharFormat  = CDC_LINEENCODING_OneStopBit,
                                           .ParityType  = CDC_PARITY_None,
                                           .DataBits    = 8                            };



inline uint16_t adc_sample()
{
	LHI;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA&(1<<ADSC));

	uint16_t data = ADC;//ADCL;
	//data |= (ADCH<<8);
	LLO;
	return data;
}


/*inline*/ uint16_t sample_x_forward()
{
	// set Y1 to Vcc
	// set Y1 to GND
	// set X1 hi-z
	// measure X2

	// we'll average the forward and backward measurements
	// each measurement adds 10 bits
	// we have 16 bits in total
	// it's safe to add 3 samples here
	// so it can still be added to the backward measurement
	// before dividing by two

	uint16_t data = 0;

	SET_ALL_INPUT_HIZ();
	SET_OUTPUT_LOW(TOUCH_Y1_PIN);
	SET_OUTPUT_HIGH(TOUCH_Y2_PIN);
	SEL_ADC(TOUCH_X2_ADCMUX);

	for(int i=0; i<SAMPLE_ACCU_COORD; ++i)
	{
		if(i<SAMPLE_ACCU_COORD/2)
			adc_sample();
		else
			data += adc_sample();
	}

	return data;
}

/*inline*/ uint16_t sample_x_backward()
{
	// same as X only we'll switch the Vcc and GND direction
	// also, since we're measuring backwards, we'll have to subtract
	// the measured value from max sample value (1023)

	uint16_t data = 0;

	SET_ALL_INPUT_HIZ();
	SET_OUTPUT_LOW(TOUCH_Y2_PIN);
	SET_OUTPUT_HIGH(TOUCH_Y1_PIN);
	SEL_ADC(TOUCH_X2_ADCMUX);

	for(int i=0; i<SAMPLE_ACCU_COORD; ++i)
	{
		if(i<SAMPLE_ACCU_COORD/2)
			adc_sample();
		else
			data += 1023-adc_sample();
	}

	return data;
}

/*inline*/ uint16_t sample_y_forward()
{
	// see sample_x_forward, just rotated 90 deg

	uint16_t data = 0;
	
	SET_ALL_INPUT_HIZ();
	SET_OUTPUT_LOW(TOUCH_X1_PIN);
	SET_OUTPUT_HIGH(TOUCH_X2_PIN);
	SEL_ADC(TOUCH_Y2_ADCMUX);

	for(int i=0; i<SAMPLE_ACCU_COORD; ++i)
	{
		if(i<SAMPLE_ACCU_COORD/2)
			adc_sample();
		else
			data += adc_sample();
	}

	return data;
}

/*inline*/ uint16_t sample_y_backward()
{
	// see sample_x_backward, just rotated 90 deg

	uint16_t data = 0;
	
	SET_ALL_INPUT_HIZ();
	SET_OUTPUT_LOW(TOUCH_X2_PIN);
	SET_OUTPUT_HIGH(TOUCH_X1_PIN);
	SEL_ADC(TOUCH_Y2_ADCMUX);

	for(int i=0; i<SAMPLE_ACCU_COORD; ++i)
	{
		if(i<SAMPLE_ACCU_COORD/2)
			adc_sample();
		else
			data += 1023-adc_sample();
	}

	return data;
}

/*inline*/ uint16_t sample_pressure()
{
	uint16_t data = 0;

	// set opposite sides (eg. X1, X2) to Vcc
	// pull down one of the leftover sides (eg. Y1) to GND
	// measure the opposite site of GND
	// then rotate this setup through all four possible combinations
	// add up the values to "average" out any errors

	// each sample adds 10 bits value
	// we have 16 bits of total space
	// so we can add up all 4 samples easily without overflow

	SET_ALL_INPUT_HIZ();
	SET_OUTPUT_HIGH(TOUCH_X2_PIN);
	SET_OUTPUT_HIGH(TOUCH_X1_PIN);
	SET_OUTPUT_LOW(TOUCH_Y1_PIN);
	SEL_ADC(TOUCH_Y2_ADCMUX);

	for(int i=0; i<SAMPLE_ACCU_PRESSURE; ++i)
	{
		if(i<SAMPLE_ACCU_PRESSURE/2)
			adc_sample();
		else
			data += adc_sample();
	}

	SET_ALL_INPUT_HIZ();
	SET_OUTPUT_HIGH(TOUCH_X2_PIN);
	SET_OUTPUT_HIGH(TOUCH_X1_PIN);
	SET_OUTPUT_LOW(TOUCH_Y2_PIN);
	SEL_ADC(TOUCH_Y1_ADCMUX);

	for(int i=0; i<SAMPLE_ACCU_PRESSURE; ++i)
	{
		if(i<SAMPLE_ACCU_PRESSURE/2)
			adc_sample();
		else
			data += adc_sample();
	}

	SET_ALL_INPUT_HIZ();
	SET_OUTPUT_HIGH(TOUCH_Y2_PIN);
	SET_OUTPUT_HIGH(TOUCH_Y1_PIN);
	SET_OUTPUT_LOW(TOUCH_X1_PIN);
	SEL_ADC(TOUCH_X2_ADCMUX);

	for(int i=0; i<SAMPLE_ACCU_PRESSURE; ++i)
	{
		if(i<SAMPLE_ACCU_PRESSURE/2)
			adc_sample();
		else
			data += adc_sample();
	}

	SET_ALL_INPUT_HIZ();
	SET_OUTPUT_HIGH(TOUCH_Y2_PIN);
	SET_OUTPUT_HIGH(TOUCH_Y1_PIN);
	SET_OUTPUT_LOW(TOUCH_X2_PIN);
	SEL_ADC(TOUCH_X1_ADCMUX);

	for(int i=0; i<SAMPLE_ACCU_PRESSURE; ++i)
	{
		if(i<SAMPLE_ACCU_PRESSURE/2)
			adc_sample();
		else
			data += adc_sample();
	}

	return data;
}

void Sample_Task()
{
	touchPosition.x = (sample_x_forward() + sample_x_backward())/2;
	touchPosition.y = (sample_y_forward() + sample_y_backward())/2;
	touchPosition.p = sample_pressure();
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */


int main(void)
{
	SetupHardware();

	GlobalInterruptEnable();

	LLO;

	for (;;)
	{
		
		if (USB_DeviceState == DEVICE_STATE_Configured)
		{
			Sample_Task();
			_delay_ms(5);
		}
		CDC_Task();
		USB_USBTask();
	}
}

void adc_init()
{

	/** Setup and enable ADC **/
	ADMUX = (0<<REFS1)|	// Reference Selection Bits
			(1<<REFS0)|		// AVcc - external cap at AREF
			(0<<ADLAR)|
			(0<<MUX4)| // select no input channels
			(0<<MUX3)|
			(0<<MUX2)|
			(0<<MUX1)|
			(0<<MUX0);
	
	ADCSRA = (1<<ADEN)|	// 1 = ADC ENable
			(0<<ADSC)|		// 1 = ADC Start Conversion ( set in adc read function)
			(0<<ADATE)|		// 1 = ADC Auto Trigger Enable
			(0<<ADIF)|		// ADC Interrupt Flag
			(0<<ADIE)|		// ADC Interrupt Enable
			(0<<ADPS2)|
			(1<<ADPS1)|		// ADC Prescaler Selects adc sample freq
			(0<<ADPS0);

	ADCSRB = (1<<ADHSM)|	// High Speed mode select
			(0<<ACME)|		// Analog Comparator Mux enable
			(0<<MUX5)|
			(0<<ADTS3)|	
			(0<<ADTS2)|		// Sets Auto Trigger source if ADATE is 1
			(0<<ADTS1)|
			(0<<ADTS0);
}


/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	clock_prescale_set(clock_div_1);	// disable clock division

	memset(&touchPosition,0x00,sizeof(struct STouchPosition));
	
	MCUCR = (1 << JTD);	// disable jtag
	MCUCR = (1 << JTD);	// disable jtag

	TOUCH_DDR |= TOUCH_ALL_PINS;				// enable output on touch pins
	TOUCH_PORT &= ~TOUCH_ALL_PINS;			// set all pins low

	adc_init();

	LED_DDR = LED_PIN;								// configure LED pin

	USB_Init();
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management and CDC management tasks.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the CDC management task started.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup CDC Data Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_NOTIFICATION_EPADDR, EP_TYPE_INTERRUPT, CDC_NOTIFICATION_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_TX_EPADDR, EP_TYPE_BULK, CDC_TXRX_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_RX_EPADDR, EP_TYPE_BULK,  CDC_TXRX_EPSIZE, 1);

	/* Reset line encoding baud rate so that the host knows to send new values */
	LineEncoding.BaudRateBPS = 0;

	/* Indicate endpoint configuration success or failure */
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Process CDC specific control requests */
	switch (USB_ControlRequest.bRequest)
	{
		case CDC_REQ_GetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the line coding data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearOUT();
			}

			break;
		case CDC_REQ_SetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Read the line coding data in from the host into the global struct */
				Endpoint_Read_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearIN();
			}

			break;
		case CDC_REQ_SetControlLineState:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				/* NOTE: Here you can read in the line state mask from the host, to get the current state of the output handshake
				         lines. The mask is read in from the wValue parameter in USB_ControlRequest, and can be masked against the
						 CONTROL_LINE_OUT_* masks to determine the RTS and DTR line states using the following code:
				*/
			}

			break;
	}
}

/** Function to manage CDC data transmission and reception to and from the host. */
void CDC_Task(void)
{
	static char ReportString[64];
	static bool ActionSent      = false;

	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	sprintf(ReportString,"%05d %05d %05d\n",touchPosition.x, touchPosition.y, touchPosition.p);

	/* Flag management - Only allow one string to be sent per action */
	if ((ReportString != NULL) && (ActionSent == false) && LineEncoding.BaudRateBPS)
	{

		ActionSent = true;

		/* Select the Serial Tx Endpoint */
		Endpoint_SelectEndpoint(CDC_TX_EPADDR);

		/* Write the String to the Endpoint */
		Endpoint_Write_Stream_LE(ReportString, strlen(ReportString), NULL);

		/* Remember if the packet to send completely fills the endpoint */
		bool IsFull = (Endpoint_BytesInEndpoint() == CDC_TXRX_EPSIZE);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();

		/* If the last packet filled the endpoint, send an empty packet to release the buffer on
		 * the receiver (otherwise all data will be cached until a non-full packet is received) */
		if (IsFull)
		{
			/* Wait until the endpoint is ready for another packet */
			Endpoint_WaitUntilReady();

			/* Send an empty packet to ensure that the host does not buffer data sent to it */
			Endpoint_ClearIN();
		}
	}

	/* Select the Serial Rx Endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	/* Throw away any received data from the host */
	if (Endpoint_IsOUTReceived())
	  Endpoint_ClearOUT();

	ActionSent = false;
}

