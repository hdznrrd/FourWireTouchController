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

#define LED_DDR DDRE
#define LED_PORT PORTE
#define LED_PIN (1<<PE6)

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
#define TOUCH_X1_ADCMUX	(5)
#define TOUCH_X2_ADCMUX	(1)
#define TOUCH_Y1_ADCMUX	(0)
#define TOUCH_Y2_ADCMUX	(4)
#define TOUCH_ADCREFV (3<<6)

#define SET_OUTPUT(x) TOUCH_DDR &= (~TOUCH_ALL_PINS | x)
#define SET_DRIVER(x) TOUCH_PORT &= (~TOUCH_ALL_PINS | x)
#define SET_INPUT(x) ADMUX = (TOUCH_ADCREFV| x)

struct STouchPosition {
	unsigned short x;
	unsigned short y;
	unsigned short p;
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



inline unsigned short adc_sample()
{
	ADCSRA |= ADSC;
	while(ADCSRA&ADSC);

	unsigned short data = ADCL;
	data |= (ADCH<<8);

	return data;
}


inline unsigned short sample_x_forward()
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

	unsigned short data = 0;
	
	SET_OUTPUT(TOUCH_Y_PINS);
	SET_DRIVER(TOUCH_Y2_PIN);
	SET_INPUT(TOUCH_X2_ADCMUX);

	data += adc_sample();
	data += adc_sample();
	data += adc_sample();

	return data;
}

inline unsigned short sample_x_backward()
{
	// same as X only we'll switch the Vcc and GND direction
	// also, since we're measuring backwards, we'll have to subtract
	// the measured value from max sample value (1023)

	unsigned short data = 0;

	SET_OUTPUT(TOUCH_Y_PINS);
	SET_DRIVER(TOUCH_Y1_PIN);
	SET_INPUT(TOUCH_X2_ADCMUX);

	data += adc_sample();
	data += adc_sample();
	data += adc_sample();

	return data;
}

inline unsigned short sample_y_forward()
{
	// see sample_x_forward, just rotated 90 deg

	unsigned short data = 0;
	
	SET_OUTPUT(TOUCH_X_PINS);
	SET_DRIVER(TOUCH_X2_PIN);
	SET_INPUT(TOUCH_Y1_ADCMUX);

	data += adc_sample();
	data += adc_sample();
	data += adc_sample();

	return data;
}

inline unsigned short sample_y_backward()
{
	// see sample_x_backward, just rotated 90 deg

	unsigned short data = 0;
	
	SET_OUTPUT(TOUCH_X_PINS);
	SET_DRIVER(TOUCH_X1_PIN);
	SET_INPUT(TOUCH_Y1_ADCMUX);

	data += adc_sample();
	data += adc_sample();
	data += adc_sample();

	return data;
}

inline unsigned short sample_pressure()
{
	unsigned short data = 0;

	// set opposite sides (eg. X1, X2) to Vcc
	// pull down one of the leftover sides (eg. Y1) to GND
	// measure the opposite site of GND
	// then rotate this setup through all four possible combinations
	// add up the values to "average" out any errors

	// each sample adds 10 bits value
	// we have 16 bits of total space
	// so we can add up all 4 samples easily without overflow

	SET_OUTPUT(TOUCH_X_PINS|TOUCH_Y1_PIN);
	SET_DRIVER(TOUCH_X_PINS);
	SET_INPUT(TOUCH_Y2_ADCMUX);

	data += adc_sample();

	SET_OUTPUT(TOUCH_X_PINS|TOUCH_Y2_PIN);
	SET_DRIVER(TOUCH_X_PINS);
	SET_INPUT(TOUCH_Y1_ADCMUX);

	data += adc_sample();

	SET_OUTPUT(TOUCH_Y_PINS|TOUCH_X1_PIN);
	SET_DRIVER(TOUCH_Y_PINS);
	SET_INPUT(TOUCH_X2_ADCMUX);

	data += adc_sample();

	SET_OUTPUT(TOUCH_Y_PINS|TOUCH_X2_PIN);
	SET_DRIVER(TOUCH_Y_PINS);
	SET_INPUT(TOUCH_X1_ADCMUX);

	data += adc_sample();

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

	for (;;)
	{
		//Sample_Task();
		CDC_Task();
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	clock_prescale_set(clock_div_1);	// disable clock division

	memset(&touchPosition,0x00,sizeof(struct STouchPosition));

	TOUCH_DDR |= TOUCH_ALL_PINS;				// enable output on touch pins
	TOUCH_PORT &= ~TOUCH_ALL_PINS;			// set all pins low

	ADCSRA |= ADEN;										// enable ADC
	ADMUX = TOUCH_ADCREFV;						// internal voltage reference, no adc channel selected

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
		LED_PORT ^= LED_PIN;

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

