/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2013 Joshua Harlan Lifton <joshua.harlan.lifton@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include "./usbserial.h"

static const struct {
	struct usb_cdc_header_descriptor 			header;
	struct usb_cdc_call_management_descriptor 	call_mgmt;
	struct usb_cdc_acm_descriptor 				acm;
	struct usb_cdc_union_descriptor 			cdc_union;
  
} __attribute__((packed)) cdcacm_functional_descriptors =
{
	/* The header shall always be the first of a Communcations Class Specific Interface Descriptor */
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,						/* USB version 1.10 */
	},
	.call_mgmt = {
		.bFunctionLength =  sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0x00,					/* under ACM call control is multiplexed with actual data */
		.bDataInterface = 0x01,					/* multiplexed commands are handled via data interface 01h */
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0x00,					/* This device supports none of the commands for the ACM interface */
	},
	/*
	 * The Union Functional Descriptor describes the relationship between a group of interfaces that can be
	 * considered to form a functional unit
	 */
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0x00,					/* the controlling interface of the union */
		.bSubordinateInterface0 = 0x01, 			/* interfaces being controlled by the controlling interface */
	}
};

/*
 * The USB configuration tree
 * 
 * dev		(device descriptor)
 *  |
 *  \- config	(configuration descriptor)
 *       |
 *       | - ifaces (Interface descriptor array)
 *              |
 *              | - interfaces[0] (Interface 0 descriptor)
 *              |      |
 *              |      \ - endpoints[2] (Endpoint descriptor address 0x83)
 *              |  
 *              \ - interfaces[1] (Interface 1 descriptor)
 *                     |
 *                     | - endpoints[0] (Endpoint descriptor address 0x01)
 *                     |
 *                     \ - endpoints[1] (Endpoint descriptor address 0x82)
 */
/*
 * Endpoint descriptors
 */
static const struct usb_endpoint_descriptor endpoints[] =
{
	{											/* Endpoint descriptors for actual data flow (BULK IN and BULK OUT) */
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,		/* This is an endpoint descriptor */
		.bEndpointAddress = 0x01,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,		/* This is an endpoint descriptor */
		.bEndpointAddress = 0x82,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = 64,
		.bInterval = 1,
	},
	{											/* Endpoint descriptor for CDC control requests (INTERRUPT) */
		/*
		* This notification endpoint isn't implemented. According to CDC spec its
		* optional, but its absence causes a NULL pointer dereference in Linux
		* cdc_acm driver.
		*/
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,		/* This is an endpoint descriptor */
		.bEndpointAddress = 0x83,
		.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
		.wMaxPacketSize = 16,
		.bInterval = 255,
	}
};

/*
 * Interface descriptors
 */
static const struct usb_interface_descriptor interfaces[] =
{
	/*
	 * The CDC standard defines two interfaces for the Communications Device Class,
	 * the Communications Interface Class and the Data Interface Class
	 */
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,			/* This is an interface descriptor */
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,								/* We have one entry in our endpoint descriptor array */
		.bInterfaceClass = USB_CLASS_CDC,				/* Communications Interface Class */
		.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,		/* Abstract Control Model */
		.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,		/* AT Commands V.25 */
		.iInterface = 0,

		/* libopencm3 data */
		.endpoint = &endpoints[2],						/* pointer to an endpoint descriptor array */
		.extra = &cdcacm_functional_descriptors,		/* extra data that needs to be tacked on at the end of the endpoint descriptor */
		.extralen = sizeof(cdcacm_functional_descriptors)
	},
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,			/* This is an interface descriptor */
		.bInterfaceNumber = 1,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,								/* We have two entries in our endpoint descriptor array */
		.bInterfaceClass = USB_CLASS_DATA,				/* Data Interface Class */
		.bInterfaceSubClass = 0,						/* unused, always 0 */
		.bInterfaceProtocol = 0,						/* No class specific protocol required */
		.iInterface = 0,

		/* libopencm3 data */
		.endpoint = &endpoints[0],						/* pointer to an endpoint descriptor array */
	}
};

/*
 * libopencm3 helper array to define our interfaces
 */
static const struct usb_interface ifaces[] =
{
	{
		.num_altsetting = 1,
		.altsetting = &interfaces[0],	/* pointer to interface descriptor */
	},
	{
		.num_altsetting = 1,
		.altsetting = &interfaces[1],	/* pointer to interface descriptor */
	}
};

/*
 * The host sends three queries:
 * 1 Give me your device description
 * 2 Give me your configuration
 * 3 Give me your strings
 * 
 * The following three structures are passed to the usbd_init function.
 */
static const struct usb_device_descriptor dev =
{
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,	/* This is a device descriptor */
	.bcdUSB = 0x0200,					/* USB Specification Number which device complies to (USB 2.0)*/
	.bDeviceClass = USB_CLASS_CDC,		/* Communications Device Class */
	.bDeviceSubClass = 0,				/* Unused at this time */
	.bDeviceProtocol = 0,				/* Unused at this time */
	.bMaxPacketSize0 = 64,				/* Maximum Packet Size for Zero Endpoint. Valid Sizes are 8, 16, 32, 64 */
	.idVendor = 0x0483,					/* ST MicroElectronics */
	.idProduct = 0x5740,				/* Virtual Comport */
	.bcdDevice = 0x0200,				/* Device Release Number */
	.iManufacturer = 1,					/* Index of Manufacturer String Descriptor */
	.iProduct = 2,						/* Index of Product String Descriptor */
	.iSerialNumber = 3,					/* Index of Serial Number String Descriptor */
	.bNumConfigurations = 1,
};

static const struct usb_config_descriptor config =
{
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,	/* This is a configuration descriptor */
	.wTotalLength = 0,							/* Total length (calculated by libopencm3) */
	.bNumInterfaces = 2,						/* Number of Interfaces */
	.bConfigurationValue = 1,					/* Value to use to select this configuration */
	.iConfiguration = 0,						/* Index of this Configurations' String Descriptor */
	.bmAttributes = 0x80,						/* Bus Powered, Not self powered, no remote wakeup */
	.bMaxPower = 0x32,							/* max power 0x32 *2mA = 100 mA */

	/* libopencm3 data */
	.interface = &ifaces[0],	/* Pointer to an array of interface definitions */
};

static const char *usb_strings[] =
{
	"LibOpenCM3 Examples",
	"CDC-ACM Echo Demo",
	"SR0001",
};




/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request( usbd_device *usbd_dev,
				struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)( usbd_device *usbd_dev, struct usb_setup_data *req )
) {
	
	(void)complete;
	(void)buf;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		{
			/*
			* This Linux cdc_acm driver requires this to be implemented
			* even though it's optional in the CDC spec, and we don't
			* advertise it in the ACM functional descriptor.
			*/
			char local_buf[10];
			struct usb_cdc_notification *notif = (void *)local_buf;

			/* We echo signals back to host as notification. */
			notif->bmRequestType = 0xA1;
			notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
			notif->wValue = 0;
			notif->wIndex = 0;
			notif->wLength = 2;
			local_buf[8] = req->wValue & 3;
			local_buf[9] = 0;
			usbd_ep_write_packet(usbd_dev, 0x83, buf, 10);
		}
		return USBD_REQ_HANDLED;

	case USB_CDC_REQ_SET_LINE_CODING: 
		if( *len < sizeof(struct usb_cdc_line_coding) )
			return USBD_REQ_NOTSUPP;

		return USBD_REQ_HANDLED;
	}

	return 0;
}

static usbd_device *device;
static uint8_t rx_data_available;

static void cdcacm_data_rx_cb( usbd_device *usbd_dev, uint8_t ep )
{
	(void)ep;
	(void)usbd_dev;
	
	rx_data_available = 1;
}

static void cdcacm_set_config( usbd_device *usbd_dev, uint16_t wValue )
{
	(void)wValue;

	usbd_ep_setup( usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb );
	usbd_ep_setup( usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL );
	usbd_ep_setup( usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL );
	
	/* setup a user defined callback for CDC control requests */
	usbd_register_control_callback( usbd_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
								USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, cdcacm_control_request );
}

void usb_setup( void )
{
	device = usbd_init( &st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer) );

	/* setup a user defined callback for configuration requests */
	usbd_register_set_config_callback( device, cdcacm_set_config );
}

int usb_read( char * buffer, uint16_t length )
{
	rx_data_available = 0;
	
	usbd_poll( device );
	
	return rx_data_available ? usbd_ep_read_packet( device, 0x01, buffer, length ) : 0;
}

int usb_write( char * buffer, uint16_t length )
{
	uint8_t index = 0;

	while( index < length ) {
		
		uint16_t to_write = length - index;
		if( to_write > 64 )
			to_write = 64;
			
		index += usbd_ep_write_packet( device, 0x82, &buffer[index], to_write );
	}

	return 0;
}

