/***************************************************************************//**
 * @file
 * @brief Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/* Application header */
#include "app.h"
#include "retargetserial.h"
#include "em_core.h"
#include <stdio.h>
#include <stdlib.h>

//Custom header files in INC directory
#include "inc/leuart.h"
#include "inc/connection_param.h"
#include "inc/external_events.h"
#include "inc/barcode.h"
#include "inc/i2c.h"


#define TIMER_CLK_FREQ 							((uint32)32768)				/* Timer Clock Frequency */
#define TIMER_S_TO_TICKS(s)						(TIMER_CLK_FREQ * s)		/* Convert seconds to timer ticks */
#define SOFT_TIMER_LEUART_INTERRUPT				(55)
#define CART_DEBUG_PRINTS						(1)							/* Comment this line to remove
																				debug prints */

#ifdef CART_DEBUG_PRINTS
	#define printf(fmt, args...) printf(fmt, ## args)
#else
	#define printf(fmt, args...)
#endif


#ifndef MAX_ADVERTISERS
#define MAX_ADVERTISERS 1
#endif

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif



uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

/* Bluetooth stack configuration parameters (see "UG136: Silicon Labs Bluetooth C Application Developer's Guide" for details on each parameter) */
static gecko_configuration_t config = {
  .config_flags = 0,                                   /* Check flag options from UG136 */
#if defined(FEATURE_LFXO)
  .sleep.flags = 3, //SLEEP_FLAGS_DEEP_SLEEP_ENABLE,        /* Sleep is enabled */
#else
  .sleep.flags = 0,
#endif // LFXO
  .bluetooth.max_connections = MAX_CONNECTIONS,        /* Maximum number of simultaneous connections */
  .bluetooth.max_advertisers = MAX_ADVERTISERS,        /* Maximum number of advertisement sets */
  .bluetooth.heap = bluetooth_stack_heap,              /* Bluetooth stack memory for connection management */
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap), /* Bluetooth stack memory for connection management */
  .bluetooth.sleep_clock_accuracy = 100,               /* Accuracy of the Low Frequency Crystal Oscillator in ppm. *
                                                       * Do not modify if you are using a module                  */
  .gattdb = &bg_gattdb_data,                           /* Pointer to GATT database */
  .ota.flags = 0,                                      /* Check flag options from UG136 */
  .ota.device_name_len = 3,                            /* Length of the device name in OTA DFU mode */
  .ota.device_name_ptr = "OTA",                        /* Device name in OTA DFU mode */
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1,                               /* Set this to be a valid PA config */
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT,               /* Configure PA input to VBAT */
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,               /* Configure PA input to DCDC */
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .rf.flags = GECKO_RF_CONFIG_ANTENNA,                 /* Enable antenna configuration. */
  .rf.antenna = GECKO_RF_ANTENNA,                      /* Select antenna path! */
};



//Global Variables for Connection Setup
static uint8_t boot_to_dfu = 0;					// Flag for indicating DFU Reset must be performed
static uint8_t connection_handle;
static uint8_t bonding_connnection_handle;


//Function Declarations
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
static void bt_connection_init(void);
static void bt_server_print_address(void);


/**
 * @brief  Main function
 */
int main(void)
{
  /* Initialize device */
  initMcu();

  /* Initialize board */
  initBoard();

  /* Initialize application */
  initApp();

  /* Initializing Bluetooth Stack Configuration */
  gecko_init(&config);

  /* UART Console Setup for Debugging */
  RETARGET_SerialInit();
  RETARGET_SerialCrLf(true);

  printf("Self Checkout Shopping Cart.\n");
  printf("Team Name: Ashwathama.\n");

  //Initializing Structures Circular Buffer and Barcode Packet to value 0
  memset(&leuart_circbuff, 0, sizeof(struct leuart_circbuff));
  memset(&barcode_packet, 0, sizeof(struct barcode_packet));

  //Starting Software Timer for leuart interrupts.
  //gecko_cmd_hardware_set_soft_timer(TIMER_S_TO_TICKS(1), SOFT_TIMER_LEUART_INTERRUPT, 0);


  while (1)
  {
	  struct gecko_cmd_packet *evt = gecko_wait_event();
	  handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
  }
}



/**
 * @brief This function handles all the gecko events.
 */
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	switch (evt_id) {
	case gecko_evt_dfu_boot_id:
		printf("Event: gecko_evt_dfu_boot_id\n");
		break;


	case gecko_evt_system_boot_id:
		printf("Event: gecko_evt_system_boot_id\n");

		//Set up Bluetooth connection parameters and start advertising.
		bt_connection_init();
		bt_server_print_address();
		leuart_init();
		i2c_init();
		//i2c_test_blocking();

		//gecko_cmd_sm_increase_security(connection_handle);

		break;


	/* This event is generated when a connected client has either
	 * 1) changed a Characteristic Client Configuration, meaning that they have enabled
	 * or disabled Notifications or Indications, or
	 * 2) sent a confirmation upon a successful reception of the indication. */
	case gecko_evt_gatt_server_characteristic_status_id:
		printf("Event: gecko_evt_gatt_server_characteristic_status_id\n");
		break;


	case gecko_evt_le_connection_opened_id:

		printf("Event: gecko_evt_le_connection_opened_id\n");
		char client_address_string[6];

		/*Configure Connection Parameters*/
		gecko_cmd_le_connection_set_parameters(evt->data.evt_le_connection_opened.connection,
													CON_INTERVAL_MIN,CON_INTERVAL_MAX,CON_LATENCY,CON_TIMEOUT);
		connection_handle = evt->data.evt_le_connection_opened.connection;

		//gecko_cmd_sm_increase_security(connection_handle);

		bd_addr client_address = evt->data.evt_le_connection_opened.address;
		sprintf(client_address_string,"X:X:X:X:%x:%x",client_address.addr[1],client_address.addr[0]);
		printf("Client Address: %s \n", client_address_string);

		break;


	case gecko_evt_sm_confirm_passkey_id:

		printf("Event: gecko_evt_sm_confirm_passkey_id\n");
		fflush(stdout);

		char password[32];

		//bonding_connnection_handle = evt->data.evt_sm_confirm_bonding.connection;
		sprintf(password,"%lu",evt->data.evt_sm_passkey_display.passkey);
		printf("Password: %s", password);
		gecko_cmd_sm_passkey_confirm( bonding_connnection_handle, 1);

		break;


	case gecko_evt_sm_confirm_bonding_id:
		printf("Event: gecko_evt_sm_confirm_bonding_id\n");
		bonding_connnection_handle = evt->data.evt_sm_confirm_bonding.connection;
		struct gecko_msg_sm_bonding_confirm_rsp_t* results;
		results = gecko_cmd_sm_bonding_confirm(bonding_connnection_handle, 1);
		printf("Result Bonding: %d\n",results->result);

		break;


	case gecko_evt_sm_bonded_id:
		printf("Event: gecko_evt_sm_bonded_id\n");
		break;


	case gecko_evt_sm_bonding_failed_id:
		printf("Event: gecko_evt_sm_bonding_failed_id\n");
		break;


	case gecko_evt_le_connection_closed_id:
		/* Check if need to boot to dfu mode */
		printf("Event: gecko_evt_le_connection_closed_id\n");
		printf("Disconnected\n");

		gecko_cmd_system_set_tx_power(0);

		if (boot_to_dfu) {
			/* Enter to DFU OTA mode */
			gecko_cmd_system_reset(2);

		} else {

			/* Stop timer in case client disconnected before indications were turned off */
			gecko_cmd_hardware_set_soft_timer(0, 0, 0);
			/* Restart advertising after client has disconnected */
			gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
		}
		break;


	case gecko_evt_hardware_soft_timer_id:
		switch (evt->data.evt_hardware_soft_timer.handle){

		case SOFT_TIMER_LEUART_INTERRUPT:
			if(!leuart_buffer_empty_status())
			{
				CORE_AtomicDisableIrq();
				external_event |= EVENT_LEUART;
				gecko_external_signal(external_event);
				CORE_AtomicEnableIrq();
			}
			break;
		}
		break;


	case gecko_evt_system_external_signal_id:
//		printf("Event: gecko_evt_system_external_signal_id\n");

//		TODO: Disable interrupts for the entire block
		if (evt->data.evt_system_external_signal.extsignals & EVENT_LEUART)
		{
//			printf("External Signal Event for LEUART received.\n");

			CORE_AtomicDisableIrq();
			external_event &= ~EVENT_LEUART;
			CORE_AtomicEnableIrq();

			//Initial data - Not necessary to initialize
			uint32_t temp_read_index = leuart_circbuff.read_index;
			int payload_size = 0;																/* Payload_size might need change depending on what
																									needs to be sent to the android application */

			/* Read data from leuart_circbuff till it is empty */
			while(!leuart_buffer_empty_status())
			{

				//TODO: Add a Software Timer for UART Receiving external event trigger.
				//TODO: Check this if condition 10 times ??? Should not be required because the scanning time for barcode is not that fast.
				if(leuart_circbuff.buffer[leuart_circbuff.read_index] == BARCODE_PREAMBLE)
				{
//					printf("Inside if loop external event\n");
					temp_read_index = leuart_circbuff.read_index + 7;							/* 7 as per the packet structure */

					/* Start making packet here */
					barcode_packet_create(&barcode_packet, &payload_size);

					//TODO: Check if this loop satisfies every condition.
				}
				else if(leuart_circbuff.buffer[leuart_circbuff.read_index] == BARCODE_POSTAMBLE)
				{
					/* Disable and Enable interrupts when copying data from leuart_circbuff to barcode_packet */
					barcode_packet.postamble = leuart_buffer_pop();
					leuart_buffer_pop();														/* To remove the redundant \r received from the barcode*/
					memcpy(&barcode_packet.payload[0], &leuart_circbuff.buffer[temp_read_index], payload_size);


					//TODO: Now Start Sending data over bluetooth here.

					/* Free the barcode_packet data structure after sending data */
					free(barcode_packet.payload);

					/* Initializing the barcode_packet data structure to zero */
					memset(&barcode_packet, 0, sizeof(struct barcode_packet));

				}
				else
				{
					leuart_buffer_pop();														/* To pop remaining data*/
				}
			}
		}
		break;



	/* Events related to OTA upgrading
			   ----------------------------------------------------------------------------- */

	/* Checks if the user-type OTA Control Characteristic was written.
	 * If written, boots the device into Device Firmware Upgrade (DFU) mode. */
	case gecko_evt_gatt_server_user_write_request_id:
		if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
			/* Set flag to enter to OTA mode */
			boot_to_dfu = 1;
			/* Send response to Write Request */
			gecko_cmd_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control,
					bg_err_success);

			/* Close connection to enter to DFU OTA mode */
			gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
		}
		break;

	default:
		break;
	}
}



/**
 * @brief This function prints the Server Bluetooth Public address
 * @param void
 * @return void
 */
static void bt_server_print_address(void)
{
	char server_address_string[6];
	struct gecko_msg_system_get_bt_address_rsp_t *bt_addr = gecko_cmd_system_get_bt_address();
	bd_addr *server_address = &bt_addr->address;

	sprintf(server_address_string,"%x:%x:%x:%x:%x:%x", server_address->addr[5],server_address->addr[4]
												,server_address->addr[3],server_address->addr[2],
												server_address->addr[1],server_address->addr[0]);
	printf("Server Address: %s \n", server_address_string);
}



/**
 * @brief This function initializes the bluetooth connection.
 * The function prints the server address, deletes all previous bondings, sets into bonding mode,
 * sets the transmit power to zero, configures and starts advertising.
 * @param void
 * @return void
 */
static void bt_connection_init(void)
{
	//Prints the Server Bluetooth Public address
	bt_server_print_address();

	//Setting Transmit Power
	gecko_cmd_system_set_tx_power(0);

	// Delete all previous bondings
	gecko_cmd_sm_delete_bondings();

	// Configure Security
	gecko_cmd_sm_configure(SECURITY_CONFIGURE_FLAG,sm_io_capability_noinputnooutput);

	//Set into Bondable Mode
	gecko_cmd_sm_set_bondable_mode(1);

	// Configure and start general advertising and enable connections.
	gecko_cmd_le_gap_set_advertise_timing(ADV_HANDLE, ADV_INTERVAL_MIN, ADV_INTERVAL_MAX, ADV_TIMING_DURATION, ADV_MAXEVENTS);
	gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
}




/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
