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
#include <stdio.h>

//Custom header files in INC directory
#include "inc/leuart.h"
#include "inc/connection_param.h"



#define CART_DEBUG_PRINTS	(1)		//Comment this line to remove debug prints

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
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,        /* Sleep is enabled */
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


// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;


//Function Declarations
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);


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
  //Initializing Bluetooth Stack Configuration
  gecko_init(&config);

  //UART Console Setup for Debugging
  RETARGET_SerialInit();
  RETARGET_SerialCrLf(true);

  printf("Self Checkout Shopping Cart.\n");
  printf("Team Name: Ashwathama.\n");

  while (1)
  {
	  struct gecko_cmd_packet *evt = gecko_wait_event();
	  handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
  }
}



static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	switch (evt_id) {
	case gecko_evt_dfu_boot_id:
		printf("Event: gecko_evt_dfu_boot_id\n");
		break;


	case gecko_evt_system_boot_id:
		printf("Event: gecko_evt_system_boot_id\n");
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
												con_interval_min,con_interval_max,con_latency,con_timeout);
		connection_handle = evt->data.evt_le_connection_opened.connection;

		gecko_cmd_sm_increase_security(connection_handle);

		bd_addr client_address = evt->data.evt_le_connection_opened.address;
		sprintf(client_address_string,"X:X:X:X:%x:%x",client_address.addr[1],client_address.addr[0]);
		printf("Client Address: %s \n", client_address_string);
		break;


	case gecko_evt_sm_confirm_passkey_id:

		printf("Event: gecko_evt_sm_confirm_passkey_id\n");
		char password[32];

		bonding_connnection_handle = evt->data.evt_sm_confirm_bonding.connection;
		sprintf(password,"%lu",evt->data.evt_sm_passkey_display.passkey);
		printf("Password: %s", password);
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


	case gecko_evt_system_external_signal_id:
		printf("Event: gecko_evt_system_external_signal_id\n");
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
/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
