/*
 * @file connection_param.h
 * @brief This file consists of all the connection parameters and macros required for bluetooth connection.
 *
 * @author: Siddhant Jajoo.
 * @date 11/16/2019
 * @copyright Copyright (c) 2019
 *
 */

#ifndef INC_CONNECTION_PARAM_H_
#define INC_CONNECTION_PARAM_H_



#define adv_handle					(0)
#define adv_interval_min			(400)
#define adv_interval_max			(400)
#define adv_timing_duration			(0)
#define adv_maxevents				(0)


#define con_interval_min			(60)
#define con_interval_max			(60)
#define con_latency					(3)
#define con_timeout					(400)


uint8_t connection_handle;
uint8_t bonding_connnection_handle;


#endif /* INC_CONNECTION_PARAM_H_ */
