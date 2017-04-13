/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2015  Intel Corporation. All rights reserved.
 *
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <sys/epoll.h>

#include "lib/bluetooth.h"
#include "lib/l2cap.h"
#include "lib/uuid.h"
#include "src/shared/mainloop.h"
#include "src/shared/util.h"
#include "src/shared/queue.h"
#include "src/shared/timeout.h"
#include "src/shared/att.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-server.h"
#include "src/shared/gatt-client.h"
#include "peripheral/ESS/ess_uuid.h"
#include "peripheral/ESS/ESS.h"


static int att_fd = -1;
static struct queue *conn_list = NULL;
static struct gatt_db *gatt_db = NULL;
static struct gatt_db *gatt_cache = NULL;
static struct ess_temp *temp_db = NULL;
static struct ess_apparent_speed *apparent_speed_db = NULL;
static struct ess_apparent_direction *apparent_direction_db=NULL;
static struct ess_dew_point *dew_point_db=NULL;
static struct ess_elevation *elevation_db=NULL;
static struct ess_gust_factor *gust_factor_db=NULL;
static struct ess_heat_index *heat_index_db=NULL;
static struct ess_humidity *humidity_db=NULL;
static struct ess_irradiance *irradiance_db=NULL;
static struct ess_pollen_concentration *pollen_concentration_db=NULL;
static struct ess_rainfall *rainfall_db=NULL;
static struct ess_pressure *pressure_db=NULL;
static struct ess_true_direction *true_direction_db=NULL;
static struct ess_true_speed *true_speed_db=NULL;
static struct ess_uv_index *uv_index_db=NULL;
static struct ess_wind_chill *wind_chill_db=NULL;
static struct ess_barometric_pressure *barometric_pressure_db=NULL;
static struct ess_magnetic_declination *magnetic_declination_db=NULL;
static struct ess_magnetic_flux_2D *magnetic_flux_2D_db=NULL;
static struct ess_magnetic_flux_3D *magnetic_flux_3D_db=NULL;
static struct gatt_conn *conn = NULL;

static uint8_t public_addr[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static uint8_t dev_name[20];
static uint8_t dev_name_len = 0;


/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for temperature characteristic and its descriptors like                                   ***
***  Temperature = 0x0A8C (27 degC)							       ***
***  User Descriptor = "Temperature charact "                                                  ***
***  range = 0 to 100 (degC) 			  					       ***
***  flags = 0 ,sampling = instantaneous, measurement_period= 20 sec, update=29sec,	       ***	
***  application = left, uncertainity = 10.5%                                                  ***

*/


static void es_temp_init(void)
{

	temp_db = new0(struct ess_temp, 1);
	temp_db->temp_ms = new0(struct ess_measurement, 1);
	temp_db->temp_tr = new0(struct trigger_setting, 1);
	temp_db->user_desc = new0(wchar_t, 21);

	memcpy(temp_db->user_desc, "Temperature Charact", 19);
	temp_db->lower_temp = 0x0000;
	temp_db->upper_temp = 0x2710;
	temp_db->temp_data = 0x0A8C;
	temp_db->temp_ms->flags = 0;
	temp_db->temp_ms->sample = 0x01;
	temp_db->temp_ms->applicatn = 0x1D;
	temp_db->temp_ms->m_uncertainity = 0x15;
	temp_db->temp_ms->m_period[0] = 0x14;
	temp_db->temp_ms->m_period[1] = 0;
	temp_db->temp_ms->m_period[2] = 0;
	temp_db->temp_ms->u_interval[0] = 0x1D;
	temp_db->temp_ms->u_interval[1] = 0;
	temp_db->temp_ms->u_interval[2] = 0;
	temp_db->indication = 0x0000;
	temp_db->temp_tr->condition = 0x01;
	temp_db->temp_tr->data[0] = 0x3C;
	temp_db->temp_tr->data[1] = 0x00;
	temp_db->temp_tr->data[2] = 0x00;

	if( temp_db->temp_tr->condition==0x01 || temp_db->temp_tr->condition==0x02)
		temp_db->temp_tr->time_enable=true;
	else if( temp_db->temp_tr->condition>=0x03 && temp_db->temp_tr->condition<= 0x09)
		temp_db->temp_tr->value_enable=true;
	else	
		temp_db->temp_tr->trigger_inactive=true;

	temp_db->g_update_timer=(temp_db->temp_tr->data[0])|(temp_db->temp_tr->data[1]<<8)| (temp_db->temp_tr->data[2]<<16);

}


/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Apparent wind Speed characteristic and its descriptors like                                ***
***  Wind Speed = 0x0100 (2.56 m/sec)						               ***
***  User Descriptor = "Apparent Wind speed"                                                  ***
***  range = 0 to 100 (m/sec) 			  					       ***
***  flags = 0 ,sampling = instantaneous, measurement_period= 10 sec, update=15sec,	       ***	
***  application = Air, uncertainity = 10.5%                                                   ***

*/

static void es_speed_init(void)
{

	apparent_speed_db = new0(struct ess_apparent_speed, 1);
	apparent_speed_db->apparent_speed_ms = new0(struct ess_measurement, 1);
	apparent_speed_db->apparent_speed_tr = new0(struct trigger_setting, 1);
	apparent_speed_db->user_desc = new0(wchar_t, 21);

	memcpy(apparent_speed_db->user_desc, "Apparent wind Speed", 20);
	apparent_speed_db->lower_speed = 0x0000;
	apparent_speed_db->upper_speed = 0x2710;
	apparent_speed_db->apparent_speed_data = 0x0100;
	apparent_speed_db->apparent_speed_ms->flags = 0;
	apparent_speed_db->apparent_speed_ms->sample = 0x01;
	apparent_speed_db->apparent_speed_ms->applicatn = 0x01;
	apparent_speed_db->apparent_speed_ms->m_uncertainity = 0x15;
	apparent_speed_db->apparent_speed_ms->m_period[0] = 0x0A;
	apparent_speed_db->apparent_speed_ms->m_period[1] = 0;
	apparent_speed_db->apparent_speed_ms->m_period[2] = 0;
	apparent_speed_db->apparent_speed_ms->u_interval[0] = 0x0F;
	apparent_speed_db->apparent_speed_ms->u_interval[1] = 0;
	apparent_speed_db->apparent_speed_ms->u_interval[2] = 0;
	apparent_speed_db->indication = 0x0000;
	apparent_speed_db->apparent_speed_tr->condition = 0x01;
	apparent_speed_db->apparent_speed_tr->data[0] = 0x3C;
	apparent_speed_db->apparent_speed_tr->data[1] = 0x00;
	apparent_speed_db->apparent_speed_tr->data[2] = 0x00;
        	 
	if( apparent_speed_db->apparent_speed_tr->condition==0x01 || apparent_speed_db->apparent_speed_tr->condition==0x02)
		apparent_speed_db->apparent_speed_tr->time_enable=true;
	else if( apparent_speed_db->apparent_speed_tr->condition>=0x03 && apparent_speed_db->apparent_speed_tr->condition<= 0x09)
		apparent_speed_db->apparent_speed_tr->value_enable=true;
	else	
		apparent_speed_db->apparent_speed_tr->trigger_inactive=true;

	apparent_speed_db->g_update_timer=(apparent_speed_db->apparent_speed_tr->data[0])|(apparent_speed_db->apparent_speed_tr->data[1]<<8)| (apparent_speed_db->apparent_speed_tr->data[2]<<16);
}


/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Apparent Wind direction characteristic and its descriptors like                       ***
***  Wind Direction = 0x0123 (2.91 deg)						               ***
***  User Descriptor = "Apparent Direction"                                                    ***
***  range = 0 to 436.9 (deg) 			  					       ***
***  flags = 0 ,sampling = RMS, measurement_period= 60 sec, update=316sec,	               ***	
***  application = Air, uncertainity = 10.5%                                                   ***

*/

static void es_apparent_direction_init(void)
{

	apparent_direction_db = new0(struct ess_apparent_direction, 1);
	apparent_direction_db->apparent_direction_ms = new0(struct ess_measurement, 1);
	apparent_direction_db->apparent_direction_tr = new0(struct trigger_setting, 1);
	apparent_direction_db->user_desc = new0(wchar_t, 21);
	
	apparent_direction_db->apparent_direction_data = 0x0123;
	memcpy(apparent_direction_db->user_desc, "Apparent Direction", 18);
	apparent_direction_db->lower_direction = 0x0000;
	apparent_direction_db->upper_direction = 0xAAAA;
	apparent_direction_db->apparent_direction_ms->flags = 0;
	apparent_direction_db->apparent_direction_ms->sample = 0x03;
	apparent_direction_db->apparent_direction_ms->applicatn = 0x01;
	apparent_direction_db->apparent_direction_ms->m_uncertainity = 0x15;
	apparent_direction_db->apparent_direction_ms->m_period[0] = 0x3C;
	apparent_direction_db->apparent_direction_ms->m_period[1] = 0;
	apparent_direction_db->apparent_direction_ms->m_period[2] = 0;
	apparent_direction_db->apparent_direction_ms->u_interval[0] = 0x3C;
	apparent_direction_db->apparent_direction_ms->u_interval[1] = 0x01;
	apparent_direction_db->apparent_direction_ms->u_interval[2] = 0;
	apparent_direction_db->indication = 0x0000;
	apparent_direction_db->apparent_direction_tr->condition = 0x01;
	apparent_direction_db->apparent_direction_tr->data[0] = 0x3C;
	apparent_direction_db->apparent_direction_tr->data[1] = 0x00;
	apparent_direction_db->apparent_direction_tr->data[2] = 0x00;

	if( apparent_direction_db->apparent_direction_tr->condition==0x01 || apparent_direction_db->apparent_direction_tr->condition==0x02)
		apparent_direction_db->apparent_direction_tr->time_enable=true;
	else if( apparent_direction_db->apparent_direction_tr->condition>=0x03 && apparent_direction_db->apparent_direction_tr->condition<= 0x09)
		apparent_direction_db->apparent_direction_tr->value_enable=true;
	else	
		apparent_direction_db->apparent_direction_tr->trigger_inactive=true;

	apparent_direction_db->g_update_timer=(apparent_direction_db->apparent_direction_tr->data[0])|(apparent_direction_db->apparent_direction_tr->data[1]<<8)| (apparent_direction_db->apparent_direction_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Dew Point characteristic and its descriptors like                                     ***
***  User Descriptor = "Dew Point"                                                             ***

*/


static void es_dew_point_init(void)
{

	dew_point_db = new0(struct ess_dew_point, 1);
	dew_point_db->dew_point_ms = new0(struct ess_measurement, 1);
	dew_point_db->dew_point_tr = new0(struct trigger_setting, 1);
	dew_point_db->user_desc = new0(wchar_t, 21);
	
	dew_point_db->dew_point_data = 0x23;
	memcpy(dew_point_db->user_desc, "Dew Point", 10);
	dew_point_db->lower_dew_point = 10;
	dew_point_db->upper_dew_point = 45;
	dew_point_db->dew_point_ms->flags = 0;
	dew_point_db->dew_point_ms->sample = 0x02;
	dew_point_db->dew_point_ms->applicatn = 0x15;
	dew_point_db->dew_point_ms->m_uncertainity = 0x15;
	dew_point_db->dew_point_ms->m_period[0] = 0x3C;
	dew_point_db->dew_point_ms->m_period[1] = 0x10;
	dew_point_db->dew_point_ms->m_period[2] = 0;
	dew_point_db->dew_point_ms->u_interval[0] = 0x3C;
	dew_point_db->dew_point_ms->u_interval[1] = 0x1F;
	dew_point_db->dew_point_ms->u_interval[2] = 0;
	dew_point_db->indication = 0x0000;
	dew_point_db->dew_point_tr->condition = 0x01;
	dew_point_db->dew_point_tr->data[0] = 0x3C;
	dew_point_db->dew_point_tr->data[1] = 0x00;
	dew_point_db->dew_point_tr->data[2] = 0x00;

	if( dew_point_db->dew_point_tr->condition==0x01 || dew_point_db->dew_point_tr->condition==0x02)
		dew_point_db->dew_point_tr->time_enable=true;
	else if( dew_point_db->dew_point_tr->condition>=0x03 && dew_point_db->dew_point_tr->condition<= 0x09)
		dew_point_db->dew_point_tr->value_enable=true;
	else	
		dew_point_db->dew_point_tr->trigger_inactive=true;

	dew_point_db->g_update_timer=(dew_point_db->dew_point_tr->data[0])|(dew_point_db->dew_point_tr->data[1]<<8)| (dew_point_db->dew_point_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Elevation characteristic and its descriptors like                                     ***
***  User Descriptor = "Elevation"                                                             ***

*/

static void es_elevation_init(void)
{

	elevation_db = new0(struct ess_elevation, 1);
	elevation_db->elevation_ms = new0(struct ess_measurement, 1);
	elevation_db->elevation_tr = new0(struct trigger_setting, 1);
	elevation_db->user_desc = new0(wchar_t, 21);
	
	elevation_db->elevation_data = rand();
	memcpy(elevation_db->user_desc, "Elevation", 10);
	elevation_db->lower_elevation[0] = 0x15;
	elevation_db->lower_elevation[1] = 0x25;
	elevation_db->lower_elevation[2] = 0x00;
	elevation_db->upper_elevation[0] = 0x55;
	elevation_db->upper_elevation[1] = 0x25;
	elevation_db->upper_elevation[2] = 0x00;
	elevation_db->elevation_ms->flags = 0;
	elevation_db->elevation_ms->sample = 0x07;
	elevation_db->elevation_ms->applicatn = 0x15;
	elevation_db->elevation_ms->m_uncertainity = 0x15;
	elevation_db->elevation_ms->m_period[0] = 0x3C;
	elevation_db->elevation_ms->m_period[1] = 0x01;
	elevation_db->elevation_ms->m_period[2] = 0;
	elevation_db->elevation_ms->u_interval[0] = 0x3C;
	elevation_db->elevation_ms->u_interval[1] = 0x1F;
	elevation_db->elevation_ms->u_interval[2] = 0;
	elevation_db->indication = 0x0000;
	elevation_db->elevation_tr->condition = 0x01;
	elevation_db->elevation_tr->data[0] = 0x3C;
	elevation_db->elevation_tr->data[1] = 0x00;
	elevation_db->elevation_tr->data[2] = 0x00;

	elevation_db->tr_value = 0;

	if( elevation_db->elevation_tr->condition==0x01 || elevation_db->elevation_tr->condition==0x02)
		elevation_db->elevation_tr->time_enable=true;
	else if( elevation_db->elevation_tr->condition>=0x03 && elevation_db->elevation_tr->condition<= 0x09)
		elevation_db->elevation_tr->value_enable=true;
	else	
		elevation_db->elevation_tr->trigger_inactive=true;

	elevation_db->g_update_timer=(elevation_db->elevation_tr->data[0])|(elevation_db->elevation_tr->data[1]<<8)| (elevation_db->elevation_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Gust Factor characteristic and its descriptors like                                   ***
***  User Descriptor = "Gust Factor"                                                           ***

*/

static void es_gust_factor_init(void)
{

	gust_factor_db = new0(struct ess_gust_factor, 1);
	gust_factor_db->gust_factor_ms = new0(struct ess_measurement, 1);
	gust_factor_db->gust_factor_tr = new0(struct trigger_setting, 1);
	gust_factor_db->user_desc = new0(wchar_t, 21);
	
	gust_factor_db->gust_factor_data = rand();
	memcpy(gust_factor_db->user_desc, "Gust Factor", 10);
	gust_factor_db->lower_gust_factor = 10;
	gust_factor_db->upper_gust_factor = 45;
	gust_factor_db->gust_factor_ms->flags = 0;
	gust_factor_db->gust_factor_ms->sample = 0x05;
	gust_factor_db->gust_factor_ms->applicatn = 0x09;
	gust_factor_db->gust_factor_ms->m_uncertainity = 0x08;
	gust_factor_db->gust_factor_ms->m_period[0] = 0x3C;
	gust_factor_db->gust_factor_ms->m_period[1] = 0x10;
	gust_factor_db->gust_factor_ms->m_period[2] = 0;
	gust_factor_db->gust_factor_ms->u_interval[0] = 0x3C;
	gust_factor_db->gust_factor_ms->u_interval[1] = 0x1F;
	gust_factor_db->gust_factor_ms->u_interval[2] = 0;
	gust_factor_db->indication = 0x0000;
	gust_factor_db->gust_factor_tr->condition = 0x01;
	gust_factor_db->gust_factor_tr->data[0] = 0x3C;
	gust_factor_db->gust_factor_tr->data[1] = 0x00;
	gust_factor_db->gust_factor_tr->data[2] = 0x00;

	if( gust_factor_db->gust_factor_tr->condition==0x01 || gust_factor_db->gust_factor_tr->condition==0x02)
		gust_factor_db->gust_factor_tr->time_enable=true;
	else if( gust_factor_db->gust_factor_tr->condition>=0x03 && gust_factor_db->gust_factor_tr->condition<= 0x09)
		gust_factor_db->gust_factor_tr->value_enable=true;
	else	
		gust_factor_db->gust_factor_tr->trigger_inactive=true;

	gust_factor_db->g_update_timer=(gust_factor_db->gust_factor_tr->data[0])|(gust_factor_db->gust_factor_tr->data[1]<<8)| (gust_factor_db->gust_factor_tr->data[2]<<16);

}


/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Heat index characteristic and its descriptors like                                    ***
***  User Descriptor = "Heat index"                                                            ***

*/

static void es_heat_index_init(void)
{

	heat_index_db = new0(struct ess_heat_index, 1);
	heat_index_db->heat_index_ms = new0(struct ess_measurement, 1);
	heat_index_db->heat_index_tr = new0(struct trigger_setting, 1);
	heat_index_db->user_desc = new0(wchar_t, 21);
	
	heat_index_db->heat_index_data = 0x23;
	memcpy(heat_index_db->user_desc, "Heat index", 10);
	heat_index_db->lower_heat_index = 10;
	heat_index_db->upper_heat_index = 45;
	heat_index_db->heat_index_ms->flags = 0;
	heat_index_db->heat_index_ms->sample = 0x02;
	heat_index_db->heat_index_ms->applicatn = 0x15;
	heat_index_db->heat_index_ms->m_uncertainity = 0x15;
	heat_index_db->heat_index_ms->m_period[0] = 0x3C;
	heat_index_db->heat_index_ms->m_period[1] = 0x10;
	heat_index_db->heat_index_ms->m_period[2] = 0;
	heat_index_db->heat_index_ms->u_interval[0] = 0x3C;
	heat_index_db->heat_index_ms->u_interval[1] = 0x1F;
	heat_index_db->heat_index_ms->u_interval[2] = 0;
	heat_index_db->indication = 0x0000;
	heat_index_db->heat_index_tr->condition = 0x01;
	heat_index_db->heat_index_tr->data[0] = 0x3C;
	heat_index_db->heat_index_tr->data[1] = 0x00;
	heat_index_db->heat_index_tr->data[2] = 0x00;

	if( heat_index_db->heat_index_tr->condition==0x01 || heat_index_db->heat_index_tr->condition==0x02)
		heat_index_db->heat_index_tr->time_enable=true;
	else if( heat_index_db->heat_index_tr->condition>=0x03 && heat_index_db->heat_index_tr->condition<= 0x09)
		heat_index_db->heat_index_tr->value_enable=true;
	else	
		heat_index_db->heat_index_tr->trigger_inactive=true;

	heat_index_db->g_update_timer=(heat_index_db->heat_index_tr->data[0])|(heat_index_db->heat_index_tr->data[1]<<8)| (heat_index_db->heat_index_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Humidity characteristic and its descriptors like                                      ***
***  User Descriptor = "Humidity"                                                              ***

*/

static void es_humidity_init(void)
{

	humidity_db = new0(struct ess_humidity, 1);
	humidity_db->humidity_ms = new0(struct ess_measurement, 1);
	humidity_db->humidity_tr = new0(struct trigger_setting, 1);
	humidity_db->user_desc = new0(wchar_t, 21);
	
	humidity_db->humidity_data = rand();
	memcpy(humidity_db->user_desc, "Humidity", 8);
	humidity_db->lower_humidity = 10;
	humidity_db->upper_humidity = 45;
	humidity_db->humidity_ms->flags = 0;
	humidity_db->humidity_ms->sample = 0x02;
	humidity_db->humidity_ms->applicatn = 0x15;
	humidity_db->humidity_ms->m_uncertainity = 0x15;
	humidity_db->humidity_ms->m_period[0] = 0x3C;
	humidity_db->humidity_ms->m_period[1] = 0x10;
	humidity_db->humidity_ms->m_period[2] = 0;
	humidity_db->humidity_ms->u_interval[0] = 0x3C;
	humidity_db->humidity_ms->u_interval[1] = 0x1F;
	humidity_db->humidity_ms->u_interval[2] = 0;
	humidity_db->indication = 0x0000;
	humidity_db->humidity_tr->condition = 0x01;
	humidity_db->humidity_tr->data[0] = 0x3C;
	humidity_db->humidity_tr->data[1] = 0x00;
	humidity_db->humidity_tr->data[2] = 0x00;

	if( humidity_db->humidity_tr->condition==0x01 || humidity_db->humidity_tr->condition==0x02)
		humidity_db->humidity_tr->time_enable=true;
	else if( humidity_db->humidity_tr->condition>=0x03 && humidity_db->humidity_tr->condition<= 0x09)
		humidity_db->humidity_tr->value_enable=true;
	else	
		humidity_db->humidity_tr->trigger_inactive=true;

	humidity_db->g_update_timer=(humidity_db->humidity_tr->data[0])|(humidity_db->humidity_tr->data[1]<<8)| (humidity_db->humidity_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Irradiance characteristic and its descriptors like                                    ***
***  User Descriptor = "Irradiance"                                                            ***

*/

static void es_irradiance_init(void)
{

	irradiance_db = new0(struct ess_irradiance, 1);
	irradiance_db->irradiance_ms = new0(struct ess_measurement, 1);
	irradiance_db->irradiance_tr = new0(struct trigger_setting, 1);
	irradiance_db->user_desc = new0(wchar_t, 21);
	
	irradiance_db->irradiance_data = rand();
	memcpy(irradiance_db->user_desc, "Irradiance", 10);
	irradiance_db->lower_irradiance = 10;
	irradiance_db->upper_irradiance = 45;
	irradiance_db->irradiance_ms->flags = 0;
	irradiance_db->irradiance_ms->sample = 0x02;
	irradiance_db->irradiance_ms->applicatn = 0x15;
	irradiance_db->irradiance_ms->m_uncertainity = 0x15;
	irradiance_db->irradiance_ms->m_period[0] = 0x3C;
	irradiance_db->irradiance_ms->m_period[1] = 0x10;
	irradiance_db->irradiance_ms->m_period[2] = 0;
	irradiance_db->irradiance_ms->u_interval[0] = 0x3C;
	irradiance_db->irradiance_ms->u_interval[1] = 0x1F;
	irradiance_db->irradiance_ms->u_interval[2] = 0;
	irradiance_db->indication = 0x0000;
	irradiance_db->irradiance_tr->condition = 0x01;
	irradiance_db->irradiance_tr->data[0] = 0x3C;
	irradiance_db->irradiance_tr->data[1] = 0x00;
	irradiance_db->irradiance_tr->data[2] = 0x00;

	if( irradiance_db->irradiance_tr->condition==0x01 || irradiance_db->irradiance_tr->condition==0x02)
		irradiance_db->irradiance_tr->time_enable=true;
	else if( irradiance_db->irradiance_tr->condition>=0x03 && irradiance_db->irradiance_tr->condition<= 0x09)
		irradiance_db->irradiance_tr->value_enable=true;
	else	
		irradiance_db->irradiance_tr->trigger_inactive=true;

	irradiance_db->g_update_timer=(irradiance_db->irradiance_tr->data[0])|(irradiance_db->irradiance_tr->data[1]<<8)| (irradiance_db->irradiance_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Pollen Concentration characteristic and its descriptors like                          ***
***  User Descriptor = "Pollen Concentration"                                                  ***

*/

static void es_pollen_concentration_init(void)
{

	pollen_concentration_db = new0(struct ess_pollen_concentration, 1);
	pollen_concentration_db->pollen_concentration_ms = new0(struct ess_measurement, 1);
	pollen_concentration_db->pollen_concentration_tr = new0(struct trigger_setting, 1);
	pollen_concentration_db->user_desc = new0(wchar_t, 21);
	
	pollen_concentration_db->pollen_concentration_data = rand();
	memcpy(pollen_concentration_db->user_desc, "Pollen Concentration", 20);
	pollen_concentration_db->lower_pollen_concentration[0] = 0x15;
	pollen_concentration_db->lower_pollen_concentration[1] = 0x25;
	pollen_concentration_db->lower_pollen_concentration[2] = 0x00;
	pollen_concentration_db->upper_pollen_concentration[0] = 0x55;
	pollen_concentration_db->upper_pollen_concentration[1] = 0x25;
	pollen_concentration_db->upper_pollen_concentration[2] = 0x00;
	pollen_concentration_db->pollen_concentration_ms->flags = 0;
	pollen_concentration_db->pollen_concentration_ms->sample = 0x07;
	pollen_concentration_db->pollen_concentration_ms->applicatn = 0x15;
	pollen_concentration_db->pollen_concentration_ms->m_uncertainity = 0x15;
	pollen_concentration_db->pollen_concentration_ms->m_period[0] = 0x3C;
	pollen_concentration_db->pollen_concentration_ms->m_period[1] = 0x01;
	pollen_concentration_db->pollen_concentration_ms->m_period[2] = 0;
	pollen_concentration_db->pollen_concentration_ms->u_interval[0] = 0x3C;
	pollen_concentration_db->pollen_concentration_ms->u_interval[1] = 0x1F;
	pollen_concentration_db->pollen_concentration_ms->u_interval[2] = 0;
	pollen_concentration_db->indication = 0x0000;
	pollen_concentration_db->pollen_concentration_tr->condition = 0x01;
	pollen_concentration_db->pollen_concentration_tr->data[0] = 0x3C;
	pollen_concentration_db->pollen_concentration_tr->data[1] = 0x00;
	pollen_concentration_db->pollen_concentration_tr->data[2] = 0x00;

	pollen_concentration_db->tr_value = 0;

	if( pollen_concentration_db->pollen_concentration_tr->condition==0x01 || pollen_concentration_db->pollen_concentration_tr->condition==0x02)
		pollen_concentration_db->pollen_concentration_tr->time_enable=true;
	else if( pollen_concentration_db->pollen_concentration_tr->condition>=0x03 && pollen_concentration_db->pollen_concentration_tr->condition<= 0x09)
		pollen_concentration_db->pollen_concentration_tr->value_enable=true;
	else	
		pollen_concentration_db->pollen_concentration_tr->trigger_inactive=true;

	pollen_concentration_db->g_update_timer=(pollen_concentration_db->pollen_concentration_tr->data[0])|(pollen_concentration_db->pollen_concentration_tr->data[1]<<8)| (pollen_concentration_db->pollen_concentration_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Rainfall characteristic and its descriptors like                                      ***
***  User Descriptor = "Rainfall"                                                              ***

*/

static void es_rainfall_init(void)
{

	rainfall_db = new0(struct ess_rainfall, 1);
	rainfall_db->rainfall_ms = new0(struct ess_measurement, 1);
	rainfall_db->rainfall_tr = new0(struct trigger_setting, 1);
	rainfall_db->user_desc = new0(wchar_t, 21);
	
	rainfall_db->rainfall_data = rand();
	memcpy(rainfall_db->user_desc, "Rain Fall", 9);
	rainfall_db->lower_rainfall = 10;
	rainfall_db->upper_rainfall = 45;
	rainfall_db->rainfall_ms->flags = 0;
	rainfall_db->rainfall_ms->sample = 0x02;
	rainfall_db->rainfall_ms->applicatn = 0x15;
	rainfall_db->rainfall_ms->m_uncertainity = 0x15;
	rainfall_db->rainfall_ms->m_period[0] = 0x3C;
	rainfall_db->rainfall_ms->m_period[1] = 0x10;
	rainfall_db->rainfall_ms->m_period[2] = 0;
	rainfall_db->rainfall_ms->u_interval[0] = 0x3C;
	rainfall_db->rainfall_ms->u_interval[1] = 0x1F;
	rainfall_db->rainfall_ms->u_interval[2] = 0;
	rainfall_db->indication = 0x0000;
	rainfall_db->rainfall_tr->condition = 0x01;
	rainfall_db->rainfall_tr->data[0] = 0x3C;
	rainfall_db->rainfall_tr->data[1] = 0x00;
	rainfall_db->rainfall_tr->data[2] = 0x00;

	if( rainfall_db->rainfall_tr->condition==0x01 || rainfall_db->rainfall_tr->condition==0x02)
		rainfall_db->rainfall_tr->time_enable=true;
	else if( rainfall_db->rainfall_tr->condition>=0x03 && rainfall_db->rainfall_tr->condition<= 0x09)
		rainfall_db->rainfall_tr->value_enable=true;
	else	
		rainfall_db->rainfall_tr->trigger_inactive=true;

	rainfall_db->g_update_timer=(rainfall_db->rainfall_tr->data[0])|(rainfall_db->rainfall_tr->data[1]<<8)| (rainfall_db->rainfall_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Pressure characteristic and its descriptors like                                      ***
***  User Descriptor = "Pressure"                                                              ***

*/

static void es_pressure_init(void)
{

	pressure_db = new0(struct ess_pressure, 1);
	pressure_db->pressure_ms = new0(struct ess_measurement, 1);
	pressure_db->pressure_tr = new0(struct trigger_setting, 1);
	pressure_db->user_desc = new0(wchar_t, 21);
	
	pressure_db->pressure_data = rand();
	memcpy(pressure_db->user_desc, "Pressure", 8);
	pressure_db->lower_pressure = 10;
	pressure_db->upper_pressure = 45;
	pressure_db->pressure_ms->flags = 0;
	pressure_db->pressure_ms->sample = 0x02;
	pressure_db->pressure_ms->applicatn = 0x15;
	pressure_db->pressure_ms->m_uncertainity = 0x15;
	pressure_db->pressure_ms->m_period[0] = 0x3C;
	pressure_db->pressure_ms->m_period[1] = 0x10;
	pressure_db->pressure_ms->m_period[2] = 0;
	pressure_db->pressure_ms->u_interval[0] = 0x3C;
	pressure_db->pressure_ms->u_interval[1] = 0x1F;
	pressure_db->pressure_ms->u_interval[2] = 0;
	pressure_db->indication = 0x0000;
	pressure_db->pressure_tr->condition = 0x01;
	pressure_db->pressure_tr->data[0] = 0x3C;
	pressure_db->pressure_tr->data[1] = 0x00;
	pressure_db->pressure_tr->data[2] = 0x00;

	if( pressure_db->pressure_tr->condition==0x01 || pressure_db->pressure_tr->condition==0x02)
		pressure_db->pressure_tr->time_enable=true;
	else if( pressure_db->pressure_tr->condition>=0x03 && pressure_db->pressure_tr->condition<= 0x09)
		pressure_db->pressure_tr->value_enable=true;
	else	
		pressure_db->pressure_tr->trigger_inactive=true;

	pressure_db->g_update_timer=(pressure_db->pressure_tr->data[0])|(pressure_db->pressure_tr->data[1]<<8)| (pressure_db->pressure_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for True Wind Direction characteristic and its descriptors like                           ***
***  User Descriptor = "True Wind Direction"                                                   ***

*/

static void es_true_direction_init(void)
{

	true_direction_db = new0(struct ess_true_direction, 1);
	true_direction_db->true_direction_ms = new0(struct ess_measurement, 1);
	true_direction_db->true_direction_tr = new0(struct trigger_setting, 1);
	true_direction_db->user_desc = new0(wchar_t, 21);
	
	true_direction_db->true_direction_data = rand();
	memcpy(true_direction_db->user_desc, "True Wind Direction", 19);
	true_direction_db->lower_true_direction = 10;
	true_direction_db->upper_true_direction = 45;
	true_direction_db->true_direction_ms->flags = 0;
	true_direction_db->true_direction_ms->sample = 0x02;
	true_direction_db->true_direction_ms->applicatn = 0x15;
	true_direction_db->true_direction_ms->m_uncertainity = 0x15;
	true_direction_db->true_direction_ms->m_period[0] = 0x3C;
	true_direction_db->true_direction_ms->m_period[1] = 0x10;
	true_direction_db->true_direction_ms->m_period[2] = 0;
	true_direction_db->true_direction_ms->u_interval[0] = 0x3C;
	true_direction_db->true_direction_ms->u_interval[1] = 0x1F;
	true_direction_db->true_direction_ms->u_interval[2] = 0;
	true_direction_db->indication = 0x0000;
	true_direction_db->true_direction_tr->condition = 0x01;
	true_direction_db->true_direction_tr->data[0] = 0x3C;
	true_direction_db->true_direction_tr->data[1] = 0x00;
	true_direction_db->true_direction_tr->data[2] = 0x00;

	if( true_direction_db->true_direction_tr->condition==0x01 || true_direction_db->true_direction_tr->condition==0x02)
		true_direction_db->true_direction_tr->time_enable=true;
	else if( true_direction_db->true_direction_tr->condition>=0x03 && true_direction_db->true_direction_tr->condition<= 0x09)
		true_direction_db->true_direction_tr->value_enable=true;
	else	
		true_direction_db->true_direction_tr->trigger_inactive=true;

	true_direction_db->g_update_timer=(true_direction_db->true_direction_tr->data[0])|(true_direction_db->true_direction_tr->data[1]<<8)| (true_direction_db->true_direction_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for True Wind Speed characteristic and its descriptors like                               ***
***  User Descriptor = "True Wind Speed"                                                       ***

*/

static void es_true_speed_init(void)
{

	true_speed_db = new0(struct ess_true_speed, 1);
	true_speed_db->true_speed_ms = new0(struct ess_measurement, 1);
	true_speed_db->true_speed_tr = new0(struct trigger_setting, 1);
	true_speed_db->user_desc = new0(wchar_t, 21);
	
	true_speed_db->true_speed_data = rand();
	memcpy(true_speed_db->user_desc, "True Wind Speed", 15);
	true_speed_db->lower_true_speed = 10;
	true_speed_db->upper_true_speed = 45;
	true_speed_db->true_speed_ms->flags = 0;
	true_speed_db->true_speed_ms->sample = 0x02;
	true_speed_db->true_speed_ms->applicatn = 0x15;
	true_speed_db->true_speed_ms->m_uncertainity = 0x15;
	true_speed_db->true_speed_ms->m_period[0] = 0x3C;
	true_speed_db->true_speed_ms->m_period[1] = 0x10;
	true_speed_db->true_speed_ms->m_period[2] = 0;
	true_speed_db->true_speed_ms->u_interval[0] = 0x3C;
	true_speed_db->true_speed_ms->u_interval[1] = 0x1F;
	true_speed_db->true_speed_ms->u_interval[2] = 0;
	true_speed_db->indication = 0x0000;
	true_speed_db->true_speed_tr->condition = 0x01;
	true_speed_db->true_speed_tr->data[0] = 0x3C;
	true_speed_db->true_speed_tr->data[1] = 0x00;
	true_speed_db->true_speed_tr->data[2] = 0x00;

	if( true_speed_db->true_speed_tr->condition==0x01 || true_speed_db->true_speed_tr->condition==0x02)
		true_speed_db->true_speed_tr->time_enable=true;
	else if( true_speed_db->true_speed_tr->condition>=0x03 && true_speed_db->true_speed_tr->condition<= 0x09)
		true_speed_db->true_speed_tr->value_enable=true;
	else	
		true_speed_db->true_speed_tr->trigger_inactive=true;

	true_speed_db->g_update_timer=(true_speed_db->true_speed_tr->data[0])|(true_speed_db->true_speed_tr->data[1]<<8)| (true_speed_db->true_speed_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for UV Index characteristic and its descriptors like                                      ***
***  User Descriptor = "UV INdex"                                                              ***

*/

static void es_uv_index_init(void)
{

	uv_index_db = new0(struct ess_uv_index, 1);
	uv_index_db->uv_index_ms = new0(struct ess_measurement, 1);
	uv_index_db->uv_index_tr = new0(struct trigger_setting, 1);
	uv_index_db->user_desc = new0(wchar_t, 21);
	
	uv_index_db->uv_index_data = rand();
	memcpy(uv_index_db->user_desc, "UV Index", 8);
	uv_index_db->lower_uv_index = 10;
	uv_index_db->upper_uv_index = 45;
	uv_index_db->uv_index_ms->flags = 0;
	uv_index_db->uv_index_ms->sample = 0x02;
	uv_index_db->uv_index_ms->applicatn = 0x15;
	uv_index_db->uv_index_ms->m_uncertainity = 0x15;
	uv_index_db->uv_index_ms->m_period[0] = 0x3C;
	uv_index_db->uv_index_ms->m_period[1] = 0x10;
	uv_index_db->uv_index_ms->m_period[2] = 0;
	uv_index_db->uv_index_ms->u_interval[0] = 0x3C;
	uv_index_db->uv_index_ms->u_interval[1] = 0x1F;
	uv_index_db->uv_index_ms->u_interval[2] = 0;
	uv_index_db->indication = 0x0000;
	uv_index_db->uv_index_tr->condition = 0x01;
	uv_index_db->uv_index_tr->data[0] = 0x3C;
	uv_index_db->uv_index_tr->data[1] = 0x00;
	uv_index_db->uv_index_tr->data[2] = 0x00;

	if( uv_index_db->uv_index_tr->condition==0x01 || uv_index_db->uv_index_tr->condition==0x02)
		uv_index_db->uv_index_tr->time_enable=true;
	else if( uv_index_db->uv_index_tr->condition>=0x03 && uv_index_db->uv_index_tr->condition<= 0x09)
		uv_index_db->uv_index_tr->value_enable=true;
	else	
		uv_index_db->uv_index_tr->trigger_inactive=true;

	uv_index_db->g_update_timer=(uv_index_db->uv_index_tr->data[0])|(uv_index_db->uv_index_tr->data[1]<<8)| (uv_index_db->uv_index_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Wind Chill characteristic and its descriptors like                                    ***
***  User Descriptor = "Wind Chill"                                                            ***

*/

static void es_wind_chill_init(void)
{

	wind_chill_db = new0(struct ess_wind_chill, 1);
	wind_chill_db->wind_chill_ms = new0(struct ess_measurement, 1);
	wind_chill_db->wind_chill_tr = new0(struct trigger_setting, 1);
	wind_chill_db->user_desc = new0(wchar_t, 21);
	
	wind_chill_db->wind_chill_data = rand();
	memcpy(wind_chill_db->user_desc, "Wind Chill", 9);
	wind_chill_db->lower_wind_chill = 10;
	wind_chill_db->upper_wind_chill = 45;
	wind_chill_db->wind_chill_ms->flags = 0;
	wind_chill_db->wind_chill_ms->sample = 0x02;
	wind_chill_db->wind_chill_ms->applicatn = 0x15;
	wind_chill_db->wind_chill_ms->m_uncertainity = 0x15;
	wind_chill_db->wind_chill_ms->m_period[0] = 0x3C;
	wind_chill_db->wind_chill_ms->m_period[1] = 0x10;
	wind_chill_db->wind_chill_ms->m_period[2] = 0;
	wind_chill_db->wind_chill_ms->u_interval[0] = 0x3C;
	wind_chill_db->wind_chill_ms->u_interval[1] = 0x1F;
	wind_chill_db->wind_chill_ms->u_interval[2] = 0;
	wind_chill_db->indication = 0x0000;
	wind_chill_db->wind_chill_tr->condition = 0x01;
	wind_chill_db->wind_chill_tr->data[0] = 0x3C;
	wind_chill_db->wind_chill_tr->data[1] = 0x00;
	wind_chill_db->wind_chill_tr->data[2] = 0x00;

	if( wind_chill_db->wind_chill_tr->condition==0x01 || wind_chill_db->wind_chill_tr->condition==0x02)
		wind_chill_db->wind_chill_tr->time_enable=true;
	else if( wind_chill_db->wind_chill_tr->condition>=0x03 && wind_chill_db->wind_chill_tr->condition<= 0x09)
		wind_chill_db->wind_chill_tr->value_enable=true;
	else	
		wind_chill_db->wind_chill_tr->trigger_inactive=true;

	wind_chill_db->g_update_timer=(wind_chill_db->wind_chill_tr->data[0])|(wind_chill_db->wind_chill_tr->data[1]<<8)| (wind_chill_db->wind_chill_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Barometric Pressure Trend characteristic and its descriptors lik                      ***
***  User Descriptor = "Barometric Pressure"                                                   ***

*/

static void es_barometric_pressure_init(void)
{

	barometric_pressure_db = new0(struct ess_barometric_pressure, 1);
	barometric_pressure_db->barometric_pressure_ms = new0(struct ess_measurement, 1);
	barometric_pressure_db->user_desc = new0(wchar_t, 21);
	
	barometric_pressure_db->barometric_pressure_data = rand();
	memcpy(barometric_pressure_db->user_desc, "Barometric pressure", 19);
	barometric_pressure_db->lower_barometric_pressure = 10;
	barometric_pressure_db->upper_barometric_pressure = 45;
	barometric_pressure_db->barometric_pressure_ms->flags = 0;
	barometric_pressure_db->barometric_pressure_ms->sample = 0x02;
	barometric_pressure_db->barometric_pressure_ms->applicatn = 0x15;
	barometric_pressure_db->barometric_pressure_ms->m_uncertainity = 0x15;
	barometric_pressure_db->barometric_pressure_ms->m_period[0] = 0x3C;
	barometric_pressure_db->barometric_pressure_ms->m_period[1] = 0x10;
	barometric_pressure_db->barometric_pressure_ms->m_period[2] = 0;
	barometric_pressure_db->barometric_pressure_ms->u_interval[0] = 0x3C;
	barometric_pressure_db->barometric_pressure_ms->u_interval[1] = 0x1F;
	barometric_pressure_db->barometric_pressure_ms->u_interval[2] = 0;

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Magnetic Declination characteristic and its descriptors like                          ***
***  User Descriptor = "Magnetic Declination"                                                  ***

*/

static void es_magnetic_declination_init(void)
{

	magnetic_declination_db = new0(struct ess_magnetic_declination, 1);
	magnetic_declination_db->magnetic_declination_ms = new0(struct ess_measurement, 1);
	magnetic_declination_db->magnetic_declination_tr = new0(struct trigger_setting, 1);
	magnetic_declination_db->user_desc = new0(wchar_t, 21);
	
	magnetic_declination_db->magnetic_declination_data = rand();
	memcpy(magnetic_declination_db->user_desc, "Magnetic Declination", 20);
	magnetic_declination_db->lower_magnetic_declination = 10;
	magnetic_declination_db->upper_magnetic_declination = 45;
	magnetic_declination_db->magnetic_declination_ms->flags = 0;
	magnetic_declination_db->magnetic_declination_ms->sample = 0x02;
	magnetic_declination_db->magnetic_declination_ms->applicatn = 0x15;
	magnetic_declination_db->magnetic_declination_ms->m_uncertainity = 0x15;
	magnetic_declination_db->magnetic_declination_ms->m_period[0] = 0x3C;
	magnetic_declination_db->magnetic_declination_ms->m_period[1] = 0x10;
	magnetic_declination_db->magnetic_declination_ms->m_period[2] = 0;
	magnetic_declination_db->magnetic_declination_ms->u_interval[0] = 0x3C;
	magnetic_declination_db->magnetic_declination_ms->u_interval[1] = 0x1F;
	magnetic_declination_db->magnetic_declination_ms->u_interval[2] = 0;
	magnetic_declination_db->indication = 0x0000;
	magnetic_declination_db->magnetic_declination_tr->condition = 0x01;
	magnetic_declination_db->magnetic_declination_tr->data[0] = 0x3C;
	magnetic_declination_db->magnetic_declination_tr->data[1] = 0x00;
	magnetic_declination_db->magnetic_declination_tr->data[2] = 0x00;

	if( magnetic_declination_db->magnetic_declination_tr->condition==0x01 || magnetic_declination_db->magnetic_declination_tr->condition==0x02)
		magnetic_declination_db->magnetic_declination_tr->time_enable=true;
	else if( magnetic_declination_db->magnetic_declination_tr->condition>=0x03 && magnetic_declination_db->magnetic_declination_tr->condition<= 0x09)
		magnetic_declination_db->magnetic_declination_tr->value_enable=true;
	else	
		magnetic_declination_db->magnetic_declination_tr->trigger_inactive=true;

	magnetic_declination_db->g_update_timer=(magnetic_declination_db->magnetic_declination_tr->data[0])|(magnetic_declination_db->magnetic_declination_tr->data[1]<<8)| (magnetic_declination_db->magnetic_declination_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Magnetic Flux 2D characteristic and its descriptors like                              ***
***  User Descriptor = "Magnetic Flux 2D"                                                      ***

*/

static void es_magnetic_flux_2D_init(void)
{

	magnetic_flux_2D_db = new0(struct ess_magnetic_flux_2D, 1);
	magnetic_flux_2D_db->magnetic_flux_2D_ms = new0(struct ess_measurement, 1);
	magnetic_flux_2D_db->magnetic_flux_2D_tr = new0(struct trigger_setting, 1);
	magnetic_flux_2D_db->user_desc = new0(wchar_t, 21);
	
	magnetic_flux_2D_db->magnetic_flux_2D_xdata = rand();
	magnetic_flux_2D_db->magnetic_flux_2D_ydata = rand();
	memcpy(magnetic_flux_2D_db->user_desc, "Magnetic Flux 2D", 16);
	magnetic_flux_2D_db->xlower_magnetic_flux_2D = rand();
	magnetic_flux_2D_db->xupper_magnetic_flux_2D = rand();
	magnetic_flux_2D_db->ylower_magnetic_flux_2D = rand();
	magnetic_flux_2D_db->yupper_magnetic_flux_2D = rand();
	magnetic_flux_2D_db->magnetic_flux_2D_ms->flags = 0;
	magnetic_flux_2D_db->magnetic_flux_2D_ms->sample = 0x02;
	magnetic_flux_2D_db->magnetic_flux_2D_ms->applicatn = 0x15;
	magnetic_flux_2D_db->magnetic_flux_2D_ms->m_uncertainity = 0x15;
	magnetic_flux_2D_db->magnetic_flux_2D_ms->m_period[0] = 0x3C;
	magnetic_flux_2D_db->magnetic_flux_2D_ms->m_period[1] = 0x10;
	magnetic_flux_2D_db->magnetic_flux_2D_ms->m_period[2] = 0;
	magnetic_flux_2D_db->magnetic_flux_2D_ms->u_interval[0] = 0x3C;
	magnetic_flux_2D_db->magnetic_flux_2D_ms->u_interval[1] = 0x1F;
	magnetic_flux_2D_db->magnetic_flux_2D_ms->u_interval[2] = 0;
	magnetic_flux_2D_db->indication = 0x0000;
	magnetic_flux_2D_db->magnetic_flux_2D_tr->condition = 0x01;
	magnetic_flux_2D_db->magnetic_flux_2D_tr->data[0] = 0x3C;
	magnetic_flux_2D_db->magnetic_flux_2D_tr->data[1] = 0x00;
	magnetic_flux_2D_db->magnetic_flux_2D_tr->data[2] = 0x00;

	if( magnetic_flux_2D_db->magnetic_flux_2D_tr->condition==0x01 || magnetic_flux_2D_db->magnetic_flux_2D_tr->condition==0x02)
		magnetic_flux_2D_db->magnetic_flux_2D_tr->time_enable=true;
	else if( magnetic_flux_2D_db->magnetic_flux_2D_tr->condition>=0x03 && magnetic_flux_2D_db->magnetic_flux_2D_tr->condition<= 0x09)
		magnetic_flux_2D_db->magnetic_flux_2D_tr->value_enable=true;
	else	
		magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive=true;

	magnetic_flux_2D_db->g_update_timer=(magnetic_flux_2D_db->magnetic_flux_2D_tr->data[0])|(magnetic_flux_2D_db->magnetic_flux_2D_tr->data[1]<<8)| (magnetic_flux_2D_db->magnetic_flux_2D_tr->data[2]<<16);

}

/*

***  This function will allocate the memory and initialize all the initials values requiered   ***
***  for Magentic Flux 3D characteristic and its descriptors like                              ***
***  User Descriptor = "Magnetic Flux 3D"                                                      ***

*/

static void es_magnetic_flux_3D_init(void)
{

	magnetic_flux_3D_db = new0(struct ess_magnetic_flux_3D, 1);
	magnetic_flux_3D_db->magnetic_flux_3D_ms = new0(struct ess_measurement, 1);
	magnetic_flux_3D_db->magnetic_flux_3D_tr = new0(struct trigger_setting, 1);
	magnetic_flux_3D_db->user_desc = new0(wchar_t, 21);
	
	magnetic_flux_3D_db->magnetic_flux_3D_xdata = rand();
	magnetic_flux_3D_db->magnetic_flux_3D_ydata = rand();
	magnetic_flux_3D_db->magnetic_flux_3D_zdata = rand();
	memcpy(magnetic_flux_3D_db->user_desc, "Magnetic Flux 3D", 16);
	magnetic_flux_3D_db->xlower_magnetic_flux_3D = rand();
	magnetic_flux_3D_db->xupper_magnetic_flux_3D = rand();
	magnetic_flux_3D_db->ylower_magnetic_flux_3D = rand();
	magnetic_flux_3D_db->yupper_magnetic_flux_3D = rand();
	magnetic_flux_3D_db->zlower_magnetic_flux_3D = rand();
	magnetic_flux_3D_db->zupper_magnetic_flux_3D = rand();
	magnetic_flux_3D_db->magnetic_flux_3D_ms->flags = 0;
	magnetic_flux_3D_db->magnetic_flux_3D_ms->sample = 0x02;
	magnetic_flux_3D_db->magnetic_flux_3D_ms->applicatn = 0x15;
	magnetic_flux_3D_db->magnetic_flux_3D_ms->m_uncertainity = 0x15;
	magnetic_flux_3D_db->magnetic_flux_3D_ms->m_period[0] = 0x3C;
	magnetic_flux_3D_db->magnetic_flux_3D_ms->m_period[1] = 0x10;
	magnetic_flux_3D_db->magnetic_flux_3D_ms->m_period[2] = 0;
	magnetic_flux_3D_db->magnetic_flux_3D_ms->u_interval[0] = 0x3C;
	magnetic_flux_3D_db->magnetic_flux_3D_ms->u_interval[1] = 0x1F;
	magnetic_flux_3D_db->magnetic_flux_3D_ms->u_interval[2] = 0;
	magnetic_flux_3D_db->indication = 0x0000;
	magnetic_flux_3D_db->magnetic_flux_3D_tr->condition = 0x01;
	magnetic_flux_3D_db->magnetic_flux_3D_tr->data[0] = 0x3C;
	magnetic_flux_3D_db->magnetic_flux_3D_tr->data[1] = 0x00;
	magnetic_flux_3D_db->magnetic_flux_3D_tr->data[2] = 0x00;

	if( magnetic_flux_3D_db->magnetic_flux_3D_tr->condition==0x01 || magnetic_flux_3D_db->magnetic_flux_3D_tr->condition==0x02)
		magnetic_flux_3D_db->magnetic_flux_3D_tr->time_enable=true;
	else if( magnetic_flux_3D_db->magnetic_flux_3D_tr->condition>=0x03 && magnetic_flux_3D_db->magnetic_flux_3D_tr->condition<= 0x09)
		magnetic_flux_3D_db->magnetic_flux_3D_tr->value_enable=true;
	else	
		magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive=true;

	magnetic_flux_3D_db->g_update_timer=(magnetic_flux_3D_db->magnetic_flux_3D_tr->data[0])|(magnetic_flux_3D_db->magnetic_flux_3D_tr->data[1]<<8)| (magnetic_flux_3D_db->magnetic_flux_3D_tr->data[2]<<16);

}

void gatt_set_public_address(uint8_t addr[6])
{
	memcpy(public_addr, addr, sizeof(public_addr));
}

void gatt_set_device_name(uint8_t name[20], uint8_t len)
{
	memcpy(dev_name, name, sizeof(dev_name));
	dev_name_len = len;
}

static void gatt_conn_destroy(void *data)
{
	struct gatt_conn *conn = data;

	bt_gatt_client_unref(conn->client);
	bt_gatt_server_unref(conn->gatt);
	bt_att_unref(conn->att);

	free(conn);
}

static void gatt_conn_disconnect(int err, void *user_data)
{
	struct gatt_conn *conn = user_data;

	printf("Device disconnected: %s\n", strerror(err));

	queue_remove(conn_list, conn);
	gatt_conn_destroy(conn);
}

static void client_ready_callback(bool success, uint8_t att_ecode,
				  void *user_data)
{
	printf("GATT client discovery complete\n");
}

static void client_service_changed_callback(uint16_t start_handle,
					    uint16_t end_handle,
					    void *user_data)
{
	printf("GATT client service changed notification\n");
}

static struct gatt_conn *gatt_conn_new(int fd)
{
	struct gatt_conn *conn;
	uint16_t mtu = 0;

	conn = new0(struct gatt_conn, 1);
	if (!conn)
		return NULL;

	conn->att = bt_att_new(fd, false);
	if (!conn->att) {
		fprintf(stderr, "Failed to initialze ATT transport layer\n");
		free(conn);
		return NULL;
	}

	bt_att_set_close_on_unref(conn->att, true);
	bt_att_register_disconnect(conn->att, gatt_conn_disconnect, conn, NULL);

	bt_att_set_security(conn->att, BT_SECURITY_SDP);

	conn->gatt = bt_gatt_server_new(gatt_db, conn->att, mtu);
	if (!conn->gatt) {
		fprintf(stderr, "Failed to create GATT server\n");
		bt_att_unref(conn->att);
		free(conn);
		return NULL;
	}

	conn->client = bt_gatt_client_new(gatt_cache, conn->att, mtu);
	if (!conn->gatt) {
		fprintf(stderr, "Failed to create GATT client\n");
		bt_gatt_server_unref(conn->gatt);
		bt_att_unref(conn->att);
		free(conn);
		return NULL;
	}

	bt_gatt_client_set_ready_handler(conn->client,
					 client_ready_callback, conn, NULL);
	bt_gatt_client_set_service_changed(conn->client,
					   client_service_changed_callback,
					   conn, NULL);

	return conn;
}

static void att_conn_callback(int fd, uint32_t events, void *user_data)
{
	struct sockaddr_l2 addr;
	socklen_t addrlen;
	int new_fd;

	if (events & (EPOLLERR | EPOLLHUP)) {
		mainloop_remove_fd(fd);
		return;
	}

	memset(&addr, 0, sizeof(addr));
	addrlen = sizeof(addr);

	new_fd = accept(att_fd, (struct sockaddr *)&addr, &addrlen);
	if (new_fd < 0) {
		fprintf(stderr, "Failed to accept new ATT connection: %m\n");
		return;
	}

	conn = gatt_conn_new(new_fd);
	if (!conn) {
		fprintf(stderr, "Failed to create GATT connection\n");
		close(new_fd);
		return;
	}

	if (!queue_push_tail(conn_list, conn)) {
		fprintf(stderr, "Failed to add GATT connection\n");
		gatt_conn_destroy(conn);
		close(new_fd);
	}

	printf("New device connected\n");
}


/*Apparent wind speed client characteristic configuration read call back */

static void apparent_speed_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=apparent_speed_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}

/* This function will be called on regular interval to send the notification */

static bool apparent_speed_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int16_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,apparent_speed_db->apparent_speed_handle,
						(uint8_t *)&pdu, 2);

	return true;
}

/* This function will be called on every second and based on the trigger condition value will be notified */
static bool apparent_speed_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int16_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(apparent_speed_db->apparent_speed_tr->condition){

		case 0x03 : if( pdu == apparent_speed_db->apparent_speed_data) 
				notify=false;
			    else
			    	apparent_speed_db->apparent_speed_data= pdu;
			    break;

		case 0x04 : if( pdu >= apparent_speed_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > apparent_speed_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= apparent_speed_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < apparent_speed_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != apparent_speed_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == apparent_speed_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,apparent_speed_db->apparent_speed_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_apparent_speed_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	apparent_speed_db->g_current_timer=((apparent_speed_db->apparent_speed_tr->data[0]) | (apparent_speed_db->apparent_speed_tr->data[1]<<8) | (apparent_speed_db->apparent_speed_tr->data[2]<<16));


	if(!apparent_speed_db->apparent_speed_enable || apparent_speed_db->apparent_speed_tr->trigger_inactive){
	timeout_remove(apparent_speed_db->timeout_id);
	return;
	}
	
	timer=apparent_speed_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(apparent_speed_db->apparent_speed_tr->time_enable)   
	apparent_speed_db->timeout_id=timeout_add(timer,apparent_speed_time_calculation,server,NULL);
	else
	apparent_speed_db->timeout_id=timeout_add(1000,apparent_speed_value_calculation,server,NULL);
	
}

/* Apparent wind speed client characteristic configuration write call back */

static void apparent_speed_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || apparent_speed_db->apparent_speed_tr->trigger_inactive)
		apparent_speed_db->apparent_speed_enable = false;
	else if (value[0] == 0x01) {
		apparent_speed_db->apparent_speed_enable = true;
		apparent_speed_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_apparent_speed_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

/*Apparent Wind Speed trigger read call back */ 

static void apparent_speed_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=apparent_speed_db->apparent_speed_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = apparent_speed_db->apparent_speed_tr->data[0];
		val[2] = apparent_speed_db->apparent_speed_tr->data[1];
		val[3] = apparent_speed_db->apparent_speed_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else {
		error=0;
		val[1]=(uint8_t)apparent_speed_db->tr_value;
		val[2]=(uint8_t)(apparent_speed_db->tr_value>>8);
		len=3;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}

/*Apparent Wind Speed trigger write call back */
 
static void apparent_speed_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		apparent_speed_db->apparent_speed_tr->trigger_inactive=false;
		apparent_speed_db->apparent_speed_tr->data[0] = value[1];		/* if trigger is time based  */
		apparent_speed_db->apparent_speed_tr->data[1] = value[2];
		apparent_speed_db->apparent_speed_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		apparent_speed_db->apparent_speed_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		apparent_speed_db->apparent_speed_tr->trigger_inactive=true;
		if(apparent_speed_db->timeout_id!=0){
			timeout_remove(apparent_speed_db->timeout_id);
			apparent_speed_db->apparent_speed_enable = false;
			apparent_speed_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		apparent_speed_db->apparent_speed_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {	
		if(len!=3){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}	
		error=0;
		apparent_speed_db->apparent_speed_tr->trigger_inactive=false;		/* if trigger is value based */
		apparent_speed_db->tr_value=value[1] | value[2]<<8 ;
		
	}

	apparent_speed_db->apparent_speed_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( apparent_speed_db->apparent_speed_tr->condition==0x01 || apparent_speed_db->apparent_speed_tr->condition==0x02){
		apparent_speed_db->apparent_speed_tr->value_enable=false;
		apparent_speed_db->apparent_speed_tr->time_enable=true;
	} else if( apparent_speed_db->apparent_speed_tr->condition>=0x03 && apparent_speed_db->apparent_speed_tr->condition<= 0x09){
		apparent_speed_db->apparent_speed_tr->time_enable=false;
		apparent_speed_db->apparent_speed_tr->value_enable=true;
		if(apparent_speed_db->apparent_speed_enable && !apparent_speed_db->apparent_speed_tr->trigger_inactive){
			timeout_remove(apparent_speed_db->timeout_id);
			update_apparent_speed_timer(conn);
		}	
	} else{	

		apparent_speed_db->apparent_speed_tr->trigger_inactive=true;
		apparent_speed_db->apparent_speed_tr->time_enable=false;
		apparent_speed_db->apparent_speed_tr->value_enable=false;
		if(apparent_speed_db->apparent_speed_enable && !apparent_speed_db->apparent_speed_tr->trigger_inactive){
			timeout_remove(apparent_speed_db->timeout_id);
			update_apparent_speed_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(apparent_speed_db->apparent_speed_tr->time_enable)
	apparent_speed_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(apparent_speed_db->g_update_timer!=apparent_speed_db->g_current_timer && apparent_speed_db->apparent_speed_tr->time_enable){
		timeout_remove(apparent_speed_db->timeout_id);
		update_apparent_speed_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*Apparent User Descriptor write call back */ 

static void apparent_speed_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error=0;
	

	/* checking if string is empty or length is more than the pdu length */
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */
	memset(apparent_speed_db->user_desc,0,21);
	memcpy(apparent_speed_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}
	

/* Speed characteristic user descriptor read call back */

static void apparent_speed_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
				       unsigned int id, uint16_t offset,
				       uint8_t opcode, struct bt_att *att,
				       void *user_data)
{

	uint8_t error;
	static char *str;
	if (offset >= 22) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
	} else {
		error = 0;
		str = (char *)apparent_speed_db->user_desc;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *) str, 21);

}

/* speed charcteristic valid range descriptor read call back */

static void apparent_speed_valid_range_read_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error;
	static uint16_t value[2];

	if (offset > 4) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
	} else {
		error = 0;
		value[0] = apparent_speed_db->lower_speed;
		value[1] = apparent_speed_db->upper_speed;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *) value, 4);

}



/* speed characteristic measurement descriptor read call back */

static void apparent_speed_measurement_read_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error;
	uint8_t value[11];

	error = 0;
	value[0] = 0;
	value[1] = 0;
	value[2] = apparent_speed_db->apparent_speed_ms->sample;
	value[3] = apparent_speed_db->apparent_speed_ms->m_period[0];
	value[4] = apparent_speed_db->apparent_speed_ms->m_period[1];
	value[5] = apparent_speed_db->apparent_speed_ms->m_period[2];
	value[6] = apparent_speed_db->apparent_speed_ms->u_interval[0];
	value[7] = apparent_speed_db->apparent_speed_ms->u_interval[1];
	value[8] = apparent_speed_db->apparent_speed_ms->u_interval[2];
	value[9] = apparent_speed_db->apparent_speed_ms->applicatn;
	value[10] = apparent_speed_db->apparent_speed_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}

/* speed characteristic read call back */

static void apparent_speed_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error;
	uint16_t value;

	if (offset > 3) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
	} else {
		error = 0;
		value = apparent_speed_db->apparent_speed_data;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *) & value,
				      2);
}


/* Tempertaure client characteristic configuration read call back */

static void temp_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=temp_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}

/* This function will notify the temperature data to client if trigger is based on time */

static bool temp_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int16_t pdu;

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,temp_db->temp_handle,
						(uint8_t *)&pdu, 2);

	return true;
}


/* This function will notify the temperature data to client if trigger is tempertaure value and this function will be called on every one sec */

static bool temp_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int16_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(temp_db->temp_tr->condition){

		case 0x03 : if( pdu == temp_db->temp_data)
				notify=false;
			    else
			    	temp_db->temp_data= pdu;
			    break;

		case 0x04 : if( pdu >= temp_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > temp_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= temp_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < temp_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != temp_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == temp_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,temp_db->temp_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_temp_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	temp_db->g_current_timer=((temp_db->temp_tr->data[0]) | (temp_db->temp_tr->data[1]<<8) | (temp_db->temp_tr->data[2]<<16));


	if(!temp_db->temp_enable || temp_db->temp_tr->trigger_inactive){
	timeout_remove(temp_db->timeout_id);
	return;
	}
	
	timer=temp_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(temp_db->temp_tr->time_enable)
	temp_db->timeout_id=timeout_add(timer,temp_time_calculation,server,NULL);
	else
	temp_db->timeout_id=timeout_add(1000,temp_value_calculation,server,NULL);

}

/* client characteristic configuration descriptor write call back */

static void temp_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || temp_db->temp_tr->trigger_inactive)
		temp_db->temp_enable = false;
	else if (value[0] == 0x01) {
		temp_db->temp_enable = true;
		temp_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_temp_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

/* Temperature characteristic user descriptor write call back */


static void temp_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error=0;
	
	
	/* checking if string is empty or length is more than the pdu length */
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(temp_db->user_desc,0,21);
	memcpy(temp_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}
	
	

/* Temperature characteristic user descriptor read call back */

static void temp_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
				       unsigned int id, uint16_t offset,
				       uint8_t opcode, struct bt_att *att,
				       void *user_data)
{

	uint8_t error;
	if (offset >= 22) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	} else {
		error = 0;
	}

done:

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)temp_db->user_desc, 21);

}

/* Temperature valid range descriptor read call back */

static void temp_valid_range_read_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error;
	static uint16_t value[2];

	if (offset > 4) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
	} else {
		error = 0;
		value[0] = temp_db->lower_temp;
		value[1] = temp_db->upper_temp;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *) value, 4);

}

/* Temperature trigger descriptor read call back */

static void temp_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=temp_db->temp_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = temp_db->temp_tr->data[0];
		val[2] = temp_db->temp_tr->data[1];
		val[3] = temp_db->temp_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else {
		error=0;
		val[1]=(uint8_t)temp_db->tr_value;
		val[2]=(uint8_t)(temp_db->tr_value>>8);
		len=3;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}

/* Temperature trigger descriptor write call back */

static void temp_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		temp_db->temp_tr->trigger_inactive=false;
		temp_db->temp_tr->data[0] = value[1];		/* if trigger is time based  */
		temp_db->temp_tr->data[1] = value[2];
		temp_db->temp_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		temp_db->temp_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		temp_db->temp_tr->trigger_inactive=true;
		if(temp_db->timeout_id!=0){
			timeout_remove(temp_db->timeout_id);
			temp_db->temp_enable = false;
			temp_db->indication = 0x0000;
			goto done;
		}
	}else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		temp_db->temp_tr->trigger_inactive=false;		/* if trigger is value based */
	}else {
		if(len!=3){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		temp_db->temp_tr->trigger_inactive=false;		/* if trigger is value based */
		temp_db->tr_value=value[1] | value[2]<<8 ;
		
	}

	temp_db->temp_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( temp_db->temp_tr->condition==0x01 || temp_db->temp_tr->condition==0x02){
		temp_db->temp_tr->value_enable=false;
		temp_db->temp_tr->time_enable=true;
	} else if( temp_db->temp_tr->condition>=0x03 && temp_db->temp_tr->condition<= 0x09){
		temp_db->temp_tr->time_enable=false;
		temp_db->temp_tr->value_enable=true;
		if(temp_db->temp_enable && !temp_db->temp_tr->trigger_inactive){
			timeout_remove(temp_db->timeout_id);
			update_temp_timer(conn);
		}	
	} else{	

		temp_db->temp_tr->trigger_inactive=true;
		temp_db->temp_tr->time_enable=false;
		temp_db->temp_tr->value_enable=false;
		if(temp_db->temp_enable && !temp_db->temp_tr->trigger_inactive){
			timeout_remove(temp_db->timeout_id);
			update_temp_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(temp_db->temp_tr->time_enable)
	temp_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(temp_db->g_update_timer!=temp_db->g_current_timer && temp_db->temp_tr->time_enable){
		timeout_remove(temp_db->timeout_id);
		update_temp_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/* Temperature measurement descriptor read Call back */ 

static void temp_measurement_read_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error;
	uint8_t value[11];

	error = 0;
	value[0] = 0;
	value[1] = 0;
	value[2] = temp_db->temp_ms->sample;
	value[3] = temp_db->temp_ms->m_period[0];
	value[4] = temp_db->temp_ms->m_period[1];
	value[5] = temp_db->temp_ms->m_period[2];
	value[6] = temp_db->temp_ms->u_interval[0];
	value[7] = temp_db->temp_ms->u_interval[1];
	value[8] = temp_db->temp_ms->u_interval[2];
	value[9] = temp_db->temp_ms->applicatn;
	value[10] = temp_db->temp_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}

/* Temperature characteristic read call back */

static void temp_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if (offset > 3) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
	} else {
		error = 0;
		value = temp_db->temp_data;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *) & value,
				      2);
}


/*
*** This section cocnsist of all the call backs ***
*** of apparent wind speed characteristics      ***
*/



/* apparent wind speed value read call back */

static void apparent_direction_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value;

	value=apparent_direction_db->apparent_direction_data;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);
}

/* Apparent wind Direction measurement descriptor read call back */

static void apparent_direction_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = apparent_direction_db->apparent_direction_ms->sample;
	value[3] = apparent_direction_db->apparent_direction_ms->m_period[0];
	value[4] = apparent_direction_db->apparent_direction_ms->m_period[1];
	value[5] = apparent_direction_db->apparent_direction_ms->m_period[2];
	value[6] = apparent_direction_db->apparent_direction_ms->u_interval[0];
	value[7] = apparent_direction_db->apparent_direction_ms->u_interval[1];
	value[8] = apparent_direction_db->apparent_direction_ms->u_interval[2];
	value[9] = apparent_direction_db->apparent_direction_ms->applicatn;
	value[10] = apparent_direction_db->apparent_direction_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}	

/* Apparent wind Direction valid range read call back */

static void apparent_direction_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value[2];
	
	value[0]=apparent_direction_db->lower_direction;
	value[1]=apparent_direction_db->upper_direction;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 4);

}

/* Apparent Wind Direction user descriptor read call back */

static void apparent_direction_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=apparent_direction_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}

/* Apparent Wind Direction user descriptor write call back */

static void apparent_direction_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(apparent_direction_db->user_desc,0,21);
	memcpy(apparent_direction_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void apparent_direction_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=apparent_direction_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool apparent_direction_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,apparent_direction_db->apparent_direction_handle,
						(uint8_t *)&pdu, 2);

	return true;
}


static bool apparent_direction_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(apparent_direction_db->apparent_direction_tr->condition){

		case 0x03 : if( pdu == apparent_direction_db->apparent_direction_data)
				notify=false;
			    else
			    	apparent_direction_db->apparent_direction_data= pdu;
			    break;

		case 0x04 : if( pdu >= apparent_direction_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > apparent_direction_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= apparent_direction_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < apparent_direction_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != apparent_direction_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == apparent_direction_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,apparent_direction_db->apparent_direction_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_apparent_direction_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	apparent_direction_db->g_current_timer=((apparent_direction_db->apparent_direction_tr->data[0]) | (apparent_direction_db->apparent_direction_tr->data[1]<<8) | (apparent_direction_db->apparent_direction_tr->data[2]<<16));


	if(!apparent_direction_db->apparent_direction_enable || apparent_direction_db->apparent_direction_tr->trigger_inactive){
	timeout_remove(apparent_direction_db->timeout_id);
	return;
	}
	
	timer=apparent_direction_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(apparent_direction_db->apparent_direction_tr->time_enable)   
	apparent_direction_db->timeout_id=timeout_add(timer,apparent_direction_time_calculation,server,NULL);
	else
	apparent_direction_db->timeout_id=timeout_add(1000,apparent_direction_value_calculation,server,NULL);
	
}


static void apparent_direction_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || apparent_direction_db->apparent_direction_tr->trigger_inactive)
		apparent_direction_db->apparent_direction_enable = false;
	else if (value[0] == 0x01) {
		apparent_direction_db->apparent_direction_enable = true;
		apparent_direction_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_apparent_direction_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void apparent_direction_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=apparent_direction_db->apparent_direction_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = apparent_direction_db->apparent_direction_tr->data[0];
		val[2] = apparent_direction_db->apparent_direction_tr->data[1];
		val[3] = apparent_direction_db->apparent_direction_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else {
		error=0;
		val[1]=(uint8_t)apparent_direction_db->tr_value;
		val[2]=(uint8_t)(apparent_direction_db->tr_value>>8);
		len=3;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void apparent_direction_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		apparent_direction_db->apparent_direction_tr->trigger_inactive=false;
		apparent_direction_db->apparent_direction_tr->data[0] = value[1];		/* if trigger is time based  */
		apparent_direction_db->apparent_direction_tr->data[1] = value[2];
		apparent_direction_db->apparent_direction_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		apparent_direction_db->apparent_direction_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		apparent_direction_db->apparent_direction_tr->trigger_inactive=true;
		if(apparent_direction_db->timeout_id!=0){
			timeout_remove(apparent_direction_db->timeout_id);
			apparent_direction_db->apparent_direction_enable = false;
			apparent_direction_db->indication = 0x0000;
			goto done;
		}
	}else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		apparent_direction_db->apparent_direction_tr->trigger_inactive=false;		/* if trigger is value based */
	}else {
		if(len!=3){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		apparent_direction_db->apparent_direction_tr->trigger_inactive=false;		/* if trigger is value based */
		apparent_direction_db->tr_value=value[1] | value[2]<<8 ;
		
	}

	apparent_direction_db->apparent_direction_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( apparent_direction_db->apparent_direction_tr->condition==0x01 || apparent_direction_db->apparent_direction_tr->condition==0x02){
		apparent_direction_db->apparent_direction_tr->value_enable=false;
		apparent_direction_db->apparent_direction_tr->time_enable=true;
	} else if( apparent_direction_db->apparent_direction_tr->condition>=0x03 && apparent_direction_db->apparent_direction_tr->condition<= 0x09){
		apparent_direction_db->apparent_direction_tr->time_enable=false;
		apparent_direction_db->apparent_direction_tr->value_enable=true;
		if(apparent_direction_db->apparent_direction_enable && !apparent_direction_db->apparent_direction_tr->trigger_inactive){
			timeout_remove(apparent_direction_db->timeout_id);
			update_apparent_direction_timer(conn);
		}	
	} else{	

		apparent_direction_db->apparent_direction_tr->trigger_inactive=true;
		apparent_direction_db->apparent_direction_tr->time_enable=false;
		apparent_direction_db->apparent_direction_tr->value_enable=false;
		if(apparent_direction_db->apparent_direction_enable && !apparent_direction_db->apparent_direction_tr->trigger_inactive){
			timeout_remove(apparent_direction_db->timeout_id);
			update_apparent_direction_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(apparent_direction_db->apparent_direction_tr->time_enable)
	apparent_direction_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(apparent_direction_db->g_update_timer!=apparent_direction_db->g_current_timer && apparent_direction_db->apparent_direction_tr->time_enable){
		timeout_remove(apparent_direction_db->timeout_id);
		update_apparent_direction_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*
*** This section cocnsist of all the call backs of ***
*** Dew Point characteristics                      ***
*/


static void dew_point_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int8_t  value;

	value=dew_point_db->dew_point_data;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 1);

}


/* Dew Point measurement descriptor read call back */

static void dew_point_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = dew_point_db->dew_point_ms->sample;
	value[3] = dew_point_db->dew_point_ms->m_period[0];
	value[4] = dew_point_db->dew_point_ms->m_period[1];
	value[5] = dew_point_db->dew_point_ms->m_period[2];
	value[6] = dew_point_db->dew_point_ms->u_interval[0];
	value[7] = dew_point_db->dew_point_ms->u_interval[1];
	value[8] = dew_point_db->dew_point_ms->u_interval[2];
	value[9] = dew_point_db->dew_point_ms->applicatn;
	value[10] = dew_point_db->dew_point_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}	

/* Dew Point valid range read call back */

static void dew_point_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int8_t value[2];

	value[0]=-40;
	value[1]=+80;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 2);

}

/* Dew Point user descriptor read call back */

static void dew_point_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=dew_point_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}

/* Dew Point user descriptor write call back */

static void dew_point_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(dew_point_db->user_desc,0,21);
	memcpy(dew_point_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

static void dew_point_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=dew_point_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool dew_point_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int8_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,dew_point_db->dew_point_handle,
						(uint8_t *)&pdu, 1);

	return true;
}


static bool dew_point_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int8_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(dew_point_db->dew_point_tr->condition){

		case 0x03 : if( pdu == dew_point_db->dew_point_data)
				notify=false;
			    else
			    	dew_point_db->dew_point_data= pdu;
			    break;

		case 0x04 : if( pdu >= dew_point_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > dew_point_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= dew_point_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < dew_point_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != dew_point_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == dew_point_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,dew_point_db->dew_point_handle,
						(uint8_t *)&pdu, 1);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_dew_point_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	dew_point_db->g_current_timer=((dew_point_db->dew_point_tr->data[0]) | (dew_point_db->dew_point_tr->data[1]<<8) | (dew_point_db->dew_point_tr->data[2]<<16));


	if(!dew_point_db->dew_point_enable || dew_point_db->dew_point_tr->trigger_inactive){
	timeout_remove(dew_point_db->timeout_id);
	return;
	}
	
	timer=dew_point_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(dew_point_db->dew_point_tr->time_enable)   
	dew_point_db->timeout_id=timeout_add(timer,dew_point_time_calculation,server,NULL);
	else
	dew_point_db->timeout_id=timeout_add(1000,dew_point_value_calculation,server,NULL);
	
}


static void dew_point_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || dew_point_db->dew_point_tr->trigger_inactive)
		dew_point_db->dew_point_enable = false;
	else if (value[0] == 0x01) {
		dew_point_db->dew_point_enable = true;
		dew_point_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_dew_point_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void dew_point_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=dew_point_db->dew_point_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = dew_point_db->dew_point_tr->data[0];
		val[2] = dew_point_db->dew_point_tr->data[1];
		val[3] = dew_point_db->dew_point_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	} else{
		error=0;
		val[1]=(uint8_t)dew_point_db->tr_value;
		len=2;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void dew_point_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		dew_point_db->dew_point_tr->trigger_inactive=false;
		dew_point_db->dew_point_tr->data[0] = value[1];		/* if trigger is time based  */
		dew_point_db->dew_point_tr->data[1] = value[2];
		dew_point_db->dew_point_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		dew_point_db->dew_point_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		dew_point_db->dew_point_tr->trigger_inactive=true;
		if(dew_point_db->timeout_id!=0){
			timeout_remove(dew_point_db->timeout_id);
			dew_point_db->dew_point_enable = false;
			dew_point_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		dew_point_db->dew_point_tr->trigger_inactive=false;		/* if trigger is value based */
	} else{
		if(len!=2){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		dew_point_db->dew_point_tr->trigger_inactive=false;		/* if trigger is value based */
		dew_point_db->tr_value=value[1];
		
	}

	dew_point_db->dew_point_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( dew_point_db->dew_point_tr->condition==0x01 || dew_point_db->dew_point_tr->condition==0x02){
		dew_point_db->dew_point_tr->value_enable=false;
		dew_point_db->dew_point_tr->time_enable=true;
	} else if( dew_point_db->dew_point_tr->condition>=0x03 && dew_point_db->dew_point_tr->condition<= 0x09){
		dew_point_db->dew_point_tr->time_enable=false;
		dew_point_db->dew_point_tr->value_enable=true;
		if(dew_point_db->dew_point_enable && !dew_point_db->dew_point_tr->trigger_inactive){
			timeout_remove(dew_point_db->timeout_id);
			update_dew_point_timer(conn);
		}	
	} else{	

		dew_point_db->dew_point_tr->trigger_inactive=true;
		dew_point_db->dew_point_tr->time_enable=false;
		dew_point_db->dew_point_tr->value_enable=false;
		if(dew_point_db->dew_point_enable && !dew_point_db->dew_point_tr->trigger_inactive){
			timeout_remove(dew_point_db->timeout_id);
			update_dew_point_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(dew_point_db->dew_point_tr->time_enable)
	dew_point_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(dew_point_db->g_update_timer!=dew_point_db->g_current_timer && dew_point_db->dew_point_tr->time_enable){
		timeout_remove(dew_point_db->timeout_id);
		update_dew_point_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*
*** This section cocnsist of all the call backs of ***
*** Elevation characteristics                      ***
*/

static void elevation_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int32_t value;

	value=elevation_db->elevation_data;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 3);

}


static void elevation_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = elevation_db->elevation_ms->sample;
	value[3] = elevation_db->elevation_ms->m_period[0];
	value[4] = elevation_db->elevation_ms->m_period[1];
	value[5] = elevation_db->elevation_ms->m_period[2];
	value[6] = elevation_db->elevation_ms->u_interval[0];
	value[7] = elevation_db->elevation_ms->u_interval[1];
	value[8] = elevation_db->elevation_ms->u_interval[2];
	value[9] = elevation_db->elevation_ms->applicatn;
	value[10] = elevation_db->elevation_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}	


static void elevation_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int8_t value[2][3];

	value[0][0]=elevation_db->lower_elevation[0];
	value[0][1]=elevation_db->upper_elevation[1];
	value[0][2]=elevation_db->upper_elevation[2];
	value[1][0]=elevation_db->lower_elevation[0];
	value[1][1]=elevation_db->upper_elevation[1];
	value[1][2]=elevation_db->upper_elevation[2];
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 6);

}


static void elevation_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=elevation_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void elevation_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(elevation_db->user_desc,0,21);
	memcpy(elevation_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

static void elevation_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=elevation_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool elevation_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int32_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,elevation_db->elevation_handle,
						(uint8_t *)&pdu, 3);

	return true;
}


static bool elevation_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int32_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();


	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(elevation_db->elevation_tr->condition){

		case 0x03 : if( pdu == elevation_db->elevation_data)
				notify=false;
			    else
			    	elevation_db->elevation_data= pdu;
			    break;

		case 0x04 : if( pdu >= elevation_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > elevation_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= elevation_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < elevation_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != elevation_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == elevation_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition 	*/		
	if(notify){
	bt_gatt_server_send_notification(server->gatt,elevation_db->elevation_handle,
						(uint8_t *)&pdu, 3);
	}

	return true;
}

 /*Timer function will read the trigger setting and call a function to send notification based on value or time */

static void update_elevation_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	elevation_db->g_current_timer=((elevation_db->elevation_tr->data[0]) | (elevation_db->elevation_tr->data[1]<<8) | (elevation_db->elevation_tr->data[2]<<16));


	if(!elevation_db->elevation_enable || elevation_db->elevation_tr->trigger_inactive){
	timeout_remove(elevation_db->timeout_id);
	return;
	}
	
	timer=elevation_db->g_current_timer*1000;
	

	/*
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(elevation_db->elevation_tr->time_enable)   
	elevation_db->timeout_id=timeout_add(timer,elevation_time_calculation,server,NULL);
	else
	elevation_db->timeout_id=timeout_add(1000,elevation_value_calculation,server,NULL);
	
}


static void elevation_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/*enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || elevation_db->elevation_tr->trigger_inactive)
		elevation_db->elevation_enable = false;
	else if (value[0] == 0x01) {
		elevation_db->elevation_enable = true;
		elevation_db->indication = 0x0001;
	} else
		error = 0x80;

	/*updating a timer function to call notification on a periodic interval */

	update_elevation_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void elevation_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=elevation_db->elevation_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = elevation_db->elevation_tr->data[0];
		val[2] = elevation_db->elevation_tr->data[1];
		val[3] = elevation_db->elevation_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	} else {
		error=0;
		val[1]=(uint8_t)elevation_db->tr_value;
		val[2]=(uint8_t)(elevation_db->tr_value>>8);
		val[3]=(uint8_t)(elevation_db->tr_value>>16);
		len=4;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void elevation_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	 /*checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		elevation_db->elevation_tr->trigger_inactive=false;
		elevation_db->elevation_tr->data[0] = value[1];		 /*if trigger is time based  */
		elevation_db->elevation_tr->data[1] = value[2];
		elevation_db->elevation_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		elevation_db->elevation_tr->condition = value[0] ;	 /*if trigger is inactive check notification is enable if yes disable the 												notification */
		elevation_db->elevation_tr->trigger_inactive=true;
		if(elevation_db->timeout_id!=0){
			timeout_remove(elevation_db->timeout_id);
			elevation_db->elevation_enable = false;
			elevation_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03) {
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		elevation_db->elevation_tr->trigger_inactive=false;		 /*if trigger is value based */
	} else {
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		elevation_db->elevation_tr->trigger_inactive=false;		 /*if trigger is value based */
		elevation_db->tr_value=value[1] | value[2]<<8 | value[3]<<16 | 0<<24;
		
	}

	elevation_db->elevation_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( elevation_db->elevation_tr->condition==0x01 || elevation_db->elevation_tr->condition==0x02){
		elevation_db->elevation_tr->value_enable=false;
		elevation_db->elevation_tr->time_enable=true;
	} else if( elevation_db->elevation_tr->condition>=0x03 && elevation_db->elevation_tr->condition<= 0x09){
		elevation_db->elevation_tr->time_enable=false;
		elevation_db->elevation_tr->value_enable=true;
		if(elevation_db->elevation_enable && !elevation_db->elevation_tr->trigger_inactive){
			timeout_remove(elevation_db->timeout_id);
			update_elevation_timer(conn);
		}	
	} else{	

		elevation_db->elevation_tr->trigger_inactive=true;
		elevation_db->elevation_tr->time_enable=false;
		elevation_db->elevation_tr->value_enable=false;
		if(elevation_db->elevation_enable && !elevation_db->elevation_tr->trigger_inactive){
			timeout_remove(elevation_db->timeout_id);
			update_elevation_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(elevation_db->elevation_tr->time_enable)
	elevation_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(elevation_db->g_update_timer!=elevation_db->g_current_timer && elevation_db->elevation_tr->time_enable){
		timeout_remove(elevation_db->timeout_id);
		update_elevation_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}



/*
*** This section cocnsist of all the call backs of ***
*** Gust Factor characteristics                    ***
*/


static void gust_factor_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value;

	value=heat_index_db->heat_index_data;	

	gatt_db_attribute_read_result(attrib, id, error, &value, 1);

}


static void gust_factor_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = gust_factor_db->gust_factor_ms->sample;
	value[3] = gust_factor_db->gust_factor_ms->m_period[0];
	value[4] = gust_factor_db->gust_factor_ms->m_period[1];
	value[5] = gust_factor_db->gust_factor_ms->m_period[2];
	value[6] = gust_factor_db->gust_factor_ms->u_interval[0];
	value[7] = gust_factor_db->gust_factor_ms->u_interval[1];
	value[8] = gust_factor_db->gust_factor_ms->u_interval[2];
	value[9] = gust_factor_db->gust_factor_ms->applicatn;
	value[10] = gust_factor_db->gust_factor_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void gust_factor_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint8_t value[2];

	value[0]=gust_factor_db->lower_gust_factor;
	value[1]=gust_factor_db->upper_gust_factor;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 2);

}
	


static void gust_factor_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=gust_factor_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void gust_factor_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(gust_factor_db->user_desc,0,21);
	memcpy(gust_factor_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

static void gust_factor_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=gust_factor_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool gust_factor_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint8_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,gust_factor_db->gust_factor_handle,
						(uint8_t *)&pdu, 1);

	return true;
}


static bool gust_factor_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint8_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(gust_factor_db->gust_factor_tr->condition){

		case 0x03 : if( pdu == gust_factor_db->gust_factor_data)
				notify=false;
			    else
			    	gust_factor_db->gust_factor_data= pdu;
			    break;

		case 0x04 : if( pdu >= gust_factor_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > gust_factor_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= gust_factor_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < gust_factor_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != gust_factor_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == gust_factor_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,gust_factor_db->gust_factor_handle,
						(uint8_t *)&pdu, 1);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_gust_factor_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	gust_factor_db->g_current_timer=((gust_factor_db->gust_factor_tr->data[0]) | (gust_factor_db->gust_factor_tr->data[1]<<8) | (gust_factor_db->gust_factor_tr->data[2]<<16));


	if(!gust_factor_db->gust_factor_enable || gust_factor_db->gust_factor_tr->trigger_inactive){
	timeout_remove(gust_factor_db->timeout_id);
	return;
	}
	
	timer=gust_factor_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(gust_factor_db->gust_factor_tr->time_enable)   
	gust_factor_db->timeout_id=timeout_add(timer,gust_factor_time_calculation,server,NULL);
	else
	gust_factor_db->timeout_id=timeout_add(1000,gust_factor_value_calculation,server,NULL);
	
}


static void gust_factor_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || gust_factor_db->gust_factor_tr->trigger_inactive)
		gust_factor_db->gust_factor_enable = false;
	else if (value[0] == 0x01) {
		gust_factor_db->gust_factor_enable = true;
		gust_factor_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_gust_factor_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void gust_factor_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=gust_factor_db->gust_factor_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = gust_factor_db->gust_factor_tr->data[0];
		val[2] = gust_factor_db->gust_factor_tr->data[1];
		val[3] = gust_factor_db->gust_factor_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	} else{
		error=0;
		val[1]=(uint8_t)gust_factor_db->tr_value;
		len=2;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void gust_factor_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		gust_factor_db->gust_factor_tr->trigger_inactive=false;
		gust_factor_db->gust_factor_tr->data[0] = value[1];		/* if trigger is time based  */
		gust_factor_db->gust_factor_tr->data[1] = value[2];
		gust_factor_db->gust_factor_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		gust_factor_db->gust_factor_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		gust_factor_db->gust_factor_tr->trigger_inactive=true;
		if(gust_factor_db->timeout_id!=0){
			timeout_remove(gust_factor_db->timeout_id);
			gust_factor_db->gust_factor_enable = false;
			gust_factor_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		gust_factor_db->gust_factor_tr->trigger_inactive=false;		/* if trigger is value based */
	} else{
		if(len!=2){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		gust_factor_db->gust_factor_tr->trigger_inactive=false;		/* if trigger is value based */
		gust_factor_db->tr_value=value[1];
		
	}

	gust_factor_db->gust_factor_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( gust_factor_db->gust_factor_tr->condition==0x01 || gust_factor_db->gust_factor_tr->condition==0x02){
		gust_factor_db->gust_factor_tr->value_enable=false;
		gust_factor_db->gust_factor_tr->time_enable=true;
	} else if( gust_factor_db->gust_factor_tr->condition>=0x03 && gust_factor_db->gust_factor_tr->condition<= 0x09){
		gust_factor_db->gust_factor_tr->time_enable=false;
		gust_factor_db->gust_factor_tr->value_enable=true;
		if(gust_factor_db->gust_factor_enable && !gust_factor_db->gust_factor_tr->trigger_inactive){
			timeout_remove(gust_factor_db->timeout_id);
			update_gust_factor_timer(conn);
		}	
	} else{	

		gust_factor_db->gust_factor_tr->trigger_inactive=true;
		gust_factor_db->gust_factor_tr->time_enable=false;
		gust_factor_db->gust_factor_tr->value_enable=false;
		if(gust_factor_db->gust_factor_enable && !gust_factor_db->gust_factor_tr->trigger_inactive){
			timeout_remove(gust_factor_db->timeout_id);
			update_gust_factor_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(gust_factor_db->gust_factor_tr->time_enable)
	gust_factor_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(gust_factor_db->g_update_timer!=gust_factor_db->g_current_timer && gust_factor_db->gust_factor_tr->time_enable){
		timeout_remove(gust_factor_db->timeout_id);
		update_gust_factor_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


/*
*** This section cocnsist of all the call backs of ***
*** Heat Index characteristics                     ***
*/


static void heat_index_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int8_t value;

	value=heat_index_db->heat_index_data;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 1);


}

/* Heat Index measurement descriptor read call back */

static void heat_index_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = heat_index_db->heat_index_ms->sample;
	value[3] = heat_index_db->heat_index_ms->m_period[0];
	value[4] = heat_index_db->heat_index_ms->m_period[1];
	value[5] = heat_index_db->heat_index_ms->m_period[2];
	value[6] = heat_index_db->heat_index_ms->u_interval[0];
	value[7] = heat_index_db->heat_index_ms->u_interval[1];
	value[8] = heat_index_db->heat_index_ms->u_interval[2];
	value[9] = heat_index_db->heat_index_ms->applicatn;
	value[10] = heat_index_db->heat_index_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void heat_index_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int8_t value[2];

	value[0]=heat_index_db->lower_heat_index;
	value[1]=heat_index_db->upper_heat_index;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 2);

}
	

/* Heat Index user descriptor read call back */

static void heat_index_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=heat_index_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}

/* Heat Index user descriptor write call back */

static void heat_index_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(heat_index_db->user_desc,0,21);
	memcpy(heat_index_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

static void heat_index_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=heat_index_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool heat_index_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int8_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,heat_index_db->heat_index_handle,
						(uint8_t *)&pdu, 1);

	return true;
}


static bool heat_index_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int8_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(heat_index_db->heat_index_tr->condition){

		case 0x03 : if( pdu == heat_index_db->heat_index_data)
				notify=false;
			    else
			    	heat_index_db->heat_index_data= pdu;
			    break;

		case 0x04 : if( pdu >= heat_index_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > heat_index_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= heat_index_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < heat_index_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != heat_index_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == heat_index_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,heat_index_db->heat_index_handle,
						(uint8_t *)&pdu, 1);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_heat_index_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	heat_index_db->g_current_timer=((heat_index_db->heat_index_tr->data[0]) | (heat_index_db->heat_index_tr->data[1]<<8) | (heat_index_db->heat_index_tr->data[2]<<16));


	if(!heat_index_db->heat_index_enable || heat_index_db->heat_index_tr->trigger_inactive){
	timeout_remove(heat_index_db->timeout_id);
	return;
	}
	
	timer=heat_index_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(heat_index_db->heat_index_tr->time_enable)   
	heat_index_db->timeout_id=timeout_add(timer,heat_index_time_calculation,server,NULL);
	else
	heat_index_db->timeout_id=timeout_add(1000,heat_index_value_calculation,server,NULL);
	
}


static void heat_index_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || heat_index_db->heat_index_tr->trigger_inactive)
		heat_index_db->heat_index_enable = false;
	else if (value[0] == 0x01) {
		heat_index_db->heat_index_enable = true;
		heat_index_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_heat_index_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void heat_index_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=heat_index_db->heat_index_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = heat_index_db->heat_index_tr->data[0];
		val[2] = heat_index_db->heat_index_tr->data[1];
		val[3] = heat_index_db->heat_index_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	} else{
		error=0;
		val[1]=(uint8_t)heat_index_db->tr_value;
		len=2;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void heat_index_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		heat_index_db->heat_index_tr->trigger_inactive=false;
		heat_index_db->heat_index_tr->data[0] = value[1];		/* if trigger is time based  */
		heat_index_db->heat_index_tr->data[1] = value[2];
		heat_index_db->heat_index_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		heat_index_db->heat_index_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		heat_index_db->heat_index_tr->trigger_inactive=true;
		if(heat_index_db->timeout_id!=0){
			timeout_remove(heat_index_db->timeout_id);
			heat_index_db->heat_index_enable = false;
			heat_index_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		heat_index_db->heat_index_tr->trigger_inactive=false;		/* if trigger is value based */
	} else{
		if(len!=2){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		heat_index_db->heat_index_tr->trigger_inactive=false;		/* if trigger is value based */
		heat_index_db->tr_value=value[1];
		
	}

	heat_index_db->heat_index_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( heat_index_db->heat_index_tr->condition==0x01 || heat_index_db->heat_index_tr->condition==0x02){
		heat_index_db->heat_index_tr->value_enable=false;
		heat_index_db->heat_index_tr->time_enable=true;
	} else if( heat_index_db->heat_index_tr->condition>=0x03 && heat_index_db->heat_index_tr->condition<= 0x09){
		heat_index_db->heat_index_tr->time_enable=false;
		heat_index_db->heat_index_tr->value_enable=true;
		if(heat_index_db->heat_index_enable && !heat_index_db->heat_index_tr->trigger_inactive){
			timeout_remove(heat_index_db->timeout_id);
			update_heat_index_timer(conn);
		}	
	} else{	

		heat_index_db->heat_index_tr->trigger_inactive=true;
		heat_index_db->heat_index_tr->time_enable=false;
		heat_index_db->heat_index_tr->value_enable=false;
		if(heat_index_db->heat_index_enable && !heat_index_db->heat_index_tr->trigger_inactive){
			timeout_remove(heat_index_db->timeout_id);
			update_heat_index_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(heat_index_db->heat_index_tr->time_enable)
	heat_index_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(heat_index_db->g_update_timer!=heat_index_db->g_current_timer && heat_index_db->heat_index_tr->time_enable){
		timeout_remove(heat_index_db->timeout_id);
		update_heat_index_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*
*** This section cocnsist of all the call backs of ***
*** Humidity characteristics                       ***
*/


static void humidity_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value;

	value=humidity_db->humidity_data;	

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static void humidity_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = humidity_db->humidity_ms->sample;
	value[3] = humidity_db->humidity_ms->m_period[0];
	value[4] = humidity_db->humidity_ms->m_period[1];
	value[5] = humidity_db->humidity_ms->m_period[2];
	value[6] = humidity_db->humidity_ms->u_interval[0];
	value[7] = humidity_db->humidity_ms->u_interval[1];
	value[8] = humidity_db->humidity_ms->u_interval[2];
	value[9] = humidity_db->humidity_ms->applicatn;
	value[10] = humidity_db->humidity_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void humidity_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value[2];

	value[0]=humidity_db->lower_humidity;
	value[1]=humidity_db->upper_humidity;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 4);

}
	


static void humidity_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=humidity_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void humidity_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	 /*initializing the string with zero's and then copying the new string to the data base*/ 

	memset(humidity_db->user_desc,0,21);
	memcpy(humidity_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

static void humidity_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=humidity_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool humidity_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,humidity_db->humidity_handle,
						(uint8_t *)&pdu, 2);

	return true;
}


static bool humidity_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(humidity_db->humidity_tr->condition){

		case 0x03 : if( pdu == humidity_db->humidity_data)
				notify=false;
			    else
			    	humidity_db->humidity_data= pdu;
			    break;

		case 0x04 : if( pdu >= humidity_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > humidity_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= humidity_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < humidity_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != humidity_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == humidity_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,humidity_db->humidity_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_humidity_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	humidity_db->g_current_timer=((humidity_db->humidity_tr->data[0]) | (humidity_db->humidity_tr->data[1]<<8) | (humidity_db->humidity_tr->data[2]<<16));


	if(!humidity_db->humidity_enable || humidity_db->humidity_tr->trigger_inactive){
	timeout_remove(humidity_db->timeout_id);
	return;
	}
	
	timer=humidity_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(humidity_db->humidity_tr->time_enable)   
	humidity_db->timeout_id=timeout_add(timer,humidity_time_calculation,server,NULL);
	else
	humidity_db->timeout_id=timeout_add(1000,humidity_value_calculation,server,NULL);
	
}


static void humidity_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || humidity_db->humidity_tr->trigger_inactive)
		humidity_db->humidity_enable = false;
	else if (value[0] == 0x01) {
		humidity_db->humidity_enable = true;
		humidity_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_humidity_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void humidity_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=humidity_db->humidity_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = humidity_db->humidity_tr->data[0];
		val[2] = humidity_db->humidity_tr->data[1];
		val[3] = humidity_db->humidity_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else {
		error=0;
		val[1]=(uint8_t)humidity_db->tr_value;
		val[2]=(uint8_t)(humidity_db->tr_value>>8);
		len=3;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void humidity_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		humidity_db->humidity_tr->trigger_inactive=false;
		humidity_db->humidity_tr->data[0] = value[1];		/* if trigger is time based  */
		humidity_db->humidity_tr->data[1] = value[2];
		humidity_db->humidity_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		humidity_db->humidity_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		humidity_db->humidity_tr->trigger_inactive=true;
		if(humidity_db->timeout_id!=0){
			timeout_remove(humidity_db->timeout_id);
			humidity_db->humidity_enable = false;
			humidity_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		humidity_db->humidity_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {
		if(len!=3){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}		
		error=0;
		humidity_db->humidity_tr->trigger_inactive=false;		/* if trigger is value based */
		humidity_db->tr_value=value[1] | value[2]<<8 ;
		
	}

	humidity_db->humidity_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( humidity_db->humidity_tr->condition==0x01 || humidity_db->humidity_tr->condition==0x02){
		humidity_db->humidity_tr->value_enable=false;
		humidity_db->humidity_tr->time_enable=true;
	} else if( humidity_db->humidity_tr->condition>=0x03 && humidity_db->humidity_tr->condition<= 0x09){
		humidity_db->humidity_tr->time_enable=false;
		humidity_db->humidity_tr->value_enable=true;
		if(humidity_db->humidity_enable && !humidity_db->humidity_tr->trigger_inactive){
			timeout_remove(humidity_db->timeout_id);
			update_humidity_timer(conn);
		}	
	} else{	

		humidity_db->humidity_tr->trigger_inactive=true;
		humidity_db->humidity_tr->time_enable=false;
		humidity_db->humidity_tr->value_enable=false;
		if(humidity_db->humidity_enable && !humidity_db->humidity_tr->trigger_inactive){
			timeout_remove(humidity_db->timeout_id);
			update_humidity_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(humidity_db->humidity_tr->time_enable)
	humidity_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(humidity_db->g_update_timer!=humidity_db->g_current_timer && humidity_db->humidity_tr->time_enable){
		timeout_remove(humidity_db->timeout_id);
		update_humidity_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*
*** This section cocnsist of all the call backs of ***
*** Irradiance characteristics                     ***
*/


static void irradiance_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value;

	value=irradiance_db->irradiance_data;	

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static void irradiance_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = irradiance_db->irradiance_ms->sample;
	value[3] = irradiance_db->irradiance_ms->m_period[0];
	value[4] = irradiance_db->irradiance_ms->m_period[1];
	value[5] = irradiance_db->irradiance_ms->m_period[2];
	value[6] = irradiance_db->irradiance_ms->u_interval[0];
	value[7] = irradiance_db->irradiance_ms->u_interval[1];
	value[8] = irradiance_db->irradiance_ms->u_interval[2];
	value[9] = irradiance_db->irradiance_ms->applicatn;
	value[10] = irradiance_db->irradiance_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void irradiance_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value[2];

	value[0]=irradiance_db->lower_irradiance;
	value[1]=irradiance_db->upper_irradiance;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 4);

}
	


static void irradiance_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=irradiance_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void irradiance_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(irradiance_db->user_desc,0,21);
	memcpy(irradiance_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void irradiance_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=irradiance_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool irradiance_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,irradiance_db->irradiance_handle,
						(uint8_t *)&pdu, 2);

	return true;
}


static bool irradiance_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(irradiance_db->irradiance_tr->condition){

		case 0x03 : if( pdu == irradiance_db->irradiance_data)
				notify=false;
			    else
			    	irradiance_db->irradiance_data= pdu;
			    break;

		case 0x04 : if( pdu >= irradiance_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > irradiance_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= irradiance_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < irradiance_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != irradiance_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == irradiance_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,irradiance_db->irradiance_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_irradiance_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	irradiance_db->g_current_timer=((irradiance_db->irradiance_tr->data[0]) | (irradiance_db->irradiance_tr->data[1]<<8) | (irradiance_db->irradiance_tr->data[2]<<16));


	if(!irradiance_db->irradiance_enable || irradiance_db->irradiance_tr->trigger_inactive){
	timeout_remove(irradiance_db->timeout_id);
	return;
	}
	
	timer=irradiance_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(irradiance_db->irradiance_tr->time_enable)   
	irradiance_db->timeout_id=timeout_add(timer,irradiance_time_calculation,server,NULL);
	else
	irradiance_db->timeout_id=timeout_add(1000,irradiance_value_calculation,server,NULL);
	
}


static void irradiance_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || irradiance_db->irradiance_tr->trigger_inactive)
		irradiance_db->irradiance_enable = false;
	else if (value[0] == 0x01) {
		irradiance_db->irradiance_enable = true;
		irradiance_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_irradiance_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void irradiance_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=irradiance_db->irradiance_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = irradiance_db->irradiance_tr->data[0];
		val[2] = irradiance_db->irradiance_tr->data[1];
		val[3] = irradiance_db->irradiance_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else {
		error=0;
		val[1]=(uint8_t)irradiance_db->tr_value;
		val[2]=(uint8_t)(irradiance_db->tr_value>>8);
		len=3;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void irradiance_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		irradiance_db->irradiance_tr->trigger_inactive=false;
		irradiance_db->irradiance_tr->data[0] = value[1];		/* if trigger is time based  */
		irradiance_db->irradiance_tr->data[1] = value[2];
		irradiance_db->irradiance_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		irradiance_db->irradiance_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		irradiance_db->irradiance_tr->trigger_inactive=true;
		if(irradiance_db->timeout_id!=0){
			timeout_remove(irradiance_db->timeout_id);
			irradiance_db->irradiance_enable = false;
			irradiance_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		irradiance_db->irradiance_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {	
		if(len!=3){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}	
		error=0;
		irradiance_db->irradiance_tr->trigger_inactive=false;		/* if trigger is value based */
		irradiance_db->tr_value=value[1] | value[2]<<8 ;
		
	}

	irradiance_db->irradiance_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( irradiance_db->irradiance_tr->condition==0x01 || irradiance_db->irradiance_tr->condition==0x02){
		irradiance_db->irradiance_tr->value_enable=false;
		irradiance_db->irradiance_tr->time_enable=true;
	} else if( irradiance_db->irradiance_tr->condition>=0x03 && irradiance_db->irradiance_tr->condition<= 0x09){
		irradiance_db->irradiance_tr->time_enable=false;
		irradiance_db->irradiance_tr->value_enable=true;
		if(irradiance_db->irradiance_enable && !irradiance_db->irradiance_tr->trigger_inactive){
			timeout_remove(irradiance_db->timeout_id);
			update_irradiance_timer(conn);
		}	
	} else{	

		irradiance_db->irradiance_tr->trigger_inactive=true;
		irradiance_db->irradiance_tr->time_enable=false;
		irradiance_db->irradiance_tr->value_enable=false;
		if(irradiance_db->irradiance_enable && !irradiance_db->irradiance_tr->trigger_inactive){
			timeout_remove(irradiance_db->timeout_id);
			update_irradiance_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(irradiance_db->irradiance_tr->time_enable)
	irradiance_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(irradiance_db->g_update_timer!=irradiance_db->g_current_timer && irradiance_db->irradiance_tr->time_enable){
		timeout_remove(irradiance_db->timeout_id);
		update_irradiance_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


/*
*** This section cocnsist of all the call backs of ***
*** Pollen Concentration characteristics           ***
*/


static void pollen_concentration_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint32_t value;

	value=pollen_concentration_db->pollen_concentration_data;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 3);

}

static void pollen_concentration_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = pollen_concentration_db->pollen_concentration_ms->sample;
	value[3] = pollen_concentration_db->pollen_concentration_ms->m_period[0];
	value[4] = pollen_concentration_db->pollen_concentration_ms->m_period[1];
	value[5] = pollen_concentration_db->pollen_concentration_ms->m_period[2];
	value[6] = pollen_concentration_db->pollen_concentration_ms->u_interval[0];
	value[7] = pollen_concentration_db->pollen_concentration_ms->u_interval[1];
	value[8] = pollen_concentration_db->pollen_concentration_ms->u_interval[2];
	value[9] = pollen_concentration_db->pollen_concentration_ms->applicatn;
	value[10] = pollen_concentration_db->pollen_concentration_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}	


static void pollen_concentration_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint8_t value[2][3];

	value[0][0]=pollen_concentration_db->lower_pollen_concentration[0];
	value[0][1]=pollen_concentration_db->upper_pollen_concentration[1];
	value[0][2]=pollen_concentration_db->upper_pollen_concentration[2];
	value[1][0]=pollen_concentration_db->lower_pollen_concentration[0];
	value[1][1]=pollen_concentration_db->upper_pollen_concentration[1];
	value[1][2]=pollen_concentration_db->upper_pollen_concentration[2];
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 6);

}


static void pollen_concentration_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=pollen_concentration_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void pollen_concentration_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(pollen_concentration_db->user_desc,0,21);
	memcpy(pollen_concentration_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void pollen_concentration_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=pollen_concentration_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool pollen_concentration_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint32_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,pollen_concentration_db->pollen_concentration_handle,
						(uint8_t *)&pdu, 2);

	return true;
}


static bool pollen_concentration_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint32_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand()&0x00FFFFFF;



	 /*checking the trigger condition and notifying based on the particular condition */
 
	switch(pollen_concentration_db->pollen_concentration_tr->condition){

		case 0x03 : if( pdu == pollen_concentration_db->pollen_concentration_data)
				notify=false;
			    else
			    	pollen_concentration_db->pollen_concentration_data= pdu;
			    break;

		case 0x04 : if( pdu >= pollen_concentration_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > pollen_concentration_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= pollen_concentration_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < pollen_concentration_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != pollen_concentration_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == pollen_concentration_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,pollen_concentration_db->pollen_concentration_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

 /*Timer function will read the trigger setting and call a function to send notification based on value or time  */

static void update_pollen_concentration_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	pollen_concentration_db->g_current_timer=((pollen_concentration_db->pollen_concentration_tr->data[0]) | (pollen_concentration_db->pollen_concentration_tr->data[1]<<8) | (pollen_concentration_db->pollen_concentration_tr->data[2]<<16));


	if(!pollen_concentration_db->pollen_concentration_enable || pollen_concentration_db->pollen_concentration_tr->trigger_inactive){
	timeout_remove(pollen_concentration_db->timeout_id);
	return;
	}
	
	timer=pollen_concentration_db->g_current_timer*1000;
	

	/*
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(pollen_concentration_db->pollen_concentration_tr->time_enable)   
	pollen_concentration_db->timeout_id=timeout_add(timer,pollen_concentration_time_calculation,server,NULL);
	else
	pollen_concentration_db->timeout_id=timeout_add(1000,pollen_concentration_value_calculation,server,NULL);
	
}


static void pollen_concentration_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/*enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || pollen_concentration_db->pollen_concentration_tr->trigger_inactive)
		pollen_concentration_db->pollen_concentration_enable = false;
	else if (value[0] == 0x01) {
		pollen_concentration_db->pollen_concentration_enable = true;
		pollen_concentration_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_pollen_concentration_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void pollen_concentration_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=pollen_concentration_db->pollen_concentration_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = pollen_concentration_db->pollen_concentration_tr->data[0];
		val[2] = pollen_concentration_db->pollen_concentration_tr->data[1];
		val[3] = pollen_concentration_db->pollen_concentration_tr->data[2];
		len=4;
	}else{
		error=0;
		val[1]=(uint8_t)pollen_concentration_db->tr_value;
		val[2]=(uint8_t)(pollen_concentration_db->tr_value>>8);
		val[3]=(uint8_t)(pollen_concentration_db->tr_value>>16);
		len=3;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void pollen_concentration_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	 /* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		pollen_concentration_db->pollen_concentration_tr->trigger_inactive=false;
		pollen_concentration_db->pollen_concentration_tr->data[0] = value[1];		/* if trigger is time based  */
		pollen_concentration_db->pollen_concentration_tr->data[1] = value[2];
		pollen_concentration_db->pollen_concentration_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		pollen_concentration_db->pollen_concentration_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		pollen_concentration_db->pollen_concentration_tr->trigger_inactive=true;
		if(pollen_concentration_db->timeout_id!=0){
			timeout_remove(pollen_concentration_db->timeout_id);
			pollen_concentration_db->pollen_concentration_enable = false;
			pollen_concentration_db->indication = 0x0000;
			goto done;
		}
	} else  if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		pollen_concentration_db->pollen_concentration_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		pollen_concentration_db->pollen_concentration_tr->trigger_inactive=false;		/* if trigger is value based */
		pollen_concentration_db->tr_value=value[1] | value[2]<<8 | value[3]<<16 | 0<<24 ;
		
	}

	pollen_concentration_db->pollen_concentration_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( pollen_concentration_db->pollen_concentration_tr->condition==0x01 || pollen_concentration_db->pollen_concentration_tr->condition==0x02){
		pollen_concentration_db->pollen_concentration_tr->value_enable=false;
		pollen_concentration_db->pollen_concentration_tr->time_enable=true;
	} else if( pollen_concentration_db->pollen_concentration_tr->condition>=0x03 && pollen_concentration_db->pollen_concentration_tr->condition<= 0x09){
		pollen_concentration_db->pollen_concentration_tr->time_enable=false;
		pollen_concentration_db->pollen_concentration_tr->value_enable=true;
		if(pollen_concentration_db->pollen_concentration_enable && !pollen_concentration_db->pollen_concentration_tr->trigger_inactive){
			timeout_remove(pollen_concentration_db->timeout_id);
			update_pollen_concentration_timer(conn);
		}	
	} else{	

		pollen_concentration_db->pollen_concentration_tr->trigger_inactive=true;
		pollen_concentration_db->pollen_concentration_tr->time_enable=false;
		pollen_concentration_db->pollen_concentration_tr->value_enable=false;
		if(pollen_concentration_db->pollen_concentration_enable && !pollen_concentration_db->pollen_concentration_tr->trigger_inactive){
			timeout_remove(pollen_concentration_db->timeout_id);
			update_pollen_concentration_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(pollen_concentration_db->pollen_concentration_tr->time_enable)
	pollen_concentration_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(pollen_concentration_db->g_update_timer!=pollen_concentration_db->g_current_timer && pollen_concentration_db->pollen_concentration_tr->time_enable){
		timeout_remove(pollen_concentration_db->timeout_id);
		update_pollen_concentration_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*
*** This section cocnsist of all the call backs of ***
*** rainfall characteristics                       ***
*/


static void rainfall_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value;

	value=rainfall_db->rainfall_data;	

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static void rainfall_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = rainfall_db->rainfall_ms->sample;
	value[3] = rainfall_db->rainfall_ms->m_period[0];
	value[4] = rainfall_db->rainfall_ms->m_period[1];
	value[5] = rainfall_db->rainfall_ms->m_period[2];
	value[6] = rainfall_db->rainfall_ms->u_interval[0];
	value[7] = rainfall_db->rainfall_ms->u_interval[1];
	value[8] = rainfall_db->rainfall_ms->u_interval[2];
	value[9] = rainfall_db->rainfall_ms->applicatn;
	value[10] = rainfall_db->rainfall_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void rainfall_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value[2];

	value[0]=rainfall_db->lower_rainfall;
	value[1]=rainfall_db->upper_rainfall;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 4);

}
	


static void rainfall_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=rainfall_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void rainfall_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(rainfall_db->user_desc,0,21);
	memcpy(rainfall_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void rainfall_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=rainfall_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool rainfall_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,rainfall_db->rainfall_handle,
						(uint8_t *)&pdu, 2);

	return true;
}


static bool rainfall_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(rainfall_db->rainfall_tr->condition){

		case 0x03 : if( pdu == rainfall_db->rainfall_data)
				notify=false;
			    else
			    	rainfall_db->rainfall_data= pdu;
			    break;

		case 0x04 : if( pdu >= rainfall_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > rainfall_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= rainfall_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < rainfall_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != rainfall_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == rainfall_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,rainfall_db->rainfall_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_rainfall_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	rainfall_db->g_current_timer=((rainfall_db->rainfall_tr->data[0]) | (rainfall_db->rainfall_tr->data[1]<<8) | (rainfall_db->rainfall_tr->data[2]<<16));


	if(!rainfall_db->rainfall_enable || rainfall_db->rainfall_tr->trigger_inactive){
	timeout_remove(rainfall_db->timeout_id);
	return;
	}
	
	timer=rainfall_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(rainfall_db->rainfall_tr->time_enable)   
	rainfall_db->timeout_id=timeout_add(timer,rainfall_time_calculation,server,NULL);
	else
	rainfall_db->timeout_id=timeout_add(1000,rainfall_value_calculation,server,NULL);
	
}


static void rainfall_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || rainfall_db->rainfall_tr->trigger_inactive)
		rainfall_db->rainfall_enable = false;
	else if (value[0] == 0x01) {
		rainfall_db->rainfall_enable = true;
		rainfall_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_rainfall_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void rainfall_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=rainfall_db->rainfall_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = rainfall_db->rainfall_tr->data[0];
		val[2] = rainfall_db->rainfall_tr->data[1];
		val[3] = rainfall_db->rainfall_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else {
		error=0;
		val[1]=(uint8_t)rainfall_db->tr_value;
		val[2]=(uint8_t)(rainfall_db->tr_value>>8);
		len=3;
	}

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void rainfall_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		rainfall_db->rainfall_tr->trigger_inactive=false;
		rainfall_db->rainfall_tr->data[0] = value[1];		/* if trigger is time based  */
		rainfall_db->rainfall_tr->data[1] = value[2];
		rainfall_db->rainfall_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		rainfall_db->rainfall_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		rainfall_db->rainfall_tr->trigger_inactive=true;
		if(rainfall_db->timeout_id!=0){
			timeout_remove(rainfall_db->timeout_id);
			rainfall_db->rainfall_enable = false;
			rainfall_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		rainfall_db->rainfall_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {	
		if(len!=3){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}	
		error=0;
		rainfall_db->rainfall_tr->trigger_inactive=false;		/* if trigger is value based */
		rainfall_db->tr_value=value[1] | value[2]<<8 ;
		
	}

	rainfall_db->rainfall_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( rainfall_db->rainfall_tr->condition==0x01 || rainfall_db->rainfall_tr->condition==0x02){
		rainfall_db->rainfall_tr->value_enable=false;
		rainfall_db->rainfall_tr->time_enable=true;
	} else if( rainfall_db->rainfall_tr->condition>=0x03 && rainfall_db->rainfall_tr->condition<= 0x09){
		rainfall_db->rainfall_tr->time_enable=false;
		rainfall_db->rainfall_tr->value_enable=true;
		if(rainfall_db->rainfall_enable && !rainfall_db->rainfall_tr->trigger_inactive){
			timeout_remove(rainfall_db->timeout_id);
			update_rainfall_timer(conn);
		}	
	} else{	

		rainfall_db->rainfall_tr->trigger_inactive=true;
		rainfall_db->rainfall_tr->time_enable=false;
		rainfall_db->rainfall_tr->value_enable=false;
		if(rainfall_db->rainfall_enable && !rainfall_db->rainfall_tr->trigger_inactive){
			timeout_remove(rainfall_db->timeout_id);
			update_rainfall_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(rainfall_db->rainfall_tr->time_enable)
	rainfall_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(rainfall_db->g_update_timer!=rainfall_db->g_current_timer && rainfall_db->rainfall_tr->time_enable){
		timeout_remove(rainfall_db->timeout_id);
		update_rainfall_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


/*
*** This section cocnsist of all the call backs of ***
*** Pressure characteristics                       ***
*/

static void pressure_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint32_t value;

	value=pressure_db->pressure_data;	

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 4);

}


static void pressure_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = pressure_db->pressure_ms->sample;
	value[3] = pressure_db->pressure_ms->m_period[0];
	value[4] = pressure_db->pressure_ms->m_period[1];
	value[5] = pressure_db->pressure_ms->m_period[2];
	value[6] = pressure_db->pressure_ms->u_interval[0];
	value[7] = pressure_db->pressure_ms->u_interval[1];
	value[8] = pressure_db->pressure_ms->u_interval[2];
	value[9] = pressure_db->pressure_ms->applicatn;
	value[10] = pressure_db->pressure_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void pressure_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint32_t value[2];

	value[0]=pressure_db->lower_pressure;
	value[1]=pressure_db->upper_pressure;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 8);

}
	


static void pressure_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=pressure_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void pressure_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(pressure_db->user_desc,0,21);
	memcpy(pressure_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void pressure_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=pressure_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool pressure_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint32_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,pressure_db->pressure_handle,
						(uint8_t *)&pdu, 4);

	return true;
}


static bool pressure_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint32_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(pressure_db->pressure_tr->condition){

		case 0x03 : if( pdu == pressure_db->pressure_data)
				notify=false;
			    else
			    	pressure_db->pressure_data= pdu;
			    break;

		case 0x04 : if( pdu >= pressure_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > pressure_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= pressure_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < pressure_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != pressure_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == pressure_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,pressure_db->pressure_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_pressure_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	pressure_db->g_current_timer=((pressure_db->pressure_tr->data[0]) | (pressure_db->pressure_tr->data[1]<<8) | (pressure_db->pressure_tr->data[2]<<16));


	if(!pressure_db->pressure_enable || pressure_db->pressure_tr->trigger_inactive){
	timeout_remove(pressure_db->timeout_id);
	return;
	}
	
	timer=pressure_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(pressure_db->pressure_tr->time_enable)   
	pressure_db->timeout_id=timeout_add(timer,pressure_time_calculation,server,NULL);
	else
	pressure_db->timeout_id=timeout_add(1000,pressure_value_calculation,server,NULL);
	
}


static void pressure_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || pressure_db->pressure_tr->trigger_inactive)
		pressure_db->pressure_enable = false;
	else if (value[0] == 0x01) {
		pressure_db->pressure_enable = true;
		pressure_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_pressure_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void pressure_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[5];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=pressure_db->pressure_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = pressure_db->pressure_tr->data[0];
		val[2] = pressure_db->pressure_tr->data[1];
		val[3] = pressure_db->pressure_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else {
		error=0;
		val[1]=(uint8_t)pressure_db->tr_value;
		val[2]=(uint8_t)(pressure_db->tr_value>>8);
		val[3]=(uint8_t)(pressure_db->tr_value>>16);
		val[4]=(uint8_t)(pressure_db->tr_value>>24);
		len=5;
	}
	
	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void pressure_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>5) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		pressure_db->pressure_tr->trigger_inactive=false;
		pressure_db->pressure_tr->data[0] = value[1];		/* if trigger is time based  */
		pressure_db->pressure_tr->data[1] = value[2];
		pressure_db->pressure_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		pressure_db->pressure_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		pressure_db->pressure_tr->trigger_inactive=true;
		if(pressure_db->timeout_id!=0){
			timeout_remove(pressure_db->timeout_id);
			pressure_db->pressure_enable = false;
			pressure_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		pressure_db->pressure_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {
		if(len!=5){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		pressure_db->pressure_tr->trigger_inactive=false;		/* if trigger is value based */
		pressure_db->tr_value=value[1] | value[2]<<8 | value[3]<<16 | value[4]<<24 ;
		
	}

	pressure_db->pressure_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( pressure_db->pressure_tr->condition==0x01 || pressure_db->pressure_tr->condition==0x02){
		pressure_db->pressure_tr->value_enable=false;
		pressure_db->pressure_tr->time_enable=true;
	} else if( pressure_db->pressure_tr->condition>=0x03 && pressure_db->pressure_tr->condition<= 0x09){
		pressure_db->pressure_tr->time_enable=false;
		pressure_db->pressure_tr->value_enable=true;
		if(pressure_db->pressure_enable && !pressure_db->pressure_tr->trigger_inactive){
			timeout_remove(pressure_db->timeout_id);
			update_pressure_timer(conn);
		}	
	} else{	

		pressure_db->pressure_tr->trigger_inactive=true;
		pressure_db->pressure_tr->time_enable=false;
		pressure_db->pressure_tr->value_enable=false;
		if(pressure_db->pressure_enable && !pressure_db->pressure_tr->trigger_inactive){
			timeout_remove(pressure_db->timeout_id);
			update_pressure_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(pressure_db->pressure_tr->time_enable)
	pressure_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(pressure_db->g_update_timer!=pressure_db->g_current_timer && pressure_db->pressure_tr->time_enable){
		timeout_remove(pressure_db->timeout_id);
		update_pressure_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*
*** This section cocnsist of all the call backs of ***
*** True Wind Direction characteristics            ***
*/


static void true_direction_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value;

	value=true_direction_db->true_direction_data;	

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static void true_direction_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = true_direction_db->true_direction_ms->sample;
	value[3] = true_direction_db->true_direction_ms->m_period[0];
	value[4] = true_direction_db->true_direction_ms->m_period[1];
	value[5] = true_direction_db->true_direction_ms->m_period[2];
	value[6] = true_direction_db->true_direction_ms->u_interval[0];
	value[7] = true_direction_db->true_direction_ms->u_interval[1];
	value[8] = true_direction_db->true_direction_ms->u_interval[2];
	value[9] = true_direction_db->true_direction_ms->applicatn;
	value[10] = true_direction_db->true_direction_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void true_direction_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value[2];

	value[0]=true_direction_db->lower_true_direction;
	value[1]=true_direction_db->upper_true_direction;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 4);

}
	


static void true_direction_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=true_direction_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void true_direction_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(true_direction_db->user_desc,0,21);
	memcpy(true_direction_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

static void true_direction_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=true_direction_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool true_direction_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,true_direction_db->true_direction_handle,
						(uint8_t *)&pdu, 2);

	return true;
}


static bool true_direction_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(true_direction_db->true_direction_tr->condition){

		case 0x03 : if( pdu == true_direction_db->true_direction_data)
				notify=false;
			    else
			    	true_direction_db->true_direction_data= pdu;
			    break;

		case 0x04 : if( pdu >= true_direction_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > true_direction_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= true_direction_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < true_direction_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != true_direction_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == true_direction_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,true_direction_db->true_direction_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_true_direction_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	true_direction_db->g_current_timer=((true_direction_db->true_direction_tr->data[0]) | (true_direction_db->true_direction_tr->data[1]<<8) | (true_direction_db->true_direction_tr->data[2]<<16));


	if(!true_direction_db->true_direction_enable || true_direction_db->true_direction_tr->trigger_inactive){
	timeout_remove(true_direction_db->timeout_id);
	return;
	}
	
	timer=true_direction_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(true_direction_db->true_direction_tr->time_enable)   
	true_direction_db->timeout_id=timeout_add(timer,true_direction_time_calculation,server,NULL);
	else
	true_direction_db->timeout_id=timeout_add(1000,true_direction_value_calculation,server,NULL);
	
}


static void true_direction_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || true_direction_db->true_direction_tr->trigger_inactive)
		true_direction_db->true_direction_enable = false;
	else if (value[0] == 0x01) {
		true_direction_db->true_direction_enable = true;
		true_direction_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_true_direction_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void true_direction_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=true_direction_db->true_direction_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = true_direction_db->true_direction_tr->data[0];
		val[2] = true_direction_db->true_direction_tr->data[1];
		val[3] = true_direction_db->true_direction_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else{
		error=0;
		val[1]=(uint8_t)true_direction_db->tr_value;
		val[2]=(uint8_t)(true_direction_db->tr_value>>8);
		len=3;
	}

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void true_direction_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		true_direction_db->true_direction_tr->trigger_inactive=false;
		true_direction_db->true_direction_tr->data[0] = value[1];		/* if trigger is time based  */
		true_direction_db->true_direction_tr->data[1] = value[2];
		true_direction_db->true_direction_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		true_direction_db->true_direction_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		true_direction_db->true_direction_tr->trigger_inactive=true;
		if(true_direction_db->timeout_id!=0){
			timeout_remove(true_direction_db->timeout_id);
			true_direction_db->true_direction_enable = false;
			true_direction_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		true_direction_db->true_direction_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {	
		if(len!=3){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}	
		error=0;
		true_direction_db->true_direction_tr->trigger_inactive=false;		/* if trigger is value based */
		true_direction_db->tr_value=value[1] | value[2]<<8 ;
		
	}

	true_direction_db->true_direction_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( true_direction_db->true_direction_tr->condition==0x01 || true_direction_db->true_direction_tr->condition==0x02){
		true_direction_db->true_direction_tr->value_enable=false;
		true_direction_db->true_direction_tr->time_enable=true;
	} else if( true_direction_db->true_direction_tr->condition>=0x03 && true_direction_db->true_direction_tr->condition<= 0x09){
		true_direction_db->true_direction_tr->time_enable=false;
		true_direction_db->true_direction_tr->value_enable=true;
		if(true_direction_db->true_direction_enable && !true_direction_db->true_direction_tr->trigger_inactive){
			timeout_remove(true_direction_db->timeout_id);
			update_true_direction_timer(conn);
		}	
	} else{	

		true_direction_db->true_direction_tr->trigger_inactive=true;
		true_direction_db->true_direction_tr->time_enable=false;
		true_direction_db->true_direction_tr->value_enable=false;
		if(true_direction_db->true_direction_enable && !true_direction_db->true_direction_tr->trigger_inactive){
			timeout_remove(true_direction_db->timeout_id);
			update_true_direction_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(true_direction_db->true_direction_tr->time_enable)
	true_direction_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(true_direction_db->g_update_timer!=true_direction_db->g_current_timer && true_direction_db->true_direction_tr->time_enable){
		timeout_remove(true_direction_db->timeout_id);
		update_true_direction_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


/*
*** This section cocnsist of all the call backs of ***
*** True Wind Speed characteristics                ***
*/


static void true_speed_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value;

	value=true_speed_db->true_speed_data;	

	gatt_db_attribute_read_result(attrib, id, error,(uint8_t  *)&value, 2);

}


static void true_speed_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = true_speed_db->true_speed_ms->sample;
	value[3] = true_speed_db->true_speed_ms->m_period[0];
	value[4] = true_speed_db->true_speed_ms->m_period[1];
	value[5] = true_speed_db->true_speed_ms->m_period[2];
	value[6] = true_speed_db->true_speed_ms->u_interval[0];
	value[7] = true_speed_db->true_speed_ms->u_interval[1];
	value[8] = true_speed_db->true_speed_ms->u_interval[2];
	value[9] = true_speed_db->true_speed_ms->applicatn;
	value[10] = true_speed_db->true_speed_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void true_speed_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value[2];

	value[0]=true_speed_db->lower_true_speed;
	value[1]=true_speed_db->upper_true_speed;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 4);

}
	


static void true_speed_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=true_speed_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void true_speed_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(true_speed_db->user_desc,0,21);
	memcpy(true_speed_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void true_speed_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=true_speed_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool true_speed_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,true_speed_db->true_speed_handle,
						(uint8_t *)&pdu, 2);

	return true;
}


static bool true_speed_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(true_speed_db->true_speed_tr->condition){

		case 0x03 : if( pdu == true_speed_db->true_speed_data)
				notify=false;
			    else
			    	true_speed_db->true_speed_data= pdu;
			    break;

		case 0x04 : if( pdu >= true_speed_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > true_speed_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= true_speed_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < true_speed_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != true_speed_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == true_speed_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,true_speed_db->true_speed_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_true_speed_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	true_speed_db->g_current_timer=((true_speed_db->true_speed_tr->data[0]) | (true_speed_db->true_speed_tr->data[1]<<8) | (true_speed_db->true_speed_tr->data[2]<<16));


	if(!true_speed_db->true_speed_enable || true_speed_db->true_speed_tr->trigger_inactive){
	timeout_remove(true_speed_db->timeout_id);
	return;
	}
	
	timer=true_speed_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(true_speed_db->true_speed_tr->time_enable)   
	true_speed_db->timeout_id=timeout_add(timer,true_speed_time_calculation,server,NULL);
	else
	true_speed_db->timeout_id=timeout_add(1000,true_speed_value_calculation,server,NULL);
	
}


static void true_speed_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || true_speed_db->true_speed_tr->trigger_inactive)
		true_speed_db->true_speed_enable = false;
	else if (value[0] == 0x01) {
		true_speed_db->true_speed_enable = true;
		true_speed_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_true_speed_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void true_speed_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=true_speed_db->true_speed_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = true_speed_db->true_speed_tr->data[0];
		val[2] = true_speed_db->true_speed_tr->data[1];
		val[3] = true_speed_db->true_speed_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else{
		error=0;
		val[1]=(uint8_t)true_speed_db->tr_value;
		val[2]=(uint8_t)(true_speed_db->tr_value>>8);
		len=3;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void true_speed_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		true_speed_db->true_speed_tr->trigger_inactive=false;
		true_speed_db->true_speed_tr->data[0] = value[1];		/* if trigger is time based  */
		true_speed_db->true_speed_tr->data[1] = value[2];
		true_speed_db->true_speed_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		true_speed_db->true_speed_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		true_speed_db->true_speed_tr->trigger_inactive=true;
		if(true_speed_db->timeout_id!=0){
			timeout_remove(true_speed_db->timeout_id);
			true_speed_db->true_speed_enable = false;
			true_speed_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		true_speed_db->true_speed_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {	
		if(len!=3){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}	
		error=0;
		true_speed_db->true_speed_tr->trigger_inactive=false;		/* if trigger is value based */
		true_speed_db->tr_value=value[1] | value[2]<<8 ;
		
	}

	true_speed_db->true_speed_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( true_speed_db->true_speed_tr->condition==0x01 || true_speed_db->true_speed_tr->condition==0x02){
		true_speed_db->true_speed_tr->value_enable=false;
		true_speed_db->true_speed_tr->time_enable=true;
	} else if( true_speed_db->true_speed_tr->condition>=0x03 && true_speed_db->true_speed_tr->condition<= 0x09){
		true_speed_db->true_speed_tr->time_enable=false;
		true_speed_db->true_speed_tr->value_enable=true;
		if(true_speed_db->true_speed_enable && !true_speed_db->true_speed_tr->trigger_inactive){
			timeout_remove(true_speed_db->timeout_id);
			update_true_speed_timer(conn);
		}	
	} else{	

		true_speed_db->true_speed_tr->trigger_inactive=true;
		true_speed_db->true_speed_tr->time_enable=false;
		true_speed_db->true_speed_tr->value_enable=false;
		if(true_speed_db->true_speed_enable && !true_speed_db->true_speed_tr->trigger_inactive){
			timeout_remove(true_speed_db->timeout_id);
			update_true_speed_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(true_speed_db->true_speed_tr->time_enable)
	true_speed_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(true_speed_db->g_update_timer!=true_speed_db->g_current_timer && true_speed_db->true_speed_tr->time_enable){
		timeout_remove(true_speed_db->timeout_id);
		update_true_speed_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*
*** This section cocnsist of all the call backs of ***
*** UV Index characteristics                       ***
*/


static void uv_index_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint8_t value;

	value=uv_index_db->uv_index_data;	

	gatt_db_attribute_read_result(attrib, id, error, &value, 1);

}


static void uv_index_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = uv_index_db->uv_index_ms->sample;
	value[3] = uv_index_db->uv_index_ms->m_period[0];
	value[4] = uv_index_db->uv_index_ms->m_period[1];
	value[5] = uv_index_db->uv_index_ms->m_period[2];
	value[6] = uv_index_db->uv_index_ms->u_interval[0];
	value[7] = uv_index_db->uv_index_ms->u_interval[1];
	value[8] = uv_index_db->uv_index_ms->u_interval[2];
	value[9] = uv_index_db->uv_index_ms->applicatn;
	value[10] = uv_index_db->uv_index_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void uv_index_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint8_t value[2];

	value[0]=uv_index_db->lower_uv_index;
	value[1]=uv_index_db->upper_uv_index;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 2);

}
	


static void uv_index_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=uv_index_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void uv_index_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(uv_index_db->user_desc,0,21);
	memcpy(uv_index_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


/*
*** This section cocnsist of all the call backs of ***
*** Wind Chill characteristics                     ***
*/

static void uv_index_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=uv_index_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool uv_index_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint8_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,uv_index_db->uv_index_handle,
						(uint8_t *)&pdu, 1);

	return true;
}


static bool uv_index_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint8_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(uv_index_db->uv_index_tr->condition){

		case 0x03 : if( pdu == uv_index_db->uv_index_data)
				notify=false;
			    else
			    	uv_index_db->uv_index_data= pdu;
			    break;

		case 0x04 : if( pdu >= uv_index_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > uv_index_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= uv_index_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < uv_index_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != uv_index_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == uv_index_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,uv_index_db->uv_index_handle,
						(uint8_t *)&pdu, 1);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_uv_index_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	uv_index_db->g_current_timer=((uv_index_db->uv_index_tr->data[0]) | (uv_index_db->uv_index_tr->data[1]<<8) | (uv_index_db->uv_index_tr->data[2]<<16));


	if(!uv_index_db->uv_index_enable || uv_index_db->uv_index_tr->trigger_inactive){
	timeout_remove(uv_index_db->timeout_id);
	return;
	}
	
	timer=uv_index_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(uv_index_db->uv_index_tr->time_enable)   
	uv_index_db->timeout_id=timeout_add(timer,uv_index_time_calculation,server,NULL);
	else
	uv_index_db->timeout_id=timeout_add(1000,uv_index_value_calculation,server,NULL);
	
}


static void uv_index_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || uv_index_db->uv_index_tr->trigger_inactive)
		uv_index_db->uv_index_enable = false;
	else if (value[0] == 0x01) {
		uv_index_db->uv_index_enable = true;
		uv_index_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_uv_index_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void uv_index_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=uv_index_db->uv_index_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = uv_index_db->uv_index_tr->data[0];
		val[2] = uv_index_db->uv_index_tr->data[1];
		val[3] = uv_index_db->uv_index_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	} else{
		error=0;
		val[1]=(uint8_t)uv_index_db->tr_value;
		len=2;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void uv_index_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		uv_index_db->uv_index_tr->trigger_inactive=false;
		uv_index_db->uv_index_tr->data[0] = value[1];		/* if trigger is time based  */
		uv_index_db->uv_index_tr->data[1] = value[2];
		uv_index_db->uv_index_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		uv_index_db->uv_index_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		uv_index_db->uv_index_tr->trigger_inactive=true;
		if(uv_index_db->timeout_id!=0){
			timeout_remove(uv_index_db->timeout_id);
			uv_index_db->uv_index_enable = false;
			uv_index_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		uv_index_db->uv_index_tr->trigger_inactive=false;		/* if trigger is value based */
	} else{
		if(len!=2){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		uv_index_db->uv_index_tr->trigger_inactive=false;		/* if trigger is value based */
		uv_index_db->tr_value=value[1];
		
	}

	uv_index_db->uv_index_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( uv_index_db->uv_index_tr->condition==0x01 || uv_index_db->uv_index_tr->condition==0x02){
		uv_index_db->uv_index_tr->value_enable=false;
		uv_index_db->uv_index_tr->time_enable=true;
	} else if( uv_index_db->uv_index_tr->condition>=0x03 && uv_index_db->uv_index_tr->condition<= 0x09){
		uv_index_db->uv_index_tr->time_enable=false;
		uv_index_db->uv_index_tr->value_enable=true;
		if(uv_index_db->uv_index_enable && !uv_index_db->uv_index_tr->trigger_inactive){
			timeout_remove(uv_index_db->timeout_id);
			update_uv_index_timer(conn);
		}	
	} else{	

		uv_index_db->uv_index_tr->trigger_inactive=true;
		uv_index_db->uv_index_tr->time_enable=false;
		uv_index_db->uv_index_tr->value_enable=false;
		if(uv_index_db->uv_index_enable && !uv_index_db->uv_index_tr->trigger_inactive){
			timeout_remove(uv_index_db->timeout_id);
			update_uv_index_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(uv_index_db->uv_index_tr->time_enable)
	uv_index_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(uv_index_db->g_update_timer!=uv_index_db->g_current_timer && uv_index_db->uv_index_tr->time_enable){
		timeout_remove(uv_index_db->timeout_id);
		update_uv_index_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void wind_chill_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int8_t value;

	value=wind_chill_db->wind_chill_data;	

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 1);

}


static void wind_chill_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = wind_chill_db->wind_chill_ms->sample;
	value[3] = wind_chill_db->wind_chill_ms->m_period[0];
	value[4] = wind_chill_db->wind_chill_ms->m_period[1];
	value[5] = wind_chill_db->wind_chill_ms->m_period[2];
	value[6] = wind_chill_db->wind_chill_ms->u_interval[0];
	value[7] = wind_chill_db->wind_chill_ms->u_interval[1];
	value[8] = wind_chill_db->wind_chill_ms->u_interval[2];
	value[9] = wind_chill_db->wind_chill_ms->applicatn;
	value[10] = wind_chill_db->wind_chill_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void wind_chill_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int8_t value[2];

	value[0]=wind_chill_db->lower_wind_chill;
	value[1]=wind_chill_db->upper_wind_chill;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 2);

}
	


static void wind_chill_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=wind_chill_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void wind_chill_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(wind_chill_db->user_desc,0,21);
	memcpy(wind_chill_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void wind_chill_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=wind_chill_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool wind_chill_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int8_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,wind_chill_db->wind_chill_handle,
						(uint8_t *)&pdu, 1);

	return true;
}


static bool wind_chill_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int8_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(wind_chill_db->wind_chill_tr->condition){

		case 0x03 : if( pdu == wind_chill_db->wind_chill_data)
				notify=false;
			    else
			    	wind_chill_db->wind_chill_data= pdu;
			    break;

		case 0x04 : if( pdu >= wind_chill_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > wind_chill_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= wind_chill_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < wind_chill_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != wind_chill_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == wind_chill_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,wind_chill_db->wind_chill_handle,
						(uint8_t *)&pdu, 1);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_wind_chill_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	wind_chill_db->g_current_timer=((wind_chill_db->wind_chill_tr->data[0]) | (wind_chill_db->wind_chill_tr->data[1]<<8) | (wind_chill_db->wind_chill_tr->data[2]<<16));


	if(!wind_chill_db->wind_chill_enable || wind_chill_db->wind_chill_tr->trigger_inactive){
	timeout_remove(wind_chill_db->timeout_id);
	return;
	}
	
	timer=wind_chill_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(wind_chill_db->wind_chill_tr->time_enable)   
	wind_chill_db->timeout_id=timeout_add(timer,wind_chill_time_calculation,server,NULL);
	else
	wind_chill_db->timeout_id=timeout_add(1000,wind_chill_value_calculation,server,NULL);
	
}


static void wind_chill_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || wind_chill_db->wind_chill_tr->trigger_inactive)
		wind_chill_db->wind_chill_enable = false;
	else if (value[0] == 0x01) {
		wind_chill_db->wind_chill_enable = true;
		wind_chill_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_wind_chill_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void wind_chill_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=wind_chill_db->wind_chill_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = wind_chill_db->wind_chill_tr->data[0];
		val[2] = wind_chill_db->wind_chill_tr->data[1];
		val[3] = wind_chill_db->wind_chill_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	} else{
		error=0;
		val[1]=(uint8_t)wind_chill_db->tr_value;
		len=2;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void wind_chill_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		wind_chill_db->wind_chill_tr->trigger_inactive=false;
		wind_chill_db->wind_chill_tr->data[0] = value[1];		/* if trigger is time based  */
		wind_chill_db->wind_chill_tr->data[1] = value[2];
		wind_chill_db->wind_chill_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		wind_chill_db->wind_chill_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		wind_chill_db->wind_chill_tr->trigger_inactive=true;
		if(wind_chill_db->timeout_id!=0){
			timeout_remove(wind_chill_db->timeout_id);
			wind_chill_db->wind_chill_enable = false;
			wind_chill_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		wind_chill_db->wind_chill_tr->trigger_inactive=false;		/* if trigger is value based */
	} else{
		if(len!=2){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		wind_chill_db->wind_chill_tr->trigger_inactive=false;		/* if trigger is value based */
		wind_chill_db->tr_value=value[1];
		
	}

	wind_chill_db->wind_chill_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( wind_chill_db->wind_chill_tr->condition==0x01 || wind_chill_db->wind_chill_tr->condition==0x02){
		wind_chill_db->wind_chill_tr->value_enable=false;
		wind_chill_db->wind_chill_tr->time_enable=true;
	} else if( wind_chill_db->wind_chill_tr->condition>=0x03 && wind_chill_db->wind_chill_tr->condition<= 0x09){
		wind_chill_db->wind_chill_tr->time_enable=false;
		wind_chill_db->wind_chill_tr->value_enable=true;
		if(wind_chill_db->wind_chill_enable && !wind_chill_db->wind_chill_tr->trigger_inactive){
			timeout_remove(wind_chill_db->timeout_id);
			update_wind_chill_timer(conn);
		}	
	} else{	

		wind_chill_db->wind_chill_tr->trigger_inactive=true;
		wind_chill_db->wind_chill_tr->time_enable=false;
		wind_chill_db->wind_chill_tr->value_enable=false;
		if(wind_chill_db->wind_chill_enable && !wind_chill_db->wind_chill_tr->trigger_inactive){
			timeout_remove(wind_chill_db->timeout_id);
			update_wind_chill_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(wind_chill_db->wind_chill_tr->time_enable)
	wind_chill_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(wind_chill_db->g_update_timer!=wind_chill_db->g_current_timer && wind_chill_db->wind_chill_tr->time_enable){
		timeout_remove(wind_chill_db->timeout_id);
		update_wind_chill_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*
*** This section cocnsist of all the call backs of ***
*** Barometric Pressure Trend characteristics      ***
*/


static void barometric_pressure_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint8_t value;

	value=barometric_pressure_db->barometric_pressure_data;	

	gatt_db_attribute_read_result(attrib, id, error, &value, 1);

}


static void barometric_pressure_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = barometric_pressure_db->barometric_pressure_ms->sample;
	value[3] = barometric_pressure_db->barometric_pressure_ms->m_period[0];
	value[4] = barometric_pressure_db->barometric_pressure_ms->m_period[1];
	value[5] = barometric_pressure_db->barometric_pressure_ms->m_period[2];
	value[6] = barometric_pressure_db->barometric_pressure_ms->u_interval[0];
	value[7] = barometric_pressure_db->barometric_pressure_ms->u_interval[1];
	value[8] = barometric_pressure_db->barometric_pressure_ms->u_interval[2];
	value[9] = barometric_pressure_db->barometric_pressure_ms->applicatn;
	value[10] = barometric_pressure_db->barometric_pressure_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void barometric_pressure_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint8_t value[2];
/*
	value[0]=barometric_pressure_db->lower_barometric_pressure;
	value[1]=barometric_pressure_db->upper_barometric_pressure;
*/	
	value[0]=rand();
	value[1]=rand();
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 2);

}
	


static void barometric_pressure_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=barometric_pressure_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void barometric_pressure_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(barometric_pressure_db->user_desc,0,21);
	memcpy(barometric_pressure_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}




/*
*** This section cocnsist of all the call backs of ***
*** Magnetic Declination characteristics           ***
*/


static void magnetic_declination_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value;

	value=magnetic_declination_db->magnetic_declination_data;	

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static void magnetic_declination_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = magnetic_declination_db->magnetic_declination_ms->sample;
	value[3] = magnetic_declination_db->magnetic_declination_ms->m_period[0];
	value[4] = magnetic_declination_db->magnetic_declination_ms->m_period[1];
	value[5] = magnetic_declination_db->magnetic_declination_ms->m_period[2];
	value[6] = magnetic_declination_db->magnetic_declination_ms->u_interval[0];
	value[7] = magnetic_declination_db->magnetic_declination_ms->u_interval[1];
	value[8] = magnetic_declination_db->magnetic_declination_ms->u_interval[2];
	value[9] = magnetic_declination_db->magnetic_declination_ms->applicatn;
	value[10] = magnetic_declination_db->magnetic_declination_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void magnetic_declination_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	uint16_t value[2];

	value[0]=magnetic_declination_db->lower_magnetic_declination;
	value[1]=magnetic_declination_db->upper_magnetic_declination;
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 4);

}
	


static void magnetic_declination_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=magnetic_declination_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void magnetic_declination_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(magnetic_declination_db->user_desc,0,21);
	memcpy(magnetic_declination_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

static void magnetic_declination_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=magnetic_declination_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool magnetic_declination_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;

	

		
	pdu=rand();

	bt_gatt_server_send_notification(server->gatt,magnetic_declination_db->magnetic_declination_handle,
						(uint8_t *)&pdu, 2);

	return true;
}


static bool magnetic_declination_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	uint16_t pdu;
	uint8_t notify=true;

	

		
	pdu=rand();



	/* checking the trigger condition and notifying based on the particular condition */
 
	switch(magnetic_declination_db->magnetic_declination_tr->condition){

		case 0x03 : if( pdu == magnetic_declination_db->magnetic_declination_data)
				notify=false;
			    else
			    	magnetic_declination_db->magnetic_declination_data= pdu;
			    break;

		case 0x04 : if( pdu >= magnetic_declination_db->tr_value)
				notify=false;
			    break;

		case 0x05 : if( pdu > magnetic_declination_db->tr_value)
				notify=false;
			    break;

		case 0x06 : if( pdu <= magnetic_declination_db->tr_value)
				notify=false;
			    break;

		case 0x07 : if( pdu < magnetic_declination_db->tr_value)
				notify=false;
			    break;

		case 0x08 : if( pdu != magnetic_declination_db->tr_value)
				notify=false;
			    break;

		default   : if( pdu == magnetic_declination_db->tr_value)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,magnetic_declination_db->magnetic_declination_handle,
						(uint8_t *)&pdu, 2);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_magnetic_declination_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	magnetic_declination_db->g_current_timer=((magnetic_declination_db->magnetic_declination_tr->data[0]) | (magnetic_declination_db->magnetic_declination_tr->data[1]<<8) | (magnetic_declination_db->magnetic_declination_tr->data[2]<<16));


	if(!magnetic_declination_db->magnetic_declination_enable || magnetic_declination_db->magnetic_declination_tr->trigger_inactive){
	timeout_remove(magnetic_declination_db->timeout_id);
	return;
	}
	
	timer=magnetic_declination_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(magnetic_declination_db->magnetic_declination_tr->time_enable)   
	magnetic_declination_db->timeout_id=timeout_add(timer,magnetic_declination_time_calculation,server,NULL);
	else
	magnetic_declination_db->timeout_id=timeout_add(1000,magnetic_declination_value_calculation,server,NULL);
	
}


static void magnetic_declination_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || magnetic_declination_db->magnetic_declination_tr->trigger_inactive)
		magnetic_declination_db->magnetic_declination_enable = false;
	else if (value[0] == 0x01) {
		magnetic_declination_db->magnetic_declination_enable = true;
		magnetic_declination_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_magnetic_declination_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void magnetic_declination_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[4];

	/* checking the trigger conditions and sending time or value based on the conition */

	val[0]=magnetic_declination_db->magnetic_declination_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = magnetic_declination_db->magnetic_declination_tr->data[0];
		val[2] = magnetic_declination_db->magnetic_declination_tr->data[1];
		val[3] = magnetic_declination_db->magnetic_declination_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else{
		error=0;
		val[1]=(uint8_t)magnetic_declination_db->tr_value;
		val[2]=(uint8_t)(magnetic_declination_db->tr_value>>8);
		len=3;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void magnetic_declination_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>4) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		magnetic_declination_db->magnetic_declination_tr->trigger_inactive=false;
		magnetic_declination_db->magnetic_declination_tr->data[0] = value[1];		/* if trigger is time based  */
		magnetic_declination_db->magnetic_declination_tr->data[1] = value[2];
		magnetic_declination_db->magnetic_declination_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		magnetic_declination_db->magnetic_declination_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		magnetic_declination_db->magnetic_declination_tr->trigger_inactive=true;
		if(magnetic_declination_db->timeout_id!=0){
			timeout_remove(magnetic_declination_db->timeout_id);
			magnetic_declination_db->magnetic_declination_enable = false;
			magnetic_declination_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		magnetic_declination_db->magnetic_declination_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {
		if(len!=3){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}		
		error=0;
		magnetic_declination_db->magnetic_declination_tr->trigger_inactive=false;		/* if trigger is value based */
		magnetic_declination_db->tr_value=value[1] | value[2]<<8 ;
		
	}

	magnetic_declination_db->magnetic_declination_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( magnetic_declination_db->magnetic_declination_tr->condition==0x01 || magnetic_declination_db->magnetic_declination_tr->condition==0x02){
		magnetic_declination_db->magnetic_declination_tr->value_enable=false;
		magnetic_declination_db->magnetic_declination_tr->time_enable=true;
	} else if( magnetic_declination_db->magnetic_declination_tr->condition>=0x03 && magnetic_declination_db->magnetic_declination_tr->condition<= 0x09){
		magnetic_declination_db->magnetic_declination_tr->time_enable=false;
		magnetic_declination_db->magnetic_declination_tr->value_enable=true;
		if(magnetic_declination_db->magnetic_declination_enable && !magnetic_declination_db->magnetic_declination_tr->trigger_inactive){
			timeout_remove(magnetic_declination_db->timeout_id);
			update_magnetic_declination_timer(conn);
		}	
	} else{	

		magnetic_declination_db->magnetic_declination_tr->trigger_inactive=true;
		magnetic_declination_db->magnetic_declination_tr->time_enable=false;
		magnetic_declination_db->magnetic_declination_tr->value_enable=false;
		if(magnetic_declination_db->magnetic_declination_enable && !magnetic_declination_db->magnetic_declination_tr->trigger_inactive){
			timeout_remove(magnetic_declination_db->timeout_id);
			update_magnetic_declination_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(magnetic_declination_db->magnetic_declination_tr->time_enable)
	magnetic_declination_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(magnetic_declination_db->g_update_timer!=magnetic_declination_db->g_current_timer && magnetic_declination_db->magnetic_declination_tr->time_enable){
		timeout_remove(magnetic_declination_db->timeout_id);
		update_magnetic_declination_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


/*
*** This section cocnsist of all the call backs of ***
*** Magnetic Flux Density 2D characteristics       ***
*/

static void magnetic_flux_2D_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int16_t value[2];

	value[0]=magnetic_flux_2D_db->magnetic_flux_2D_xdata;
	value[1]=magnetic_flux_2D_db->magnetic_flux_2D_ydata;		

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 4);

}


static void magnetic_flux_2D_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = magnetic_flux_2D_db->magnetic_flux_2D_ms->sample;
	value[3] = magnetic_flux_2D_db->magnetic_flux_2D_ms->m_period[0];
	value[4] = magnetic_flux_2D_db->magnetic_flux_2D_ms->m_period[1];
	value[5] = magnetic_flux_2D_db->magnetic_flux_2D_ms->m_period[2];
	value[6] = magnetic_flux_2D_db->magnetic_flux_2D_ms->u_interval[0];
	value[7] = magnetic_flux_2D_db->magnetic_flux_2D_ms->u_interval[1];
	value[8] = magnetic_flux_2D_db->magnetic_flux_2D_ms->u_interval[2];
	value[9] = magnetic_flux_2D_db->magnetic_flux_2D_ms->applicatn;
	value[10] = magnetic_flux_2D_db->magnetic_flux_2D_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void magnetic_flux_2D_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int16_t value[4];

	value[0]=magnetic_flux_2D_db->xlower_magnetic_flux_2D;
	value[1]=magnetic_flux_2D_db->xupper_magnetic_flux_2D;
	value[2]=magnetic_flux_2D_db->ylower_magnetic_flux_2D;
	value[3]=magnetic_flux_2D_db->yupper_magnetic_flux_2D;
	
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 8);

}
	


static void magnetic_flux_2D_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=magnetic_flux_2D_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void magnetic_flux_2D_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(magnetic_flux_2D_db->user_desc,0,21);
	memcpy(magnetic_flux_2D_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void magnetic_flux_2D_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=magnetic_flux_2D_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool magnetic_flux_2D_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int16_t pdu[2];

	

		
	pdu[0]=rand();
	pdu[1]=rand();

	bt_gatt_server_send_notification(server->gatt,magnetic_flux_2D_db->magnetic_flux_2D_handle,
						(uint8_t *)pdu, 4);

	return true;
}


static bool magnetic_flux_2D_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int16_t pdu[2];
	uint8_t notify=true;

	

		
	pdu[0]=rand();
	pdu[1]=rand();



	/*
	*** checking the trigger condition and notifying based on the particular condition                 ***
	*** in Condition 0x04,0x05,0x06,0x07,0x08,0x09 the x,y flux values are checked with trigger values ***
	*** if any one or two values are matching the trigger condition they will be notified              ***
	*** and previously value will be notified for the one's not matching trigger condition             ***   
	*/
 
	switch(magnetic_flux_2D_db->magnetic_flux_2D_tr->condition){

		case 0x03 : if( pdu[0] == magnetic_flux_2D_db->magnetic_flux_2D_xdata && pdu[1]== magnetic_flux_2D_db->magnetic_flux_2D_ydata)
				notify=false;
			    break;

		case 0x04 : if( pdu[0]>=magnetic_flux_2D_db->tr_valuex && pdu[1]>=magnetic_flux_2D_db->tr_valuey)
				notify=false;
			    break;

		case 0x05 : if( pdu[0] > magnetic_flux_2D_db->tr_valuex && pdu[1] > magnetic_flux_2D_db->tr_valuey)
				notify=false;
			    break;

		case 0x06 : if( pdu[0] <= magnetic_flux_2D_db->tr_valuex &&  pdu[1] <= magnetic_flux_2D_db->tr_valuey)
				notify=false;
			    break;

		case 0x07 : if( pdu[0] < magnetic_flux_2D_db->tr_valuex && pdu[1] < magnetic_flux_2D_db->tr_valuey )
				notify=false;
			    break;

		case 0x08 : if( pdu[0] != magnetic_flux_2D_db->tr_valuex && pdu[1] != magnetic_flux_2D_db->tr_valuey )
				notify=false;
			    break;

		default   : if( pdu[0] == magnetic_flux_2D_db->tr_valuex && pdu[1]== magnetic_flux_2D_db->tr_valuey)
				notify=false;
	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,magnetic_flux_2D_db->magnetic_flux_2D_handle,
						(uint8_t *)pdu, 4);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_magnetic_flux_2D_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	magnetic_flux_2D_db->g_current_timer=((magnetic_flux_2D_db->magnetic_flux_2D_tr->data[0]) | (magnetic_flux_2D_db->magnetic_flux_2D_tr->data[1]<<8) | (magnetic_flux_2D_db->magnetic_flux_2D_tr->data[2]<<16));


	if(!magnetic_flux_2D_db->magnetic_flux_2D_enable || magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive){
	timeout_remove(magnetic_flux_2D_db->timeout_id);
	return;
	}
	
	timer=magnetic_flux_2D_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(magnetic_flux_2D_db->magnetic_flux_2D_tr->time_enable)   
	magnetic_flux_2D_db->timeout_id=timeout_add(timer,magnetic_flux_2D_time_calculation,server,NULL);
	else
	magnetic_flux_2D_db->timeout_id=timeout_add(1000,magnetic_flux_2D_value_calculation,server,NULL);
	
}


static void magnetic_flux_2D_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive)
		magnetic_flux_2D_db->magnetic_flux_2D_enable = false;
	else if (value[0] == 0x01) {
		magnetic_flux_2D_db->magnetic_flux_2D_enable = true;
		magnetic_flux_2D_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_magnetic_flux_2D_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void magnetic_flux_2D_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[5];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=magnetic_flux_2D_db->magnetic_flux_2D_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = magnetic_flux_2D_db->magnetic_flux_2D_tr->data[0];
		val[2] = magnetic_flux_2D_db->magnetic_flux_2D_tr->data[1];
		val[3] = magnetic_flux_2D_db->magnetic_flux_2D_tr->data[2];
		len=4;
	}else if(val[0]==0x03){
		error=0;
		len=1;
	}else {
		error=0;
		val[1]=(uint8_t)magnetic_flux_2D_db->tr_valuex;
		val[2]=(uint8_t)(magnetic_flux_2D_db->tr_valuex>>8);
		val[3]=(uint8_t)magnetic_flux_2D_db->tr_valuey;
		val[4]=(uint8_t)(magnetic_flux_2D_db->tr_valuey>>8);
		len=5;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void magnetic_flux_2D_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>5) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive=false;
		magnetic_flux_2D_db->magnetic_flux_2D_tr->data[0] = value[1];		/* if trigger is time based  */
		magnetic_flux_2D_db->magnetic_flux_2D_tr->data[1] = value[2];
		magnetic_flux_2D_db->magnetic_flux_2D_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		magnetic_flux_2D_db->magnetic_flux_2D_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive=true;
		if(magnetic_flux_2D_db->timeout_id!=0){
			timeout_remove(magnetic_flux_2D_db->timeout_id);
			magnetic_flux_2D_db->magnetic_flux_2D_enable = false;
			magnetic_flux_2D_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive=false;		/* if trigger is value based */
	}else {
		if(len!=5){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive=false;		/* if trigger is value based */
		magnetic_flux_2D_db->tr_valuex=value[1] | value[2]<<8 ;
		magnetic_flux_2D_db->tr_valuey=value[3] | value[4]<<8 ;
		
	}

	magnetic_flux_2D_db->magnetic_flux_2D_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( magnetic_flux_2D_db->magnetic_flux_2D_tr->condition==0x01 || magnetic_flux_2D_db->magnetic_flux_2D_tr->condition==0x02){
		magnetic_flux_2D_db->magnetic_flux_2D_tr->value_enable=false;
		magnetic_flux_2D_db->magnetic_flux_2D_tr->time_enable=true;
	} else if( magnetic_flux_2D_db->magnetic_flux_2D_tr->condition>=0x03 && magnetic_flux_2D_db->magnetic_flux_2D_tr->condition<= 0x09){
		magnetic_flux_2D_db->magnetic_flux_2D_tr->time_enable=false;
		magnetic_flux_2D_db->magnetic_flux_2D_tr->value_enable=true;
		if(magnetic_flux_2D_db->magnetic_flux_2D_enable && !magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive){
			timeout_remove(magnetic_flux_2D_db->timeout_id);
			update_magnetic_flux_2D_timer(conn);
		}	
	} else{	

		magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive=true;
		magnetic_flux_2D_db->magnetic_flux_2D_tr->time_enable=false;
		magnetic_flux_2D_db->magnetic_flux_2D_tr->value_enable=false;
		if(magnetic_flux_2D_db->magnetic_flux_2D_enable && !magnetic_flux_2D_db->magnetic_flux_2D_tr->trigger_inactive){
			timeout_remove(magnetic_flux_2D_db->timeout_id);
			update_magnetic_flux_2D_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(magnetic_flux_2D_db->magnetic_flux_2D_tr->time_enable)
	magnetic_flux_2D_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(magnetic_flux_2D_db->g_update_timer!=magnetic_flux_2D_db->g_current_timer && magnetic_flux_2D_db->magnetic_flux_2D_tr->time_enable){
		timeout_remove(magnetic_flux_2D_db->timeout_id);
		update_magnetic_flux_2D_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

/*
*** This section cocnsist of all the call backs of ***
*** Magnetic Flux Density 3D characteristics       ***
*/


static void magnetic_flux_3D_read(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int16_t value[3];

	value[0]=magnetic_flux_3D_db->magnetic_flux_3D_xdata;
	value[1]=magnetic_flux_3D_db->magnetic_flux_3D_ydata;		
	value[2]=magnetic_flux_3D_db->magnetic_flux_3D_zdata;		
	
	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 6);

}


static void magnetic_flux_3D_measurement_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0,value[11];

	value[0] = 0;
	value[1] = 0;
	value[2] = magnetic_flux_3D_db->magnetic_flux_3D_ms->sample;
	value[3] = magnetic_flux_3D_db->magnetic_flux_3D_ms->m_period[0];
	value[4] = magnetic_flux_3D_db->magnetic_flux_3D_ms->m_period[1];
	value[5] = magnetic_flux_3D_db->magnetic_flux_3D_ms->m_period[2];
	value[6] = magnetic_flux_3D_db->magnetic_flux_3D_ms->u_interval[0];
	value[7] = magnetic_flux_3D_db->magnetic_flux_3D_ms->u_interval[1];
	value[8] = magnetic_flux_3D_db->magnetic_flux_3D_ms->u_interval[2];
	value[9] = magnetic_flux_3D_db->magnetic_flux_3D_ms->applicatn;
	value[10] = magnetic_flux_3D_db->magnetic_flux_3D_ms->m_uncertainity;

	gatt_db_attribute_read_result(attrib, id, error, value, 11);

}



static void magnetic_flux_3D_valid_range_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	int16_t value[6];

	value[0]=magnetic_flux_3D_db->xlower_magnetic_flux_3D;
	value[1]=magnetic_flux_3D_db->xupper_magnetic_flux_3D;
	value[2]=magnetic_flux_3D_db->ylower_magnetic_flux_3D;
	value[3]=magnetic_flux_3D_db->yupper_magnetic_flux_3D;
	value[4]=magnetic_flux_3D_db->zlower_magnetic_flux_3D;
	value[5]=magnetic_flux_3D_db->zupper_magnetic_flux_3D;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 12);

}
	


static void magnetic_flux_3D_char_user_desc_read_cb(struct gatt_db_attribute *attrib,
		      unsigned int id, uint16_t offset,
		      uint8_t opcode, struct bt_att *att, void *user_data)
{

	uint8_t error=0;
	wchar_t *value;

	value=magnetic_flux_3D_db->user_desc;

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)value, 21);

}


static void magnetic_flux_3D_char_user_desc_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	/* checking if user string is empty or more than the default size */
	
	if(!value || len>=22){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if(offset){
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	/* initializing the string with zero's and then copying the new string to the data base */

	memset(magnetic_flux_3D_db->user_desc,0,21);
	memcpy(magnetic_flux_3D_db->user_desc,value,len+1);

done :

	gatt_db_attribute_write_result(attrib, id, error);

}


static void magnetic_flux_3D_msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{

	uint8_t error;
	static uint16_t value;

	if(offset>2)
	{
	error=BT_ATT_ERROR_INVALID_OFFSET;
	}
	else
	{
	error=0;
	value=magnetic_flux_3D_db->indication;
	}

	gatt_db_attribute_read_result(attrib, id, error, (uint8_t *)&value, 2);

}


static bool magnetic_flux_3D_time_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int16_t pdu[3];

	

		
	pdu[0]=rand();
	pdu[1]=rand();
	pdu[2]=rand();

	bt_gatt_server_send_notification(server->gatt,magnetic_flux_3D_db->magnetic_flux_3D_handle,
						(uint8_t *)pdu, 6);

	return true;
}


static bool magnetic_flux_3D_value_calculation(void *user_data)
{

	struct gatt_conn *server=user_data;
	int16_t pdu[3];
	uint8_t notify=true;

	

		
	pdu[0]=rand();
	pdu[1]=rand();
	pdu[2]=rand();



	/*
	*** checking the trigger condition and notifying based on the particular condition         ***
	*** in Condition 0x04,0x05,0x06,0x07 the x,y,z flux values are checked with trigger values ***
	*** if any one,two or all values are matching the trigger condition they will be notified  ***
	*** and previously value will be notified for the one's not matching trigger condition     ***   
	*/
 
	switch(magnetic_flux_3D_db->magnetic_flux_3D_tr->condition){

		case 0x03 : if( pdu[0] == magnetic_flux_3D_db->magnetic_flux_3D_xdata &&  pdu[1] == magnetic_flux_3D_db->magnetic_flux_3D_ydata &&  pdu[2] == magnetic_flux_3D_db->magnetic_flux_3D_zdata)
				notify=false;
			    else{
			    	magnetic_flux_3D_db->magnetic_flux_3D_xdata= pdu[0];
			    	magnetic_flux_3D_db->magnetic_flux_3D_ydata= pdu[1];
			    	magnetic_flux_3D_db->magnetic_flux_3D_zdata= pdu[2];
			    }
			    break;

		case 0x04 : if( pdu[0] >= magnetic_flux_3D_db->tr_valuex && pdu[1] >= magnetic_flux_3D_db->tr_valuey && pdu[2] >= magnetic_flux_3D_db->tr_valuez )
				notify=false;
			    break;

		case 0x05 : if( pdu[0] > magnetic_flux_3D_db->tr_valuex && pdu[1] > magnetic_flux_3D_db->tr_valuey && pdu[2] > magnetic_flux_3D_db->tr_valuez)
				notify=false;
			    break;

		case 0x06 : if( pdu[0] <= magnetic_flux_3D_db->tr_valuex && pdu[1] <= magnetic_flux_3D_db->tr_valuey && pdu[2] <= magnetic_flux_3D_db->tr_valuez)
				notify=false;
			    break;

		case 0x07 : if( pdu[0] < magnetic_flux_3D_db->tr_valuex && pdu[1] < magnetic_flux_3D_db->tr_valuey && pdu[2] < magnetic_flux_3D_db->tr_valuez)
				notify=false;
			    break;

		default   : if( pdu[0] == magnetic_flux_3D_db->tr_valuex && pdu[1] == magnetic_flux_3D_db->tr_valuey && pdu[2] == magnetic_flux_3D_db->tr_valuez)
				notify=false;

	}
	
	/* notify only if its satisfying the trigger condition */			
	if(notify){
	bt_gatt_server_send_notification(server->gatt,magnetic_flux_3D_db->magnetic_flux_3D_handle,
						(uint8_t *)pdu, 6);
	}

	return true;
}

/* Timer function will read the trigger setting and call a function to send notification based on value or time */ 

static void update_magnetic_flux_3D_timer(struct gatt_conn *server)
{
	
	unsigned int timer;

	magnetic_flux_3D_db->g_current_timer=((magnetic_flux_3D_db->magnetic_flux_3D_tr->data[0]) | (magnetic_flux_3D_db->magnetic_flux_3D_tr->data[1]<<8) | (magnetic_flux_3D_db->magnetic_flux_3D_tr->data[2]<<16));


	if(!magnetic_flux_3D_db->magnetic_flux_3D_enable || magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive){
	timeout_remove(magnetic_flux_3D_db->timeout_id);
	return;
	}
	
	timer=magnetic_flux_3D_db->g_current_timer*1000;
	

	/* 
	** if trigger is based on time call the notification based on trigger value **
	** if trigger is based on value call the function on every one sec and check the trigger condition and notify **
	*/

 
	if(magnetic_flux_3D_db->magnetic_flux_3D_tr->time_enable)   
	magnetic_flux_3D_db->timeout_id=timeout_add(timer,magnetic_flux_3D_time_calculation,server,NULL);
	else
	magnetic_flux_3D_db->timeout_id=timeout_add(1000,magnetic_flux_3D_value_calculation,server,NULL);
	
}


static void magnetic_flux_3D_msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
				    unsigned int id, uint16_t offset,
				    const uint8_t * value, size_t len,
				    uint8_t opcode, struct bt_att *att,
				    void *user_data)
{

	uint8_t error = 0;
	struct gatt_conn *server = conn;

	if (!value || len != 2) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
		goto done;
	}

	if (offset) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		goto done;
	}

	/* enabling the notification and updating the data base */
	
	if (value[0] == 0x00 || magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive)
		magnetic_flux_3D_db->magnetic_flux_3D_enable = false;
	else if (value[0] == 0x01) {
		magnetic_flux_3D_db->magnetic_flux_3D_enable = true;
		magnetic_flux_3D_db->indication = 0x0001;
	} else
		error = 0x80;

	/* updating a timer function to call notification on a periodic interval */

	update_magnetic_flux_3D_timer(server);

 done:

	gatt_db_attribute_write_result(attrib, id, error);

}

static void magnetic_flux_3D_triger_read_cb(struct gatt_db_attribute *attrib,
			       unsigned int id, uint16_t offset,
			       uint8_t opcode, struct bt_att *att,
			       void *user_data)
{
	uint8_t error,len;
	uint8_t val[7];

	/* checking the trigger conditions and sending time or value based on the conition */
	val[0]=magnetic_flux_3D_db->magnetic_flux_3D_tr->condition;

	if(val[0] == 0x00){
		error=0;
		len=1;
	}else if(val[0] == 0x01 || val[0]==0x02){
		error = 0;
		val[1] = magnetic_flux_3D_db->magnetic_flux_3D_tr->data[0];
		val[2] = magnetic_flux_3D_db->magnetic_flux_3D_tr->data[1];
		val[3] = magnetic_flux_3D_db->magnetic_flux_3D_tr->data[2];
		len=4;
	}else if(val[0]==0x03) {
		error=0;
		len=1;
	} else {
		error=0;
		val[1]=(uint8_t)magnetic_flux_3D_db->tr_valuex;
		val[2]=(uint8_t)(magnetic_flux_3D_db->tr_valuex>>8);
		val[3]=(uint8_t)magnetic_flux_3D_db->tr_valuey;
		val[4]=(uint8_t)(magnetic_flux_3D_db->tr_valuey>>8);
		val[5]=(uint8_t)magnetic_flux_3D_db->tr_valuez;
		val[6]=(uint8_t)(magnetic_flux_3D_db->tr_valuez>>8);
		len=7;
	}
	

	gatt_db_attribute_read_result(attrib, id, error, val, len);

}


static void magnetic_flux_3D_triger_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t * value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{

	uint8_t error;

	if (len<1 || len>7) {
		error = BT_ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LEN;
	} else if(value[0] < 255 && value[0]>= 10){ 	/* checking if the value is RFU */
		error = BT_ERROR_OUT_OF_RANGE ;	
	} else if(value[0]==0x01 || value[0]==0x02){
		if(len!=4){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error = 0;
		magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive=false;
		magnetic_flux_3D_db->magnetic_flux_3D_tr->data[0] = value[1];		/* if trigger is time based  */
		magnetic_flux_3D_db->magnetic_flux_3D_tr->data[1] = value[2];
		magnetic_flux_3D_db->magnetic_flux_3D_tr->data[2] = value[3];
	}else if(value[0]==0x00){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		magnetic_flux_3D_db->magnetic_flux_3D_tr->condition = value[0] ;	/* if trigger is inactive check notification is enable if yes disable the 												notification */
		magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive=true;
		if(magnetic_flux_3D_db->timeout_id!=0){
			timeout_remove(magnetic_flux_3D_db->timeout_id);
			magnetic_flux_3D_db->magnetic_flux_3D_enable = false;
			magnetic_flux_3D_db->indication = 0x0000;
			goto done;
		}
	} else if(value[0]==0x03){
		if(len!=1){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive=false;		/* if trigger is value based */
	} else {
		if(len!=7){
			error=BT_ATT_ERROR_INVALID_PDU;
			goto done;
		}
		error=0;
		magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive=false;		/* if trigger is value based */
		magnetic_flux_3D_db->tr_valuex=value[1] | value[2]<<8 ;
		magnetic_flux_3D_db->tr_valuey=value[3] | value[4]<<8 ;
		magnetic_flux_3D_db->tr_valuez=value[5] | value[6]<<8 ;
		
	}

	magnetic_flux_3D_db->magnetic_flux_3D_tr->condition=value[0] ;
	

	/*
	** checking if trigger is time or value based which will use in enabling the notification **
	** if notification is already enabled update the timer with new values  **
	*/

	if( magnetic_flux_3D_db->magnetic_flux_3D_tr->condition==0x01 || magnetic_flux_3D_db->magnetic_flux_3D_tr->condition==0x02){
		magnetic_flux_3D_db->magnetic_flux_3D_tr->value_enable=false;
		magnetic_flux_3D_db->magnetic_flux_3D_tr->time_enable=true;
	} else if( magnetic_flux_3D_db->magnetic_flux_3D_tr->condition>=0x03 && magnetic_flux_3D_db->magnetic_flux_3D_tr->condition<= 0x09){
		magnetic_flux_3D_db->magnetic_flux_3D_tr->time_enable=false;
		magnetic_flux_3D_db->magnetic_flux_3D_tr->value_enable=true;
		if(magnetic_flux_3D_db->magnetic_flux_3D_enable && !magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive){
			timeout_remove(magnetic_flux_3D_db->timeout_id);
			update_magnetic_flux_3D_timer(conn);
		}	
	} else{	

		magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive=true;
		magnetic_flux_3D_db->magnetic_flux_3D_tr->time_enable=false;
		magnetic_flux_3D_db->magnetic_flux_3D_tr->value_enable=false;
		if(magnetic_flux_3D_db->magnetic_flux_3D_enable && !magnetic_flux_3D_db->magnetic_flux_3D_tr->trigger_inactive){
			timeout_remove(magnetic_flux_3D_db->timeout_id);
			update_magnetic_flux_3D_timer(conn);
		}	
		
	}
	
	/*
	*** updating the gloabal timer and checking if its not equal to current timer *** 
	*** then calling the update timer function to update the timer to notify      ***
	*/
	
	if(magnetic_flux_3D_db->magnetic_flux_3D_tr->time_enable)
	magnetic_flux_3D_db->g_update_timer=(value[1])|(value[2]<<8)| (value[3]<<16);


	if(magnetic_flux_3D_db->g_update_timer!=magnetic_flux_3D_db->g_current_timer && magnetic_flux_3D_db->magnetic_flux_3D_tr->time_enable){
		timeout_remove(magnetic_flux_3D_db->timeout_id);
		update_magnetic_flux_3D_timer(conn);
	}
	

done :

	gatt_db_attribute_write_result(attrib, id, error);

}

static void gap_device_name_read(struct gatt_db_attribute *attrib,
				 unsigned int id, uint16_t offset,
				 uint8_t opcode, struct bt_att *att,
				 void *user_data)
{
	uint8_t error;
	const uint8_t *value;
	size_t len;

	if (offset > dev_name_len) {
		error = BT_ATT_ERROR_INVALID_OFFSET;
		value = NULL;
		len = dev_name_len;
	} else {
		error = 0;
		len = dev_name_len - offset;
		value = len ? &dev_name[offset] : NULL;
	}

	gatt_db_attribute_read_result(attrib, id, error, value, len);
}

static void populate_gap_service(struct gatt_db *db)
{
	struct gatt_db_attribute *service;
	bt_uuid_t uuid;

	bt_uuid16_create(&uuid, UUID_GAP);
	service = gatt_db_add_service(db, &uuid, true, 6);

	bt_uuid16_create(&uuid, GATT_CHARAC_DEVICE_NAME);
	gatt_db_service_add_characteristic(service, &uuid,
					   BT_ATT_PERM_READ,
					   BT_GATT_CHRC_PROP_READ,
					   gap_device_name_read, NULL, NULL);

	gatt_db_service_set_active(service, true);
}

static void populate_devinfo_service(struct gatt_db *db)
{
	struct gatt_db_attribute *service;
	bt_uuid_t uuid;

	bt_uuid16_create(&uuid, 0x180a);
	service = gatt_db_add_service(db, &uuid, true, 9);

	gatt_db_service_set_active(service, true);
}

/* This function will initialize the fields of all the characteristic and descriptor with server designed initial values */

static void ess_all_characteristics_init(void)
{

	es_temp_init();
	es_speed_init();
	es_apparent_direction_init();
	es_dew_point_init();
	es_elevation_init();
	es_gust_factor_init();
	es_heat_index_init();
	es_humidity_init();
	es_irradiance_init();
	es_pollen_concentration_init();
	es_rainfall_init();
	es_pressure_init();
	es_true_direction_init();
	es_true_speed_init();
	es_uv_index_init();
	es_wind_chill_init();
	es_barometric_pressure_init();
	es_magnetic_declination_init();
	es_magnetic_flux_2D_init();
	es_magnetic_flux_3D_init();

}


/*
*   This Functions populate the gatt data base with all the service ,   *
*   characteristic and descriptors of ESS Profile                       *
*/

static void populate_environmental_service(struct gatt_db *db)
{
	struct gatt_db_attribute *service, *temp,*apparent_direction,*apparent_speed,*elevation,*dew_point,*gust_factor,*heat_index,*humidity,*irradiance,*rainfall,*pressure,*true_direction,*true_speed,*uv_index,*wind_chill,*magnetic_declination,*magnetic_flux_2D,*magnetic_flux_3D;
	bt_uuid_t uuid;

	/*  Adding ESS service as primary service  */ 

	bt_uuid16_create(&uuid, UUID_ESS_SERVICE);
	service = gatt_db_add_service(db, &uuid, true, 160);	


	ess_all_characteristics_init();

	/*  Adding Tempertaure characteristics and it's descriptors  */ 

	bt_uuid16_create(&uuid, UUID_TEMPERATURE);

	temp = gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  temp_read, NULL, NULL);

	temp_db->temp_handle = gatt_db_attribute_get_handle(temp);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       temp_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       temp_triger_read_cb, temp_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       temp_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE, 
				       temp_char_user_desc_read_cb,temp_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       temp_msrmt_ccc_read_cb,
				       temp_msrmt_ccc_write_cb, conn);


	/*  Adding apparent wind Speed characteristics and it's descriptors  */ 

	bt_uuid16_create(&uuid, UUID_APPARENT_WIND_SPEED);

	apparent_speed=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  apparent_speed_read, NULL, NULL);

	apparent_speed_db->apparent_speed_handle = gatt_db_attribute_get_handle(apparent_speed);


	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       apparent_speed_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       apparent_speed_triger_read_cb, apparent_speed_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       apparent_speed_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       apparent_speed_char_user_desc_read_cb,apparent_speed_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       apparent_speed_msrmt_ccc_read_cb,
				       apparent_speed_msrmt_ccc_write_cb, conn);

	/*  Adding apparent wind Direction characteristics and it's descriptors  */ 

	bt_uuid16_create(&uuid, UUID_APPARENT_WIND_DIRECTION);

	apparent_direction=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  apparent_direction_read, NULL, NULL);

	apparent_direction_db->apparent_direction_handle = gatt_db_attribute_get_handle(apparent_direction);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       apparent_direction_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       apparent_direction_triger_read_cb, apparent_direction_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       apparent_direction_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       apparent_direction_char_user_desc_read_cb,apparent_direction_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       apparent_direction_msrmt_ccc_read_cb,
				       apparent_direction_msrmt_ccc_write_cb, conn);

	/*  Adding Dew point characteristics and it's descriptors  */ 

	bt_uuid16_create(&uuid, UUID_DEW_POINT);

	dew_point=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  dew_point_read, NULL, NULL);

	dew_point_db->dew_point_handle = gatt_db_attribute_get_handle(dew_point);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       dew_point_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       dew_point_triger_read_cb, dew_point_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       dew_point_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       dew_point_char_user_desc_read_cb,dew_point_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       dew_point_msrmt_ccc_read_cb,
				       dew_point_msrmt_ccc_write_cb, conn);
	
	/*  Adding Elevation characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_ELEVATION);

	elevation=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  elevation_read, NULL, NULL);

	elevation_db->elevation_handle = gatt_db_attribute_get_handle(elevation);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       elevation_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       elevation_triger_read_cb, elevation_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       elevation_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       elevation_char_user_desc_read_cb,elevation_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       elevation_msrmt_ccc_read_cb,
				       elevation_msrmt_ccc_write_cb, conn);

	/* Adding Gust Factor characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_GUST_FACTOR);

	gust_factor=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  gust_factor_read, NULL, NULL);

	gust_factor_db->gust_factor_handle = gatt_db_attribute_get_handle(gust_factor);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       gust_factor_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       gust_factor_triger_read_cb, gust_factor_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       gust_factor_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       gust_factor_char_user_desc_read_cb,gust_factor_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       gust_factor_msrmt_ccc_read_cb,
				       gust_factor_msrmt_ccc_write_cb, conn);

	/*  Adding Heat Index characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_HEAT_INDEX);

	heat_index=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  heat_index_read, NULL, NULL);

	heat_index_db->heat_index_handle = gatt_db_attribute_get_handle(heat_index);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       heat_index_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       heat_index_triger_read_cb, heat_index_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);

	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       heat_index_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       heat_index_char_user_desc_read_cb,heat_index_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       heat_index_msrmt_ccc_read_cb,
				       heat_index_msrmt_ccc_write_cb, conn);

	/*  Adding Humidity characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_HUMIDITY);

	humidity=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  humidity_read, NULL, NULL);

	humidity_db->humidity_handle = gatt_db_attribute_get_handle(humidity);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       humidity_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       humidity_triger_read_cb, humidity_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);

	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       humidity_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       humidity_char_user_desc_read_cb,humidity_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       humidity_msrmt_ccc_read_cb,
				       humidity_msrmt_ccc_write_cb, conn);

	/*  Adding Irradiance characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_IRRADIANCE);	

	irradiance=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  irradiance_read, NULL, NULL);

	irradiance_db->irradiance_handle = gatt_db_attribute_get_handle(irradiance);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       irradiance_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       irradiance_triger_read_cb, irradiance_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       irradiance_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       irradiance_char_user_desc_read_cb,irradiance_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       irradiance_msrmt_ccc_read_cb,
				       irradiance_msrmt_ccc_write_cb, conn);

	/*  Adding Pollen concentration characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_POLLEN_CONCENTRATION);

	gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  pollen_concentration_read, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       pollen_concentration_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       pollen_concentration_triger_read_cb, pollen_concentration_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       pollen_concentration_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       pollen_concentration_char_user_desc_read_cb,pollen_concentration_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       pollen_concentration_msrmt_ccc_read_cb,
				       pollen_concentration_msrmt_ccc_write_cb, conn);

	/* Adding Rainfall characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_RAIN_FALL);

	rainfall=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  rainfall_read, NULL, NULL);

	rainfall_db->rainfall_handle = gatt_db_attribute_get_handle(rainfall);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       rainfall_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       rainfall_triger_read_cb, rainfall_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       rainfall_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       rainfall_char_user_desc_read_cb,rainfall_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       rainfall_msrmt_ccc_read_cb,
				       rainfall_msrmt_ccc_write_cb, conn);

	/*  Adding Pressure characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_PRESSURE);

	pressure=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  pressure_read, NULL, NULL);

	pressure_db->pressure_handle = gatt_db_attribute_get_handle(pressure);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       pressure_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       pressure_triger_read_cb, pressure_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       pressure_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       pressure_char_user_desc_read_cb,pressure_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       pressure_msrmt_ccc_read_cb,
				       pressure_msrmt_ccc_write_cb, conn);

	/*  Adding True Wind Direction characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_TRUE_WIND_DIRECTION);

	true_direction=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ|
						  BT_GATT_CHRC_PROP_NOTIFY,
						  true_direction_read, NULL, NULL);

	true_direction_db->true_direction_handle = gatt_db_attribute_get_handle(true_direction);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       true_direction_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       true_direction_triger_read_cb, true_direction_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       true_direction_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       true_direction_char_user_desc_read_cb,true_direction_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       true_direction_msrmt_ccc_read_cb,
				       true_direction_msrmt_ccc_write_cb, conn);

	/*  Adding True Wind Speed characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_TRUE_WIND_SPEED);

	true_speed=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  true_speed_read, NULL, NULL);

	true_speed_db->true_speed_handle = gatt_db_attribute_get_handle(true_speed);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       true_speed_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       true_speed_triger_read_cb, true_speed_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       true_speed_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       true_speed_char_user_desc_read_cb,true_speed_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       true_speed_msrmt_ccc_read_cb,
				       true_speed_msrmt_ccc_write_cb, conn);

	/*  Adding UV Index characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_UV_INDEX);

	uv_index=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  uv_index_read, NULL, NULL);

	uv_index_db->uv_index_handle = gatt_db_attribute_get_handle(uv_index);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       uv_index_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       uv_index_triger_read_cb, uv_index_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       uv_index_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       uv_index_char_user_desc_read_cb,uv_index_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       uv_index_msrmt_ccc_read_cb,
				       uv_index_msrmt_ccc_write_cb, conn);

	/*  Adding Wind Chill characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_WIND_CHILL);

	wind_chill=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  wind_chill_read, NULL, NULL);

	wind_chill_db->wind_chill_handle = gatt_db_attribute_get_handle(wind_chill);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       wind_chill_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       wind_chill_triger_read_cb, wind_chill_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       wind_chill_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       wind_chill_char_user_desc_read_cb,wind_chill_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       wind_chill_msrmt_ccc_read_cb,
				       wind_chill_msrmt_ccc_write_cb, conn);

	/*  Adding Barometric Pressure characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_BAROMETRIC_PRESSURE_TREND);
	
	gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ ,
						  barometric_pressure_read, NULL, NULL);


	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       barometric_pressure_measurement_read_cb, NULL, NULL);


	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       barometric_pressure_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       barometric_pressure_char_user_desc_read_cb,barometric_pressure_char_user_desc_write_cb, NULL);

	/*  Adding Magnetic Declination characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_MAGNETIC_DECLINATION);

	magnetic_declination=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  magnetic_declination_read, NULL, NULL);

	magnetic_declination_db->magnetic_declination_handle = gatt_db_attribute_get_handle(magnetic_declination);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       magnetic_declination_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       magnetic_declination_triger_read_cb, magnetic_declination_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       magnetic_declination_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       magnetic_declination_char_user_desc_read_cb,magnetic_declination_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       magnetic_declination_msrmt_ccc_read_cb,
				       magnetic_declination_msrmt_ccc_write_cb, conn);

	/*  Adding Magnetic flux 2D characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_MAGNETIC_FLUX_DENSITY_2D);

	magnetic_flux_2D=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  magnetic_flux_2D_read, NULL, NULL);

	magnetic_flux_2D_db->magnetic_flux_2D_handle = gatt_db_attribute_get_handle(magnetic_flux_2D);

	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       magnetic_flux_2D_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       magnetic_flux_2D_triger_read_cb, magnetic_flux_2D_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       magnetic_flux_2D_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       magnetic_flux_2D_char_user_desc_read_cb,magnetic_flux_2D_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       magnetic_flux_2D_msrmt_ccc_read_cb,
				       magnetic_flux_2D_msrmt_ccc_write_cb, conn);

	/*  Adding Magnetic flux 3D characteristics and it's descriptors  */

	bt_uuid16_create(&uuid, UUID_MAGNETIC_FLUX_DENSITY_3D);

	magnetic_flux_3D=gatt_db_service_add_characteristic(service, &uuid,
						  BT_ATT_PERM_READ,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY,
						  magnetic_flux_3D_read, NULL, NULL);

	magnetic_flux_3D_db->magnetic_flux_3D_handle = gatt_db_attribute_get_handle(magnetic_flux_3D);


	bt_uuid16_create(&uuid, ESS_MEASUREMENT_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       magnetic_flux_3D_measurement_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_TRIGER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       magnetic_flux_3D_triger_read_cb, magnetic_flux_3D_triger_write_cb,
				       NULL);

	bt_uuid16_create(&uuid, ESS_VALID_RANGE_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ,
				       magnetic_flux_3D_valid_range_read_cb, NULL, NULL);

	bt_uuid16_create(&uuid, ESS_CHAR_USER_DESC);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       magnetic_flux_3D_char_user_desc_read_cb,magnetic_flux_3D_char_user_desc_write_cb, NULL);

	bt_uuid16_create(&uuid, CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
				       BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
				       magnetic_flux_3D_msrmt_ccc_read_cb,
				       magnetic_flux_3D_msrmt_ccc_write_cb, conn);
	
	/* activating the ESS service all the characteristic and descriptor */

	gatt_db_service_set_active(service, true);

}

void gatt_server_start(void)
{
	struct sockaddr_l2 addr;

	if (att_fd >= 0)
		return;

	att_fd = socket(PF_BLUETOOTH, SOCK_SEQPACKET | SOCK_CLOEXEC,
			BTPROTO_L2CAP);
	if (att_fd < 0) {
		fprintf(stderr, "Failed to create ATT server socket: %m\n");
		return;
	}

	memset(&addr, 0, sizeof(addr));
	addr.l2_family = AF_BLUETOOTH;
	addr.l2_cid = htobs(ATT_CID);
	memcpy(&addr.l2_bdaddr, public_addr, 6);
	addr.l2_bdaddr_type = BDADDR_LE_PUBLIC;

	if (bind(att_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		fprintf(stderr, "Failed to bind ATT server socket: %m\n");
		close(att_fd);
		att_fd = -1;
		return;
	}

	if (listen(att_fd, 1) < 0) {
		fprintf(stderr, "Failed to listen on ATT server socket: %m\n");
		close(att_fd);
		att_fd = -1;
		return;
	}

	gatt_db = gatt_db_new();
	if (!gatt_db) {
		close(att_fd);
		att_fd = -1;
		return;
	}

	populate_environmental_service(gatt_db);
	populate_gap_service(gatt_db);
	populate_devinfo_service(gatt_db);

	gatt_cache = gatt_db_new();

	conn_list = queue_new();
	if (!conn_list) {
		gatt_db_unref(gatt_db);
		gatt_db = NULL;
		close(att_fd);
		att_fd = -1;
		return;
	}

	mainloop_add_fd(att_fd, EPOLLIN, att_conn_callback, NULL, NULL);
}

void gatt_server_stop(void)
{
	if (att_fd < 0)
		return;

	mainloop_remove_fd(att_fd);

	queue_destroy(conn_list, gatt_conn_destroy);

	gatt_db_unref(gatt_cache);
	gatt_cache = NULL;

	gatt_db_unref(gatt_db);
	gatt_db = NULL;

	close(att_fd);
	att_fd = -1;
}
