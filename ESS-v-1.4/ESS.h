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

#include <stdint.h>


/* ess_measurement structure is measurement descriptor structure with all necessary fields */

struct ess_measurement {
	uint16_t flags;
	uint8_t sample;
	uint8_t m_period[3];  //measurement period
	uint8_t u_interval[3]; //update interval
	uint8_t applicatn;
	uint8_t m_uncertainity;
};

/* This trigger_setting holds the trigger setting  */

struct trigger_setting {
	uint8_t condition;
	uint8_t data[3]; // Time based data
	bool time_enable;
	bool value_enable;
	bool trigger_inactive;
};


/* This structure hold all the fields required for Temprature characteristic and its descriptors */
struct ess_temp {
	int16_t temp_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t temp_handle;
	int16_t tr_value;
	unsigned int timeout_id;
	bool temp_enable;
	int16_t lower_temp;
	int16_t upper_temp;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	struct ess_measurement *temp_ms;
	struct trigger_setting *temp_tr;

};

/* This structure hold all the fields required for Apparent Wind Speed characteristic and its descriptors */

struct ess_apparent_speed {
	uint16_t apparent_speed_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t apparent_speed_handle;
	uint16_t tr_value;
	unsigned int timeout_id;
	bool apparent_speed_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint16_t lower_speed;
	uint16_t upper_speed;
	struct ess_measurement *apparent_speed_ms;
	struct trigger_setting *apparent_speed_tr;
};

/* This structure hold all the fields required for Apparent Wind Direction characteristic and its descriptors */

struct ess_apparent_direction {
	uint16_t apparent_direction_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t apparent_direction_handle;
	uint16_t tr_value;
	unsigned int timeout_id;
	bool apparent_direction_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint16_t lower_direction;
	uint16_t upper_direction;
	struct ess_measurement *apparent_direction_ms;
	struct trigger_setting *apparent_direction_tr;
};

/* This structure hold all the fields required for Dew Point characteristic and its descriptors */

struct ess_dew_point {
	int8_t dew_point_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t dew_point_handle;
	int8_t tr_value;
	unsigned int timeout_id;
	bool dew_point_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	int8_t lower_dew_point;
	int8_t upper_dew_point;
	struct ess_measurement *dew_point_ms;
	struct trigger_setting *dew_point_tr;
};

/* This structure hold all the fields required for Elevation characteristic and its descriptors */

struct ess_elevation {
	int32_t elevation_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t elevation_handle;
	int32_t tr_value;
	unsigned int timeout_id;
	bool elevation_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	int8_t lower_elevation[3];
	int8_t upper_elevation[3];
	struct ess_measurement *elevation_ms;
	struct trigger_setting *elevation_tr;
};

/* This structure hold all the fields required for Gust Factor characteristic and its descriptors */

struct ess_gust_factor {
	uint8_t gust_factor_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t gust_factor_handle;
	uint8_t tr_value;
	unsigned int timeout_id;
	bool gust_factor_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint8_t lower_gust_factor;
	uint8_t upper_gust_factor;
	struct ess_measurement *gust_factor_ms;
	struct trigger_setting *gust_factor_tr;
};

/* This structure hold all the fields required for Heat Index characteristic and its descriptors */

struct ess_heat_index {
	int8_t heat_index_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t heat_index_handle;
	int8_t tr_value;
	unsigned int timeout_id;
	bool heat_index_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	int8_t lower_heat_index;
	int8_t upper_heat_index;
	struct ess_measurement *heat_index_ms;
	struct trigger_setting *heat_index_tr;
};

/* This structure hold all the fields required for Humidity characteristic and its descriptors */

struct ess_humidity {
	uint16_t humidity_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t humidity_handle;
	uint16_t tr_value;
	unsigned int timeout_id;
	bool humidity_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint16_t lower_humidity;
	uint16_t upper_humidity;
	struct ess_measurement *humidity_ms;
	struct trigger_setting *humidity_tr;
};

/* This structure hold all the fields required for Irradiance characteristic and its descriptors */

struct ess_irradiance {
	uint16_t irradiance_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t irradiance_handle;
	uint16_t tr_value;
	unsigned int timeout_id;
	bool irradiance_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint16_t lower_irradiance;
	uint16_t upper_irradiance;
	struct ess_measurement *irradiance_ms;
	struct trigger_setting *irradiance_tr;
};

/* This structure hold all the fields required for Pollen Concentration characteristic and its descriptors */

struct ess_pollen_concentration {
	uint32_t pollen_concentration_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t pollen_concentration_handle;
	uint32_t tr_value;
	unsigned int timeout_id;
	bool pollen_concentration_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint8_t lower_pollen_concentration[3];
	uint8_t upper_pollen_concentration[3];
	struct ess_measurement *pollen_concentration_ms;
	struct trigger_setting *pollen_concentration_tr;
};

/* This structure hold all the fields required for Rainfall characteristic and its descriptors */

struct ess_rainfall {
	uint16_t rainfall_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t rainfall_handle;
	uint16_t tr_value;
	unsigned int timeout_id;
	bool rainfall_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint16_t lower_rainfall;
	uint16_t upper_rainfall;
	struct ess_measurement *rainfall_ms;
	struct trigger_setting *rainfall_tr;
};

/* This structure hold all the fields required for Pressure characteristic and its descriptors */

struct ess_pressure {
	uint32_t pressure_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t pressure_handle;
	uint32_t tr_value;
	unsigned int timeout_id;
	bool pressure_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint32_t lower_pressure;
	uint32_t upper_pressure;
	struct ess_measurement *pressure_ms;
	struct trigger_setting *pressure_tr;
};

/* This structure hold all the fields required for True Wind Direction characteristic and its descriptors */

struct ess_true_direction {
	uint16_t true_direction_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t true_direction_handle;
	uint16_t tr_value;
	unsigned int timeout_id;
	bool true_direction_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint16_t lower_true_direction;
	uint16_t upper_true_direction;
	struct ess_measurement *true_direction_ms;
	struct trigger_setting *true_direction_tr;
};

/* This structure hold all the fields required for True Wind Speed characteristic and its descriptors */

struct ess_true_speed {
	uint16_t true_speed_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t true_speed_handle;
	uint16_t tr_value;
	unsigned int timeout_id;
	bool true_speed_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint16_t lower_true_speed;
	uint16_t upper_true_speed;
	struct ess_measurement *true_speed_ms;
	struct trigger_setting *true_speed_tr;
};

/* This structure hold all the fields required for UV Index characteristic and its descriptors */

struct ess_uv_index {
	uint8_t uv_index_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t uv_index_handle;
	uint8_t tr_value;
	unsigned int timeout_id;
	bool uv_index_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint8_t lower_uv_index;
	uint8_t upper_uv_index;
	struct ess_measurement *uv_index_ms;
	struct trigger_setting *uv_index_tr;
};

/* This structure hold all the fields required for Wind Chill characteristic and its descriptors */

struct ess_wind_chill {
	int8_t wind_chill_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t wind_chill_handle;
	int8_t tr_value;
	unsigned int timeout_id;
	bool wind_chill_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	int8_t lower_wind_chill;
	int8_t upper_wind_chill;
	struct ess_measurement *wind_chill_ms;
	struct trigger_setting *wind_chill_tr;
};

/* This structure hold all the fields required for Barometric pressure characteristic and its descriptors */

struct ess_barometric_pressure {
	uint8_t barometric_pressure_data;
	wchar_t *user_desc;
	uint8_t lower_barometric_pressure;
	uint8_t upper_barometric_pressure;
	struct ess_measurement *barometric_pressure_ms;
};

/* This structure hold all the fields required for Magnetic Declination characteristic and its descriptors */

struct ess_magnetic_declination {
	uint16_t magnetic_declination_data;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t magnetic_declination_handle;
	uint16_t tr_value;
	unsigned int timeout_id;
	bool magnetic_declination_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	uint16_t lower_magnetic_declination;
	uint16_t upper_magnetic_declination;
	struct ess_measurement *magnetic_declination_ms;
	struct trigger_setting *magnetic_declination_tr;
};

/* This structure hold all the fields required for Magnetic Flux 2D characteristic and its descriptors */

struct ess_magnetic_flux_2D {
	uint16_t magnetic_flux_2D_xdata;
	uint16_t magnetic_flux_2D_ydata;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t magnetic_flux_2D_handle;
	uint16_t tr_valuex;
	uint16_t tr_valuey;
	unsigned int timeout_id;
	bool magnetic_flux_2D_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;
	int16_t xlower_magnetic_flux_2D;
	int16_t xupper_magnetic_flux_2D;
	int16_t ylower_magnetic_flux_2D;
	int16_t yupper_magnetic_flux_2D;
	struct ess_measurement *magnetic_flux_2D_ms;
	struct trigger_setting *magnetic_flux_2D_tr;
};

/* This structure hold all the fields required for Magnetic Flux 3D characteristic and its descriptors */

struct ess_magnetic_flux_3D {
	uint16_t magnetic_flux_3D_xdata;
	uint16_t magnetic_flux_3D_ydata;
	uint16_t magnetic_flux_3D_zdata;
	wchar_t *user_desc;
	uint16_t indication;
	uint16_t magnetic_flux_3D_handle;
	uint16_t tr_valuex;
	uint16_t tr_valuey;
	uint16_t tr_valuez;
	unsigned int timeout_id;
	bool magnetic_flux_3D_enable;
	uint32_t g_update_timer;
	uint32_t g_current_timer;	
	int16_t xlower_magnetic_flux_3D;
	int16_t xupper_magnetic_flux_3D;
	int16_t ylower_magnetic_flux_3D;
	int16_t yupper_magnetic_flux_3D;
	int16_t zlower_magnetic_flux_3D;
	int16_t zupper_magnetic_flux_3D;
	struct ess_measurement *magnetic_flux_3D_ms;
	struct trigger_setting *magnetic_flux_3D_tr;
};


struct gatt_conn {
	struct bt_att *att;
	struct bt_gatt_server *gatt;
	struct bt_gatt_client *client;
};

void gatt_set_public_address(uint8_t addr[6]);
void gatt_set_device_name(uint8_t name[20], uint8_t len);

void gatt_server_start(void);
void gatt_server_stop(void);
