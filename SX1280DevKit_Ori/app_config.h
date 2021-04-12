#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/*
 * This file is inteded to be used for tuning the application at compile time
 * by activating/deactivating feature
 *
 * To activate a feature, ensure the corresponding preprocessor token
 * definition is not commented out.
 * To deactivate it, comment it out.
 */

/*
 * GPS feature.
 * Need a ubloc device soldered on the PCB
 */
//#define HAS_GPS_SENSOR

/*
 * Proximity sensor feature
 */
//#define HAS_PROXIMITY_SENSOR

#endif  // __APP_CONFIG_H__

