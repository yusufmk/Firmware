/****************************************************************************
 *
 ****************************************************************************/

/**
 * @file RedundancyManager.h
 * redundancy manager helper functions definitions
 *
 * @author M.Yusuf Korkut <mkorkut@thy.com>
 */

#ifndef REDUNDANCY_MANAGER_H_
#define REDUNDANCY_MANAGER_H_

// #include <drivers/drv_hrt.h>

// UORB Topic'leri buraya eklencek
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>


bool anyArmed(arming_state_t armingState);
bool anyStandby(arming_state_t armingState);
bool anyStandbyError(arming_state_t armingState);

bool anyOp(arming_state_t armingState);
bool anyMon(arming_state_t armingState);


#endif /* REDUNDANCY_MANAGER_H_ */
