#include "hl.hpp"

extern "C" {

#include <order/hub.h>

#include "hl.h"
#include "motion.h"

} // extern "C"

static void Hub_Interrupt();

static Shared_Hub_Mode mode = SHARED_HUB_MODE_BASE;
static union {
	struct {
		Motion_Tick last_left_tick, last_right_tick;
	} tracker;
} hub_state;

//
// Order implementations
//

extern "C" void Hub_Set_Mode(Shared_Hub_Mode _mode) {
	mode = _mode;
	switch(mode) {
	case SHARED_HUB_MODE_BASE:
		hub_state.tracker = {
				.last_left_tick = Motion_Get_Left_Ticks(),
				.last_right_tick = Motion_Get_Right_Ticks()
		};
		break;
	default:
		break;
	}
}

//
// Interrupt routines
//

extern "C" void HAL_IncTick() {
	uwTick++;
	Hub_Interrupt();
}

static void Hub_Interrupt() {
	static uint32_t last_invokation_date = HAL_GetTick();

	switch(mode) {
	case SHARED_HUB_MODE_TRACKER:
		if (HAL_GetTick() <= last_invokation_date + 10) return;
		if (hub_state.tracker.last_left_tick != Motion_Get_Left_Ticks() || hub_state.tracker.last_right_tick != Motion_Get_Right_Ticks()) {
			HL_Call_Remote<Controller_Report_Measure>(HAL_GetTick(), Motion_Get_Left_Ticks(), Motion_Get_Right_Ticks());
			hub_state.tracker.last_left_tick = Motion_Get_Left_Ticks();
			hub_state.tracker.last_right_tick = Motion_Get_Right_Ticks();
		}
		break;
	default:
		return;
	}

	last_invokation_date = HAL_GetTick();
}
