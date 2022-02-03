#include "hub.h"

#include "hl.hpp"

static Hub_Mode mode = SHARED_HUB_MODE_BASE;

void Hub_Set_Mode(Hub_Mode _mode) {
	mode = _mode;
}

void Hub_Interrupt(void) {
	switch(mode) {
	case SHARED_HUB_MODE_TRACKER:
		HL_Call_Remote<controller_report_measure>(0, 0, 0);
		return;
	default:
		return;
	}
}
