/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Wingtip.h"
#include "Wingtip_Backend.h"

// Base class constructor

AP_Wingtip_Backend::AP_Wingtip_Backend(AP_Wingtip &_wingtip, uint8_t instance, AP_Wingtip::Wingtip_State &_state)
    : wingtip(_wingtip)
    , state(_state)
{
}
