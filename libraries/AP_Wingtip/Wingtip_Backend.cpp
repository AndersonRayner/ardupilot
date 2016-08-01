/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#include "AP_Wingtip.h"
#include "Wingtip_Backend.h"

AP_Wingtip_Backend::AP_Wingtip_Backend(AP_Wingtip &wingtip) :
    _wingtip(wingtip)
{}

