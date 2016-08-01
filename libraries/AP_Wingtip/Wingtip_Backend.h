// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  I have no idea what I'm doing...
 */

#pragma once

#include "AP_Wingtip.h"

class AP_Wingtip;
class AP_Wingtip_Backend
{
public:
    AP_Wingtip_Backend(AP_Wingtip &wingtip);

    virtual ~AP_Wingtip_Backend(void) {}

    virtual bool init_device(void);// = 0;

    virtual void collect(void);// = 0;

protected:
    // access to frontend
    AP_Wingtip &_wingtip;

//private:


};
