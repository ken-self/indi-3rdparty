#ifndef STARGO_SYSTEM_H
#define STARGO_SYSTEM_H

/*
    Avalon StarGo System
    Copyright (C) 2018 Christopher Contaxis (chrconta@gmail.com) and
    Wolfgang Reissenberger (sterne-jaeger@t-online.de)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "stargo.h"
#include "stargofocuser.h"
class StarGoFocuser;
class StarGoSystem : public StarGoTelescope
{
public:
    // AUX focusers control
    ISwitchVectorProperty Aux1FocuserSP;
    ISwitch Aux1FocuserS[2];

    StarGoSystem();
    virtual ~StarGoSystem() = default;
//protected:
    bool initProperties() override;
    bool updateProperties() override;
    bool ReadScopeStatus() override;
    bool saveConfigItems(FILE *fp) override;

    bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    bool sendQuery(const char* cmd, char* response, char end, int wait=AVALON_TIMEOUT);
    bool sendQuery(const char* cmd, char* response, int wait=AVALON_TIMEOUT);
    
    StarGoFocuser* m_focuser;
};
inline bool StarGoSystem::sendQuery(const char* cmd, char* response, int wait)
{
    return sendQuery(cmd, response, '#', wait);
}

#endif // STARGO_SYSTEM_H
