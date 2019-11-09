#ifndef STARGO_FOCUSER_H
#define STARGO_FOCUSER_H

/*
    Avalon StarGo Focusser
    Copyright (C) 2018 Christopher Contaxis (chrconta@gmail.com) and
    Wolfgang Reissenberger (sterne-jaeger@t-online.de) and
    Ken Self (ken.kgself@gmail.com)

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

class StarGoSystem;
#include "stargosystem.h"

#include "indifocuserinterface.h"
#include "defaultdevice.h"

class StarGoFocuser : public INDI::FocuserInterface
{
    friend class StarGoSystem;
public:
    StarGoFocuser(StarGoSystem* dev);
    virtual ~StarGoFocuser() = default;

    bool initProperties();
    bool updateProperties();
//    bool ReadScopeStatus() override;

    bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
    bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);

protected:
    StarGoSystem* m_device;
    bool SetFocuserSpeed(int speed) override;
    IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration) override;
    IPState MoveAbsFocuser(uint32_t absolutePosition) override;
    IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
//    bool ReverseFocuser(bool enabled) override;
    bool AbortFocuser() override;
    bool SyncFocuser(uint32_t ticks) override;
//    bool SetFocuserMaxPosition(uint32_t ticks) override;

    int targetFocuserPosition;
    bool startMovingFocuserInward;
    bool startMovingFocuserOutward;
    uint32_t moveFocuserDurationRemaining;

    // Stargo commands
    bool getFocuserPosition(int* position);

    bool sendQuery(const char* cmd, char* response, char end, int wait=AVALON_TIMEOUT);
    bool sendQuery(const char* cmd, char* response, int wait=AVALON_TIMEOUT);
    const char* getDeviceName(); 
};
inline bool StarGoFocuser::sendQuery(const char* cmd, char* response, int wait)
{
    return sendQuery(cmd, response, '#', wait);
}

#endif // STARGO_FOCUSER_H
