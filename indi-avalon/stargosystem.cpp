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

#include "stargosystem.h"
#include <cstring>
#include <memory>


static class Loader
{
public:
    std::unique_ptr<StarGoSystem> device;
public:
    Loader()
    {
        device.reset(new StarGoSystem());
    }
} loader;

#define AVALON_FOCUSER_POSITION_OFFSET                  500000

/***************************************************************************
 * @brief Constructor
 * @param defaultDevice the telescope
 * @param name device name
 ***************************************************************************/
StarGoSystem::StarGoSystem()
{
    m_focuser =  new StarGoFocuser(this);
}

/***************************************************************************
 * @brief Initialize the focuser UI controls
 * @param groupName tab where the UI controls are grouped
 ***************************************************************************/
bool StarGoSystem::initProperties()
{
    IUFillSwitch(&Aux1FocuserS[DefaultDevice::INDI_ENABLED], "INDI_ENABLED", "Enabled", ISS_OFF);
    IUFillSwitch(&Aux1FocuserS[DefaultDevice::INDI_DISABLED], "INDI_DISABLED", "Disabled", ISS_ON);
    IUFillSwitchVector(&Aux1FocuserSP, Aux1FocuserS, 2, getDeviceName(), "AUX1_FOCUSER_CONTROL", "AUX1 Focuser",
                       MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    StarGoTelescope::initProperties();
    m_focuser->initProperties();

    return true;
}

/***************************************************************************
 * @brief Fill the UI controls with current values
 * @return true iff everything went fine
 ***************************************************************************/
bool StarGoSystem::updateProperties()
{
    if (isConnected())
    {
        defineProperty(&Aux1FocuserSP);
    }
    else
    {
        deleteProperty(Aux1FocuserSP.name);
    }
    StarGoTelescope::updateProperties();
    m_focuser->updateProperties();
    return true;
}

/***************************************************************************
 * Reaction to UI commands
 ***************************************************************************/
bool StarGoSystem::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    INDI_UNUSED(states);
    INDI_UNUSED(names);
    INDI_UNUSED(n);

    //  first check if it's for our device
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(name, Aux1FocuserSP.name))
        {
            if (IUUpdateSwitch(&Aux1FocuserSP, states, names, n) < 0)
                return false;
            bool activated = (IUFindOnSwitchIndex(&Aux1FocuserSP) == DefaultDevice::INDI_ENABLED);
            Aux1FocuserSP.s = activated ? IPS_OK : IPS_IDLE;
            IDSetSwitch(&Aux1FocuserSP, nullptr);
            return true;
        }
        bool activated = (IUFindOnSwitchIndex(&Aux1FocuserSP) == DefaultDevice::INDI_ENABLED);
        if (strstr(name, "FOCUS") && activated)
        {
            return m_focuser->processSwitch(dev, name, states, names, n);
        }
        return StarGoTelescope::ISNewSwitch(dev, name, states, names, n);
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoSystem::saveConfigItems(FILE *fp)
{
    LOG_DEBUG(__FUNCTION__);
    IUSaveConfigSwitch(fp, &Aux1FocuserSP);
    StarGoFocuser::saveConfigItems(fp);
    StarGoTelescope::saveConfigItems(fp);
    return true;
}

/***************************************************************************
 *
 ***************************************************************************/
bool StarGoSystem::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    INDI_UNUSED(values);
    INDI_UNUSED(names);
    INDI_UNUSED(n);

    //  first check if it's for our device
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        bool activated = (IUFindOnSwitchIndex(&Aux1FocuserSP) == DefaultDevice::INDI_ENABLED);
        if (strstr(name, "FOCUS") && activated)
        {
            return m_focuser->processNumber(dev, name, values, names, n);
        }
        return StarGoTelescope::ISNewNumber(dev, name, values, names, n);
    }
    return true;
}

bool StarGoSystem::sendQuery(const char* cmd, char* response, char end, int wait)
{
    return StarGoTelescope::sendQuery(cmd, response, end, wait);
}
bool StarGoSystem::ReadScopeStatus()
{
    if(! StarGoTelescope::ReadScopeStatus())
    {
            return false;
    }
    if(TrackState == SCOPE_SLEWING || TrackState == SCOPE_PARKING)
    {
        return true;  // While scope is slewing focuser cannot respond
    }
    bool activated = (IUFindOnSwitchIndex(&Aux1FocuserSP) == DefaultDevice::INDI_ENABLED);
    if (activated) return m_focuser->ReadStatus();
}

