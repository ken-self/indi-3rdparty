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

// Unique pointers
static std::unique_ptr<StarGoSystem> device;

void ISInit()
{
    static int isInit = 0;

    if (isInit)
        return;

    isInit = 1;
    if (device.get() == nullptr)
    {
        StarGoSystem* myDevice = new StarGoSystem();
        device.reset(myDevice);
    }
}

void ISGetProperties(const char *dev)
{
    ISInit();
    device->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    ISInit();
    device->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    ISInit();
    device->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    ISInit();
    device->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}
void ISSnoopDevice(XMLEle *root)
{
    ISInit();
    device->ISSnoopDevice(root);
}

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
        if (strstr(name, "FOCUS"))
        {
            return m_focuser->processSwitch(dev, name, states, names, n);
        }
        return StarGoTelescope::ISNewSwitch(dev, name, states, names, n);
    }
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
        if (strstr(name, "FOCUS"))
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
    return m_focuser->ReadStatus();
}

