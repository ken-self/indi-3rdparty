/*
    Avalon Star GO Focuser
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

#include "stargofocuser.h"
#include <cstring>

#define AVALON_FOCUSER_POSITION_OFFSET                  500000

/***************************************************************************
 * @brief Constructor
 * @param defaultDevice the telescope
 * @param name device name
 ***************************************************************************/
//StarGoFocuser::StarGoFocuser(StarGoTelescope* defaultDevice, const char *name) : INDI::FocuserInterface(defaultDevice)
StarGoFocuser::StarGoFocuser(StarGoSystem *dev) : FI(dev), m_device(dev)
{
}

/***************************************************************************
 * @brief Initialize the focuser UI controls
 * @param groupName tab where the UI controls are grouped
  ***************************************************************************/
bool StarGoFocuser::initProperties()
{
    FI::initProperties(FOCUS_TAB);

    FI::SetCapability(
        FOCUSER_CAN_ABS_MOVE        | /*!< Can the focuser move by absolute position? */
        FOCUSER_CAN_REL_MOVE        | /*!< Can the focuser move by relative position? */
        FOCUSER_CAN_ABORT           | /*!< Is it possible to abort focuser motion? */
        FOCUSER_CAN_REVERSE         | /*!< Is it possible to reverse focuser motion? */
        FOCUSER_CAN_SYNC            | /*!< Can the focuser sync to a custom position */
        FOCUSER_HAS_VARIABLE_SPEED    /*!< Can the focuser move in different configurable speeds? */
    );

    // set default values
    FocusAbsPosN[0].min = 0.0;
    FocusAbsPosN[0].max = 100000.0;
    FocusAbsPosN[0].step = 1000.0;
    FocusRelPosN[0].step = 1000.0;
    FocusSyncN[0].step = 1000.0;
    
    FocusSpeedN[0].min = 0.0;
    FocusSpeedN[0].max = 10.0;
    FocusSpeedN[0].step  = 1.0;
    FocusSpeedN[0].value = 1.0;

    return true;
}

/***************************************************************************
 * @brief Fill the UI controls with current values
 * @return true iff everything went fine
 ***************************************************************************/
bool StarGoFocuser::updateProperties()
{
    FI::updateProperties();
    return true;

}

/***************************************************************************
 * 
 ***************************************************************************/
bool StarGoFocuser::SetFocuserSpeed(int speed)
{
    // Command  - :X1Caaaa*bb#
    // Response - 0#
    bool valid = false;
    unsigned int param[10][2] = 
    {
        {9000,1},
        {6000,1},
        {4000,1},
        {2500,1},
        {1000,5},
        {750,10},
        {500,20},
        {250,30},
        {100,40},
        {60,50}
    };
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    sprintf(cmd, ":X1C%4d*%2d#", param[speed-1][0],param[speed-1][1]);
    if (!sendQuery(cmd, response))
    {
        LOGF_ERROR("Failed to send new focuser speed command %s", cmd);
        return false;
    }
    return valid;
}

/***************************************************************************
 * 
 ***************************************************************************/
IPState StarGoFocuser::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    INDI_UNUSED(speed);
    if (duration == 0) {
        return IPS_OK;
    }
    uint32_t position = static_cast<uint32_t>(FocusAbsPosN[0].min);
    if (dir == FOCUS_INWARD) {
        position = static_cast<uint32_t>(FocusAbsPosN[0].max);
    }
//    moveFocuserDurationRemaining = duration;

    return MoveAbsFocuser(position);
}

/***************************************************************************
 * 
 ***************************************************************************/
IPState StarGoFocuser::MoveAbsFocuser(uint32_t position)
{
    // Command  - :X16pppppp#
    // Response - Nothing
    bool result = true;
    targetFocuserPosition = position;
    char command[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    sprintf(command, ":X16%06d#", AVALON_FOCUSER_POSITION_OFFSET + targetFocuserPosition);
    if ((result = sendQuery(command, response,0)) )
    {
        LOGF_ERROR("Failed to send AUX1 goto command $s", command);
    }
    return result? IPS_BUSY: IPS_ALERT;
}

/***************************************************************************
 * 
 ***************************************************************************/
IPState StarGoFocuser::MoveRelFocuser(FocusDirection dir, uint32_t relativePosition)
{
    uint32_t absolutePosition = relativePosition * (dir==FOCUS_INWARD?-1:+1);
    return MoveAbsFocuser(absolutePosition);
}

/***************************************************************************
 * 
 ***************************************************************************/
bool StarGoFocuser::AbortFocuser()
{
    // Command  - :X0AAUX1ST#
    // Response - Nothing
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":X0AAUX1ST#", response, 0)) 
    {
        LOG_ERROR("Failed to send AUX1 stop command." );
        return false;
    }
    return true;
}

/***************************************************************************
 * 
 ***************************************************************************/
bool StarGoFocuser::ReadStatus()
{
    int position;
    if(! getFocuserPosition(&position) )
    {
        return false;
    }
    FocusAbsPosN[0].value = position;
    IDSetNumber(&FocusSpeedNP, nullptr);
    return true;
}

/***************************************************************************
 * 
 ***************************************************************************/
const char *StarGoFocuser::getDeviceName()
{
//    if (baseDevice == nullptr) return "";
//    return baseDevice->getDeviceName();
    if (m_device == nullptr) return "";
    return m_device->getDeviceName();
}

/***************************************************************************
 *
 ***************************************************************************/
bool StarGoFocuser::SyncFocuser(uint32_t  position)
{
    // Command  - :X0Cpppppp#
    // Response - Nothing
    char response[AVALON_RESPONSE_BUFFER_LENGTH]={0};
    char command[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    sprintf(command, ":X0C%06d#", AVALON_FOCUSER_POSITION_OFFSET + position);
    if (!sendQuery(command, response,0)) 
    {
        LOG_ERROR("Failed to send AUX1 sync command.");
        return false;
    }
    return true;
}

/***************************************************************************
 *
 ***************************************************************************/
bool StarGoFocuser::getFocuserPosition(int* position) 
{
    // Command  - :X0BAUX1AS#
    // Response - AX1=ppppppp #
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if(!sendQuery(":X0BAUX1AS#", response)) 
    {
        LOG_ERROR("Failed to get AUX1 position request.");
        return false;
    }
    int tempPosition = 0;
    int returnCode = sscanf(response, "%*c%*c%*c%*c%07d", &tempPosition);
    if (returnCode <= 0) 
    {
        LOGF_ERROR("Failed to parse AUX1 position response '%s'.", response);
        return false;
    }
    (*position) = (tempPosition - AVALON_FOCUSER_POSITION_OFFSET);
    return true;
}

/************************************************************************
 * 
 ************************************************************************/
bool StarGoFocuser::sendQuery(const char* cmd, char* response, char end, int wait)
{
    return m_device->sendQuery(cmd, response, end, wait);
}
