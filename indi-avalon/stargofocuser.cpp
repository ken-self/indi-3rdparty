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
/*
    baseDevice = defaultDevice;
    deviceName = name;
    focuserActivated = false;
*/
}

/***************************************************************************
 * @brief Initialize the focuser UI controls
 * @param groupName tab where the UI controls are grouped
  ***************************************************************************/
//void StarGoFocuser::initProperties(const char *groupName)
bool StarGoFocuser::initProperties()
{
//    INDI::FocuserInterface::initProperties(groupName);
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
/*    if (isConnected()) {
        baseDevice->defineProperty(&FocusSpeedNP);
        baseDevice->defineProperty(&FocusMotionSP);
        baseDevice->defineProperty(&FocusTimerNP);
        baseDevice->defineProperty(&FocusAbsPosNP);
        baseDevice->defineProperty(&FocusRelPosNP);
        baseDevice->defineProperty(&FocusAbortSP);
        baseDevice->defineProperty(&FocusSyncNP);
        baseDevice->defineProperty(&FocusReverseSP);
    }
    else {
        baseDevice->deleteProperty(FocusSpeedNP.name);
        baseDevice->deleteProperty(FocusMotionSP.name);
        baseDevice->deleteProperty(FocusTimerNP.name);
        baseDevice->deleteProperty(FocusAbsPosNP.name);
        baseDevice->deleteProperty(FocusRelPosNP.name);
        baseDevice->deleteProperty(FocusAbortSP.name);
        baseDevice->deleteProperty(FocusSyncNP.name);
        baseDevice->deleteProperty(FocusReverseSP.name);
    }
*/
    FI::updateProperties();
    return true;

}

/***************************************************************************
 *
bool StarGoFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    INDI_UNUSED(states);
    INDI_UNUSED(names);
    INDI_UNUSED(n);

    //  first check if it's for our device
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {

        if (!strcmp(name, FocusMotionSP.name))
        {
            return changeFocusMotion(states, names, n);
        }
        else if (!strcmp(name, FocusAbortSP.name))
        {
            return changeFocusAbort(states, names, n);
        }
        else if (!strcmp(name, FocusReverseSP.name))
        {
            return setFocuserDirection(states, names, n);
        }
    }

    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    INDI_UNUSED(values);
    INDI_UNUSED(names);
    INDI_UNUSED(n);

    //  first check if it's for our device
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(name, FocusSpeedNP.name))
        {
            return changeFocusSpeed(values, names, n);
        } else if (!strcmp(name, FocusTimerNP.name))
        {
            return changeFocusTimer(values, names, n);
        } else if (!strcmp(name, FocusAbsPosNP.name))
        {
            return changeFocusAbsPos(values, names, n);
        } else if (!strcmp(name, FocusRelPosNP.name))
        {
            return changeFocusRelPos(values, names, n);
        } else if (!strcmp(name, FocusSyncNP.name))
        {
            return changeFocusSyncPos(values, names, n);
        }
    }

    return true;
}
 ***************************************************************************/

/***************************************************************************
 *
bool StarGoFocuser::changeFocusTimer(double values[], char* names[], int n) {
    int time = static_cast<int>(values[0]);
    if (validateFocusTimer(time)) {
        IUUpdateNumber(&FocusTimerNP, values, names, n);
        FocusTimerNP.s = MoveFocuser(FocusMotionS[0].s == ISS_ON ? FOCUS_INWARD : FOCUS_OUTWARD,
                static_cast<int>(FocusSpeedN[0].value), static_cast<uint16_t>(FocusTimerN[0].value));
        IDSetNumber(&FocusTimerNP, nullptr);
    }
    return true;
}
 ***************************************************************************/


/***************************************************************************
 * 
bool StarGoFocuser::changeFocusMotion(ISState* states, char* names[], int n) {
    IUUpdateSwitch(&FocusMotionSP, states, names, n);
    FocusMotionSP.s = IPS_OK;
    IDSetSwitch(&FocusMotionSP, nullptr);
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::changeFocusAbsPos(double values[], char* names[], int n) {
    uint32_t absolutePosition = static_cast<uint32_t>(values[0]);
    if (validateFocusAbsPos(absolutePosition)) {
        double currentPosition = FocusAbsPosN[0].value;
        IUUpdateNumber(&FocusAbsPosNP, values, names, n);
        // After updating the property the current position is temporarily reset to
        // the target position, I personally didn't like that so I am going to have
        // it only display the last known focuser position
        FocusAbsPosN[0].value = currentPosition;
        FocusAbsPosNP.s = MoveAbsFocuser(absolutePosition);
        IDSetNumber(&FocusAbsPosNP, nullptr);
    }
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::changeFocusRelPos(double values[], char* names[], int n) {
    int relativePosition = static_cast<int>(values[0]);
    if (validateFocusRelPos(relativePosition)) {
        IUUpdateNumber(&FocusRelPosNP, values, names, n);
        FocusRelPosNP.s = moveFocuserRelative(relativePosition);
        IDSetNumber(&FocusRelPosNP, nullptr);
        // reflect the relative position status to the absolute position
        FocusAbsPosNP.s = FocusRelPosNP.s;
        IDSetNumber(&FocusAbsPosNP, nullptr);
    }
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::changeFocusSpeed(double values[], char* names[], int n) {
    int speed = static_cast<int>(values[0]);
    if (validateFocusSpeed(speed)) {
        IUUpdateNumber(&FocusSpeedNP, values, names, n);
        FocusSpeedNP.s = SetFocuserSpeed(speed) ? IPS_OK : IPS_ALERT;

        IDSetNumber(&FocusSpeedNP, nullptr);
    }
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::setFocuserDirection(ISState* states, char* names[], int n) {

    if (IUUpdateSwitch(&FocusReverseSP, states, names, n) < 0)
        return false;

    focuserReversed = (IUFindOnSwitchIndex(&FocusReverseSP) == 0 ? INDI_ENABLED : INDI_DISABLED);

    FocusReverseSP.s = IPS_OK;
    IDSetSwitch(&FocusReverseSP, nullptr);

    return true;
}
 ***************************************************************************/


/***************************************************************************
 * 
bool StarGoFocuser::changeFocusAbort(ISState* states, char* names[], int n) {
    INDI_UNUSED(states);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
    IUResetSwitch(&FocusAbortSP);
    FocusAbortSP.s = AbortFocuser() ? IPS_OK : IPS_ALERT;
    FocusAbsPosNP.s = IPS_OK;
    IDSetNumber(&FocusAbsPosNP, nullptr);
    FocusRelPosNP.s = IPS_OK;
    IDSetNumber(&FocusRelPosNP, nullptr);
    IDSetSwitch(&FocusAbortSP, nullptr);
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::changeFocusSyncPos(double values[], char* names[], int n) {
    int absolutePosition = static_cast<int>(values[0]);
    if (validateFocusSyncPos(absolutePosition)) {
        IUUpdateNumber(&FocusSyncNP, values, names, n);
        FocusSyncNP.s = syncFocuser(absolutePosition);
        IDSetNumber(&FocusSyncNP, nullptr);
    }
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::validateFocusSpeed(int speed) {
    int minSpeed = static_cast<int>(FocusSpeedN[0].min);
    int maxSpeed = static_cast<int>(FocusSpeedN[0].max);
    if (speed < minSpeed || speed > maxSpeed) {
        DEBUGF(INDI::Logger::DBG_ERROR, "%s: Cannot set focuser speed to %d, it is outside the valid range of [%d, %d]", getDeviceName(), speed, minSpeed, maxSpeed);
        return false;
    }
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::validateFocusTimer(int time) {
    int minTime = static_cast<int>(FocusTimerN[0].min);
    int maxTime = static_cast<int>(FocusTimerN[0].max);
    if (time < minTime || time > maxTime) {
        DEBUGF(INDI::Logger::DBG_ERROR, "%s: Cannot set focuser timer to %d, it is outside the valid range of [%d, %d]", getDeviceName(), time, minTime, maxTime);
        return false;
    }
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::validateFocusAbsPos(uint32_t absolutePosition) {
    uint32_t minPosition = static_cast<uint32_t>(FocusAbsPosN[0].min);
    uint32_t maxPosition = static_cast<uint32_t>(FocusAbsPosN[0].max);
    if (absolutePosition < minPosition || absolutePosition > maxPosition) {
        DEBUGF(INDI::Logger::DBG_ERROR, "%s: Cannot set focuser absolute position to %d, it is outside the valid range of [%d, %d]", getDeviceName(), absolutePosition, minPosition, maxPosition);
        return false;
    }
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::validateFocusRelPos(int relativePosition) {
    int minRelativePosition = static_cast<int>(FocusRelPosN[0].min);
    int maxRelativePosition = static_cast<int>(FocusRelPosN[0].max);
    if (relativePosition < minRelativePosition || relativePosition > maxRelativePosition) {
        DEBUGF(INDI::Logger::DBG_ERROR, "%s: Cannot set focuser relative position to %d, it is outside the valid range of [%d, %d]", getDeviceName(), relativePosition, minRelativePosition, maxRelativePosition);
        return false;
    }
    uint32_t absolutePosition = getAbsoluteFocuserPositionFromRelative(relativePosition);
    return validateFocusAbsPos(absolutePosition);
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::validateFocusSyncPos(int absolutePosition) {
    int minPosition = static_cast<int>(FocusAbsPosN[0].min);
    int maxPosition = static_cast<int>(FocusAbsPosN[0].max);
    if (absolutePosition < minPosition || absolutePosition > maxPosition) {
        DEBUGF(INDI::Logger::DBG_ERROR, "%s: Cannot sync focuser to position %d, it is outside the valid range of [%d, %d]", getDeviceName(), absolutePosition, minPosition, maxPosition);
        return false;
    }
    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
uint32_t StarGoFocuser::getAbsoluteFocuserPositionFromRelative(int relativePosition) {
    bool inward = FocusMotionS[0].s == ISS_ON;
    if (inward) {
        relativePosition *= -1;
    }
    return static_cast<uint32_t>(FocusAbsPosN[0].value + relativePosition);
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::ReadFocuserStatus() {
    // do nothing if not active
    if (!isConnected())
        return true;

    int absolutePosition = 0;
    if (sendQueryFocuserPosition(&absolutePosition)) {
        FocusAbsPosN[0].value = (focuserReversed == INDI_DISABLED) ? absolutePosition : -absolutePosition;
        IDSetNumber(&FocusAbsPosNP, nullptr);
    }
    else
        return false;

    if (isFocuserMoving() && atFocuserTargetPosition()) {
        FocusAbsPosNP.s = IPS_OK;
        IDSetNumber(&FocusAbsPosNP, nullptr);
        FocusRelPosNP.s = IPS_OK;
        IDSetNumber(&FocusRelPosNP, nullptr);
    }

    return true;
}
 ***************************************************************************/

/***************************************************************************
 * 
 ***************************************************************************/
bool StarGoFocuser::SetFocuserSpeed(int speed)
{
//    return sendNewFocuserSpeed(speed);
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
IPState StarGoFocuser::MoveFocuser(FocusDirection dir, int speed, uint16_t duration) {
    INDI_UNUSED(speed);
    if (duration == 0) {
        return IPS_OK;
    }
    uint32_t position = static_cast<uint32_t>(FocusAbsPosN[0].min);
    if (dir == FOCUS_INWARD) {
        position = static_cast<uint32_t>(FocusAbsPosN[0].max);
    }
    moveFocuserDurationRemaining = duration;
    
/*
    bool result = sendMoveFocuserToPosition(position);
    if (!result) {
        return IPS_ALERT;
    }
    return IPS_BUSY;
*/
    return MoveAbsFocuser(position);
}

/***************************************************************************
 * 
 ***************************************************************************/
IPState StarGoFocuser::MoveAbsFocuser(uint32_t position)
{
/*
    bool result = sendMoveFocuserToPosition(absolutePosition);
    if (!result) {
        return IPS_ALERT;
    }
    return IPS_BUSY;
*/
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
/*
IPState StarGoFocuser::moveFocuserRelative(int relativePosition) {
    if (relativePosition == 0) {
        return IPS_OK;
    }
    uint32_t absolutePosition = getAbsoluteFocuserPositionFromRelative(relativePosition);
    return MoveAbsFocuser(absolutePosition);
*/
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
//    return sendAbortFocuser();
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
/*
IPState StarGoFocuser::syncFocuser(int absolutePosition) 
{
    bool result = sendSyncFocuserToPosition(absolutePosition);
    if (!result) {
        return IPS_ALERT;
    }
    return IPS_OK;
}
*/

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
bool StarGoFocuser::isConnected() {
    if (baseDevice == nullptr) return false;
    return focuserActivated;
}
 ***************************************************************************/

/***************************************************************************
 * 
 ***************************************************************************/
const char *StarGoFocuser::getDeviceName() {
//    if (baseDevice == nullptr) return "";
//    return baseDevice->getDeviceName();
    if (m_device == nullptr) return "";
    return m_device->getDeviceName();
}

/***************************************************************************
 * 
const char *StarGoFocuser::getDefaultName()
{
    return deviceName;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::activate(bool activate)
{
    bool result = true;
    if (activate == true && focuserActivated == false)
    {
        initProperties(deviceName);
        focuserActivated = activate;
        result = updateProperties();
    }
    else if (activate == false)
    {
        focuserActivated = activate;
        result = updateProperties();
    }
    return result;
}
 ***************************************************************************/

/***************************************************************************
 * 
bool StarGoFocuser::saveConfigItems(FILE *fp)
{
    if (focuserActivated)
    {
        IUSaveConfigSwitch(fp, &FocusReverseSP);
        IUSaveConfigNumber(fp, &FocusSpeedNP);
    }

    return true;
}
 ***************************************************************************/

/***************************************************************************
 * LX200 queries, sent to baseDevice
 ***************************************************************************/

/***************************************************************************
 *
bool StarGoFocuser::sendNewFocuserSpeed(int speed) {
    // Command  - :X1Caaaa*bb#
    // Response - Unknown
    bool valid = false;
    switch(speed) {
    case 1: valid = baseDevice->transmit(":X1C9000*01#"); break;
    case 2: valid = baseDevice->transmit(":X1C6000*01#"); break;
    case 3: valid = baseDevice->transmit(":X1C4000*01#"); break;
    case 4: valid = baseDevice->transmit(":X1C2500*01#"); break;
    case 5: valid = baseDevice->transmit(":X1C1000*05#"); break;
    case 6: valid = baseDevice->transmit(":X1C0750*10#"); break;
    case 7: valid = baseDevice->transmit(":X1C0500*20#"); break;
    case 8: valid = baseDevice->transmit(":X1C0250*30#"); break;
    case 9: valid = baseDevice->transmit(":X1C0100*40#"); break;
    case 10: valid = baseDevice->transmit(":X1C0060*50#"); break;
    default: DEBUGF(INDI::Logger::DBG_ERROR, "%s: Invalid focuser speed %d specified.", getDeviceName(), speed);
    }
    if (!valid) {
        DEBUGF(INDI::Logger::DBG_ERROR, "%s: Failed to send new focuser speed command.", getDeviceName());
        return false;
    }
    return valid;
}
 ***************************************************************************/

/***************************************************************************
 *
 ***************************************************************************/
//bool StarGoFocuser::sendSyncFocuserToPosition(int position) {
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
//bool StarGoFocuser::sendQueryFocuserPosition(int* position) {
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

/***************************************************************************
 *
bool StarGoFocuser::sendMoveFocuserToPosition(uint32_t position) {
    // Command  - :X16pppppp#
    // Response - Nothing
    targetFocuserPosition = (focuserReversed == INDI_DISABLED) ? position : -position;
    char command[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    sprintf(command, ":X16%06d#", AVALON_FOCUSER_POSITION_OFFSET + targetFocuserPosition);
    if (!baseDevice->transmit(command)) {
        LOGF_ERROR("%s: Failed to send AUX1 goto command.", getDeviceName());
        return false;
    }
    return true;
}
 ***************************************************************************/

/***************************************************************************
 *
bool StarGoFocuser::sendAbortFocuser() {
    // Command  - :X0AAUX1ST#
    // Response - Nothing
    if (!baseDevice->transmit(":X0AAUX1ST#")) {
        DEBUGF(INDI::Logger::DBG_ERROR, "%s: Failed to send AUX1 stop command.", getDeviceName());
        return false;
    }
    return true;
}
 ***************************************************************************/

/************************************************************************
 * helper functions
 ************************************************************************/

/***************************************************************************
 *
bool StarGoFocuser::isFocuserMoving() {
    return FocusAbsPosNP.s == IPS_BUSY || FocusRelPosNP.s == IPS_BUSY;
}
 ***************************************************************************/

/***************************************************************************
 *
bool StarGoFocuser::atFocuserTargetPosition() {
    return static_cast<uint32_t>(FocusAbsPosN[0].value) == (focuserReversed == INDI_DISABLED) ? targetFocuserPosition : -targetFocuserPosition;
}
 ***************************************************************************/

/************************************************************************
 * 
 ************************************************************************/
bool StarGoFocuser::sendQuery(const char* cmd, char* response, char end, int wait)
{
    return m_device->sendQuery(cmd, response, end, wait);
}
