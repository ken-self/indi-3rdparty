/*
    Avalon StarGo driver

    Copyright (C) 2019 Christopher Contaxis, Wolfgang Reissenberger,
    Ken Self and Tonino Tasselli

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

#include <cmath>
#include <memory>
#include <cstring>
#include <unistd.h>
#ifndef _WIN32
#include <termios.h>
#endif
#include <libnova/julian_day.h>
#include <libnova/sidereal_time.h>

#include "config.h"

/*******************************************************************************
*** StarGo Implementation
*******************************************************************************/
const char *ADVANCED_TAB = "Advanced";

StarGoTelescope::StarGoTelescope()
{
    LOG_DEBUG(__FUNCTION__);
    setVersion(AVALON_VERSION_MAJOR, AVALON_VERSION_MINOR);

    DBG_SCOPE = INDI::Logger::DBG_DEBUG;

    SetTelescopeCapability(TELESCOPE_CAN_PARK | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT |
                           TELESCOPE_HAS_TRACK_MODE | TELESCOPE_HAS_LOCATION | TELESCOPE_CAN_CONTROL_TRACK |
                           TELESCOPE_HAS_PIER_SIDE | TELESCOPE_HAS_TIME, 4);

    autoRa = new AutoAdjust(this);
}

/*******************************************************************************
**
** VIRTUAL METHODS
**
*******************************************************************************/

/*******************************************************************************
**
*******************************************************************************/
const char *StarGoTelescope::getDefaultName()
{
    return "Avalon StarGo";
}

/*******************************************************************************
** Handshake is called when the driver first connects (physically) to the mount
*******************************************************************************/
bool StarGoTelescope::Handshake()
{
    LOG_DEBUG(__FUNCTION__);
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

// Use GetScopeAlignmentStatus as a basic form of Handshake.
// Checks that the mount responds to the GW query (Polar or AltAztracking mode)
    char mountType;
    bool isTracking;
    int alignmentPoints;
    if(!getScopeAlignmentStatus(&mountType, &isTracking, &alignmentPoints))
    {
        LOG_ERROR("Error communication with telescope.");
        return false;
    }
    
// Handshake commands used in the StarGo ASCOM driver. 
    char cmdsync[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char cmdlst[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char cmddate[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char lst[AVALON_COMMAND_BUFFER_LENGTH-5] = {0};
    if(getLST_String(lst))
    {
        sprintf(cmdsync,":X31%s#", lst);
        sprintf(cmdlst, ":X32%s#", lst);
    }
    time_t now = time (nullptr);
    strftime(cmddate, AVALON_COMMAND_BUFFER_LENGTH, ":X50%d%m%y#", localtime(&now));

    const char* cmds[12][2]={
        ":TTSFG#", "0",
        ":X3E1#", nullptr,
        ":TTHS1#", nullptr,
        cmddate, nullptr,             // Set current date
        ":TTRFr#", "0",
        ":X4B1#", nullptr,
        ":TTSFS#", "0",
        ":X474#", nullptr,
        ":TTSFR#", "0",
        ":X351#", "0",
        cmdlst, "0",                // Set LST
        ":TTRFd#", "0"              // Reset Force Meridian flip
    };
    for( int i=0; i < 12; i++)
    {
    LOGF_DEBUG("cmd %d: %s (%s)", i, cmds[i][0], cmds[i][1]);
        if(!sendQuery(cmds[i][0], response, cmds[i][1]==nullptr?0:5))
        {
            LOGF_ERROR("Error sending command %s", cmds[i][0]);
            continue;
        }
        if (cmds[i][1]!=nullptr && strcmp(response, cmds[i][1]) != 0)
        {
            LOGF_ERROR("Unexpected response %s", response);
            continue;
        }
    }

    getBasicData();

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // sync home position
        if (!strcmp(name, SyncHomeSP.name))
        {
            return setHomeSync();
        }

        // goto home position
        if (!strcmp(name, MountGotoHomeSP.name))
        {
            IUUpdateSwitch(&MountGotoHomeSP, states, names, n);
            if (setGotoHome())
            {
                MountGotoHomeSP.s = IPS_BUSY;
                TrackState = SCOPE_SLEWING;
            }
            else
            {
                MountGotoHomeSP.s = IPS_ALERT;
            }
            MountGotoHomeS[0].s = ISS_OFF;
            IDSetSwitch(&MountGotoHomeSP, nullptr);

            LOG_INFO("Slewing to home position...");
            return true;
        }
        // tracking mode
        else if (!strcmp(name, TrackModeSP.name))
        {
            if (IUUpdateSwitch(&TrackModeSP, states, names, n) < 0)
                return false;
            int trackMode = IUFindOnSwitchIndex(&TrackModeSP);

            bool result = SetTrackMode(trackMode);

            switch (trackMode)
            {
            case TRACK_SIDEREAL:
                LOG_INFO("Sidereal tracking rate selected.");
                break;
            case TRACK_SOLAR:
                LOG_INFO("Solar tracking rate selected.");
                break;
            case TRACK_LUNAR:
                LOG_INFO("Lunar tracking rate selected");
                break;
            }
            TrackModeSP.s = result ? IPS_OK : IPS_ALERT;

            IDSetSwitch(&TrackModeSP, nullptr);
            return result;
        }
        else if (!strcmp(name, ST4StatusSP.name))
        {
            bool enabled = !strcmp(IUFindOnSwitchName(states, names, n), ST4StatusS[INDI_ENABLED].name);
            bool result = setST4Enabled(enabled);

            if(result)
            {
                ST4StatusS[INDI_ENABLED].s = enabled ? ISS_ON : ISS_OFF;
                ST4StatusS[INDI_DISABLED].s = enabled ? ISS_OFF : ISS_ON;
                ST4StatusSP.s = IPS_OK;
            }
            else
            {
                ST4StatusSP.s = IPS_ALERT;
            }
            IDSetSwitch(&ST4StatusSP, nullptr);
            return result;
        }
        else if (!strcmp(name, KeypadStatusSP.name))
        {
            bool enabled = !strcmp(IUFindOnSwitchName(states, names, n), KeypadStatusS[INDI_ENABLED].name);
            bool result = setKeyPadEnabled(enabled);

            if(result)
            {
                KeypadStatusS[INDI_ENABLED].s = enabled ? ISS_ON : ISS_OFF;
                KeypadStatusS[INDI_DISABLED].s = enabled ? ISS_OFF : ISS_ON;
                KeypadStatusSP.s = IPS_OK;
            }
            else
            {
                KeypadStatusSP.s = IPS_ALERT;
            }
            IDSetSwitch(&KeypadStatusSP, nullptr);
            return result;
        }
        else if (!strcmp(name, MeridianFlipModeSP.name))
        {
            int preIndex = IUFindOnSwitchIndex(&MeridianFlipModeSP);
            IUUpdateSwitch(&MeridianFlipModeSP, states, names, n);
            int nowIndex = IUFindOnSwitchIndex(&MeridianFlipModeSP);
            if (SetMeridianFlipMode(nowIndex) == false)
            {
                IUResetSwitch(&MeridianFlipModeSP);
                MeridianFlipModeS[preIndex].s = ISS_ON;
                MeridianFlipModeSP.s          = IPS_ALERT;
            }
            else
                MeridianFlipModeSP.s = IPS_OK;
            IDSetSwitch(&MeridianFlipModeSP, nullptr);
            return true;
        }
        else if (!strcmp(name, RaMotorReverseSP.name))
        {
            int preIndex = IUFindOnSwitchIndex(&RaMotorReverseSP);
            int decIndex = IUFindOnSwitchIndex(&DecMotorReverseSP);
            IUUpdateSwitch(&RaMotorReverseSP, states, names, n);
            int raIndex = IUFindOnSwitchIndex(&RaMotorReverseSP);
            if (setMotorReverse(raIndex, decIndex) == false)
            {
                IUResetSwitch(&RaMotorReverseSP);
                RaMotorReverseS[preIndex].s = ISS_ON;
                RaMotorReverseSP.s          = IPS_ALERT;
            }
            else
                RaMotorReverseSP.s = IPS_OK;
            IDSetSwitch(&RaMotorReverseSP, nullptr);
            return true;
        }
        else if (!strcmp(name, DecMotorReverseSP.name))
        {
            int preIndex = IUFindOnSwitchIndex(&DecMotorReverseSP);
            int raIndex = IUFindOnSwitchIndex(&RaMotorReverseSP);
            IUUpdateSwitch(&DecMotorReverseSP, states, names, n);
            int decIndex = IUFindOnSwitchIndex(&DecMotorReverseSP);
            if (setMotorReverse(raIndex, decIndex) == false)
            {
                IUResetSwitch(&DecMotorReverseSP);
                DecMotorReverseS[preIndex].s = ISS_ON;
                DecMotorReverseSP.s          = IPS_ALERT;
            }
            else
                DecMotorReverseSP.s = IPS_OK;
            IDSetSwitch(&DecMotorReverseSP, nullptr);
            return true;
        }
        else if (!strcmp(name, RaAutoAdjustSP.name))
        {
            bool enabled = !strcmp(IUFindOnSwitchName(states, names, n), RaAutoAdjustS[INDI_ENABLED].name);
            bool result = autoRa->setEnabled(enabled);

            if(result)
            {
                RaAutoAdjustS[INDI_ENABLED].s = enabled ? ISS_ON : ISS_OFF;
                RaAutoAdjustS[INDI_DISABLED].s = enabled ? ISS_OFF : ISS_ON;
                RaAutoAdjustSP.s = IPS_OK;
            }
            else
            {
                RaAutoAdjustSP.s = IPS_ALERT;
            }
            IDSetSwitch(&RaAutoAdjustSP, nullptr);
            return result;
        }
    }

//  Nobody has claimed this, so pass it to the parent
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        processGuiderProperties(name, values, names, n);
        // sync home position
        if (!strcmp(name, GuidingSpeedNP.name))
        {
            int raSpeed  = static_cast<int>(round(values[0] * 100.0));
            int decSpeed = static_cast<int>(round(values[1] * 100.0));
            bool result  = setGuidingSpeeds(raSpeed, decSpeed);

            if(result)
            {
                GuidingSpeedN[0].value = static_cast<double>(raSpeed) / 100.0;
                GuidingSpeedN[1].value = static_cast<double>(decSpeed) / 100.0;
                GuidingSpeedNP.s = IPS_OK;
            }
            else
            {
                GuidingSpeedNP.s = IPS_ALERT;
            }
            IDSetNumber(&GuidingSpeedNP, nullptr);
            return result;
        }
        else if (!strcmp(name, MountRequestDelayNP.name))
        {
            int secs   = static_cast<int>(floor(values[0] / 1000.0));
            long nsecs = static_cast<long>(round((values[0] - 1000.0 * secs) * 1000000.0));
            setMountRequestDelay(secs, nsecs);

            MountRequestDelayN[0].value = secs*1000 + nsecs/1000000;
            MountRequestDelayNP.s = IPS_OK;
            IDSetNumber(&MountRequestDelayNP, nullptr);
            return true;
        }
        else if (!strcmp(name, MaxSlewNP.name))
        {
            int raSlew  = values[0];
            int decSlew = values[1];
            bool result  = setMaxSlews(raSlew, decSlew);

            if(result)
            {
                MaxSlewN[0].value = static_cast<double>(raSlew);
                MaxSlewN[1].value = static_cast<double>(decSlew);
                MaxSlewNP.s = IPS_OK;
            }
            else
            {
                MaxSlewNP.s = IPS_ALERT;
            }
            IDSetNumber(&MaxSlewNP, nullptr);
            return result;
        }
        else if (!strcmp(name, MoveSpeedNP.name))
        {
            int center  = values[0];
            int find = values[1];
            bool result  = setMoveSpeed(center, find);

            if(result)
            {
                MoveSpeedN[0].value = static_cast<double>(center);
                MoveSpeedN[1].value = static_cast<double>(find);
                MoveSpeedNP.s = IPS_OK;
            }
            else
            {
                MoveSpeedNP.s = IPS_ALERT;
            }
            IDSetNumber(&MoveSpeedNP, nullptr);
            return result;
        }
        else if (!strcmp(name, TrackAdjustNP.name))
        {
            // change tracking adjustment
            bool success = setTrackingAdjustment(values[0]);
            double adjust;
            if (success)
            {
                success = getTrackingAdjustment(&adjust);  // Get the value set in the mount
                TrackAdjustN[0].value = adjust;
                TrackAdjustNP.s       = IPS_OK;
            }
            else
                TrackAdjustNP.s = IPS_ALERT;

            IDSetNumber(&TrackAdjustNP, nullptr);
            return success;
        }
        else if (!strcmp(name, TorqueNP.name))
        {
            int torque  = static_cast<int>(values[0]);
            bool result  = setTorque(torque);
            if(result)
            {
                result = getTorque(&torque);  // Get the value set in the mount
                TorqueN[0].value = torque;
                TorqueNP.s = IPS_OK;
            }
            else
            {
                TorqueNP.s = IPS_ALERT;
            }
            IDSetNumber(&TorqueNP, nullptr);
            return result;
        }
    }

    //  Nobody has claimed this, so pass it to the parent
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

/*******************************************************************************
** ISGetProperties defines the minimal set of controls needed for 
** the client to connect to the driver. These should be moved to updateProperties
*******************************************************************************/
/*
void StarGoTelescope::ISGetProperties(const char *dev)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) != 0)
        return;

    INDI::Telescope::ISGetProperties(dev);
    if (isConnected())
    {
// Track mode: sidereal, lunar, solar, custom
// Track state: Track on, track off
// These steps are performed in INDI::telescope::updateProperties
        if (HasTrackMode() && TrackModeS != nullptr)
            defineProperty(&TrackModeSP);
        if (CanControlTrack())
            defineProperty(&TrackStateSP);
    }
}
*/
/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::initProperties()
{
    /* Make sure to init parent properties first */
    if (!INDI::Telescope::initProperties()) return false;

    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    AddTrackMode("TRACK_SOLAR", "Solar");
    AddTrackMode("TRACK_LUNAR", "Lunar");

    TrackState = SCOPE_IDLE;

    initGuiderProperties(getDeviceName(), GUIDE_TAB);

    /* Add debug/simulation/config controls so we may debug driver if necessary */
    addAuxControls();

    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

// In INDI::Telescope something similar is done in initProperties.
// Below assumes that at startup the mount is pointing at HA=0 (so RA=LST) and the pole
// But currentRa and currentDec are not really used as class properties
// Except maybe in simulation mode
// so this bit of code looks meaningless
// Could use instead EqN[AXIS_RA].value and EqN[AXIS_DE].value which are set by NewRaDec()
/*
    double longitude=0.0, latitude=90.0;
    // Get value from config file if it exists.
    IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LONG", &longitude);
    currentRA  = get_local_sidereal_time(longitude);
    IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LAT", &latitude);
    currentDEC = latitude > 0.0 ? 90.0 : -90.0;
*/
    IUFillSwitch(&MountGotoHomeS[0], "MOUNT_GOTO_HOME_VALUE", "Goto Home", ISS_OFF);
    IUFillSwitchVector(&MountGotoHomeSP, MountGotoHomeS, 1, getDeviceName(), "MOUNT_GOTO_HOME", "Goto Home", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 60, IPS_OK);

    SetParkDataType(PARK_HA_DEC);

    IUFillSwitch(&SyncHomeS[0], "SYNC_HOME", "Sync Home", ISS_OFF);
    IUFillSwitchVector(&SyncHomeSP, SyncHomeS, 1, getDeviceName(), "TELESCOPE_SYNC_HOME", "Home Position", MAIN_CONTROL_TAB,
                       IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    IUFillText(&MountFirmwareInfoT[0], "MOUNT_FIRMWARE_INFO", "Firmware", "");
    IUFillText(&MountFirmwareInfoT[1], "MOUNT_TYPE", "Mount Type", "");
    IUFillText(&MountFirmwareInfoT[2], "MOUNT_TCB", "TCB", "");
    IUFillTextVector(&MountFirmwareInfoTP, MountFirmwareInfoT, 3, getDeviceName(), "MOUNT_INFO", "Mount Info", INFO_TAB, IP_RO, 60, IPS_OK);

    // Gear Ratios
    IUFillNumber(&GearRatioN[0], "GEAR_RATIO_RA", "RA Gearing", "%.2f", 0.0, 1000.0, 1, 0);
    IUFillNumber(&GearRatioN[1], "GEAR_RATIO_DEC", "DEC Gearing", "%.2f", 0.0, 1000.0, 1, 0);
    IUFillNumberVector(&GearRatioNP, GearRatioN, 2, getDeviceName(), "Gear Ratio","Gearing", INFO_TAB, IP_RO, 60, IPS_IDLE);

    // Max Slew Speeds
    IUFillNumber(&MaxSlewN[0], "MAX_SLEW_RA", "RA Max Slew", "%.2f", 0.0, 100.0, 1, 0);
    IUFillNumber(&MaxSlewN[1], "MAX_SLEW_DEC", "DEC Max Slew", "%.2f", 0.0, 100.0, 1, 0);
    IUFillNumberVector(&MaxSlewNP, MaxSlewN, 2, getDeviceName(), "Max Slew","Slewing", MOTION_TAB, IP_RW, 60, IPS_IDLE);

    // Move Speeds
    IUFillNumber(&MoveSpeedN[0], "MOVE_SPEED_CENTER", "Center Speed", "%.2f", 1.0, 10.0, 1, 0);
    IUFillNumber(&MoveSpeedN[1], "MOVE_SPEED_FIND", "Find Speed", "%.2f", 1.0, 150.0, 1, 0);
    IUFillNumberVector(&MoveSpeedNP, MoveSpeedN, 2, getDeviceName(), "Goto Speed","Slewing", MOTION_TAB, IP_RW, 60, IPS_IDLE);

    // RA and Dec motor direction
    IUFillSwitch(&RaMotorReverseS[INDI_ENABLED], "INDI_ENABLED", "Reverse", ISS_OFF);
    IUFillSwitch(&RaMotorReverseS[INDI_DISABLED], "INDI_DISABLED", "Normal", ISS_OFF);
    IUFillSwitchVector(&RaMotorReverseSP, RaMotorReverseS, 2, getDeviceName(), "RA_REVERSE", "RA Reverse", MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    IUFillSwitch(&DecMotorReverseS[INDI_ENABLED], "INDI_ENABLED", "Reverse", ISS_OFF);
    IUFillSwitch(&DecMotorReverseS[INDI_DISABLED], "INDI_DISABLED", "Normal", ISS_OFF);
    IUFillSwitchVector(&DecMotorReverseSP, DecMotorReverseS, 2, getDeviceName(), "DEC_REVERSE", "Dec Reverse", MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Torque
    IUFillNumber(&TorqueN[0], "TORQUE_RA", "Motor Torque", "%.0f", 0.0, 100.0, 1, 0);
    IUFillNumberVector(&TorqueNP, TorqueN, 1, getDeviceName(), "Torque","Torque", ADVANCED_TAB, IP_RW, 60, IPS_IDLE);

    // Motor Step Position
    IUFillNumber(&MotorStepN[0], "MOTOR_STEP_RA", "RA Step Pos", "%.2f", -100000.0, 100000.0, 1, 0);
    IUFillNumber(&MotorStepN[1], "MOTOR_STEP_DEC", "DEC Step Pos", "%.2f", -100000.0, 100000.0, 1, 0);
    IUFillNumberVector(&MotorStepNP, MotorStepN, 2, getDeviceName(), "Motor Steps","Position", INFO_TAB, IP_RO, 60, IPS_IDLE);

    // Guiding settings
    IUFillNumber(&GuidingSpeedN[0], "GUIDE_RATE_WE", "RA Speed", "%.2f", 0.0, 2.0, 0.1, 0);
    IUFillNumber(&GuidingSpeedN[1], "GUIDE_RATE_NS", "DEC Speed", "%.2f", 0.0, 2.0, 0.1, 0);
    IUFillNumberVector(&GuidingSpeedNP, GuidingSpeedN, 2, getDeviceName(), "GUIDE_RATE","Autoguiding", GUIDE_TAB, IP_RW, 60, IPS_IDLE);

    // Tracking Adjustment
    IUFillNumber(&TrackAdjustN[0], "RA_TRACK_ADJ", "RA Tracking Adjust (%)", "%.2f", -5.0, 5.0, 0.01, 0);
    IUFillNumberVector(&TrackAdjustNP, TrackAdjustN, 1, getDeviceName(), "Track Adjust","Tracking", MOTION_TAB, IP_RW, 60, IPS_IDLE);

    // Auto Tracking Adjustment
    IUFillSwitch(&RaAutoAdjustS[INDI_ENABLED], "INDI_ENABLED", "Enabled", ISS_ON);
    IUFillSwitch(&RaAutoAdjustS[INDI_DISABLED], "INDI_DISABLED", "Disabled", ISS_OFF);
    IUFillSwitchVector(&RaAutoAdjustSP, RaAutoAdjustS, 2, getDeviceName(), "RA_AUTO_ADJ", "RA Auto Adjust", MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);


    IUFillSwitch(&ST4StatusS[INDI_ENABLED], "INDI_ENABLED", "Enabled", ISS_OFF);
    IUFillSwitch(&ST4StatusS[INDI_DISABLED], "INDI_DISABLED", "Disabled", ISS_OFF);
    IUFillSwitchVector(&ST4StatusSP, ST4StatusS, 2, getDeviceName(), "ST4", "ST4", GUIDE_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // keypad enabled / disabled
    IUFillSwitch(&KeypadStatusS[INDI_ENABLED], "INDI_ENABLED", "Enabled", ISS_ON);
    IUFillSwitch(&KeypadStatusS[INDI_DISABLED], "INDI_DISABLED", "Disabled", ISS_OFF);
    IUFillSwitchVector(&KeypadStatusSP, KeypadStatusS, 2, getDeviceName(), "Keypad", "Keypad", ADVANCED_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // meridian flip
    IUFillSwitch(&MeridianFlipModeS[0], "MERIDIAN_FLIP_AUTO", "Auto", ISS_OFF);
    IUFillSwitch(&MeridianFlipModeS[1], "MERIDIAN_FLIP_DISABLED", "Disabled", ISS_OFF);
    IUFillSwitch(&MeridianFlipModeS[2], "MERIDIAN_FLIP_FORCED", "Forced", ISS_OFF);
    IUFillSwitchVector(&MeridianFlipModeSP, MeridianFlipModeS, 3, getDeviceName(), "MERIDIAN_FLIP_MODE", "Meridian Flip", MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // mount command delay
    IUFillNumber(&MountRequestDelayN[0], "MOUNT_REQUEST_DELAY", "Request Delay (ms)", "%.0f", 0.0, 1000, 1.0, 50.0);
    IUFillNumberVector(&MountRequestDelayNP, MountRequestDelayN, 1, getDeviceName(), "REQUEST_DELAY", "StarGO", OPTIONS_TAB, IP_RW, 60, IPS_OK);

    IUFillNumber(&HaLstN[0], "HA", "HA (hh:mm:ss)", "%010.6m", 0, 24, 0, 0);
    IUFillNumber(&HaLstN[1], "LST", "LST (hh:mm:ss)", "%010.6m", 0, 24, 0, 0);
    IUFillNumberVector(&HaLstNP, HaLstN, 2, getDeviceName(), "HA-LST", "Hour Angle", INFO_TAB, IP_RO, 60, IPS_IDLE);

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::updateProperties()
{
    if (! INDI::Telescope::updateProperties()) return false;

    if (isConnected())
    {
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
        defineProperty(&SyncHomeSP);
        defineProperty(&MountGotoHomeSP);
        defineProperty(&GuidingSpeedNP);
        defineProperty(&ST4StatusSP);
        defineProperty(&KeypadStatusSP);
        defineProperty(&MeridianFlipModeSP);
        defineProperty(&MountRequestDelayNP);
        defineProperty(&MountFirmwareInfoTP);
        defineProperty(&TrackAdjustNP);
        defineProperty(&RaAutoAdjustSP);
        defineProperty(&GearRatioNP);
        defineProperty(&TorqueNP);
        defineProperty(&RaMotorReverseSP);
        defineProperty(&DecMotorReverseSP);
        defineProperty(&MaxSlewNP);
        defineProperty(&MoveSpeedNP);
        defineProperty(&MotorStepNP);
        defineProperty(&HaLstNP);

//        getBasicData(); // Call from Handshake()
    }
    else
    {
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(SyncHomeSP.name);
        deleteProperty(MountGotoHomeSP.name);
        deleteProperty(GuidingSpeedNP.name);
        deleteProperty(ST4StatusSP.name);
        deleteProperty(KeypadStatusSP.name);
        deleteProperty(MeridianFlipModeSP.name);
        deleteProperty(MountRequestDelayNP.name);
        deleteProperty(MountFirmwareInfoTP.name);
        deleteProperty(TrackAdjustNP.name);
        deleteProperty(RaAutoAdjustSP.name);
        deleteProperty(GearRatioNP.name);
        deleteProperty(TorqueNP.name);
        deleteProperty(RaMotorReverseSP.name);
        deleteProperty(DecMotorReverseSP.name);
        deleteProperty(MaxSlewNP.name);
        deleteProperty(MoveSpeedNP.name);
        deleteProperty(MotorStepNP.name);
        deleteProperty(HaLstNP.name);
    }

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::saveConfigItems(FILE *fp)
{
    LOG_DEBUG(__FUNCTION__);
    INDI::Telescope::saveConfigItems(fp);
    return true;
}

/*******************************************************************************
** ReadScopeStatus called by polling
*******************************************************************************/
bool StarGoTelescope::ReadScopeStatus()
{
    LOG_DEBUG(__FUNCTION__);
    if (!isConnected())
        return false;

    if (isSimulation())
    {
        mountSim();
        return true;
    }

    int x, y;
    if (! getMotorStatus(&x, &y))
    {
        LOG_INFO("Failed to parse motor state. Retrying...");
        // retry once
        if (! getMotorStatus(&x, &y))
        {
            LOG_ERROR("Cannot determine scope status, failed to parse motor state.");
            return false;
        }
    }
    int motion;
    if( x == MOTION_SLEW || y == MOTION_SLEW)
    {
        motion = MOTION_SLEW;
    }
    else if( x == MOTION_ACCEL || y == MOTION_ACCEL)
    {
        motion = MOTION_SLEW;
    }
    else if( x == MOTION_DECEL || y == MOTION_DECEL)
    {
        motion = MOTION_SLEW;
    }
    else if( x == MOTION_GUIDE || y == MOTION_GUIDE)
    {
        motion = MOTION_GUIDE;
    }
    else if( x == MOTION_TRACK || y == MOTION_TRACK)
    {
        motion = MOTION_TRACK;
    }
    else if( x == MOTION_STATIC && y == MOTION_STATIC)
    {
        motion = MOTION_STATIC;
    }
    else
    {
       LOGF_ERROR("Invlid motion state: %d, %d", x, y);
       return false;
    }
    if( motion == MOTION_GUIDE)
    {
        LOG_DEBUG("Guiding in progress");
        return true;
    }
    if( x != 4) GuideComplete(AXIS_RA);
    if( y != 4) GuideComplete(AXIS_DE);

    char parkHomeStatus[2] = {'\0','\0'};
    if (! getParkHomeStatus(parkHomeStatus))
    {
       LOG_ERROR("Cannot determine scope status, failed to determine park/sync state.");
       return false;
    }
    LOGF_DEBUG("Motor state(RA,DE): (%d, %d); Park state = %s", x, y, parkHomeStatus);

    INDI::Telescope::TelescopeStatus newTrackState = TrackState;

    // handle parking / unparking
    if(strcmp(parkHomeStatus, "2") == 0)
    {
        newTrackState = SCOPE_PARKED;
        if (TrackState != newTrackState)
            SetParked(true);
    }
    else
    {
        if (TrackState == SCOPE_PARKED)
            SetParked(false);

        // handle tracking state
        if(x==0 && y==0)
        {
            newTrackState = SCOPE_IDLE;
            if (TrackState != newTrackState)
                LOGF_INFO("%sTracking is off.", TrackState == SCOPE_PARKING ? "Scope parked. ": "");

            if (MountGotoHomeSP.s == IPS_BUSY)
            {
                MountGotoHomeSP.s = IPS_OK;
                IDSetSwitch(&MountGotoHomeSP, nullptr);
            }
        }
        else if(x==1 && y==0)
        {
            newTrackState = SCOPE_TRACKING;  // or GUIDING
            if (TrackState != newTrackState)
                LOGF_INFO("%sTracking...", TrackState == SCOPE_SLEWING ? "Slewing completed. ": "");
        }
    }

    double raStep, decStep;
    if (getMotorSteps(&raStep, &decStep))
    {
        MotorStepN[0].value =  raStep;
        MotorStepN[1].value =  decStep;
        MotorStepNP.s = IPS_OK;
    }
    else
    {
        MotorStepNP.s = IPS_ALERT;
    }
    IDSetNumber(&MotorStepNP, nullptr);

    double r, d;
    if(!getEqCoordinates(&r, &d))
    {
        LOG_ERROR("Retrieving equatorial coordinates failed.");
        return false;
    }
//    currentRA = r;
//    currentDEC = d;

    TrackState = newTrackState;
//    NewRaDec(currentRA, currentDEC);
    NewRaDec(r, d);
    double ha, lst;
    char clst[12];
    if(getLST_String(clst))
    {
        f_scansexa(clst, &lst);
        ha = lst - r;
        HaLstN[0].value =  ha;
        HaLstN[1].value =  lst;
        HaLstNP.s = IPS_OK;
    }
    else
    {
        LOG_ERROR("Retrieving scope LST failed.");
        HaLstNP.s = IPS_ALERT;
    }

    WaitParkOptionReady();

    return getSideOfPier();
}

/*******************************************************************************
** virtual updateLocation
*******************************************************************************/
bool StarGoTelescope::updateLocation(double latitude, double longitude, double elevation)
{
    LOGF_DEBUG("%s Lat:%.3lf Lon:%.3lf",__FUNCTION__, latitude, longitude);
    INDI_UNUSED(elevation);

    if (isSimulation())
        return true;

//    LOGF_DEBUG("Setting site longitude '%lf'", longitude);
    if (!isSimulation() && ! setSiteLongitude(longitude))
    {
        LOGF_ERROR("Error setting site longitude %lf", longitude);
        return false;
    }

    if (!isSimulation() && ! setSiteLatitude(latitude))
    {
        LOGF_ERROR("Error setting site latitude %lf", latitude);
        return false;
    }

    char l[32]={0}, L[32]={0};
    fs_sexa(l, latitude, 3, 3600);
    fs_sexa(L, longitude, 4, 3600);

//    LOGF_INFO("Site location updated to Lat %.32s - Long %.32s", l, L);
// Set local sidereal time for the new longitude
    if(!setLocalSiderealTime(longitude))
    {
        LOG_ERROR("Error setting local sidereal time");
        return false;
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::Sync(double ra, double dec)
{
    LOGF_DEBUG("%s ra=%lf, dec=%lf", __FUNCTION__, ra, dec);
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if(!isSimulation() && !setObjectCoords(ra,dec))
    {
         LOG_ERROR("Error setting coords for sync");
         return false;
    }

    if (!isSimulation() && !sendQuery(":CM#", response))
    {
        EqNP.s = IPS_ALERT;
        IDSetNumber(&EqNP, "Synchronization failed.");
        return false;
    }

//    currentRA  = ra;
//    currentDEC = dec;

    LOG_INFO("Synchronization successful.");

    EqNP.s     = IPS_OK;

//    NewRaDec(currentRA, currentDEC);
    NewRaDec(ra, dec);

    return true;
}

/*******************************************************************************
** SetParkPosition
** Set desired parking position to the supplied value.
** This ONLY sets the desired park position value and does not perform parking.
** Input arguments are as defined by SetParkDataType(PARK_HA_DEC) (see initProperties)
**
*******************************************************************************/
bool StarGoTelescope::SetParkPosition(double Axis1Value, double Axis2Value)
{
    INDI_UNUSED(Axis1Value);
    INDI_UNUSED(Axis2Value);

// Convert HA/Dec to RA/Dec
    double longitude;
    if (!getSiteLongitude(&longitude))
    {
        LOG_WARN("Failed to get site Longitude from device.");
        return false;
    }
    // determine local sidereal time
    double lst = get_local_sidereal_time(longitude);

// Use LST to calculate RA from input HA then slew to that position
// StarGo can only set the current mount position as the Park position
// Caution. If mount LST does not match driver LST then the mount can slew
// to an unexpected location.
    if(!Goto(lst - Axis1Value, Axis2Value)) return false;
    return SetCurrentPark();
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::SetDefaultPark()
{
    LOG_DEBUG(__FUNCTION__);

// Not sure why we do this
    double latitude;
    if (!getSiteLatitude(&latitude))
    {
        LOG_WARN("Failed to get site Latitude from device.");
        return false;
    }

    if(!setGotoHome()) return false;
    return SetCurrentPark();
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::SetCurrentPark()
{
    LOG_DEBUG(__FUNCTION__);

    ParkOptionBusy = true;
    WaitParkOptionReady();
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::Park()
{
    LOG_DEBUG(__FUNCTION__);
    // in: :X362#
    // out: "pB#"

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (sendQuery(":X362#", response) && strcmp(response, "pB") == 0)
    {
        LOG_INFO("Parking mount...");
        TrackState = SCOPE_PARKING;
        return true;
    }
    else
    {
        LOGF_ERROR("Parking failed. Response %s", response);
        return false;
    }
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::UnPark()
{
    LOG_DEBUG(__FUNCTION__);
    // in: :X370#
    // out: "p0#"

    double siteLong;

    // step one: determine site longitude
    if (!getSiteLongitude(&siteLong))
    {
        LOG_WARN("Failed to get site Longitude from device.");
        return false;
    }
    // set LST to avoid errors
    if (!setLocalSiderealTime(siteLong))
    {
        LOGF_ERROR("Failed to set LST before unparking %lf", siteLong);
        return false;
    }
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    // and now execute unparking
    if (sendQuery(":X370#", response) && strcmp(response, "p0") == 0)
    {
        LOG_INFO("Unparking mount...");
        return true;
    }
    else
    {
        LOGF_ERROR("Unpark failed with response: %s", response);
        return false;
    }
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::SetSlewRate(int index)
{
    LOGF_DEBUG("%s %d", __FUNCTION__, index);

    if (!isSimulation() && !setSlewMode(index))
    {
        SlewRateSP.s = IPS_ALERT;
        IDSetSwitch(&SlewRateSP, "Error setting slew mode.");
        return false;
    }

    SlewRateSP.s = IPS_OK;
    IDSetSwitch(&SlewRateSP, nullptr);
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::Goto(double ra, double dec)
{
    LOGF_DEBUG("%s ra:%lf, dec:%lf", __FUNCTION__, ra, dec);
    // in :MS#
    //  after setObjectCoords
    const struct timespec timeout = {0, 100000000L};

//    targetRA  = ra;
//    targetDEC = dec;

    // If moving, let's stop it first.
    if (EqNP.s == IPS_BUSY)
    {
        if (!isSimulation() && !Abort())
        {
            AbortSP.s = IPS_ALERT;
            IDSetSwitch(&AbortSP, "Abort slew failed.");
            return false;
        }

        AbortSP.s = IPS_OK;
        EqNP.s    = IPS_IDLE;
        IDSetSwitch(&AbortSP, "Slew aborted.");
        IDSetNumber(&EqNP, nullptr);

        if (MovementNSSP.s == IPS_BUSY || MovementWESP.s == IPS_BUSY)
        {
            MovementNSSP.s = MovementWESP.s = IPS_IDLE;
            EqNP.s                          = IPS_IDLE;
            IUResetSwitch(&MovementNSSP);
            IUResetSwitch(&MovementWESP);
            IDSetSwitch(&MovementNSSP, nullptr);
            IDSetSwitch(&MovementWESP, nullptr);
        }

        // sleep for 100 mseconds
        nanosleep(&timeout, nullptr);
    }
    if(!isSimulation() && !setObjectCoords(ra,dec))
    {
         LOG_ERROR("Error setting coords for goto");
         return false;
    }

    if (!isSimulation())
    {
        char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
        if(!sendQuery(":MS#", response))
        {
            LOG_ERROR("Error Slewing");
            EqNP.s = IPS_ALERT;
            IDSetNumber(&EqNP, nullptr);
            return false;
        }
    }

    TrackState = SCOPE_SLEWING;
    EqNP.s     = IPS_BUSY;

//    LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::Abort()
{
    LOG_DEBUG(__FUNCTION__);
    // in :Q#
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    ParkOptionBusy = false;
    if (!isSimulation() && !sendQuery(":Q#", response, 0))
    {
        LOG_ERROR("Failed to abort slew.");
        return false;
    }

    if (GuideNSNP.s == IPS_BUSY || GuideWENP.s == IPS_BUSY)
    {
        GuideNSNP.s = GuideWENP.s = IPS_IDLE;
        GuideNSN[0].value = GuideNSN[1].value = 0.0;
        GuideWEN[0].value = GuideWEN[1].value = 0.0;

        LOG_INFO("Guide aborted.");
        IDSetNumber(&GuideNSNP, nullptr);
        IDSetNumber(&GuideWENP, nullptr);

        return true;
    }

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::SetTrackMode(uint8_t mode)
{
    LOGF_DEBUG("%s: Set Track Mode %d", __FUNCTION__, mode);
    if (isSimulation())
        return true;

    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    char s_mode[10]={0};

    switch (mode)
    {
        case TRACK_SIDEREAL:
            strcpy(cmd, ":TQ#");
            strcpy(s_mode, "Sidereal");
            break;
        case TRACK_SOLAR:
            strcpy(cmd, ":TS#");
            strcpy(s_mode, "Solar");
            break;
        case TRACK_LUNAR:
            strcpy(cmd, ":TL#");
            strcpy(s_mode, "Lunar");
            break;
        default:
            return false;
    }
    if ( !sendQuery(cmd, response, 0))  // Dont wait for response - there is none
        return false;
    LOGF_INFO("Tracking mode set to %s.", s_mode );

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::SetTrackEnabled(bool enabled)
{
    LOGF_DEBUG("%s enabled=%d",__FUNCTION__, enabled);
    // Command tracking on  - :X122#
    //         tracking off - :X120#

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (! sendQuery(enabled ? ":X122#" : ":X120#", response, 0))
    {
        LOGF_ERROR("Failed to %s tracking", enabled ? "enable" : "disable");
        return false;
    }
    LOGF_INFO("Tracking %s.", enabled?"enabled":"disabled");
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::SetTrackRate(double raRate, double deRate)
{
// * Set tracking rate
// * X1Ennnn where nnnn=0500 to 1500; 1000 is base rate
// See also SetTrackingAdjustment.
// This virtual function should set values in arcsec/second
// Convert to %age:
// (raRate/15-1)*10000 + 1000

    LOGF_DEBUG("%s rarate=%lf deRate=%lf",__FUNCTION__,raRate, deRate);
    INDI_UNUSED(raRate);
    INDI_UNUSED(deRate);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    int rate = static_cast<int>((raRate/15.0 - 1.0)*10000.0 + 1000.0);
    sprintf(cmd, ":X1E%04d", rate);
    if(!sendQuery(cmd, response, 0))
    {
        LOGF_ERROR("Failed to set tracking t %d", rate);
        return false;
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
IPState StarGoTelescope::GuideNorth(uint32_t ms)
{
    LOGF_DEBUG("%s %dms",__FUNCTION__, ms);
    if(!SendPulseCmd(STARGO_NORTH, ms))
    {
        return IPS_ALERT;
    }
    return IPS_IDLE;
}

/*******************************************************************************
**
*******************************************************************************/
IPState StarGoTelescope::GuideSouth(uint32_t ms)
{
    LOGF_DEBUG("%s %dms",__FUNCTION__, ms);
    if(!SendPulseCmd(STARGO_SOUTH, ms))
    {
        return IPS_ALERT;
    }
    return IPS_IDLE;
}

/*******************************************************************************
**
*******************************************************************************/
IPState StarGoTelescope::GuideEast(uint32_t ms)
{
    LOGF_DEBUG("%s %dms",__FUNCTION__, ms);
    if(!SendPulseCmd(STARGO_EAST, ms))
    {
        return IPS_ALERT;
    }
    return IPS_IDLE;
}

/*******************************************************************************
**
*******************************************************************************/
IPState StarGoTelescope::GuideWest(uint32_t ms)
{
    LOGF_DEBUG("%s %dms",__FUNCTION__, ms );
    if(!SendPulseCmd(STARGO_WEST, ms))
    {
        return IPS_ALERT;
    }
    return IPS_IDLE;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    LOGF_DEBUG("%s dir=%d cmd=%d", __FUNCTION__, dir, command);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    sprintf(cmd, ":%s%s#", command==MOTION_START?"M":"Q", dir == DIRECTION_NORTH?"n":"s");
    if (!isSimulation() && !sendQuery(cmd, response, 0))
    {
        LOG_ERROR("Error N/S motion direction.");
        return false;
    }

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    LOGF_DEBUG("%s dir=%d cmd=%d", __FUNCTION__, dir, command);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    sprintf(cmd, ":%s%s#", command==MOTION_START?"M":"Q", dir == DIRECTION_WEST?"w":"e");

    if (!isSimulation() && !sendQuery(cmd, response, 0))
    {
        LOG_ERROR("Error W/E motion direction.");
        return false;
    }

    return true;
}
/*******************************************************************************
** StarGo functions
*******************************************************************************/
/*******************************************************************************
** getScopeLocation called from getBasicData i.e. when a client connects
*******************************************************************************/
bool StarGoTelescope::getScopeLocation()
{
    LOG_DEBUG(__FUNCTION__);
    if (isSimulation())
    {
        LocationNP.np[LOCATION_LATITUDE].value = 29.5;
        LocationNP.np[LOCATION_LONGITUDE].value = 48.0;
        LocationNP.np[LOCATION_ELEVATION].value = 10;
        LocationNP.s           = IPS_OK;
        IDSetNumber(&LocationNP, nullptr);
        return true;
    }

    double siteLat = 0.0, siteLong = 0.0;
    if (!getSiteLatitude(&siteLat))
    {
        LOG_WARN("Failed to get site latitude from device.");
        return false;
    }
    if (!getSiteLongitude(&siteLong))
    {
        LOG_WARN("Failed to get site longitude from device.");
        return false;
    }
    LocationNP.np[LOCATION_LATITUDE].value = siteLat;
    LocationNP.np[LOCATION_LONGITUDE].value = siteLong;

    LOGF_DEBUG("Mount Controller Latitude: %lg Longitude: %lg", LocationN[LOCATION_LATITUDE].value, LocationN[LOCATION_LONGITUDE].value);

    IDSetNumber(&LocationNP, nullptr);
// Not sure why the driver does this in a get function
//    if(!setLocalSiderealTime(siteLong))
//    {
//        LOG_ERROR("Error setting local sidereal time");
//        return false;
//    }

    return true;
}

/*******************************************************************************
 * Determine the site latitude. In contrast to a standard LX200 implementation,
 * StarGo returns the location in arc seconds precision.
*******************************************************************************/
bool StarGoTelescope::getSiteLatitude(double *siteLat)
{
    LOG_DEBUG(__FUNCTION__);
    // Command :Gt#
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":Gt#", response)) 
    {
        LOG_ERROR("Failed to send query get Site Latitude command.");
        return false;
    }
    if (f_scansexa(response, siteLat))
    {
        LOGF_ERROR("Unable to parse get Site Latitude response %s", response);
        return false;
    }
    return true;
}

/*******************************************************************************
** setSiteLatitude called from updateLocation
 * @brief Set the site latitude
 * @param latitude value
 * @return true iff the command succeeded
*******************************************************************************/
bool StarGoTelescope::setSiteLatitude(double Lat)
{
    LOGF_DEBUG("%s Lat=%lf", __FUNCTION__, Lat);
    int d, m, s;
    char command[AVALON_COMMAND_BUFFER_LENGTH] = {0};

    getSexComponents(Lat, &d, &m, &s);

    snprintf(command, sizeof(command), ":St%+03d*%02d:%02d#", d, m, s);

    LOGF_DEBUG("Sending set site latitude request '%s'", command);

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    return (sendQuery(command, response));
}

/*******************************************************************************
 * Determine the site longitude from the mount
 * StarGo returns the location in arc seconds precision.
*******************************************************************************/
bool StarGoTelescope::getSiteLongitude(double *siteLong)
{
    LOG_DEBUG(__FUNCTION__);
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":Gg#", response))
    {
        LOG_ERROR("Failed to send query get Site Longitude command.");
        return false;
    }
    if (f_scansexa(response, siteLong))
    {
        LOG_ERROR("Unable to parse get Site Longitude response.");
        return false;
    }
    return true;
}

/*******************************************************************************
** setSiteLongitude called from updateLocation
 * Determine the site longitude.
 * StarGo saves the location in arc seconds precision.
*******************************************************************************/
bool StarGoTelescope::setSiteLongitude(double longitude)
{
    LOGF_DEBUG("%s longitude=%lf", __FUNCTION__, longitude);
    int d, m, s;
    char command[AVALON_COMMAND_BUFFER_LENGTH]={0};
    if (longitude > 180) longitude = longitude - 360;
    if (longitude < -180) longitude = 360 + longitude;

    getSexComponents(longitude, &d, &m, &s);

    if (d < 0 || m < 0 || s < 0)
        snprintf(command, sizeof(command), ":Sg%04d*%02u:%02u#",
                 d,
                 static_cast<uint32_t>(std::abs(m)),
                 static_cast<uint32_t>(std::abs(s)));
    else
        snprintf(command, sizeof(command), ":Sg+%03d*%02d:%02d#", d, m, s);

    LOGF_DEBUG("Sending set site longitude request '%s'", command);

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    bool result = sendQuery(command, response);

    return (result);
}

/*******************************************************************************
**setLocalSiderealTime
 * called from updateLocation and UnPark
*******************************************************************************/
bool StarGoTelescope::setLocalSiderealTime(double longitude)
{
    LOGF_DEBUG("%s longitude=%lf", __FUNCTION__, longitude);
    double lst = get_local_sidereal_time(longitude);
    LOGF_DEBUG("Current local sidereal time = %lf", lst);
    int h=0, m=0, s=0;
    getSexComponents(lst, &h, &m, &s);

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    sprintf(cmd, ":X32%02hd%02hd%02hd#",
            static_cast<int16_t>(h),
            static_cast<int16_t>(m),
            static_cast<int16_t>(s));
    if(!sendQuery(cmd, response))
    {
        LOG_ERROR("Failed to set LST");
        return false;
    }
    return true;
}

/*******************************************************************************
 * getLST_String
 * called from Handshake and syncHomePosition
 * @brief Determine the LST with format HHMMSS
 * @return LST value for the current scope locateion
*******************************************************************************/
bool StarGoTelescope::getLST_String(char* input)
{
    LOG_DEBUG(__FUNCTION__);

    double longitude;
    if (!getSiteLongitude(&longitude))
    {
        LOG_WARN("getLST Failed to get site Longitude from device.");
        return false;
    }
    // determine local sidereal time
    double lst = get_local_sidereal_time(longitude);
    int h=0, m=0, s=0;
    LOGF_DEBUG("Current local sidereal time = %.8lf", lst);
    // translate into hh:mm:ss
    getSexComponents(lst, &h, &m, &s);

    sprintf(input, "%02d%02d%02d", h, m, s);
    return true;
}
/*******************************************************************************
**
*******************************************************************************/
/*
bool StarGoTelescope::getScopeLST(double *lst)
{
    LOG_DEBUG(__FUNCTION__);
    // command :GS#
    // returns LST as hh:mm:ss#
    // seems to always return 00:34:56
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if(!sendQuery(":GS#", response))
    {
        LOG_ERROR("Failed to get LST");
        return false;
    }
    double mlst;
    if (f_scansexa(response, &mlst))
    {
        LOG_ERROR("Unable to parse get LST response.");
        return false;
    }
    double longitude;
    if (!getSiteLongitude(&longitude))
    {
        LOG_WARN("Failed to get site Longitude from device.");
        return false;
    }
    double syslst = get_local_sidereal_time(longitude);
    if( fabs(mlst-syslst) > 0.1 )
    {
        char clst[12], csys[12];
        fs_sexa(csys, syslst, 3, 3600);
        fs_sexa(clst, mlst, 3, 3600);
        LOGF_WARN("Mount LST varies from System LST %s %s", clst, csys );
    }
    *lst = mlst;
    return true;
}
*/
/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getScopeTime()
{
    LOG_DEBUG(__FUNCTION__);
    char cdate[MAXINDINAME]={0};
    char ctime[MAXINDINAME]={0};
    struct tm ltm;
    struct tm utm;
    time_t time_epoch;

    double offset=0;
    if (getUTCOffset(&offset))
    {
        char utcStr[8]={0};
        snprintf(utcStr, 8, "%.2f", offset);
        IUSaveText(&TimeT[1], utcStr);
    }
    else
    {
        LOG_WARN("Could not obtain UTC offset from mount!");
        return false;
    }

    if (getLocalTime(ctime) == false)
    {
        LOG_WARN("Could not obtain local time from mount!");
        return false;
    }

    if (getLocalDate(cdate) == false)
    {
        LOG_WARN("Could not obtain local date from mount!");
        return false;
    }

    // To ISO 8601 format in LOCAL TIME!
    char datetime[MAXINDINAME]={0};
    snprintf(datetime, MAXINDINAME, "%sT%s", cdate, ctime);

    // Now that date+time are combined, let's get tm representation of it.
    if (strptime(datetime, "%FT%T", &ltm) == nullptr)
    {
        LOGF_WARN("Could not process mount date and time: %s", datetime);
        return false;
    }

    // Get local time epoch in UNIX seconds
    time_epoch = mktime(&ltm);

    // LOCAL to UTC by subtracting offset.
    time_epoch -= static_cast<int>(offset * 3600.0);

    // Get UTC (we're using localtime_r, but since we shifted time_epoch above by UTCOffset, we should be getting the real UTC time)
    localtime_r(&time_epoch, &utm);

    // Format it into the final UTC ISO 8601
    strftime(cdate, MAXINDINAME, "%Y-%m-%dT%H:%M:%S", &utm);
    IUSaveText(&TimeT[0], cdate);

    LOGF_DEBUG("Mount controller UTC Time: %s", TimeT[0].text);
    LOGF_DEBUG("Mount controller UTC Offset: %s", TimeT[1].text);

    // Let's send everything to the client
    TimeTP.s = IPS_OK;
    IDSetText(&TimeTP, nullptr);

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getLocalDate(char *dateString)
{
    LOG_DEBUG(__FUNCTION__);
// The StarGo does not save local date or time (or at least does not
// provide a way to query them
// So just use the driver date
    time_t now = time (nullptr);
    strftime(dateString, 32, "%F", localtime(&now));
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::setLocalDate(uint8_t days, uint8_t months, uint16_t years)
{
    LOGF_DEBUG("%s days=%d, months=%d, years=%d", __FUNCTION__, days, months, years);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH]={0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH]={0};

    int yy = years % 100;

// Use X50 using DDMMYY
//    snprintf(cmd, sizeof(cmd), ":SC %02d%02d%02d#", months, days, yy);
    snprintf(cmd, sizeof(cmd), ":X50%02d%02d%02d#", days, months, yy);
    if (!sendQuery(cmd, response, 0))  // No response
    {
        LOG_ERROR("Failed to set date");
        return false;
    }

    if (response[0] == '0')
    {
        LOG_ERROR("Invalid reponse to set date");
        return false;
    }

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getLocalTime(char *timeString)
{
    LOG_DEBUG(__FUNCTION__);
// StarGo does not store local date or time
// It does store LST as time of day so that can be converted back to a time of day
    time_t now = 0;
    strftime(timeString, 32, "%T", localtime(&now));
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::setLocalTime24(uint8_t hour, uint8_t minute, uint8_t second)
{
    LOGF_DEBUG("s hour=%d, minute=%d second=%d", __FUNCTION__, hour, minute, second);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH]={0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH]={0};

    snprintf(cmd, sizeof(cmd), ":SL %02d:%02d:%02d#", hour, minute, second);

    return (sendQuery(cmd, response, 0));
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getUTCOffset(double *offset)
{
    LOG_DEBUG(__FUNCTION__);
    // StarGo does not store the UTC offset
    if (isSimulation())
    {
        *offset = 3;
        return true;
    }
    time_t t = time(NULL);
    struct tm lt = {0,0,0,0,0,0,0,0,0,0,""};
    localtime_r(&t, &lt);

    *offset = static_cast<double>(lt.tm_gmtoff)/3600;
    
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::setUTCOffset(double offset)
{
    LOGF_DEBUG("%s offset=%lf", __FUNCTION__, offset);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH]={0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH]={0};
    int hours = offset * -1.0;

    snprintf(cmd, sizeof(cmd), ":SG %+03d#", hours);

    return (sendQuery(cmd, response, 0));
}

/*******************************************************************************
 * @brief Check whether the mount is synched or parked.
 * @param status 0=unparked, 1=at home position, 2=parked
 *               A=slewing home, B=slewing to park position
 * @return true if the command succeeded, false otherwise
*******************************************************************************/
bool StarGoTelescope::getParkHomeStatus (char* status)
{
    LOG_DEBUG(__FUNCTION__);
    // Command   - :X38#
    // Answers:
    // p0 - unparked
    // p1 - at home position
    // p2 - parked
    // pA - slewing home
    // pB - slewing to park position

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":X38#", response))
    {
        LOG_ERROR("Failed to send get parking status request.");
        return false;
    }

//    LOGF_DEBUG("response: %s", response);

    if (! sscanf(response, "p%s[012AB]", status))
    {
        LOGF_ERROR("Unexpected park home status response '%s'.", response);
        return false;
    }

    return true;
}

/*******************************************************************************
** setHomeSync
** Sets the current mount position as the Home position
** Called from ISNewSwitch when the Synch Home button is clicked
*******************************************************************************/
bool StarGoTelescope::setHomeSync()
{
    LOG_DEBUG(__FUNCTION__);
    // Command Sync Home     :X31hhmmss#
    // hhmmss is local sidereal time
    char input[AVALON_COMMAND_BUFFER_LENGTH-5] = {0};
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    if (!getLST_String(input))
    {
        LOG_WARN("Synching home get LST failed.");
        SyncHomeSP.s = IPS_ALERT;
        SyncHomeS[0].s = ISS_OFF;
        IDSetSwitch(&SyncHomeSP, nullptr);
        return false;
    }

    sprintf(cmd, ":X31%s#", input);
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if (sendQuery(cmd, response))
    {
        LOG_INFO("Synching home position succeeded.");
        SyncHomeSP.s = IPS_OK;
    }
    else
    {
        LOG_WARN("Synching home position failed.");
        SyncHomeSP.s = IPS_ALERT;
        SyncHomeS[0].s = ISS_OFF;
        IDSetSwitch(&SyncHomeSP, nullptr);
        return false;
    }
    SyncHomeS[0].s = ISS_OFF;
    IDSetSwitch(&SyncHomeSP, nullptr);
    
// Confirm by getting RA/Dec (X590) and mount LST (GS)
// Calculate HA = LST-RA
// Dec should be the pole

    
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::setParkPosition()
{
    LOG_DEBUG(__FUNCTION__);
    // Command  - :X352#
    // Response - 0#
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":X352#", response))
    {
        LOG_ERROR("Failed to send mount set park position command.");
        return false;
    }
    if (response[0] != '0')
    {
        LOGF_ERROR("Invalid mount set park position response '%s'.", response);
        return false;
    }
    return true;
}

/*******************************************************************************
*
*******************************************************************************/
bool StarGoTelescope::setGotoHome()
{
    LOG_DEBUG(__FUNCTION__);
    // Command  - :X361#
    // Response - pA#
    //            :Z1303#
    //            p0#
    //            :Z1003#
    //            p0#
    char response[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    if (!sendQuery(":X361#", response))
    {
        LOG_ERROR("Failed to send mount goto home command.");
        return false;
    }
    if (strcmp(response, "pA") != 0)
    {
        LOGF_ERROR("Invalid send mount goto home response '%s'.", response);
        return false;
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
void StarGoTelescope::WaitParkOptionReady()
{
    // Check if waiting for park position to set. Reset in Abort())
    if(!ParkOptionBusy) return;

    // Check if the mount has stopped slewing
//        SCOPE_IDLE        ready
//        SCOPE_SLEWING -   not ready
//        SCOPE_TRACKING    ready
//        SCOPE_PARKING     not ready - error
//        SCOPE_PARKED      not ready error
// If it has then set park position. Otherwise wait for next call
    if(TrackState != SCOPE_IDLE &&
       TrackState != SCOPE_TRACKING ) return;

    double longitude;
    bool rc = setParkPosition();
    ParkOptionSP.s = IPS_ALERT;
    if(!rc)
    {
        LOG_WARN("Unable to set Park Position.");
    }
    else if (!getSiteLongitude(&longitude))
    {
        LOG_WARN("Failed to get site Longitude from device.");
    }
    else
    {
    // determine local sidereal time
        double lst = get_local_sidereal_time(longitude);
        SetAxis1Park(lst - EqN[AXIS_RA].value);
        SetAxis2Park(EqN[AXIS_DE].value);
        ParkOptionSP.s = IPS_OK;
    }
    IDSetSwitch(&ParkOptionSP, nullptr);
    ParkOptionBusy = false;
    return;
}

/*******************************************************************************
 * @brief Determine the max slew rates for RA and DEC axis
 * @param raSlew max slew for RA axis
 * @param decSlew max slew for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::getMaxSlews(int *raSlew, int *decSlew)
{
    LOG_DEBUG(__FUNCTION__);
    // Command query max slew rates  - :TTGMX#
    //         response              - xxayy#
    //         xx RA; yy DEC

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":TTGMX#", response))
    {
        LOG_ERROR("Failed to send get RAmax slew rates request.");
        return false;
    }
    if (! sscanf(response, "%02da%2d", raSlew, decSlew))
    {
        LOGF_ERROR("Unexpected max slew response '%s'.", response);
        return false;
    }

    return true;
}

/*******************************************************************************
 * @brief Set the max slew rates for RA and DEC axis
 * @param raSlew max slew for RA axis
 * @param decSlew max slew for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::setMaxSlews(int raSlew, int decSlew)
{
    LOG_DEBUG(__FUNCTION__);
    // Command query max slew rates  - :TTMX#
    //         parama                - xxyy#
    //         xx RA; yy DEC

    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    sprintf(cmd, ":TTMX%2d%2d", raSlew, decSlew);
    if (sendQuery(cmd, response))
    {
        LOGF_INFO("Setting Max Slews to %2d %2d.", raSlew, decSlew);
    }
    else
    {
        LOGF_ERROR("Setting Max Slews to %2d %2d FAILED", raSlew, decSlew);
        return false;
    }
    return true;
}

/*******************************************************************************
 * @brief Determine the centering and finding speeds for RA and DEC axis
 * @param raSpeed factor for RA axis
 * @param decSpeed factor for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::getMoveSpeed(int *center, int * find )
{
    LOG_DEBUG(__FUNCTION__);
    INDI_UNUSED(center);
    INDI_UNUSED(find);
    /*
 * Set centre and find speeds
 * X03aaaabbbb aaaa=center speed; bbbb=findspeed
 * valid center speeds:
2x  = 007:  = 0x7a = 122
3x  = 0051  = 0x51 = 81
4x  = 003=  = 0x3d = 61
6x  = 0028  = 0x28 = 40
8x  = 001>  = 0x1e = 30
10x = 0018  = 0x18 = 24

valid find speeds:
10x  = 0031  = x31 = 49
15x  = 0020  = x20 = 32
20x  = 0018  = x18 = 24
30x  = 0010  = x10 = 16
50x  = 000:  = x0a = 10
75x  = 0006  = x06 = 6
100x = 0005  = x05 = 5
150x = 0003  = x03 = 3
*/
    LOGF_DEBUG("%s %s", __FUNCTION__, "Not Implemented");
    return false;

}
/*******************************************************************************
 * @brief Set the centering and findign speeds for RA and DEC axis
 * @param raSpeed factor for RA axis
 * @param decSpeed factor for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::setMoveSpeed(int center, int find )
{
    LOG_DEBUG(__FUNCTION__);
    /*
 * Set centre and find speeds
 * X03aaaabbbb aaaa=center speed; bbbb=findspeed
 * valid center speeds:
2x  = 007:  = 0x7a = 122
3x  = 0051  = 0x51 = 81
4x  = 003=  = 0x3d = 61
6x  = 0028  = 0x28 = 40
8x  = 001>  = 0x1e = 40
10x = 0018  = 0x18 = 24
Parameter = 240/factor
valid find speeds:
10x  = 0031  = x31 = 49
15x  = 0020  = x20 = 32
20x  = 0018  = x18 = 24
30x  = 0010  = x10 = 16
50x  = 000:  = x0a = 10
75x  = 0006  = x06 = 6
100x = 0005  = x05 = 5
150x = 0003  = x03 = 3
Parameter = 480/factor
*/
    double centerpar = double(center) / 240.0;
    double findpar = double(find) / 480.0;

    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    char centerstr[9] = {0};
    char findstr[9] = {0};
    int2ahex(centerstr, centerpar);
    int2ahex(findstr, findpar);
    sprintf(cmd, ":X03%4s%4s", centerstr, findstr);
    if (sendQuery(cmd, response))
    {
        LOGF_INFO("Setting Center and Find to %2d %2d.", center, find);
    }
    else
    {
        LOGF_ERROR("Setting Center and Find to %2d %2d FAILED", center, find);
        return false;
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::setSlewMode(int slewMode)
{
    LOGF_DEBUG("%s mode=%d", __FUNCTION__, slewMode);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    switch (slewMode)
    {
        case SLEW_MAX:
            strcpy(cmd, ":RS#");
            break;
        case SLEW_FIND:
            strcpy(cmd, ":RM#");
            break;
        case SLEW_CENTERING:
            strcpy(cmd, ":RC#");
            break;
        case SLEW_GUIDE:
            strcpy(cmd, ":RG#");
            break;
        default:
            return false;
    }
    if (!sendQuery(cmd, response, 0)) // Don't wait for response - there isn't one
    {
        LOG_ERROR("Error communication with telescope.");
        return false;
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getEqCoordinates (double *ra, double *dec)
{
    LOG_DEBUG(__FUNCTION__);
    // Use X590 for RA DEC
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if(!sendQuery(":X590#", response))
    {
        LOGF_ERROR("Unable to get RA and DEC %s", response);
        return false;
    }
    double r, d;
    int returnCode = sscanf(response, "RD%08lf%08lf", &r, &d);
    if (returnCode < 2)
    {
       LOGF_ERROR("Failed to parse RA and Dec response '%s'.", response);
       return false;
    }
    *ra  = r / 1.0e6;
    *dec = d / 1.0e5;

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::setObjectCoords(double ra, double dec)
{
    LOGF_DEBUG("%s ra=%lf dec=%lf", __FUNCTION__, ra, dec);

    char RAStr[AVALON_COMMAND_BUFFER_LENGTH]={0};
    char DecStr[AVALON_COMMAND_BUFFER_LENGTH]={0};
    int h, m, s, d;
        getSexComponents(ra, &h, &m, &s);
        snprintf(RAStr, sizeof(RAStr), ":Sr%02d:%02d:%02d#", h, m, s);
        getSexComponents(dec, &d, &m, &s);
        /* case with negative zero */
        if (!d && dec < 0)
            snprintf(DecStr, sizeof(DecStr), ":Sd-%02d*%02d:%02d#", d, m, s);
        else
            snprintf(DecStr, sizeof(DecStr), ":Sd%+03d*%02d:%02d#", d, m, s);
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (isSimulation()) return true;
// These commands receive a response without a terminating #
    if(!sendQuery(RAStr, response, '1', 2)  || !sendQuery(DecStr, response, '1', 2) )
    {
        EqNP.s = IPS_ALERT;
        IDSetNumber(&EqNP, "Error setting RA/DEC.");
        return false;
    }

    return true;
}

/*******************************************************************************
 * @brief Retrieve pier side of the mount and sync it back to the client
 * @return true iff synching succeeds
*******************************************************************************/
bool StarGoTelescope::getSideOfPier()
{
    LOG_DEBUG(__FUNCTION__);
    // Command query side of pier - :X39#
    //         side unknown       - PX#
    //         east pointing west - PE#
    //         west pointing east - PW#

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":X39#", response))
    {
        LOG_ERROR("Failed to send query pier side.");
        return false;
    }
    char answer;

    if (! sscanf(response, "P%c", &answer))
    {
        LOGF_ERROR("Unexpected query pier side response '%s'.", response);
        return false;
    }

    switch (answer)
    {
    case 'X':
        LOG_DEBUG("Detected pier side unknown.");
        setPierSide(INDI::Telescope::PIER_UNKNOWN);
        break;
    case 'W':
        // seems to be vice versa
        LOG_DEBUG("Detected pier side west.");
        setPierSide(INDI::Telescope::PIER_WEST);
        break;
    case 'E':
        LOG_DEBUG("Detected pier side east.");
        setPierSide(INDI::Telescope::PIER_EAST);
        break;
    default:
        break;
    }

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::GetMeridianFlipMode(int* index)
{
    LOG_DEBUG(__FUNCTION__);

// 0: Auto mode: Enabled and not Forced
// 1: Disabled mode: Disabled and not Forced
// 2: Forced mode: Enabled and Forced
    const char* disablecmd = ":TTGFs#";
    const char* forcecmd  = ":TTGFd#";
    char disableresp[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    char forceresp[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if(!sendQuery(disablecmd, disableresp) || !sendQuery(forcecmd, forceresp))
    {
        LOGF_ERROR("Cannot get Meridian Flip Mode %s %s", disableresp, forceresp);
        return false;
    }
    int disable = 0;
    if (! sscanf(disableresp, "vs%01d", &disable))
    {
        LOGF_ERROR("Invalid meridian flip disabled response '%s", disableresp);
        return false;
    }
    int force = 0;
    if (! sscanf(forceresp, "vd%01d", &force))
    {
        LOGF_ERROR("Invalid meridian flip forced response '%s", forceresp);
        return false;
    }
    if( disable == 1)
    {
        *index = 1; // disabled
        LOG_WARN("Meridian flip DISABLED. BE CAREFUL, THIS MAY CAUSE DAMAGE TO YOUR MOUNT!");
    }
    else if( force == 0)
    {
        *index = 0; // auto
        LOG_INFO("Meridian flip enabled.");
    }
    else
    {
        *index = 2; // forced
        LOG_WARN("Meridian flip FORCED. BE CAREFUL, THIS MAY CAUSE DAMAGE TO YOUR MOUNT!");
    }

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::SetMeridianFlipMode(int index)
{
    LOGF_DEBUG("%s index=%d", __FUNCTION__, index);
    // 0: Auto mode: Enabled and not Forced
    // 1: Disabled mode: Disabled and not Forced
    // 2: Forced mode: Enabled and Forced

    if (isSimulation())
    {
        MeridianFlipModeSP.s = IPS_OK;
        IDSetSwitch(&MeridianFlipModeSP, nullptr);
        return true;
    }
    if( index > 2)
    {
        LOGF_ERROR("Invalid Meridian Flip Mode %d", index);
        return false;
    }
    const char* disablecmd = index==1 ? ":TTSFs#" : ":TTRFs#";
    const char* forcecmd  = index==2 ? ":TTSFd#" : ":TTRFd#";
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if(!sendQuery(disablecmd, response) || !sendQuery(forcecmd, response))
    {
        LOGF_ERROR("Cannot set Meridian Flip Mode %d", index);
        return false;
    }

    switch (index)
    {
    case 0:
        LOG_INFO("Meridian flip enabled.");
        break;
    case 1:
        LOG_WARN("Meridian flip DISABLED. BE CAREFUL, THIS MAY CAUSE DAMAGE TO YOUR MOUNT!");
        break;
    case 2:
        LOG_WARN("Meridian flip FORCED. BE CAREFUL, THIS MAY CAUSE DAMAGE TO YOUR MOUNT!");
        break;
    }

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getTrackingAdjustment(double *valueRA)
{
    /*
     * :X42# to read the tracking adjustment value as orsRRR#
     * :X44# to read the tracking adjustment value as odsDDD#

     */
    LOG_DEBUG(__FUNCTION__);
    int raValue;
    char response[RB_MAX_LEN] = {0};

    if (!sendQuery(":X42#", response))
        return false;

    if (sscanf(response, "or%04i#", &raValue) < 1)
    {
        LOG_ERROR("Unable to parse tracking adjustment response");
        return false;
    }

    *valueRA = static_cast<double>(raValue / 100.0);
    return true;
}

/*******************************************************************************
 * Adjust RA tracking speed.
*******************************************************************************/
bool StarGoTelescope::setTrackingAdjustment(double adjustRA)
{
    LOG_DEBUG(__FUNCTION__);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH];
    /*
     * :X41sRRR# to adjust the RA tracking speed where s is the sign + or -  and RRR are three digits whose meaning is parts per 10000 of  RA correction .
     * :X43sDDD# to fix the cf DEC offset
     X41 command only applies whole number percentages i.e +/- 100, 200, 300, 400, 500.
     
     :X1Ennnn # where nnnn is between 0500 and 1500. 1000 represents no adjustment and 0500 is -5% and 1500 is +5%
     Ascertained from the StarGo ASCOM driver
     */

    // ensure that -5 <= adjust <= 5
    if (adjustRA > 5.0)
    {
        LOGF_ERROR("Adjusting tracking by %0.2f%% not allowed. Maximal value is 5.0%%", adjustRA);
        return false;
    }
    else if (adjustRA < -5.0)
    {
        LOGF_ERROR("Adjusting tracking by %0.2f%% not allowed. Minimal value is -5.0%%", adjustRA);
        return false;
    }

    int parameter = static_cast<int>(adjustRA * 100); // + 1000; // Add 1000 to X41 value for X1E1 value
    sprintf(cmd, ":X41%+04i#", parameter);
//    sprintf(cmd, ":X1E%04d", parameter);

//   if(!transmit(cmd))
//   {
//       LOGF_ERROR("Cannot adjust tracking by %d%%", adjustRA);
//       return false;
//   }
//    LOG_ERROR("Adjusting tracking is disabled");

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if(!sendQuery(cmd, response, 0))
    {
        LOGF_ERROR("Cannot adjust tracking by %d%%", adjustRA);
        return false;
    }
    if (adjustRA == 0.0)
        LOG_INFO("RA tracking adjustment cleared.");
    else
        LOGF_INFO("RA tracking adjustment to %+0.2f%% succeded.", adjustRA);

    return true;
}

/*******************************************************************************
 * @brief Determine the guiding speeds for RA and DEC axis
 * @param raSpeed percentage for RA axis
 * @param decSpeed percenage for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::getGuidingSpeeds (int *raSpeed, int *decSpeed)
{
    LOG_DEBUG(__FUNCTION__);
    // Command query guiding speeds  - :X22#
    //         response              - rrbdd#
    //         rr RA speed percentage, dd DEC speed percentage

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if (!sendQuery(":X22#", response))
    {
        LOG_ERROR("Failed to send query guiding speeds request.");
        return false;
    }
    if (! sscanf(response, "%02db%2d", raSpeed, decSpeed))
    {
        LOGF_ERROR("Unexpected guiding speed response '%s'.", response);
        return false;
    }

    return true;
}

/*******************************************************************************
 * @brief Set the guiding speeds for RA and DEC axis
 * @param raSpeed percentage for RA axis
 * @param decSpeed percenage for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::setGuidingSpeeds (int raSpeed, int decSpeed)
{
    LOGF_DEBUG("%s raSpeed=%d, decSpeed=%d", __FUNCTION__, raSpeed, decSpeed);
    // in RA guiding speed  -  :X20rr#
    // in DEC guiding speed - :X21dd#

    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    sprintf(cmd, ":X20%2d#", raSpeed);
    if (sendQuery(cmd, response, 0)) // No response from mount
    {
        LOGF_INFO("Setting RA speed to %2d%%.", raSpeed);
    }
    else
    {
        LOGF_ERROR("Setting RA speed to %2d %% FAILED", raSpeed);
        return false;
    }
    const struct timespec timeout = {0, 100000000L};
    // sleep for 100 mseconds
    nanosleep(&timeout, nullptr);

    sprintf(cmd, ":X21%2d#", decSpeed);
    if (sendQuery(cmd, response, 0))  // No response from mount
    {
        LOGF_INFO("Setting DEC speed to %2d%%.", decSpeed);
    }
    else
    {
        LOGF_ERROR("Setting DEC speed to %2d%% FAILED", decSpeed);
        return false;
    }
    return true;
}

/*******************************************************************************
 * @brief Check if the ST4 port is enabled
 * @param isEnabled - true iff the ST4 port is enabled
 * @return
*******************************************************************************/
bool StarGoTelescope::getST4Status (bool *isEnabled)
{
     LOG_DEBUG(__FUNCTION__);
    // Command query ST4 status  - :TTGFh#
    //         response enabled  - vh1
    //                  disabled - vh0

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if (!sendQuery(":TTGFh#", response))
    {
        LOG_ERROR("Failed to send query ST4 status request.");
        return false;
    }
    int answer = 0;
    if (! sscanf(response, "vh%01d", &answer))
    {
        LOGF_ERROR("Unexpected ST4 status response '%s'.", response);
        return false;
    }

    *isEnabled = (answer == 1);
    return true;
}

/*******************************************************************************
 * @brief Enable or disable the ST4 guiding port
 * @param enabled flag whether enable or disable
 * @return
*******************************************************************************/
bool StarGoTelescope::setST4Enabled(bool enabled)
{
    LOGF_DEBUG("%s enabled=%d", __FUNCTION__, enabled);

    const char *cmd = enabled ? ":TTSFh#" : ":TTRFh#";
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (sendQuery(cmd, response))
    {
        LOG_INFO(enabled ? "ST4 port enabled." : "ST4 port disabled.");
        usePulseCommand = !(enabled);
        return true;
    }
    else
    {
        LOG_ERROR("Setting ST4 port FAILED");
        return false;
    }
}

/*******************************************************************************
**
*******************************************************************************/
int StarGoTelescope::SendPulseCmd(int8_t direction, uint32_t duration_msec)
{
    LOGF_DEBUG("%s dir=%d dur=%d ms", __FUNCTION__, direction, duration_msec );
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if(!usePulseCommand) return true;

    if (MovementNSSP.s == IPS_BUSY || MovementWESP.s == IPS_BUSY)
    {
        LOG_ERROR("Cannot guide while moving.");
        return IPS_ALERT;
    }
    if (isParked())
    {
        LOG_ERROR("Cannot guide while parked.");
        return IPS_ALERT;
    }

    switch (direction)
    {
        case STARGO_NORTH:
            sprintf(cmd, ":Mgn%04d#", duration_msec);
            break;
        case STARGO_SOUTH:
            sprintf(cmd, ":Mgs%04d#", duration_msec);
            break;
        case STARGO_EAST:
            sprintf(cmd, ":Mge%04d#", duration_msec);
            break;
        case STARGO_WEST:
            sprintf(cmd, ":Mgw%04d#", duration_msec);
            break;
        default:
            return 1;
    }
    if (!sendQuery(cmd, response, 0)) // Don't wait for response - there isn't one
    {
        LOG_ERROR("Failed to send guide pulse request.");
        return false;
    }
// Assume the guide pulse was issued and acted upon.
    bool adjEnabled = (IUFindOnSwitchIndex(&RaAutoAdjustSP) == DefaultDevice::INDI_ENABLED);

    if((direction == STARGO_EAST || direction == STARGO_WEST) && adjEnabled)
    {
        autoRa->setRaAdjust(direction, duration_msec);
    }
    return true;
}

/**************************************************************************************
**getBasicData is called from updateProperties whenever a client connects to the driver
* It could instead be called from Handshake whenever the driver connects to the mount
* It initialises driver properties from the mount before they are updated from the config file
***************************************************************************************/
void StarGoTelescope::getBasicData()
{
    LOG_DEBUG(__FUNCTION__);

    if (!isSimulation())
    {
        MountFirmwareInfoT[0].text = new char[64];
        MountFirmwareInfoT[1].text = new char[64];
        MountFirmwareInfoT[2].text = new char[64];
        if (!getFirmwareInfo(MountFirmwareInfoT[0].text,
                MountFirmwareInfoT[1].text,
                MountFirmwareInfoT[2].text ))
            LOG_ERROR("Failed to get firmware from device.");
        else
            IDSetText(&MountFirmwareInfoTP, nullptr);

        char parkHomeStatus[2] = {'\0','\0'};
        if (getParkHomeStatus(parkHomeStatus))
        {
            SetParked(strcmp(parkHomeStatus, "2") == 0);
            if (strcmp(parkHomeStatus, "1") == 0)
            {
                SyncHomeSP.s = IPS_OK;
                IDSetSwitch(&SyncHomeSP, nullptr);
            }
        }
        bool isEnabled;
        if (getST4Status(&isEnabled))
        {
            ST4StatusS[INDI_ENABLED].s = isEnabled ? ISS_ON : ISS_OFF;
            ST4StatusS[INDI_DISABLED].s = isEnabled ? ISS_OFF : ISS_ON;
            ST4StatusSP.s = IPS_OK;
        }
        else
        {
            ST4StatusSP.s = IPS_ALERT;
        }
        IDSetSwitch(&ST4StatusSP, nullptr);

        if (getKeypadStatus(&isEnabled))
        {
            KeypadStatusS[INDI_ENABLED].s = isEnabled ? ISS_ON : ISS_OFF;
            KeypadStatusS[INDI_DISABLED].s = isEnabled ? ISS_OFF : ISS_ON;
            KeypadStatusSP.s = IPS_OK;
        }
        else
        {
            KeypadStatusSP.s = IPS_ALERT;
        }
        IDSetSwitch(&KeypadStatusSP, nullptr);

        int index;
        if (GetMeridianFlipMode(&index))
        {
                IUResetSwitch(&MeridianFlipModeSP);
                MeridianFlipModeS[index].s = ISS_ON;
                MeridianFlipModeSP.s   = IPS_OK;
        }
        else
        {
            MeridianFlipModeSP.s = IPS_ALERT;
        }
        IDSetSwitch(&MeridianFlipModeSP, nullptr);

// Get the guiding speed
        int raSpeed, decSpeed;
        if(getGuidingSpeeds(&raSpeed, &decSpeed))
        {
            GuidingSpeedN[0].value =  raSpeed/100.0;
            GuidingSpeedN[1].value =  decSpeed/100.0;
            GuidingSpeedNP.s = IPS_OK;
        }
        else
        {
            LOG_ERROR("Unable to get guiding speed");
            GuidingSpeedNP.s = IPS_ALERT;
        }
        IDSetNumber(&GuidingSpeedNP, nullptr);   

        double raCorrection;
        if (getTrackingAdjustment(&raCorrection))
        {
            TrackAdjustN[0].value = raCorrection;
            TrackAdjustNP.s      = IPS_OK;
        }
        else
        {
            TrackAdjustNP.s = IPS_ALERT;
        }
        IDSetNumber(&TrackAdjustNP, nullptr);

        int raRatio, decRatio;
        if (getGearRatios(&raRatio, &decRatio))
        {
            GearRatioN[0].value = static_cast<double>(raRatio);
            GearRatioN[1].value = static_cast<double>(decRatio);
            GearRatioNP.s = IPS_OK;
        }
        else
        {
            GearRatioNP.s = IPS_ALERT;
        }
        IDSetNumber(&GearRatioNP, nullptr);

        int torque;
        if(getTorque(&torque))
        {
            TorqueN[0].value =  static_cast<double>(torque);
            TorqueNP.s = IPS_OK;
        }
        else
        {
            LOG_ERROR("Unable to get torque");
            TorqueNP.s = IPS_ALERT;
        }
        IDSetNumber(&TorqueNP, nullptr);   

        bool raDir, decDir;
        if (getMotorReverse(&raDir, &decDir))
        {
            RaMotorReverseS[INDI_ENABLED].s = raDir ? ISS_ON : ISS_OFF;
            RaMotorReverseS[INDI_DISABLED].s = raDir ? ISS_OFF : ISS_ON;
            RaMotorReverseSP.s = IPS_OK;
            DecMotorReverseS[INDI_ENABLED].s = decDir ? ISS_ON : ISS_OFF;
            DecMotorReverseS[INDI_DISABLED].s = decDir ? ISS_OFF : ISS_ON;
            DecMotorReverseSP.s = IPS_OK;
        }
        else
        {
            RaMotorReverseSP.s = IPS_ALERT;
            DecMotorReverseSP.s = IPS_ALERT;
        }
        IDSetSwitch(&RaMotorReverseSP, nullptr);
        IDSetSwitch(&DecMotorReverseSP, nullptr);

        int raSlew, decSlew;
        if (getMaxSlews(&raSlew, &decSlew))
        {
            MaxSlewN[0].value = static_cast<double>(raSlew);
            MaxSlewN[1].value = static_cast<double>(decSlew);
            MaxSlewNP.s = IPS_OK;
        }
        else
        {
            MaxSlewNP.s = IPS_ALERT;
        }
        IDSetNumber(&MaxSlewNP, nullptr);

        int centerSpeed, findSpeed;
        if (getMoveSpeed(&centerSpeed, &findSpeed ))
        {
            MoveSpeedN[0].value = static_cast<double>(centerSpeed);
            MoveSpeedN[1].value = static_cast<double>(findSpeed);
            MoveSpeedNP.s = IPS_OK;
        }
        else
        {
            MoveSpeedNP.s = IPS_ALERT;
        }
        IDSetNumber(&MoveSpeedNP, nullptr);

    }
    if (getLocationOnStartup && (GetTelescopeCapability() & TELESCOPE_HAS_LOCATION))
        getScopeLocation();

    if (getTimeOnStartup && (GetTelescopeCapability() & TELESCOPE_HAS_TIME))
        getScopeTime();

    usePulseCommand = true;

}

/*******************************************************************************
// getScopeAlignmentStatus
// Returns: <mount><tracking><alignment># where:
*******************************************************************************/
bool StarGoTelescope::getScopeAlignmentStatus(char *mountType, bool *isTracking, int *alignmentPoints)
{
// mount: A-AzEl mounted, P-Equatorially mounted, G-german mounted equatorial
// tracking: T-tracking, N-not tracking
// alignment: 0-needs alignment, 1-one star aligned, 2-two star aligned, 3-three star aligned.

    LOG_DEBUG(__FUNCTION__);
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if(!sendQuery(":GW#", response))
    {
        LOG_ERROR("Error communication with telescope.");
        return false;
    }

    char mt, tracking;
    int nr;
    int returnCode = sscanf(response, "%c%c%01d", &mt, &tracking, &nr);
    if (returnCode < 3)
    {
       LOGF_ERROR("Failed to parse scope alignment status response '%s'.", response);
       return false;
    }

    *mountType = mt;
    *isTracking = (tracking == 'T');
    *alignmentPoints = nr;
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getMotorStatus(int *xSpeed, int *ySpeed)
{
    LOG_DEBUG(__FUNCTION__);
    // Command  - :X34#
    // the StarGo replies mxy# where x is the RA / AZ motor status and y
    // the DEC / ALT motor status meaning:
    //    x (y) = 0 motor x (y) stopped or unpowered
    //             (use :X3C# if you want  distinguish if stopped or unpowered)
    //    x (y) = 1 motor x (y) returned in tracking mode
    //    x (y) = 2 motor x (y) acelerating
    //    x (y) = 3 motor x (y) decelerating
    //    x (y) = 4 motor x (y) moving at low speed to refine
    //    x (y) = 5 motor x (y) moving at high speed to target

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if(!sendQuery(":X34#", response))
    {
        LOG_ERROR("Failed to get motor state");
        return false;
    }
    int x, y;
    int returnCode = sscanf(response, "m%01d%01d", &x, &y);
    if (returnCode < 2)
    {
       LOGF_ERROR("Failed to parse motor state response '%s'.", response);
       return false;
    }
    *xSpeed = x;
    *ySpeed = y;
    LOGF_DEBUG("Motor state = (%d, %d)", *xSpeed, *ySpeed);
    return true;
}

/*******************************************************************************
** @brief Check if the Keypad port is enabled
** @param isEnabled - true iff the Keypad port is enabled
** @return
*******************************************************************************/
bool StarGoTelescope::getKeypadStatus (bool *isEnabled)
{
    LOG_DEBUG(__FUNCTION__);
    // Command query Keypad status  - :TTGFr#
    //            response enabled  - vr1
    //                     disabled - vr0

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if (!sendQuery(":TTGFr#", response))
    {
        LOG_ERROR("Failed to send query Keypad status request.");
        return false;
    }
    int answer = 0;
    if (! sscanf(response, "vr%01d", &answer))
    {
        LOGF_ERROR("Unexpected Keypad status response '%s'.", response);
        return false;
    }

    *isEnabled = (answer == 0);
    return true;
}

/*******************************************************************************
 * @brief Enable or disable the Keypad port
 * @param enabled flag whether enable or disable
 * @return
*******************************************************************************/
bool StarGoTelescope::setKeyPadEnabled(bool enabled)
{

    const char *cmd = enabled ? ":TTRFr#" : ":TTSFr#";
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (sendQuery(cmd, response))
    {
        LOG_INFO(enabled ? "Keypad port enabled." : "Keypad port disabled.");
        return true;
    }
    else
    {
        LOG_ERROR("Setting Keypad port FAILED");
        return false;
    }

}

/*******************************************************************************
 * @brief Retrieve the firmware info from the mount
 * @param firmwareInfo - firmware description
 * @return
*******************************************************************************/
bool StarGoTelescope::getFirmwareInfo (char* firmwareInfo, char *mount, char *tcb )
{
/*
 * Manufactuer
 *      GVP
 * Firmware version
 *      GVN
 * Firmware date
 *      GVD
 * TCB version:
 *      X29 TCB=0000247
 * Get Mount Type
 *      TTGM
 */
    LOG_DEBUG(__FUNCTION__);
    std::string infoStr;
    char manufacturer[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    // step 1: retrieve manufacturer
    if (!sendQuery(":GVP#", manufacturer))
    {
        LOG_ERROR("Failed to send get manufacturer request.");
        return false;
    }
    infoStr.assign(manufacturer);

    // step 2: retrieve firmware version
    char firmwareVersion[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":GVN#", firmwareVersion))
    {
        LOG_ERROR("Failed to send get firmware version request.");
        return false;
    }
    infoStr.append(" - ").append(firmwareVersion);

    // step 3: retrieve firmware date
    char firmwareDate[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":GVD#", firmwareDate))
    {
        LOG_ERROR("Failed to send get firmware date request.");
        return false;
    }
    std::string dateStr = firmwareDate;
    infoStr.append(" - ").append(dateStr, 1, dateStr.length()-1);

    strcpy(firmwareInfo, infoStr.c_str());

    // step 4: get mount type
    char mountType[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":TTGM#", mountType))
    {
        LOG_ERROR("Failed to send get mount type request.");
        return false;
    }
    strcpy(mount, mountType);

    // step 5: get TCB version
    char tcbVer[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":X29#", tcbVer))
    {
        LOG_ERROR("Failed to send get TCB version request.");
        return false;
    }
    strcpy(tcb, tcbVer);

    return true;
}

/*******************************************************************************
** mountSim 
** called from ReadScopeStatus
*******************************************************************************/
void StarGoTelescope::mountSim()
{
    LOG_DEBUG(__FUNCTION__);
/* Simulation Parameters */
    static struct timeval ltv;
    struct timeval tv;
    double dt=0, da=0, dx=0;
    int nlocked=0;

    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday(&tv, nullptr);

    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;

    dt  = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
    ltv = tv;
    da  = STARGO_GENERIC_SLEWRATE * dt;
    
    double currentRA = EqN[AXIS_RA].value;
    double currentDEC = EqN[AXIS_DE].value;
    double targetRA = TargetN[AXIS_RA].value;
    double targetDEC = TargetN[AXIS_DE].value;

    /* Process per current state. We check the state of EQUATORIAL_COORDS and act acoordingly */
    switch (TrackState)
    {

    case SCOPE_IDLE:
        currentRA  += (TRACKRATE_SIDEREAL/3600.0 * dt / 15.);
        break;

    case SCOPE_TRACKING:
        switch (IUFindOnSwitchIndex(&TrackModeSP))
        {
        case TRACK_SIDEREAL:
            da = 0;
            dx = 0;
            break;

        case TRACK_LUNAR:
            da = ((TRACKRATE_LUNAR-TRACKRATE_SIDEREAL)/3600.0 * dt / 15.);
            dx = 0;
            break;

        case TRACK_SOLAR:
            da = ((TRACKRATE_SOLAR-TRACKRATE_SIDEREAL)/3600.0 * dt / 15.);
            dx = 0;
            break;

        case TRACK_CUSTOM:
            da = ((TrackRateN[AXIS_RA].value-TRACKRATE_SIDEREAL)/3600.0 * dt / 15.);
            dx = (TrackRateN[AXIS_DE].value/3600.0 * dt);
            break;

        }

        currentRA  += da;
        currentDEC += dx;
        break;

    case SCOPE_SLEWING:
    case SCOPE_PARKING:
        /* slewing - nail it when both within one pulse @ STARGO_GENERIC_SLEWRATE */
        nlocked = 0;

        dx = targetRA - currentRA;

        if (fabs(dx) <= da)
        {
            currentRA = targetRA;
            nlocked++;
        }
        else if (dx > 0)
            currentRA += da / 15.;
        else
            currentRA -= da / 15.;

        dx = targetDEC - currentDEC;
        if (fabs(dx) <= da)
        {
            currentDEC = targetDEC;
            nlocked++;
        }
        else if (dx > 0)
            currentDEC += da;
        else
            currentDEC -= da;

        if (nlocked == 2)
        {
            if (TrackState == SCOPE_SLEWING)
                TrackState = SCOPE_TRACKING;
            else
                SetParked(true);
        }

        break;

    default:
        break;
    }

    NewRaDec(currentRA, currentDEC);
}

/*******************************************************************************
 * @brief Determine the gear ratios for RA and DEC axis
 * @param raRatio for RA axis
 * @param decRatio for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::getGearRatios(int *raRatio, int *decRatio)
{
    LOG_DEBUG(__FUNCTION__);
    // Command query gear ratios  - :X480# and :X481#
    //         response              - innnnnnnn#
    //         nnnnnnnn Avalon hex

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":X480#", response))
    {
        LOG_ERROR("Failed to send get RA gear ratiorequest.");
        return false;
    }
    *raRatio = ahex2int(&response[2]);
    if (!sendQuery(":X481#", response))
    {
        LOG_ERROR("Failed to send get DEC gear ratiorequest.");
        return false;
    }
    *decRatio = ahex2int(&response[2]);

    return true;
}

/*******************************************************************************
 * @brief Determine the motor position for RA and DEC axis
 * @param raSteps position for RA axis
 * @param decSteps position for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::getMotorSteps(double *raSteps, double *decSteps)
{
    LOG_DEBUG(__FUNCTION__);
    // Command query motor step pos  - :TTGMs0# and :TTGMs1#
    //         response              - xxxxxxxxr; yyyyyyyyd#
    //         nnnnnnnn Avalon hex

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    const double hi=std::pow(2.0,31);
    const double fi=std::pow(2.0,32);
    if (!sendQuery(":TTGMs0#", response))
    {
        LOG_ERROR("Failed to send get RA motor step pos request.");
        return false;
    }
    response[8] = '\0';
    *raSteps = ahex2int(response);
    LOGF_DEBUG("%s RA pos %s %lf", __FUNCTION__, response, *raSteps);
    *raSteps = *raSteps<=hi? *raSteps: (*raSteps - fi);
    if (!sendQuery(":TTGMs1#", response))
    {
        LOG_ERROR("Failed to send get DEC motor steps request.");
        return false;
    }
    response[8] = '\0';
    *decSteps = ahex2int(response);
    LOGF_DEBUG("%s DEC pos %s %lf", __FUNCTION__, response, *decSteps);
    *decSteps = *decSteps<=hi? *decSteps: (*decSteps - fi);

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getMotorReverse(bool *raDir, bool *decDir)
{
    LOG_DEBUG(__FUNCTION__);
//    INDI_UNUSED(raDir);
//    INDI_UNUSED(decDir);
/*
 * Get RA and Dec motor directions (Forward, Reverse)
 * Get X1B => wrd where r=RA; d=DEC
 *
*/
//    LOGF_DEBUG("%s %s", __FUNCTION__, "Not Implemented");
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if (!sendQuery(":X1B#", response))
    {
        LOG_ERROR("Failed to send query Motor Reverse request.");
        return false;
    }
    int radir = 0, decdir=0;
    if (! sscanf(response, "w%01d%01d", &radir, &decdir))
    {
        LOGF_ERROR("Unexpected Motor reverse response '%s'.", response);
        return false;
    }

    *raDir = (radir == 1);
    *decDir = (decdir == 1);
    return true;

}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::setMotorReverse(bool raDir, bool decDir)
{
    LOG_DEBUG(__FUNCTION__);
//    INDI_UNUSED(raDir);
//    INDI_UNUSED(decDir);
/*
 * Set RA and Dec motor directions (Forward, Reverse)
 * Reverse RA X1A0n; DEC X1A1n
 * where n = 0/1
*/
//    LOGF_DEBUG("%s %s", __FUNCTION__, "Not Implemented");
    const char *racmd = raDir ? ":X1A00#" : ":X1A01#";
    const char *deccmd = decDir ? ":X1A10#" : ":X1A11#";
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (sendQuery(racmd, response))
    {
        LOG_INFO(raDir ? "RA reversed." : "RA normal.");
    }
    else
    {
        LOG_ERROR("Setting RA Reverse FAILED");
        return false;
    }
    if (sendQuery(deccmd, response))
    {
        LOG_INFO(decDir ? "DEC reversed." : "DEC normal.");
    }
    else
    {
        LOG_ERROR("Setting DEC Reverse FAILED");
        return false;
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getTorque(int *torque)
{
    LOG_DEBUG(__FUNCTION__);
//    INDI_UNUSED(torque);
/*
 Get: TTGT
 Return tnnn# where nnn is torque %
*/
//    LOGF_DEBUG("%s %s", __FUNCTION__, "Not Implemented");
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(":TTGT#", response))
    {
        LOG_ERROR("Failed to send query get Torque command.");
        return false;
    }
    if (! sscanf(response, "t%03d", torque))
    {
        LOGF_ERROR("Unexpected torque response '%s'.", response);
        return false;
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::setTorque(int torque)
{
    LOG_DEBUG(__FUNCTION__);
//    INDI_UNUSED(torque);
/*
 * Set motor torque %
 :TTTnnn#
*/
//    LOGF_DEBUG("%s %s", __FUNCTION__, "Not Implemented");
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    sprintf(cmd, ":TTT%03d", torque);
    if (sendQuery(cmd, response))
    {
        LOGF_INFO("Setting Torque to %3d%%.", torque);
    }
    else
    {
        LOGF_ERROR("Setting Torque to %3d %% FAILED", torque);
        return false;
    }
    return true;
}

/*******************************************************************************
 * @brief Send a STARGO query to the communication port and read the result.
 * @param cmd STARGO query
 * @param response answer
 * @return true if the command succeeded, false otherwise
*******************************************************************************/
bool StarGoTelescope::sendQuery(const char* cmd, char* response, char end, int wait)
{
    LOGF_DEBUG("%s %s End:%c Wait:%ds", __FUNCTION__, cmd, end, wait);
    response[0] = '\0';
    char lresponse[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    int lbytes=0;
    lresponse [0] = '\0';
    while (receive(lresponse, &lbytes, '#', 0))
    {
        lbytes=0;
        ParseMotionState(lresponse);
        lresponse [0] = '\0';
    }
    flush();
    if(!transmit(cmd))
    {
        LOGF_ERROR("Command <%s> failed.", cmd);
        // sleep for 50 mseconds to avoid flooding the mount with commands
        nanosleep(&mount_request_delay, nullptr);
        return false;
    }
    lresponse[0] = '\0';
    int lwait = wait;
    bool found = false;
    while (receive(lresponse, &lbytes, end, lwait))
    {
//        LOGF_DEBUG("Found response after %ds %s", lwait, lresponse);
        lbytes=0;
        if(! ParseMotionState(lresponse))
        {
            // Take the first response that is no motion state
            if (!found)
                strcpy(response, lresponse);
            found = true;
            lwait = 0;
        }
    }
    flush();
    
    // sleep for 50 mseconds to avoid flooding the mount with commands
    nanosleep(&mount_request_delay, nullptr);
    
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::ParseMotionState(char* state)
{
    LOGF_DEBUG("%s %s", __FUNCTION__, state);
    int lmotor, lmode, lslew;
    if(sscanf(state, ":Z1%01d%01d%01d", &lmotor, &lmode, &lslew)==3
        || sscanf(state, ":Z%01d%01d%01d", &lmotor, &lmode, &lslew)==3) // Starting to see :Znnn responses
    {
        LOGF_DEBUG("Motion state %s=>Motors: %d, Track: %d, SlewSpeed: %d", state, lmotor, lmode, lslew);
        // m = 0 both motors are OFF (no power)
        // m = 1 RA motor OFF DEC motor ON
        // m = 2 RA motor ON DEC motor OFF
        // m = 3 both motors are ON
        switch(lmotor)
        {
            case 0:
                CurrentMotorsState = MOTORS_OFF;
                break;
            case 1:
                CurrentMotorsState = MOTORS_DEC_ONLY;
                break;
            case 2:
                CurrentMotorsState = MOTORS_RA_ONLY;
                break;
            case 3:
                CurrentMotorsState = MOTORS_ON;
                break;
        };
    // Tracking modes
    // t = 0 no tracking at all (not used)
    // t = 1 tracking at moon speed
    // t = 2 tracking at sun speed
    // t = 3 tracking at stars speed (sidereal speed)
        switch(lmode)
        {
            case 0:  // Not used
                // TRACK_NONE removed, do nothing
                //CurrentTrackMode = TRACK_NONE;
                break;
            case 1:
                CurrentTrackMode = TRACK_LUNAR;
                break;
            case 2:
                CurrentTrackMode = TRACK_SOLAR;
                break;
            case 3:
                CurrentTrackMode = TRACK_SIDEREAL;
                break;
        };
    // Slew speed index
    // s = 0 GUIDE speed
    // s = 1 CENTERING speed
    // s = 2 FINDING speed
    // s = 3 MAX speed
        switch(lslew)
        {
            case 0:
                CurrentSlewRate = SLEW_GUIDE;
                break;
            case 1:
                CurrentSlewRate = SLEW_CENTERING;
                break;
            case 2:
                CurrentSlewRate = SLEW_FIND;
                break;
            case 3:
                CurrentSlewRate = SLEW_MAX;
                break;
        };
        return true;
    }
    else
    {
        return false;
    }
}

/*********************************************************************************
 * Communication methods
 *********************************************************************************/

/*******************************************************************************
 * @brief Receive answer from the communication port.
 * @param buffer - buffer holding the answer
 * @param bytes - number of bytes contained in the answer
 * @author CanisUrsa
 * @return true if communication succeeded, false otherwise
*******************************************************************************/
bool StarGoTelescope::receive(char* buffer, int* bytes, char end, int wait)
{
//    LOGF_DEBUG("%s timeout=%ds",__FUNCTION__, wait);
    int timeout = wait; //? AVALON_TIMEOUT: 0;
    int returnCode = tty_read_section(PortFD, buffer, end, timeout, bytes);
    if (returnCode != TTY_OK)
    {
        char errorString[MAXRBUF];
        tty_error_msg(returnCode, errorString, MAXRBUF);
        if(returnCode==TTY_TIME_OUT && wait <= 0) return false;
        LOGF_WARN("Failed to receive full response: %s. (Return code: %d)", errorString, returnCode);
        return false;
    }
    if(buffer[*bytes-1]=='#')
        buffer[*bytes - 1] = '\0'; // remove #
    else
        buffer[*bytes] = '\0';

    return true;
}

/*******************************************************************************
 * @brief Flush the communication port.
 * @author CanisUrsa
 * StarGo sends information asynchronously so flushing is disabled
*******************************************************************************/
void StarGoTelescope::flush()
{
//    LOG_DEBUG(__FUNCTION__);
//    tcflush(PortFD, TCIOFLUSH);
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::transmit(const char* buffer)
{
//    LOG_DEBUG(__FUNCTION__);
    int bytesWritten = 0;
    flush();
    int returnCode = tty_write_string(PortFD, buffer, &bytesWritten);
    if (returnCode != TTY_OK)
    {
        char errorString[MAXRBUF];
        tty_error_msg(returnCode, errorString, MAXRBUF);
        LOGF_WARN("Failed to transmit %s. Wrote %d bytes and got error %s.", buffer, bytesWritten, errorString);
        return false;
    }
    return true;
}
/*******************************************************************************
**
*******************************************************************************/
StarGoTelescope::AutoAdjust::AutoAdjust(StarGoTelescope *ptr)
{
    p = ptr;
    xmin = 20000;   // Minimum duration of set in milliseconds
    xmax = 30000;   // Maximum gap between samples
    nmin = 5;       // Minimum number of samples
    reset();
}
/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::AutoAdjust::setEnabled(bool isenabled)
{
    LOGF_DEBUG("%s enabled=%d",__FUNCTION__, enabled);
    enabled = isenabled;
    LOGF_INFO("RA Auto Adjust %s.", enabled?"enabled":"disabled");
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
void StarGoTelescope::AutoAdjust::reset()
{
    LOG_DEBUG(__FUNCTION__);
    sumx=sumy=sumxy=sumx2=0;
    x.clear();
    y.clear();
    start = std::chrono::system_clock::now();
    p->setTrackingAdjustment(0.0);
}
/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::AutoAdjust::setRaAdjust(int8_t direction, uint32_t duration_msec)
{
    LOG_DEBUG(__FUNCTION__);
    if (!enabled)
    {
        LOG_ERROR("Auto tracking adjustment is currently DISABLED");
        return false;
    }

// Get the guiding speed
    double guidingSpeed = p->GuidingSpeedN[0].value;
    
    int raSpeed, decSpeed;
    if(p->getGuidingSpeeds(&raSpeed, &decSpeed))
    {
        guidingSpeed =  raSpeed/100.0;
    }
    else
    {
        LOG_ERROR("Unable to get guiding speed");
        return false;
    }

//    get trackrate = 1 + adj/100
    double trackAdjust = 1.0 + p->TrackAdjustN[0].value/100.0;
    
    double raCorrection;
    if (p->getTrackingAdjustment(&raCorrection))
    {
        trackAdjust = 1.0 + raCorrection/100;
    }
    else
    {
        LOG_ERROR("Unable to get tracking adjustment ");
        return false;
    }
    
// Calculate number of milliseconds since the driver was started
    double xnewest = std::chrono::duration<double,std::milli>(std::chrono::system_clock::now() - start).count();
    double ddir = 0.0;
    if (direction == STARGO_EAST)
    {
        ddir = -1.0;
    }
    else if (direction == STARGO_WEST)
    {
        ddir = -1.0;
    }
    else
    {
        LOG_ERROR("Invalid direction.");
        return false;
    }
    
    // Reset if the gap to the last reading is too large
    if( !x.empty() && (xnewest - x.back()) > xmax)
    {
        LOG_INFO("Reset RA auto adjust after hiatus");
        reset();
    }
    
    // Calculate the cumulative corrections so far in milliseconds normalised to sidereal rate
    double ynewest = ddir * static_cast<double>(duration_msec) * guidingSpeed * trackAdjust + (y.empty() ? 0.0: y.back());
    while (xnewest - x[1] > xmin && x.size() > 2) // enough samples. lose the excess
    {
        sumx  -= x.front();
        sumy  -= y.front();
        sumxy -= x.front()*y.front();
        sumx2 -= x.front()*x.front();
        x.pop_front();
        y.pop_front();
    }
    x.push_back(xnewest);
    y.push_back(ynewest);
    sumx  += xnewest;
    sumy  += ynewest;
    sumxy += xnewest*ynewest;
    sumx2 += xnewest*xnewest;
    
    if(x.size() < nmin)
    {
        LOG_INFO("RA auto adjust needs more samples");
        return true; // not enough samples
    }

    uint32_t n = x.size();
    double slope = (n*sumxy - sumx*sumy)/(n*sumx2 - sumy*sumy);
    double adjustRA = slope*100.0;  // Convert to a percentage of Sidereal rate
    p->setTrackingAdjustment(adjustRA);
    LOGF_INFO("RA auto adjust rate to %.2f", adjustRA);
    return true;

}
