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
#include "zfilterfactory.h"

#include <cmath>
#include <memory>
#include <cstring>
#include <unistd.h>
#include <exception>
//#include <cassert>
//????? Why WIN32
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

StarGoTelescope::StarGoTelescope(): GI(this)
{
    LOG_DEBUG(__FUNCTION__);
    setVersion(AVALON_VERSION_MAJOR, AVALON_VERSION_MINOR);

    DBG_SCOPE = INDI::Logger::DBG_DEBUG;

    SetTelescopeCapability(TELESCOPE_CAN_PARK | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT |
                           TELESCOPE_HAS_TRACK_MODE | TELESCOPE_HAS_LOCATION | TELESCOPE_CAN_CONTROL_TRACK |
                           TELESCOPE_HAS_PIER_SIDE, 4);
}

/*******************************************************************************
**
** VIRTUAL METHODS
**
*******************************************************************************/

/*******************************************************************************
 * 
 ******************************************************************************/
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
    if (!getScopeAlignmentStatus(&mountType, &isTracking, &alignmentPoints))
    {
        LOG_ERROR("Error communication with telescope.");
        return false;
    }

// Handshake commands used in the StarGo ASCOM driver.
    char cmdsync[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char cmdlst[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char cmddate[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char lst[AVALON_COMMAND_BUFFER_LENGTH-5] = {0};
    if (getLST_String(lst))
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
        ":TTRFr#", "0",               // Enable the Keypad
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
        if (!sendQuery(cmds[i][0], response, cmds[i][1]==nullptr?0:5))
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
    INDI::Telescope::Handshake(); // calls ReadScopeStatus

    autoRa = new AutoAdjust(this);

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
        if (SyncHomeSP.isNameMatch(name))
        {
            return setHomeSync();
        }

        // goto home position
        if (MountGotoHomeSP.isNameMatch(name))
        {
            MountGotoHomeSP.update(states, names, n);
            if (setGotoHome())
            {
                MountGotoHomeSP.setState(IPS_BUSY);
                TrackState = SCOPE_SLEWING;
            }
            else
            {
                MountGotoHomeSP.setState(IPS_ALERT);
            }
            MountGotoHomeSP[0].setState(ISS_OFF);
            MountGotoHomeSP.apply();

            LOG_INFO("Slewing to home position...");
            return true;
        }
        // tracking mode
        else if (TrackModeSP.isNameMatch(name))
        {
            if (TrackModeSP.update(states, names, n) == false)
                return false;
            uint8_t trackMode = static_cast<uint8_t>(TrackModeSP.findOnSwitchIndex());
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
            TrackModeSP.setState(result ? IPS_OK : IPS_ALERT);
            TrackModeSP.apply(nullptr);

            return result;
        }
        else if (ST4StatusSP.isNameMatch(name))
        {
            bool enabled = !strcmp(IUFindOnSwitchName(states, names, n), ST4StatusSP[INDI_ENABLED].name);
            bool result = setST4Enabled(enabled);

            if (result)
            {
                ST4StatusSP[INDI_ENABLED].setState(enabled ? ISS_ON : ISS_OFF);
                ST4StatusSP[INDI_DISABLED].setState(enabled ? ISS_OFF : ISS_ON);
                ST4StatusSP.setState(IPS_OK);
            }
            else
            {
                ST4StatusSP.setState(IPS_ALERT);
            }
            ST4StatusSP.apply();
            return result;
        }
        else if (KeypadStatusSP.isNameMatch(name))
        {
            ///////////
            bool enabled = !strcmp(IUFindOnSwitchName(states, names, n), KeypadStatusSP[INDI_ENABLED].name);
            bool result = setKeyPadEnabled(enabled);

            if (result)
            {
                KeypadStatusSP[INDI_ENABLED].setState(enabled ? ISS_ON : ISS_OFF);
                KeypadStatusSP[INDI_DISABLED].setState(enabled ? ISS_OFF : ISS_ON);
                KeypadStatusSP.setState(IPS_OK);
            }
            else
            {
                KeypadStatusSP.setState(IPS_ALERT);
            }
            KeypadStatusSP.apply();
            return result;
        }
        else if (MaxSlewSpeedSP.isNameMatch(name))
        {
            if (MaxSlewSpeedSP.update(states, names, n) == false)
                return false;
            int index = MaxSlewSpeedSP.findOnSwitchIndex();

            bool result = setMaxSlewSpeed(index);

            switch (index)
            {
                case 0:
                    LOG_INFO("System slew rate set to low.");
                    break;
                case 1:
                    LOG_INFO("System slew rate set to medium.");
                    break;
                case 2:
                    LOG_INFO("System slew rate set to fast.");
                    break;
                case 3:
                    LOG_WARN("System slew rate set to high. ONLY AVAILABLE FOR 15V or 18V!");
                    break;
                default:
                    LOGF_WARN("Unexpected slew rate %d", index);
                    result = false;
                    break;
            }
            MaxSlewSpeedSP.setState(result ? IPS_OK : IPS_ALERT);
            MaxSlewSpeedSP.apply();
            return result;
        }
        else if (CenterSpeedSP.isNameMatch(name))
        {
            if (CenterSpeedSP.update(states, names, n) == false)
                return false;
            int index = CenterSpeedSP.findOnSwitchIndex();
            int find = FindSpeedSP.findOnSwitchIndex();

            bool result = setCenterFindSpeed(index, find);
            if (!result)
            {
                LOGF_WARN("Set Center speed failed Center: %d Find: %d", index, find);
                result = false;
            }
            CenterSpeedSP.setState(result ? IPS_OK : IPS_ALERT);
            CenterSpeedSP.apply();
            return result;
        }
        else if (FindSpeedSP.isNameMatch(name))
        {
            if (FindSpeedSP.update(states, names, n) == false)
                return false;
            int index = FindSpeedSP.findOnSwitchIndex();
            int center = CenterSpeedSP.findOnSwitchIndex();

            bool result = setCenterFindSpeed(center, index);
            if (!result)
            {
                LOGF_WARN("Set Find speed failed Center: %d Find: %d", center, index);
                result = false;
            }
            FindSpeedSP.setState(result ? IPS_OK : IPS_ALERT);
            FindSpeedSP.apply();
            return result;
        }
        else if (MeridianFlipModeSP.isNameMatch(name))
        {
            int preIndex = MeridianFlipModeSP.findOnSwitchIndex();
            MeridianFlipModeSP.update(states, names, n);
            int nowIndex = MeridianFlipModeSP.findOnSwitchIndex();
            if (SetMeridianFlipMode(nowIndex) == false)
            {
                MeridianFlipModeSP.reset();
                MeridianFlipModeSP[preIndex].setState(ISS_ON);
                MeridianFlipModeSP.setState(IPS_ALERT);
            }
            else
                MeridianFlipModeSP.setState(IPS_OK);
            MeridianFlipModeSP.apply();
            return true;
        }
        else if (RaMotorReverseSP.isNameMatch(name))
        {
            int preIndex = RaMotorReverseSP.findOnSwitchIndex();
            int decIndex = DecMotorReverseSP.findOnSwitchIndex();
            RaMotorReverseSP.update(states, names, n);
            int raIndex = RaMotorReverseSP.findOnSwitchIndex();
            if (setMotorReverse(raIndex, decIndex) == false)
            {
                RaMotorReverseSP.reset();
                RaMotorReverseSP[preIndex].setState(ISS_ON);
                RaMotorReverseSP.setState(IPS_ALERT);
            }
            else
                RaMotorReverseSP.setState(IPS_OK);
            RaMotorReverseSP.apply();
            return true;
        }
        else if (DecMotorReverseSP.isNameMatch(name))
        {
            int preIndex = DecMotorReverseSP.findOnSwitchIndex();
            int raIndex = RaMotorReverseSP.findOnSwitchIndex();
            DecMotorReverseSP.update(states, names, n);
            int decIndex = DecMotorReverseSP.findOnSwitchIndex();
            if (setMotorReverse(raIndex, decIndex) == false)
            {
                DecMotorReverseSP.reset();
                DecMotorReverseSP[preIndex].setState(ISS_ON);
                DecMotorReverseSP.setState(IPS_ALERT);
            }
            else
                DecMotorReverseSP.setState(IPS_OK);
            DecMotorReverseSP.apply();
            return true;
        }
        else if (RaAutoAdjustSP.isNameMatch(name))
        {
            ////////////////
            bool enabled = !strcmp(IUFindOnSwitchName(states, names, n), RaAutoAdjustSP[INDI_ENABLED].name);
            bool result = autoRa->setEnabled(enabled);

            if (result)
            {
                RaAutoAdjustSP[INDI_ENABLED].setState(enabled ? ISS_ON : ISS_OFF);
                RaAutoAdjustSP[INDI_DISABLED].setState(enabled ? ISS_OFF : ISS_ON);
                RaAutoAdjustSP.setState(IPS_OK);
            }
            else
            {
                RaAutoAdjustSP.setState(IPS_ALERT);
            }
            RaAutoAdjustSP.apply();
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
        if (GI::processNumber(dev, name, values, names, n))
            return true;
        // sync home position
        if (GuidingSpeedNP.isNameMatch(name))
        {
            int raSpeed  = static_cast<int>(round(values[0] * 100.0));
            int decSpeed = static_cast<int>(round(values[1] * 100.0));
            bool result  = setGuidingSpeeds(raSpeed, decSpeed);

            if (result)
            {
                GuidingSpeedNP[0].value = static_cast<double>(raSpeed) / 100.0;
                GuidingSpeedNP[1].value = static_cast<double>(decSpeed) / 100.0;
                GuidingSpeedNP.setState(IPS_OK);
            }
            else
            {
                GuidingSpeedNP.setState(IPS_ALERT);
            }
            GuidingSpeedNP.apply();
            return result;
        }
        else if (MountRequestDelayNP.isNameMatch(name))
        {
            setMountRequestDelay(values[0]);
            MountRequestDelayNP[0].value = values[0];
            MountRequestDelayNP.setState(IPS_OK);
            MountRequestDelayNP.apply();
            return true;
        }
        else if (TrackingAdjustmentNP.isNameMatch(name))
        {
            if( autoRa->isEnabled() )
            {
                LOG_ERROR("Cannot adjust tracking rate when auto-adjustment is enabled");
                TrackingAdjustmentNP.setState(IPS_ALERT);
                TrackingAdjustmentNP.apply();
                return false;
            }
            // change tracking adjustment
            bool success = setTrackingAdjustment(values[0]);
            if (success)
            {
                double adjust;
                success = getTrackingAdjustment(&adjust);  // Get the value set in the mount
                TrackingAdjustmentNP[0].value = adjust;
                TrackingAdjustmentNP.setState(IPS_OK);
            }
            else
                TrackingAdjustmentNP.setState(IPS_ALERT);

            TrackingAdjustmentNP.apply();
            return success;
        }
        else if (TorqueNP.isNameMatch(name))
        {
            int torque  = static_cast<int>(values[0]);
            bool result  = setTorque(torque);
            if (result)
            {
                result = getTorque(&torque);  // Get the value set in the mount
                TorqueNP[0].value = torque;
                TorqueNP.setState(IPS_OK);
            }
            else
            {
                TorqueNP.setState(IPS_ALERT);
            }
            TorqueNP.apply();
            return result;
        }
    }

    //  Nobody has claimed this, so pass it to the parent
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

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

    GI::initProperties(GUIDE_TAB);

    // Add debug/simulation/config controls so we may debug driver if necessary
    addAuxControls();

    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    MountGotoHomeSP[0].fill("MOUNT_GOTO_HOME_VALUE", "Goto Home", ISS_OFF);
    MountGotoHomeSP.fill(getDeviceName(), "MOUNT_GOTO_HOME", "Goto Home", MAIN_CONTROL_TAB,
                       IP_RW, ISR_ATMOST1, 60, IPS_OK);

    SetParkDataType(PARK_HA_DEC);

    SyncHomeSP[0].fill("SYNC_HOME", "Sync Home", ISS_OFF);
    SyncHomeSP.fill(getDeviceName(), "TELESCOPE_SYNC_HOME", "Home Position", MAIN_CONTROL_TAB,
                       IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    MountFirmwareInfoTP[0].fill("MOUNT_FIRMWARE_INFO", "Firmware", "");
    MountFirmwareInfoTP[1].fill("MOUNT_TYPE", "Mount Type", "");
    MountFirmwareInfoTP[2].fill("MOUNT_TCB", "TCB", "");
    MountFirmwareInfoTP.fill(getDeviceName(), "MOUNT_INFO", "Mount Info", INFO_TAB, IP_RO,
                     60, IPS_OK);

    // Guiding settings
    GuidingSpeedNP[0].fill("GUIDE_RATE_WE", "RA Speed", "%.2f", 0.0, 2.0, 0.1, 0);
    GuidingSpeedNP[1].fill("GUIDE_RATE_NS", "DEC Speed", "%.2f", 0.0, 2.0, 0.1, 0);
    GuidingSpeedNP.fill(getDeviceName(), "GUIDE_RATE","Autoguiding", GUIDE_TAB, IP_RW, 60,
                       IPS_IDLE);

    // ST4 guiding enabled / disabled
    ST4StatusSP[INDI_ENABLED].fill("INDI_ENABLED", "Enabled", ISS_OFF);
    ST4StatusSP[INDI_DISABLED].fill("INDI_DISABLED", "Disabled", ISS_ON);
    ST4StatusSP.fill(getDeviceName(), "ST4", "ST4", GUIDE_TAB, IP_RW, ISR_1OFMANY, 60,
                       IPS_IDLE);

    // keypad enabled / disabled
    KeypadStatusSP[INDI_ENABLED].fill("INDI_ENABLED", "Enabled", ISS_ON);
    KeypadStatusSP[INDI_DISABLED].fill("INDI_DISABLED", "Disabled", ISS_OFF);
    KeypadStatusSP.fill(getDeviceName(), "Keypad", "Keypad", MOTION_TAB, IP_RW, ISR_1OFMANY,
                       60, IPS_IDLE);

    // Max Slew Speeds
    MaxSlewSpeedSP[0].fill("MAX_SLEW_SPEED_LOW", "Low", ISS_OFF);
    MaxSlewSpeedSP[1].fill("MAX_SLEW_SPEED_MEDIUM", "Medium", ISS_OFF);
    MaxSlewSpeedSP[2].fill("MAX_SLEW_SPEED_FAST", "Fast", ISS_ON);
    MaxSlewSpeedSP[3].fill("MAX_SLEW_SPEED_HIGH", "High", ISS_OFF);
    MaxSlewSpeedSP.fill(getDeviceName(), "MAX_SLEW_SPEED", "Max Slew Speed", MOTION_TAB,
                       IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Center Speeds
    CenterSpeedSP[0].fill("CENTER_SPEED_2X",  "2x", ISS_OFF);
    CenterSpeedSP[1].fill("CENTER_SPEED_3X",  "3x", ISS_OFF);
    CenterSpeedSP[2].fill("CENTER_SPEED_4X",  "4x", ISS_OFF);
    CenterSpeedSP[3].fill("CENTER_SPEED_6X",  "6x", ISS_OFF);
    CenterSpeedSP[4].fill("CENTER_SPEED_8X",  "8x", ISS_ON);
    CenterSpeedSP[5].fill("CENTER_SPEED_10X", "10x", ISS_OFF);
    CenterSpeedSP.fill(getDeviceName(), "CENTER_SPEED", "Center Speed", MOTION_TAB,
                       IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Find Speeds
    FindSpeedSP[0].fill("FIND_SPEED_10X",  "10x", ISS_OFF);
    FindSpeedSP[1].fill("FIND_SPEED_15X",  "15x", ISS_OFF);
    FindSpeedSP[2].fill("FIND_SPEED_20X",  "20x", ISS_OFF);
    FindSpeedSP[3].fill("FIND_SPEED_30X",  "30x", ISS_OFF);
    FindSpeedSP[4].fill("FIND_SPEED_50X",  "50x", ISS_OFF);
    FindSpeedSP[5].fill("FIND_SPEED_75X",  "75x", ISS_ON);
    FindSpeedSP[6].fill("FIND_SPEED_100X", "100x", ISS_OFF);
    FindSpeedSP[7].fill("FIND_SPEED_150X", "150x", ISS_OFF);
    FindSpeedSP.fill(getDeviceName(), "FIND_SPEED", "Find Speed", MOTION_TAB,
                       IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Tracking Adjustment
    TrackingAdjustmentNP[0].fill("RA_TRACK_ADJ", "RA Tracking Adjust (%)", "%.2f", -5.0, 5.0, 0.01, 0);
    TrackingAdjustmentNP.fill(getDeviceName(), "Track Adjust","Tracking", MOTION_TAB,
                       IP_RW, 60, IPS_IDLE);

    // meridian flip
    MeridianFlipModeSP[0].fill("MERIDIAN_FLIP_AUTO", "Auto", ISS_OFF);
    MeridianFlipModeSP[1].fill("MERIDIAN_FLIP_DISABLED", "Disabled", ISS_OFF);
    MeridianFlipModeSP[2].fill("MERIDIAN_FLIP_FORCED", "Forced", ISS_OFF);
    MeridianFlipModeSP.fill(getDeviceName(), "MERIDIAN_FLIP_MODE", "Meridian Flip",
                       MOTION_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // mount command delay
    /*
    default delay is static_cast<double>(xmitDelay.count())/1000.0;
    */
    MountRequestDelayNP[0].fill("MOUNT_REQUEST_DELAY", "Request Delay (ms)", "%.0f", 0.0, 1000, 1.0, 50.0);
    MountRequestDelayNP.fill(getDeviceName(), "REQUEST_DELAY", "StarGO", OPTIONS_TAB,
                       IP_RW, 60, IPS_OK);

    // HA and LST for reference
    HaLstNP[0].fill("HA", "HA (hh:mm:ss)", "%010.6m", 0, 24, 0, 0);
    HaLstNP[1].fill("LST", "LST (hh:mm:ss)", "%010.6m", 0, 24, 0, 0);
    HaLstNP.fill(getDeviceName(), "HA-LST", "Hour Angle", SITE_TAB, IP_RO, 60, IPS_IDLE);

    // Gear Ratios
    GearRatioNP[0].fill("GEAR_RATIO_RA", "RA Gearing", "%.2f", 0.0, 1000.0, 1, 0);
    GearRatioNP[1].fill("GEAR_RATIO_DEC", "DEC Gearing", "%.2f", 0.0, 1000.0, 1, 0);
    GearRatioNP.fill(getDeviceName(), "Gear Ratio","Gearing", INFO_TAB, IP_RO, 60, IPS_IDLE);

    // RA and Dec motor direction
    RaMotorReverseSP[INDI_ENABLED].fill("INDI_ENABLED", "Reverse", ISS_OFF);
    RaMotorReverseSP[INDI_DISABLED].fill("INDI_DISABLED", "Normal", ISS_OFF);
    RaMotorReverseSP.fill(getDeviceName(), "RA_REVERSE", "RA Reverse", MOTION_TAB, IP_RW,
                       ISR_1OFMANY, 60, IPS_IDLE);

    DecMotorReverseSP[INDI_ENABLED].fill("INDI_ENABLED", "Reverse", ISS_OFF);
    DecMotorReverseSP[INDI_DISABLED].fill("INDI_DISABLED", "Normal", ISS_OFF);
    DecMotorReverseSP.fill(getDeviceName(), "DEC_REVERSE", "Dec Reverse", MOTION_TAB,
                       IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Torque
    TorqueNP[0].fill("TORQUE_RA", "Motor Torque", "%.0f", 0.0, 100.0, 10.0, 0);
    TorqueNP.fill(getDeviceName(), "Torque","Torque", MOTION_TAB, IP_RW, 60, IPS_IDLE);

    // Motor Step Position
    MotorStepNP[0].fill("MOTOR_STEP_RA", "RA Step Pos", "%.2f", -100000.0, 100000.0, 1, 0);
    MotorStepNP[1].fill("MOTOR_STEP_DEC", "DEC Step Pos", "%.2f", -100000.0, 100000.0, 1, 0);
    MotorStepNP.fill(getDeviceName(), "Motor Steps","Position", INFO_TAB, IP_RO, 60, IPS_IDLE);

    // Auto Tracking Adjustment
    RaAutoAdjustSP[INDI_ENABLED].fill("INDI_ENABLED", "Enabled", ISS_OFF);
    RaAutoAdjustSP[INDI_DISABLED].fill("INDI_DISABLED", "Disabled", ISS_ON);
    RaAutoAdjustSP.fill(getDeviceName(), "RA_AUTO_ADJ", "RA Auto Adjust", GUIDE_TAB, IP_RW,
                       ISR_1OFMANY, 60, IPS_IDLE);

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
        defineProperty(SyncHomeSP);
        defineProperty(MountGotoHomeSP);
        defineProperty(GuidingSpeedNP);
        defineProperty(ST4StatusSP);
        defineProperty(KeypadStatusSP);
        defineProperty(MaxSlewSpeedSP);
        defineProperty(CenterSpeedSP);
        defineProperty(FindSpeedSP);
        defineProperty(TrackingAdjustmentNP);
        defineProperty(MeridianFlipModeSP);
        defineProperty(MountRequestDelayNP);
        defineProperty(MountFirmwareInfoTP);
        defineProperty(RaAutoAdjustSP);
        defineProperty(GearRatioNP);
        defineProperty(TorqueNP);
        defineProperty(RaMotorReverseSP);
        defineProperty(DecMotorReverseSP);
        defineProperty(MotorStepNP);
        defineProperty(HaLstNP);
    }
    else
    {
        deleteProperty(SyncHomeSP);
        deleteProperty(MountGotoHomeSP);
        deleteProperty(GuidingSpeedNP);
        deleteProperty(ST4StatusSP);
        deleteProperty(KeypadStatusSP);
        deleteProperty(MaxSlewSpeedSP);
        deleteProperty(CenterSpeedSP);
        deleteProperty(FindSpeedSP);
        deleteProperty(TrackingAdjustmentNP);
        deleteProperty(MeridianFlipModeSP);
        deleteProperty(MountRequestDelayNP);
        deleteProperty(MountFirmwareInfoTP);
        deleteProperty(RaAutoAdjustSP);
        deleteProperty(GearRatioNP);
        deleteProperty(TorqueNP);
        deleteProperty(RaMotorReverseSP);
        deleteProperty(DecMotorReverseSP);
        deleteProperty(MotorStepNP);
        deleteProperty(HaLstNP);
    }
    GI::updateProperties();

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::saveConfigItems(FILE *fp)
{
    LOG_DEBUG(__FUNCTION__);
// There is no get function for Center and Find speeds so save in config
    CenterSpeedSP.save(fp);
    FindSpeedSP.save(fp);
    RaAutoAdjustSP.save(fp);

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
    if ( x == MOTION_SLEW || y == MOTION_SLEW)
    {
        motion = MOTION_SLEW;
    }
    else if ( x == MOTION_ACCEL || y == MOTION_ACCEL)
    {
        motion = MOTION_SLEW;
    }
    else if ( x == MOTION_DECEL || y == MOTION_DECEL)
    {
        motion = MOTION_SLEW;
    }
    else if ( x == MOTION_GUIDE || y == MOTION_GUIDE)
    {
        motion = MOTION_GUIDE;
    }
    else if ( x == MOTION_TRACK || y == MOTION_TRACK)
    {
        motion = MOTION_TRACK;
    }
    else if ( x == MOTION_STATIC && y == MOTION_STATIC)
    {
        motion = MOTION_STATIC;
    }
    else
    {
       LOGF_ERROR("Invlid motion state: %d, %d", x, y);
       return false;
    }
    if ( motion == MOTION_GUIDE)
    {
        LOG_DEBUG("Guiding in progress");
        return true;
    }
    if ( x != 4) GuideComplete(AXIS_RA);
    if ( y != 4) GuideComplete(AXIS_DE);

    char parkHomeStatus[2] = {'\0','\0'};
    if (! getParkHomeStatus(parkHomeStatus))
    {
       LOG_ERROR("Cannot determine scope status, failed to determine park/sync state.");
       return false;
    }
    LOGF_DEBUG("Motor state(RA,DE): (%d, %d); Park state = %s", x, y, parkHomeStatus);

    INDI::Telescope::TelescopeStatus newTrackState = TrackState;

    // handle parking / unparking
    if (strcmp(parkHomeStatus, "2") == 0)
    {
        newTrackState = SCOPE_PARKED;
        if (TrackState != newTrackState)
            SetParked(true);
        updateParkPosition();
    }
    else
    {
        if (TrackState == SCOPE_PARKED)
            SetParked(false);

        // handle tracking state
        if (x == 0 && y == 0)
        {
            newTrackState = SCOPE_IDLE;
            if (TrackState != newTrackState)
                LOGF_INFO("%sTracking is off.", TrackState == SCOPE_PARKING ? "Scope parked. ": "");

            if (MountGotoHomeSP.getState() == IPS_BUSY)
            {
                MountGotoHomeSP.setState(IPS_OK);
                MountGotoHomeSP.apply();
            }
        }
        else if (x == 1 && y == 0)
        {
            newTrackState = SCOPE_TRACKING;  // or GUIDING
            if (TrackState != newTrackState)
                LOGF_INFO("%sTracking...", TrackState == SCOPE_SLEWING ? "Slewing completed. ": "");
        }
    }

    double raStep, decStep;
    if (getMotorSteps(&raStep, &decStep))
    {
        MotorStepNP[0].value =  raStep;
        MotorStepNP[1].value =  decStep;
        MotorStepNP.setState(IPS_OK);
    }
    else
    {
        MotorStepNP.setState(IPS_ALERT);
    }
    MotorStepNP.apply();

    double r, d;
    if (!getEqCoordinates(&r, &d))
    {
        LOG_ERROR("Retrieving equatorial coordinates failed.");
        return false;
    }

    TrackState = newTrackState;
    NewRaDec(r, d);
    double ha, lst;
    if (getLST(&lst))
    {
        ha = lst - r;
        HaLstNP[0].value =  fmod(ha, 24.0);
        HaLstNP[1].value =  fmod(lst, 24.0);
        HaLstNP.setState(IPS_OK);
    }
    else
    {
        LOG_ERROR("Retrieving scope LST failed.");
        HaLstNP.setState(IPS_ALERT);
    }
    HaLstNP.apply();

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

    if (!isConnected())
        return false;
    
    if (isSimulation())
        return true;

    if (!setSiteLongitude(longitude))
    {
        LOGF_ERROR("Error setting site longitude %lf", longitude);
        return false;
    }

    if (!setSiteLatitude(latitude))
    {
        LOGF_ERROR("Error setting site latitude %lf", latitude);
        return false;
    }

    char l[32]={0}, L[32]={0};
    fs_sexa(l, latitude, 3, 3600);
    fs_sexa(L, longitude, 4, 3600);

   LOGF_DEBUG("Site location updated to Lat %.32s - Long %.32s", l, L);

// Set local sidereal time for the new longitude
    if (!setLocalSiderealTime(longitude))
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

    if (!isSimulation() && !setObjectCoords(ra,dec))
    {
         LOG_ERROR("Error setting coords for sync");
         return false;
    }

    if (!isSimulation() && !sendQuery(":CM#", response))
    {
        EqNP.setState(IPS_ALERT);
        LOG_ERROR("Synchronization failed.");
        EqNP.apply();
        return false;
    }
    LOG_INFO("Synchronization successful.");

    EqNP.setState(IPS_OK);
    EqNP.apply();
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
    if (!Goto(lst - Axis1Value, Axis2Value)) return false;
    return SetCurrentPark();
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::SetDefaultPark()
{
    LOG_DEBUG(__FUNCTION__);

// Not sure why we do this
/*
    double latitude;
    if (!getSiteLatitude(&latitude))
    {
        LOG_WARN("Failed to get site Latitude from device.");
        return false;
    }
*/

// Slew to the Home position then set it as the Park position
    if (!setGotoHome()) return false;
    return SetCurrentPark();
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::SetCurrentPark()
{
    LOG_DEBUG(__FUNCTION__);

// Setting ParkOptionBusy causes WaitParkOptionReady to set the mount park position 
// once the scope has stopped moving.
// WaitParkOptionReady is called from ReadScopeStatus
// An Abort will reset ParkOptionBusy to false
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
        SlewRateSP.setState(IPS_ALERT);
        SlewRateSP.apply();
        LOG_ERROR( "Error setting slew mode.");
        return false;
    }

    SlewRateSP.setState(IPS_OK);
    SlewRateSP.apply();
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

    // If moving, let's stop it first.
//    if (EqNP.s == IPS_BUSY)
    if (EqNP.getState() == IPS_BUSY)
    {
        if (!isSimulation() && !Abort())
        {
            AbortSP.setState(IPS_ALERT);
            AbortSP.apply();
            LOG_ERROR("Abort slew failed.");
            return false;
        }

        AbortSP.setState(IPS_OK);
        AbortSP.apply();
        EqNP.setState(IPS_IDLE);
        EqNP.apply();
        
        LOG_INFO("Slew aborted.");
//        IDSetNumber(&EqNP, nullptr);

        if (MovementNSSP.getState() == IPS_BUSY || MovementWESP.getState() == IPS_BUSY)
        {
            MovementNSSP.setState(IPS_IDLE);
            MovementWESP.setState(IPS_IDLE);
            EqNP.setState(IPS_IDLE);
            EqNP.apply();

            MovementNSSP.reset();
            MovementWESP.reset();
            MovementNSSP.apply();
            MovementWESP.apply();
        }

        // sleep for 100 mseconds
        nanosleep(&timeout, nullptr);
    }
    if (!isSimulation() && !setObjectCoords(ra,dec))
    {
         LOG_ERROR("Error setting coords for goto");
         return false;
    }

    if (!isSimulation())
    {
        char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
        if (!sendQuery(":MS#", response))
        {
            LOG_ERROR("Error Slewing");
            EqNP.setState(IPS_ALERT);
            EqNP.apply();
            return false;
        }
    }

    TrackState = SCOPE_SLEWING;
    EqNP.setState(IPS_BUSY);
    EqNP.apply();

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

    if (GuideNSNP.getState() == IPS_BUSY || GuideWENP.getState() == IPS_BUSY)
    {
        GuideNSNP.setState(IPS_IDLE);
        GuideWENP.setState(IPS_IDLE);
        GuideNSNP[0].setValue(0.0);
        GuideNSNP[1].setValue(0.0);
        GuideWENP[0].setValue(0.0);
        GuideWENP[1].setValue(0.0);

        LOG_INFO("Guide aborted.");
        GuideNSNP.apply();
        GuideWENP.apply();

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
// This virtual function should set values in arcsec per second
// Convert arcsec per second to percentage for StarGo
// Used when capability HAS_TRACK_RATE is set
// In this case HAS_TRACK_RATE is not set so the procedure is not needed.

    LOGF_DEBUG("%s rarate=%lf deRate=%lf",__FUNCTION__,raRate, deRate);
    INDI_UNUSED(raRate);
    INDI_UNUSED(deRate);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    int rate = static_cast<int>((raRate/15.0 - 1.0)*10000.0 + 1000.0);
    sprintf(cmd, ":X1E%04d", rate);
    if (!sendQuery(cmd, response, 0))
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
    if (!SendPulseCmd(STARGO_NORTH, ms))
    {
        return IPS_ALERT;
    }
    return IPS_BUSY;
}

/*******************************************************************************
**
*******************************************************************************/
IPState StarGoTelescope::GuideSouth(uint32_t ms)
{
    LOGF_DEBUG("%s %dms",__FUNCTION__, ms);
    if (!SendPulseCmd(STARGO_SOUTH, ms))
    {
        return IPS_ALERT;
    }
    return IPS_BUSY;
}

/*******************************************************************************
**
*******************************************************************************/
IPState StarGoTelescope::GuideEast(uint32_t ms)
{
    LOGF_DEBUG("%s %dms",__FUNCTION__, ms);
    if (!SendPulseCmd(STARGO_EAST, ms))
    {
        return IPS_ALERT;
    }

// Doco says:
// Returns IPS_OK if operation is completed successfully, IPS_BUSY if operation will take timr to complete,
// or IPS_ALERT if operation failed.

    return IPS_BUSY;
}

/*******************************************************************************
**
*******************************************************************************/
IPState StarGoTelescope::GuideWest(uint32_t ms)
{
    LOGF_DEBUG("%s %dms",__FUNCTION__, ms );
    if (!SendPulseCmd(STARGO_WEST, ms))
    {
        return IPS_ALERT;
    }
    return IPS_BUSY;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    LOGF_DEBUG("%s dir=%d cmd=%d", __FUNCTION__, dir, command);
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

// Any other goto prior to this command sets the slew speed to MAX
// Set the slew speed as requested by the client
    int premode = SlewRateSP.findOnSwitchIndex();
    if (SetSlewRate(premode) == false)
    {
        SlewRateSP.setState(IPS_ALERT);
    }
    else
        SlewRateSP.setState(IPS_OK);
    SlewRateSP.apply();

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

// Any other goto prior to this command sets the slew speed to MAX
// Set the slew speed as requested by the client
    int premode = SlewRateSP.findOnSwitchIndex();
    if (SetSlewRate(premode) == false)
    {
        SlewRateSP.setState(IPS_ALERT);
    }
    else
        SlewRateSP.setState(IPS_OK);
    SlewRateSP.apply();

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
        LocationNP[LOCATION_LATITUDE].setValue( 29.5 );
        LocationNP[LOCATION_LONGITUDE].setValue( 48.0 );
        LocationNP[LOCATION_ELEVATION].setValue( 10 );
        LocationNP.setState(IPS_OK);
        LocationNP.apply();
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
    LocationNP[LOCATION_LATITUDE].setValue(siteLat);
    LocationNP[LOCATION_LONGITUDE].setValue(siteLong);

    LOGF_DEBUG("Mount Controller Latitude: %lg Longitude: %lg", LocationNP[LOCATION_LATITUDE].getValue(),
               LocationNP[LOCATION_LONGITUDE].getValue());

    LocationNP.apply();
// Not sure why the driver does this in a get function
//    if (!setLocalSiderealTime(siteLong))
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
    if (!sendQuery(cmd, response))
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
    LOGF_DEBUG("LST = %s", input);
    return true;
}
/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::getLST(double *lst)
{
    LOG_DEBUG(__FUNCTION__);
    double longitude;
    if (!getSiteLongitude(&longitude))
    {
        LOG_WARN("getLST Failed to get site Longitude from device.");
        return false;
    }
    // determine local sidereal time
    *lst = get_local_sidereal_time(longitude);
    LOGF_DEBUG("Current local sidereal time = %.8lf", *lst);

    return true;
}

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
        TimeTP[1].setText(utcStr);
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

    // Get UTC (we're using localtime_r, but since we shifted time_epoch above by UTCOffset we should be getting the real
    // UTC time)
    localtime_r(&time_epoch, &utm);

    // Format it into the final UTC ISO 8601
    strftime(cdate, MAXINDINAME, "%Y-%m-%dT%H:%M:%S", &utm);
    TimeTP[0].setText(cdate);

    LOGF_DEBUG("Mount controller UTC Time: %s", TimeTP[0].text);
    LOGF_DEBUG("Mount controller UTC Offset: %s", TimeTP[1].text);

    // Let's send everything to the client
    TimeTP.setState(IPS_OK);
    TimeTP.apply();

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
    
// X50 does not get a response so this is unnecessary
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
    int hours = static_cast<int>(offset * -1.0);

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

    if (! sscanf(response, "p%32s[012AB]", status))
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
        SyncHomeSP.setState(IPS_ALERT);
        SyncHomeSP[0].setState(ISS_OFF);
        SyncHomeSP.apply();
        return false;
    }

    sprintf(cmd, ":X31%s#", input);
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if (sendQuery(cmd, response))
    {
        LOG_INFO("Synching home position succeeded.");
        SyncHomeSP.setState(IPS_OK);
    }
    else
    {
        LOG_WARN("Synching home position failed.");
        SyncHomeSP.setState(IPS_ALERT);
        SyncHomeSP[0].setState(ISS_OFF);
        SyncHomeSP.apply();
        return false;
    }
    SyncHomeSP[0].setState(ISS_OFF);
    SyncHomeSP.apply();

// Confirm by getting RA/Dec (X590) and mount LST (GS)
// Calculate HA = LST-RA
// Dec should be the pole
    double r, d;
    if (!getEqCoordinates(&r, &d))
    {
        LOG_ERROR("Retrieving equatorial coordinates failed.");
        return false;
    }
    double ha, lst;
    if (getLST(&lst))
    {
        ha = lst - r;
    }
    else
    {
        LOG_ERROR("Retrieving scope LST failed.");
        return false;
    }
    LOGF_INFO("Home coordinates HA: %.6f DE: %.6f", ha, d);

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
    LOGF_DEBUG("%s ParkOptionBusy: %d TrackState: %d", __FUNCTION__, ParkOptionBusy, TrackState);
   // Check if waiting for park position to set. Reset in Abort())
    if (!ParkOptionBusy) return;

    // Check if the mount has stopped slewing
//        SCOPE_IDLE        ready
//        SCOPE_SLEWING -   not ready
//        SCOPE_TRACKING    ready
//        SCOPE_PARKING     not ready - error
//        SCOPE_PARKED      not ready error
// If it has then set park position. Otherwise wait for next call
    if (TrackState != SCOPE_IDLE &&
       TrackState != SCOPE_TRACKING ) return;

    ParkOptionSP.setState(IPS_ALERT);
    if (!setParkPosition())
    {
        LOG_WARN("Unable to set Park Position.");
    }
    else
    {
        updateParkPosition();
        ParkOptionSP.setState(IPS_OK);
    }
    ParkOptionSP.apply();
    ParkOptionBusy = false;
    return;
}

/*******************************************************************************
**
*******************************************************************************/
void StarGoTelescope::updateParkPosition()
{
    LOGF_DEBUG("%s TrackState: %d", __FUNCTION__, TrackState);
    // Check if the mount is parked
    //        SCOPE_PARKED      ready
    // If it has then update park position. Otherwise wait for next call
    if (TrackState != SCOPE_PARKED ) return;

    double lst;
    if (!getLST(&lst))
    {
        LOG_WARN("Failed to get site LST from device.");
    }
    else
    {
    // Update HA and Dec of parking position
        SetAxis1Park(lst - EqNP[AXIS_RA].value);
        SetAxis2Park(EqNP[AXIS_DE].value);
    }
    return;
}

/*******************************************************************************
 * @brief Determine the max slew rates for RA and DEC axis
 * @param raSlew max slew for RA axis
 * @param decSlew max slew for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::getMaxSlewSpeed(int *index)
{
    LOG_DEBUG(__FUNCTION__);
    // Command query Max Slew speed - :TTGMX#

    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if (!sendQuery(":TTGMX#", response))
    {
        LOG_ERROR("Failed to send query max slew speed.");
        return false;
    }
    int xx = 0, yy = 0;
    if (! sscanf(response, "%02da%02d", &xx, &yy))
    {
        LOGF_ERROR("Unexpected max slew speed status response '%s'.", response);
        return false;
    }

    switch (xx)
    {
        case 6:
            *index = 0;
            break;
        case 8:
            *index = 1;
            break;
        case 9:
            *index = 2;
            break;
        case 12:
            *index = 3;
            break;
        default:
            LOGF_ERROR("Unexpected max slew speed status response '%s'.", response);
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
bool StarGoTelescope::setMaxSlewSpeed(int index)
{
    LOG_DEBUG(__FUNCTION__);
    // Command query max slew rates  - :TTMX#
    //         parama                - xxyy#
    //         xx RA; yy DEC

    std::string cmd = ":TTMX";
    switch (index)
    {
        case 0:
            cmd.append("0606#");
            break;
        case 1:
            cmd.append("0808#");
            break;
        case 2:
            cmd.append("0909#");
            break;
        case 3:
            cmd.append("1212#");
            break;
        default:
            LOGF_ERROR("Unexpected max slew speed mode '%02d'.", index);
            return false;
    }
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (sendQuery(cmd.c_str(), response))
    {
        return true;
    }
    else
    {
        LOG_ERROR("Setting max slew speed mode FAILED");
        return false;
    }
}

/*******************************************************************************
 * @brief Determine the centering and finding speeds for RA and DEC axis
 * @param raSpeed factor for RA axis
 * @param decSpeed factor for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::getCenterFindSpeed(int *center, int * find )
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
 * @brief Set the centering and finding speeds for RA and DEC axis
 * @param raSpeed factor for RA axis
 * @param decSpeed factor for DEC axis
 * @return
*******************************************************************************/
bool StarGoTelescope::setCenterFindSpeed(int center, int find )
{
    LOG_DEBUG(__FUNCTION__);
    /*
 * Set centre and find speeds
 * X03aaaabbbb aaaa=center speed; bbbb=findspeed
 * valid center speeds:
*** deprecated 2x  = 007:  = 0x7a = 122
*** deprecated 3x  = 0051  = 0x51 = 81
*** deprecated 4x  = 003=  = 0x3d = 61
*** deprecated 6x  = 0028  = 0x28 = 40
*** deprecated 8x  = 001>  = 0x1e = 30
*** deprecated 10x = 0018  = 0x18 = 24
*** deprecated Parameter = 240/factor

2x  = 0042  = 0x42 = 66
3x  = 002<  = 0x2c = 44
4x  = 0021  = 0x21 = 33
6x  = 0016  = 0x16 = 22
8x  = 0010  = 0x10 = 16
10x = 000=  = 0x0d = 13

valid find speeds:
*** deprecated 10x  = 0031  = x31 = 49
*** deprecated 15x  = 0020  = x20 = 32
*** deprecated 20x  = 0018  = x18 = 24
*** deprecated 30x  = 0010  = x10 = 16
*** deprecated 50x  = 000:  = x0a = 10
*** deprecated 75x  = 0006  = x06 = 6
*** deprecated 100x = 0005  = x05 = 5
*** deprecated 150x = 0003  = x03 = 3
*** deprecated Parameter = 480/factor

10x  = 001:  = x1a = 26
15x  = 0012  = x12 = 18
20x  = 000=  = x0d = 13
30x  = 0009  = x09 = 9
50x  = 0005  = x05 = 5
75x  = 0004  = x04 = 4
100x = 0003  = x03 = 3
150x = 0002  = x02 = 2
*/
    double centerSpeeds[6]={66,44,33,22,16,13};
    double findSpeeds[8]={26,18,13,9,5,4,3,2};

    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    char centerstr[9] = {0};
    char findstr[9] = {0};
    int2ahex(centerstr, centerSpeeds[center]);
    int2ahex(findstr, findSpeeds[find]);
    sprintf(cmd, ":X03%4s%4s#", &centerstr[4], &findstr[4]);
    if (sendQuery(cmd, response, 0))
    {
        LOGF_INFO("Setting Center and Find: %s", cmd);
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
    if (!sendQuery(":X590#", response))
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
    // case with negative zero
    if (!d && dec < 0)
        snprintf(DecStr, sizeof(DecStr), ":Sd-%02d*%02d:%02d#", d, m, s);
    else
        snprintf(DecStr, sizeof(DecStr), ":Sd%+03d*%02d:%02d#", d, m, s);
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (isSimulation()) return true;
// These commands receive a response without a terminating #
    if (!sendQuery(RAStr, response, '1', 2)  || !sendQuery(DecStr, response, '1', 2) )
    {
        EqNP.setState(IPS_ALERT);
        EqNP.apply();
        LOG_ERROR("Error setting RA/DEC.");
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
    if (!sendQuery(disablecmd, disableresp) || !sendQuery(forcecmd, forceresp))
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
    if ( disable == 1)
    {
        *index = 1; // disabled
        LOG_WARN("Meridian flip DISABLED. BE CAREFUL, THIS MAY CAUSE DAMAGE TO YOUR MOUNT!");
    }
    else if ( force == 0)
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
        MeridianFlipModeSP.setState(IPS_OK);
        MeridianFlipModeSP.apply();
        return true;
    }
    if ( index > 2)
    {
        LOGF_ERROR("Invalid Meridian Flip Mode %d", index);
        return false;
    }
    const char* disablecmd = index==1 ? ":TTSFs#" : ":TTRFs#";
    const char* forcecmd  = index==2 ? ":TTSFd#" : ":TTRFd#";
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};
    if (!sendQuery(disablecmd, response) || !sendQuery(forcecmd, response))
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
     * if :X41 has invalid parameters, then :X42 also returns invalid parameters
     */
    LOG_DEBUG(__FUNCTION__);
    int raValue;
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    if (!sendQuery(":X42#", response))
        return false;

    if (sscanf(response, "or%04d#", &raValue) < 1)
    {
        LOG_ERROR("Unable to parse tracking adjustment response");
        return false;
    }

    *valueRA = static_cast<double>(raValue / 100.0);
    LOGF_DEBUG("%s RA adj: %.3f", __FUNCTION__, *valueRA);
    return true;
}

/*******************************************************************************
 * Adjust RA tracking speed.
*******************************************************************************/
bool StarGoTelescope::setTrackingAdjustment(double adjustRA)
{
    LOGF_DEBUG("%s RA adj: %.3f", __FUNCTION__, adjustRA);
    /*
     * :X41sRRR# to adjust the RA tracking speed where s is the sign + or -  and RRR are three digits whose meaning is parts
     * per 10000 of  RA correction .
     * :X43sDDD# to fix the cf DEC offset
     * :X41 accepts invalid parameters. Not sure what it does with them
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
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

    sprintf(cmd, ":X41%+04i#", parameter);
//    sprintf(cmd, ":X1E%04d", parameter);
    if (!sendQuery(cmd, response, 0))
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
bool StarGoTelescope::SendPulseCmd(TDirection direction, uint32_t duration_msec)
{
    LOGF_DEBUG("%s dir=%d dur=%d ms", __FUNCTION__, direction, duration_msec );
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

// 
    if (!usePulseCommand)
    {
        LOG_ERROR("Cannot pulse guide with ST4 enabled.");
        return false;
    }

    if (MovementNSSP.getState() == IPS_BUSY || MovementWESP.getState() == IPS_BUSY)
    {
        LOG_ERROR("Cannot guide while moving.");
        return false;
    }
    if (isParked())
    {
        LOG_ERROR("Cannot guide while parked.");
        return false;
    }
    if( direction < 0 && direction >= 4)
    {
        LOGF_ERROR("Invalid direction %d", direction);
        return false;
    }
    const INDI_EQ_AXIS axis[4] = {AXIS_DE,AXIS_DE,AXIS_RA,AXIS_RA};
    const char cdir[4] = { 'n','s','w','e' };
    const char* caxis[] = { "RA", "DE" };
    INDI_EQ_AXIS laxis = axis[direction];

// Use GetMotorStatus to find out what is happening with the motors
// Should be either 1 or 0 (tracking or idle) to allow guiding
    int motion[2] = {-1,-1};
    if (!getMotorStatus(&motion[AXIS_RA],&motion[AXIS_DE]))
    {
        LOG_ERROR("Cannot determine motor status.");
        return false;
    }
    if (motion[laxis] != MOTION_STATIC and motion[laxis] != MOTION_TRACK)
    {
        LOGF_ERROR("motor on %s axis is in use", caxis[laxis]);
        return false;
    }
 
    sprintf(cmd, ":Mg%c%04u#", cdir[direction], duration_msec);
    if (!sendQuery(cmd, response, 0)) // Don't wait for response - there isn't one
    {
        LOG_ERROR("Failed to send guide pulse request.");
        return false;
    }
    
// Set a timer to call back when guiding should have finished
// If there is already a timer remove it
    if (GuideTID[laxis])
    {
        IERmTimer(GuideTID[laxis]);
        GuideTID[laxis] = 0;
    }

// Set up the timer;
    timeoutArgs[laxis].me = this;
    timeoutArgs[laxis].axis = laxis;

    GuideTID[laxis] = IEAddTimer(static_cast<int>(duration_msec), guideTimeoutHelper, &timeoutArgs[laxis]);

// Assume the guide pulse was issued and acted upon.
    bool adjEnabled = (RaAutoAdjustSP.findOnSwitchIndex() == DefaultDevice::INDI_ENABLED);
// or autoRa->isEnabled()
// We could possibly move this to the timer handler
    if (laxis == AXIS_RA && adjEnabled)
    {
        autoRa->addsample(direction, duration_msec);
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
void StarGoTelescope::guideTimeoutHelper(void * p)
 {
     static_cast<guideTimeoutArgs *>(p)->me->guideTimeout(static_cast<guideTimeoutArgs *>(p)->axis);
 }
  
/*******************************************************************************
**
*******************************************************************************/
void StarGoTelescope::guideTimeout(INDI_EQ_AXIS axis)
{
    const char* caxis[] = { "RA", "DE" };
    LOGF_DEBUG("%s Axis: %s", __FUNCTION__, caxis[axis]);
    
// Check motor status
    int motion[2] = {-1,-1};
    
    INDI::PropertyNumber GuideNP[2] = {GuideWENP, GuideNSNP};
//////INumberVectorProperty* GuideNP[2] = {&GuideWENP, &GuideNSNP};
    const int direction[2][2] = {{DIRECTION_WEST, DIRECTION_EAST},
                                {DIRECTION_NORTH, DIRECTION_SOUTH} };

    if (!getMotorStatus(&motion[AXIS_RA], &motion[AXIS_DE]))
    {
        LOG_ERROR("Cannot determine motor status.");
        GuideNP[axis].setState(IPS_ALERT);
    }
    else if (motion[axis] != MOTION_STATIC and motion[axis] != MOTION_TRACK)
    {
        LOGF_WARN("Motor is still moving on axis %s", caxis[axis]);
        GuideNP[axis].setState(IPS_ALERT);
    }
    else
    {
        (GuideNP[axis])[direction[axis][0]].setValue( 0 );
        (GuideNP[axis])[direction[axis][1]].setValue( 0 );

        GuideComplete(axis);
        LOGF_DEBUG("Guiding completed on axis %s", caxis[axis]);
        return;
    }
    GuideNP[axis].apply();
    GuideTID[axis] = 0;     // Cancel the timer
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
        MountFirmwareInfoTP[0].text = new char[64];
        MountFirmwareInfoTP[1].text = new char[64];
        MountFirmwareInfoTP[2].text = new char[64];
        if (!getFirmwareInfo(MountFirmwareInfoTP[0].text,
                MountFirmwareInfoTP[1].text,
                MountFirmwareInfoTP[2].text ))
            LOG_ERROR("Failed to get firmware from device.");
        else
            MountFirmwareInfoTP.apply();

        char parkHomeStatus[2] = {'\0','\0'};
        if (getParkHomeStatus(parkHomeStatus))
        {
            SetParked(strcmp(parkHomeStatus, "2") == 0);
            if (strcmp(parkHomeStatus, "1") == 0)
            {
                SyncHomeSP.setState(IPS_OK);
                SyncHomeSP.apply();
            }
        }
        bool isEnabled;
        if (getST4Status(&isEnabled))
        {
            ST4StatusSP[INDI_ENABLED].setState(isEnabled ? ISS_ON : ISS_OFF);
            ST4StatusSP[INDI_DISABLED].setState(isEnabled ? ISS_OFF : ISS_ON);
            ST4StatusSP.setState(IPS_OK);
        }
        else
        {
            ST4StatusSP.setState(IPS_ALERT);
        }
        ST4StatusSP.apply();

        double raCorrection;
        if (getTrackingAdjustment(&raCorrection))
        {
            TrackingAdjustmentNP[0].setValue(raCorrection);
            TrackingAdjustmentNP.setState(IPS_OK);
        }
        else
        {
            TrackingAdjustmentNP.setState(IPS_ALERT);
        }
        TrackingAdjustmentNP.apply();

        if (getKeypadStatus(&isEnabled))
        {
            KeypadStatusSP[INDI_ENABLED].setState(isEnabled ? ISS_ON : ISS_OFF);
            KeypadStatusSP[INDI_DISABLED].setState(isEnabled ? ISS_OFF : ISS_ON);
            KeypadStatusSP.setState( IPS_OK);
        }
        else
        {
            KeypadStatusSP.setState(IPS_ALERT);
        }
        KeypadStatusSP.apply();

        int index;
        if (GetMeridianFlipMode(&index))
        {
            MeridianFlipModeSP.reset();
            MeridianFlipModeSP[index].setState(ISS_ON);
            MeridianFlipModeSP.setState(IPS_OK);
        }
        else
        {
            MeridianFlipModeSP.setState(IPS_ALERT);
        }
        MeridianFlipModeSP.apply();

        int raSlew;
        if (getMaxSlewSpeed(&raSlew))
        {
            MaxSlewSpeedSP.reset();
            MaxSlewSpeedSP[raSlew].setState(ISS_ON);
            MaxSlewSpeedSP.setState(IPS_OK);
        }
        else
        {
            MaxSlewSpeedSP.setState(IPS_ALERT);
        }
        MaxSlewSpeedSP.apply();

        int centerSpeed, findSpeed;
        if (getCenterFindSpeed(&centerSpeed, &findSpeed ))
        {
            CenterSpeedSP.reset();
            FindSpeedSP.reset();
            CenterSpeedSP[centerSpeed].setState(ISS_ON);
            FindSpeedSP[findSpeed].setState(ISS_ON);
            CenterSpeedSP.setState(IPS_OK);
            FindSpeedSP.setState(IPS_OK);
        }
        else
        {
            CenterSpeedSP.setState(IPS_ALERT);
            FindSpeedSP.setState(IPS_ALERT);
        }
        CenterSpeedSP.apply();
        FindSpeedSP.apply();

// Get the guiding speed
        int raSpeed, decSpeed;
        if (getGuidingSpeeds(&raSpeed, &decSpeed))
        {
            GuidingSpeedNP[0].setValue( static_cast<double>(raSpeed / 100.0) );
            GuidingSpeedNP[1].setValue( static_cast<double>(decSpeed / 100.0) );
            GuidingSpeedNP.setState(IPS_OK);
        }
        else
        {
            LOG_ERROR("Unable to get guiding speed");
            GuidingSpeedNP.setState(IPS_ALERT);
        }
        GuidingSpeedNP.apply();

        int raRatio, decRatio;
        if (getGearRatios(&raRatio, &decRatio))
        {
            GearRatioNP[0].setValue( static_cast<double>(raRatio) );
            GearRatioNP[1].setValue( static_cast<double>(decRatio) );
            GearRatioNP.setState(IPS_OK);
        }
        else
        {
            GearRatioNP.setState(IPS_ALERT);
        }
        GearRatioNP.apply();

        int torque;
        if (getTorque(&torque))
        {
            TorqueNP[0].setValue( static_cast<double>(torque) );
            TorqueNP.setState(IPS_OK);
        }
        else
        {
            LOG_ERROR("Unable to get torque");
            TorqueNP.setState(IPS_ALERT);
        }
        TorqueNP.apply();

        bool raDir, decDir;
        if (getMotorReverse(&raDir, &decDir))
        {
            RaMotorReverseSP[INDI_ENABLED].setState(raDir ? ISS_ON : ISS_OFF);
            RaMotorReverseSP[INDI_DISABLED].setState(raDir ? ISS_OFF : ISS_ON);
            RaMotorReverseSP.setState(IPS_OK);
            DecMotorReverseSP[INDI_ENABLED].setState(decDir ? ISS_ON : ISS_OFF);
            DecMotorReverseSP[INDI_DISABLED].setState( decDir ? ISS_OFF : ISS_ON);
            DecMotorReverseSP.setState(IPS_OK);
        }
        else
        {
            RaMotorReverseSP.setState(IPS_ALERT);
            DecMotorReverseSP.setState(IPS_ALERT);
        }
        RaMotorReverseSP.apply();
        DecMotorReverseSP.apply();
    }

// Time and Location capabilites are hard coded in this driver
    if (getLocationOnStartup && (GetTelescopeCapability() & TELESCOPE_HAS_LOCATION))
        getScopeLocation();

// Should get the time in ReadScopeStatus which is called from Handshake
    if (getTimeOnStartup && (GetTelescopeCapability() & TELESCOPE_HAS_TIME))
        getScopeTime();

// FIXME. Need to decide if pulseguiding is dependent on ST4 guiding enablement
// It seems unwise to permit both ST4 guiding and pulse guiding simultaneously
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
    if (!sendQuery(":GW#", response))
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
    if (!sendQuery(":X34#", response))
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

    double currentRA = EqNP[AXIS_RA].value;
    double currentDEC = EqNP[AXIS_DE].value;
    double targetRA = TargetNP[AXIS_RA].value;
    double targetDEC = TargetNP[AXIS_DE].value;

    /* Process per current state. We check the state of EQUATORIAL_COORDS and act acoordingly */
    switch (TrackState)
    {

    case SCOPE_IDLE:
        currentRA  += (TRACKRATE_SIDEREAL/3600.0 * dt / 15.);
        break;

    case SCOPE_TRACKING:
        switch (TrackModeSP.findOnSwitchIndex())
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
            da = ((TrackRateNP[AXIS_RA].value-TRACKRATE_SIDEREAL)/3600.0 * dt / 15.);
            dx = (TrackRateNP[AXIS_DE].value/3600.0 * dt);
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
        LOG_ERROR("Failed to send get RA gear ratio request.");
        return false;
    }
    *raRatio = ahex2int(&response[2]);
    if (!sendQuery(":X481#", response))
    {
        LOG_ERROR("Failed to send get DEC gear ratio request.");
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
/*
 * Get RA and Dec motor directions (Forward, Reverse)
 * Get X1B => wrd where r=RA; d=DEC
*/
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
/*
 * Set RA and Dec motor directions (Forward, Reverse)
 * Reverse RA X1A0n; DEC X1A1n
 * where n = 0/1
*/
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
/*
 Get: TTGT
 Return tnnn# where nnn is torque %
*/
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
/*
 * Set motor torque %
 :TTTnnn#
*/
    char cmd[AVALON_COMMAND_BUFFER_LENGTH] = {0};
    char response[AVALON_RESPONSE_BUFFER_LENGTH] = {0};

// Consider rounding to nearest 10%
//  torque = torque/10 * 10;
// Need to send the PAUSE commnd to reset the mount
// Need to see how Torque is then set on restart - presumably from the config file
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
    
// Get the time and compare to last transmit. If > xmitDelay then ok, else wait
    std::chrono::nanoseconds delay = xmitDelay - 
                                     std::chrono::nanoseconds(std::chrono::system_clock::now() - lastXmit);
    if (delay.count() > 0)
    {
        LOGF_DEBUG("Delay transmit for %.1f ms / %.1f ms", static_cast<double>(delay.count())/1000000.0, 
                   static_cast<double>(xmitDelay.count())/1000000.0);
        // Convert duration to timespec values
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(delay);
        delay -= secs;
        timespec sleep_ts{static_cast<time_t>(secs.count()), static_cast<long>(delay.count())};  // int (time_t), long
        nanosleep(&sleep_ts, nullptr);
    }
    // Update the transmit timer
    lastXmit = std::chrono::system_clock::now();

    if (!transmit(cmd))
    {
        LOGF_ERROR("Command <%s> failed.", cmd);
        return false;
    }
    lresponse[0] = '\0';
    int lwait = wait;
    bool found = false;
    while (receive(lresponse, &lbytes, end, lwait))
    {
        lbytes=0;
        if (! ParseMotionState(lresponse))
        {
            // Take the first response that is no motion state
            if (!found)
                strcpy(response, lresponse);
            found = true;
            lwait = 0;
        }
    }
    flush();

    return true;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::ParseMotionState(char* state)
{
    LOGF_DEBUG("%s %s", __FUNCTION__, state);
    int lmotor, lmode, lslew;
    if (sscanf(state, ":Z1%01d%01d%01d", &lmotor, &lmode, &lslew)==3
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
    int timeout = wait; //? AVALON_TIMEOUT: 0;
    int returnCode = tty_read_section(PortFD, buffer, end, timeout, bytes);
    if (returnCode != TTY_OK)
    {
        char errorString[MAXRBUF];
        tty_error_msg(returnCode, errorString, MAXRBUF);
        if (returnCode==TTY_TIME_OUT && wait <= 0) return false;
        LOGF_WARN("Failed to receive full response: %s. (Return code: %d)", errorString, returnCode);
        return false;
    }
    if (buffer[*bytes-1]=='#')
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
** sub class AutoAdjust
*******************************************************************************/
const double StarGoTelescope::AutoAdjust::Z_SAMPLE_DURATION_MS = 20000.0;  // Z-filter sample duration

StarGoTelescope::AutoAdjust::AutoAdjust(StarGoTelescope *ptr)
{
    p = ptr;
    // Set RA Auto Adjust Z-filter parameters
    // Based on a 20 second sample period (Z_SAMPLE_DURATION_MS) and the long period drift
    // of the M-Uno mount (and others??) of 1200 seconds with 50" p-p (~0.13"/s max drift)
    // There are further frequency spikes at 600s (14" p-p or ~0.07"/s max)
    // and 180s (7.4" p-p or 0.125"/s max)
    // Corner period: For Butterworth and Bessel lowpass designs, the corner frequency is the frequency 
    // at which the magnitude of the response is -3 dB.
    // We want the corner to be at around 600s or less so that full attenuation occurs at 1200s.
    // So the corner period vs sample period is 600/20 = 30x
    // Consider the corner at 400s
    zfilter = new ZFilterFactory(ptr);
    zfilter->rebuild( BUTTERWORTH, 4, 20 );
}
/*******************************************************************************
**
*******************************************************************************/
StarGoTelescope::AutoAdjust::~AutoAdjust()
{
    stop();
    delete zfilter;
}

/*******************************************************************************
**
*******************************************************************************/
bool StarGoTelescope::AutoAdjust::setEnabled(bool isenabled)
{
    LOGF_DEBUG("%s enabled=%d",__FUNCTION__, enabled);
    enabled = isenabled;
    LOGF_INFO("RA Auto Adjust %s.", enabled?"enabled":"disabled");
    if (!enabled)
    {
        stop();
    }
    else
    {
        start();
    }
    return true;
}

/*******************************************************************************
**
*******************************************************************************/
void StarGoTelescope::AutoAdjust::start()
{
    LOG_DEBUG(__FUNCTION__);
    p->setTrackingAdjustment(0.0);
    zfilter->resetsamples();
    samples.clear();
    stop();
    sampleTimerID = IEAddPeriodicTimer(static_cast<int>(Z_SAMPLE_DURATION_MS), sampleTimerHelper, this);

}
/*******************************************************************************
**
*******************************************************************************/
void StarGoTelescope::AutoAdjust::stop()
{
    // If there is a timer remove it
    if (sampleTimerID)
    {
        IERmTimer(sampleTimerID);
        sampleTimerID = 0;
    }
    samples.clear();
}

/*******************************************************************************
** Zfilter version
*******************************************************************************/
bool StarGoTelescope::AutoAdjust::addsample(int8_t direction, uint32_t duration_msec)
{
    LOGF_DEBUG("%s Dir: %d; Dur: %d", __FUNCTION__, direction, duration_msec);
try{
    if (!enabled)
    {
        LOG_ERROR("Auto tracking adjustment is currently DISABLED");
        return false;
    }

    if(!(direction == STARGO_EAST || direction == STARGO_WEST))
    {
        LOG_ERROR("Invalid direction");
        return false;
    }
    double ddir = direction == STARGO_EAST ? -1.0 : 1.0;
    
//  Get the guiding speed
    double guidingSpeed = p->GuidingSpeedNP[0].value;
    int raSpeed, decSpeed;

    if (!p->getGuidingSpeeds(&raSpeed, &decSpeed))
    {
        LOG_ERROR("Unable to get guiding speed");
        return false;
    }
    guidingSpeed =  raSpeed/100.0;

    // Calculate the correction in milliseconds normalised to sidereal rate
    double ynewest = ddir * static_cast<double>(duration_msec) * guidingSpeed;
    samples.push_back(ynewest);
    LOGF_DEBUG("Correction: %.0f ms @ Sidereal. Guide rate: %.2f", ynewest, guidingSpeed);

    return true;
}
catch (std::exception& e)
{
    LOGF_ERROR("%s >>> %s", __FUNCTION__, e.what());
    return false;
}

}

/*******************************************************************************
**
*******************************************************************************/
void StarGoTelescope::AutoAdjust::sampleTimerHelper(void *p)
{
    static_cast<StarGoTelescope::AutoAdjust *>(p)->sampleTimerProcess();
}

/*******************************************************************************
**
*******************************************************************************/
void StarGoTelescope::AutoAdjust::sampleTimerProcess()
{
    LOG_DEBUG(__FUNCTION__);
try{
// Sum the corrections in msec @ sidereal
    double sumCorr = 0.0;
    int i=0;

// This could take place while a sample is being added to the queue 
// and may be the cause of crashes. 
// Should lock the queue while processing it either here or when adding samples
// Maybe easier to do here even though adding should take priority
    while (!samples.empty())
    {
        sumCorr += samples.front();
        samples.pop_front();
        i++;
    }
    LOGF_DEBUG("%d samples sum to %.0f ms correction", i, sumCorr);
    
// Calculate the correction to long period drift with the zfilter
// Convert it to a tracking adjustment by dividing by the sampling period
// i.e. we have sidereal ms correction over duration ms => sidereal adjustment
// FIXME: the corrections seem to be a factor of 10x too small
    double newcorrection = zfilter->addsample(sumCorr);
    double slope = newcorrection / Z_SAMPLE_DURATION_MS;

//  get trackrate adjustment percentage
    double currAdjust = p->TrackingAdjustmentNP[0].value;
    if (!p->getTrackingAdjustment(&currAdjust))
    {
        LOG_ERROR("Unable to get tracking adjustment ");
        return;
    }

// Convert to a percentage (of Sidereal rate)
// Need to add current adjustment as the correction is on top of that
// The return from addsample is the LP filtered deviation.
// The correction needs to be in the same direction to take some of the burden
// of corrections away from guiding
    double newAdjust = slope*100.0 + currAdjust;
    LOGF_DEBUG("Correction %.4e ms Adjustment: %.4e %% sidereal", newcorrection, newAdjust);

    if (fabs(newAdjust-currAdjust) > 0.005)
    {
        LOGF_INFO("RA auto adjust rate from %.2f to %.2f", currAdjust, newAdjust);
        if(p->setTrackingAdjustment(newAdjust))
        {
            p->TrackingAdjustmentNP[0].value = newAdjust;
            p->TrackingAdjustmentNP.setState(IPS_OK);
        }
        else
        {
            LOGF_ERROR("RA tracking adjust from %.2f to %.2f failed", currAdjust, newAdjust);
            p->TrackingAdjustmentNP.setState(IPS_ALERT);
        }
        p->TrackingAdjustmentNP.apply();
    }
    else
    {
        LOGF_INFO("No change in RA auto adjust rate %.2f to %.2f", currAdjust, newAdjust);
    }

}
catch (std::exception& e)
{
    LOGF_ERROR("%s >>> %s", __FUNCTION__, e.what());
    return;
}
}
