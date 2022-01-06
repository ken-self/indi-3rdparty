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

#ifndef STARGO_TELESCOPE_H
#define STARGO_TELESCOPE_H

//#pragma once

#include <indiguiderinterface.h>
#include <inditelescope.h>

#include <indicom.h>
#include <indilogger.h>
#include <termios.h>

#include <cstring>
#include <string>
#include <cmath>
#include <unistd.h>
#include <deque>
#include <chrono>

#define STARGO_TIMEOUT 5 /* FD timeout in seconds */
#define RB_MAX_LEN    64
#define AVALON_TIMEOUT                                  2
#define AVALON_COMMAND_BUFFER_LENGTH                    32
#define AVALON_RESPONSE_BUFFER_LENGTH                   32
#define STARGO_GENERIC_SLEWRATE 5        /* slew rate, degrees/s */


enum TDirection
{
    STARGO_NORTH,
    STARGO_WEST,
    STARGO_EAST,
    STARGO_SOUTH,
    STARGO_ALL
};

// StarGo specific tabs
extern const char *ADVANCED_TAB;

class StarGoTelescope : public INDI::Telescope, public INDI::GuiderInterface
{
public:
    enum TrackMode
    {
        TRACK_SIDEREAL=0, //=Telescope::TelescopeTrackMode::TRACK_SIDEREAL,
        TRACK_SOLAR=1, //=Telescope::TelescopeTrackMode::TRACK_SOLAR,
        TRACK_LUNAR=2, //=Telescope::TelescopeTrackMode::TRACK_LUNAR,
        TRACK_NONE=3
    };
    enum MotorsState
    {
        MOTORS_OFF=0,
        MOTORS_DEC_ONLY=1,
        MOTORS_RA_ONLY=2,
        MOTORS_ON=3
    };
    enum MotionState
    {
        MOTION_STATIC=0,
        MOTION_TRACK=1,
        MOTION_ACCEL=2,
        MOTION_DECEL=3,
        MOTION_GUIDE=4,
        MOTION_SLEW=5
    };
    enum MotorReverseection
    {
        DIRECTION_NORMAL=0,
        DIRECTION_REVERSE=1
    };
    TrackMode CurrentTrackMode {TRACK_SIDEREAL};
    MotorsState CurrentMotorsState {MOTORS_OFF};
    TelescopeSlewRate CurrentSlewRate {SLEW_MAX};

    StarGoTelescope();

    virtual const char *getDefaultName() override;
    virtual bool Handshake() override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual bool saveConfigItems(FILE *fp) override;

protected:

    // Sync Home Position
    ISwitchVectorProperty SyncHomeSP;
    ISwitch SyncHomeS[1];

    // firmware info
    ITextVectorProperty MountFirmwareInfoTP;
    IText MountFirmwareInfoT[3] = {};

    // goto home
    ISwitchVectorProperty MountGotoHomeSP;
    ISwitch MountGotoHomeS[1];

    // Guiding
    INumberVectorProperty GuidingSpeedNP;
    INumber GuidingSpeedN[2];

    // ST4 status
    ISwitchVectorProperty ST4StatusSP;
    ISwitch ST4StatusS[2];

    // Keypad
    ISwitchVectorProperty KeypadStatusSP;
    ISwitch KeypadStatusS[2];

    // Max slew speed
    ISwitchVectorProperty MaxSlewSpeedSP;
    ISwitch MaxSlewSpeedS[4];

    // Center speeds
    ISwitchVectorProperty CenterSpeedSP;
    ISwitch CenterSpeedS[6];

    // Center and Find speeds
    ISwitchVectorProperty FindSpeedSP;
    ISwitch FindSpeedS[8];

    // RA Track Adjust
    INumberVectorProperty TrackingAdjustmentNP;
    INumber TrackingAdjustmentN[1];

    // Auto RA Tracking Adjustment
    ISwitchVectorProperty RaAutoAdjustSP;
    ISwitch RaAutoAdjustS[2];

    // meridian flip
    ISwitchVectorProperty MeridianFlipModeSP;
    ISwitch MeridianFlipModeS[3];

    // configurable delay between two commands to avoid flooding StarGO
    INumberVectorProperty MountRequestDelayNP;
    INumber MountRequestDelayN[1];

    // Gear ratios
    INumberVectorProperty GearRatioNP;
    INumber GearRatioN[2];

    // Torque
    INumberVectorProperty TorqueNP;
    INumber TorqueN[1];

    // MotorReverse section
    ISwitchVectorProperty RaMotorReverseSP;
    ISwitch RaMotorReverseS[2];
    ISwitchVectorProperty DecMotorReverseSP;
    ISwitch DecMotorReverseS[2];

    // Motor Step Position
    INumberVectorProperty MotorStepNP;
    INumber MotorStepN[2];

    // HA and LST
    INumberVectorProperty HaLstNP;
    INumber HaLstN[2];

    bool usePulseCommand { true };
    struct timespec mount_request_delay = {0, 50000000L};

    bool getTimeOnStartup=true, getLocationOnStartup=true;
    uint8_t DBG_SCOPE;

    bool ParkOptionBusy { false };

/***********************************************************************************************
* Virtual functions
 ***********************************************************************************************/
        // Telescope:: virtual functions
    virtual bool ReadScopeStatus() override;
    virtual bool updateLocation(double latitude, double longitude, double elevation) override;
//    virtual bool updateTime(ln_date *utc, double utc_offset);
    virtual bool Sync(double ra, double dec) override;
    virtual bool SetParkPosition(double Axis1Value, double Axis2Value) override;
    virtual bool SetDefaultPark() override;
    virtual bool SetCurrentPark() override;
    virtual bool Park() override;
    virtual bool UnPark() override;
    virtual bool SetSlewRate(int index) override;
    virtual bool Goto(double ra, double dec) override;
    virtual bool Abort() override;
    virtual bool SetTrackMode(uint8_t mode) override;
    virtual bool SetTrackEnabled(bool enabled) override;
    virtual bool SetTrackRate(double raRate, double deRate) override;

        // GuiderInterface virtual functions
    virtual IPState GuideNorth(uint32_t ms) override;
    virtual IPState GuideSouth(uint32_t ms) override;
    virtual IPState GuideEast(uint32_t ms) override;
    virtual IPState GuideWest(uint32_t ms) override;
//    virtual void GuideComplete(INDI_EQ_AXIS axis) override;
    virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;



/***********************************************************************************************
* StarGo specific functions
 ***********************************************************************************************/
// Location and time
    bool getScopeLocation();
    bool getSiteLatitude(double *siteLat);
    bool setSiteLatitude(double Lat);
    bool getSiteLongitude(double *siteLong);
    bool setSiteLongitude(double Long);
    bool setLocalSiderealTime(double longitude);
    bool getLST_String(char* input);
    bool getLST(double *lst);
    bool getScopeTime();
    bool getLocalDate(char *dateString) ;
    bool setLocalDate(uint8_t days, uint8_t months, uint16_t years) ;
    bool getLocalTime(char *timeString) ;
    bool setLocalTime24(uint8_t hour, uint8_t minute, uint8_t second) ;
    bool getUTCOffset(double *offset) ;
    bool setUTCOffset(double offset) ;

// Parking
    bool getParkHomeStatus (char* status);
    bool setHomeSync();
    bool setParkPosition();
    bool setGotoHome();
    void WaitParkOptionReady();
    void updateParkPosition();

// Slewing
    bool getMaxSlewSpeed(int *slewSpeed);
    bool setMaxSlewSpeed(int slewSpeed);
    bool getCenterFindSpeed(int *centerSpeed, int *findSpeed ); // Not implemented
    bool setCenterFindSpeed(int centerSpeed, int findSpeed );
    bool setSlewMode(int slewMode);
    bool getEqCoordinates(double *ra, double *dec);
    bool setObjectCoords(double ra, double dec);
    bool getSideOfPier();
    bool SetMeridianFlipMode(int index);
    bool GetMeridianFlipMode(int *index);

// autoguiding and tracking
    bool getTrackingAdjustment(double *valueRA);
    bool setTrackingAdjustment(double adjustRA);
    bool isGuiding();
    bool setGuidingSpeeds(int raSpeed, int decSpeed);
    bool getGuidingSpeeds(int *raSpeed, int *decSpeed);
    bool setST4Enabled(bool enabled);
    bool getST4Status(bool *isEnabled);
    int SendPulseCmd(int8_t direction, uint32_t duration_msec) ;

// Misc
    void getBasicData();
    bool getScopeAlignmentStatus(char *mountType, bool *isTracking, int *alignmentPoints);
    bool getMotorStatus(int *xSpeed, int *ySpeed);
    bool getKeypadStatus (bool *isEnabled);
    bool setKeyPadEnabled(bool enabled);
    bool getFirmwareInfo(char *version, char *mount, char *tcb );
    void mountSim();

// Motors
    bool getGearRatios(int *raRatio, int *decRatio);
    bool getMotorSteps(double *raSteps, double *decSteps);
    bool getMotorReverse(bool *raDir, bool *decDir);   //Not implemented
    bool setMotorReverse(bool raDir, bool decDir);  //Not implemented
    bool getTorque(int *torque);  //Not implemented
    bool setTorque(int torque);  //Not implemented

protected:
// Helper functions
    bool sendQuery(const char* cmd, char* response, char end, int wait=AVALON_TIMEOUT);
    bool sendQuery(const char* cmd, char* response, int wait=AVALON_TIMEOUT);
    bool ParseMotionState(char* state);
    void setMountRequestDelay(int secs, long nanosecs);
    bool receive(char* buffer, int* bytes, int wait=AVALON_TIMEOUT);
    bool receive(char* buffer, int* bytes, char end, int wait=AVALON_TIMEOUT);
    void flush();
    bool transmit(const char* buffer);
    double ahex2int(char* ahex);
    void int2ahex(char * ahex, double val);

// Manage the RA auto adjustment functionality
    class AutoAdjust
    {
    public:
        AutoAdjust(StarGoTelescope *ptr);
        bool setEnabled(bool isenabled);
        bool setRaAdjust(int8_t direction, uint32_t duration_msec);
    private:
        std::deque<double> x;
        std::deque<double> y;
        std::chrono::time_point<std::chrono::system_clock> start;
        double  sumx, sumy, sumxy, sumx2;
        double  xmin, xmax;
        uint32_t nmin;
        bool enabled;
        StarGoTelescope *p;
        void reset();
        const char* getDeviceName(){return p->getDeviceName();}
    };
    AutoAdjust *autoRa;
};
inline bool StarGoTelescope::isGuiding(){
    return (GuideNSNP.s == IPS_BUSY || GuideWENP.s == IPS_BUSY);
}
inline bool StarGoTelescope::sendQuery(const char* cmd, char* response, int wait)
{
    return sendQuery(cmd, response, '#', wait);
}
inline void StarGoTelescope::setMountRequestDelay(int secs, long nanosecs)
{
        mount_request_delay.tv_sec = secs;
        mount_request_delay.tv_nsec = nanosecs;
};
inline bool StarGoTelescope::receive(char* buffer, int* bytes, int wait)
{
    return receive(buffer, bytes, '#', wait);
}
//Convert to Hex using ASCII-48 (ASCII-x30)
inline double StarGoTelescope::ahex2int(char* ahex)
{
    double val=0;
    for(unsigned int i=0; i<strlen(ahex); i++)
    {
        val = val*16 + ahex[i] - 48;
    }
    return val;
}
//Convert to ASCII using Hex+48
inline void StarGoTelescope::int2ahex(char * ahex, double val)
{
    uint32_t ival = static_cast<uint32_t>(round(val));
    for(int i=7; i>=0; i--)
    {
        ahex[i] = (ival & 0x0000000F) + 48;
        ival = ival >> 4;
    }
    return;
}

#endif // STARGO_TELESCOPE_H
