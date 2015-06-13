#ifndef __API_MICROP_H
#define __API_MICROP_H

#define PROXIMITY_KDATA_SIZE 11

enum Batt_ID{
    Batt_P01 = 0,
    Batt_Dock = 1,
};

enum p01_Cable_Status{
    P01_CABLE_UNKNOWN = -1,
    P01_CABLE_NO = 0,
    P01_CABLE_CHARGER = 1,
    P01_CABLE_USB = 2,
    P01_CABLE_OTG = 7,
};

enum p01_Charging_Status{
    P01_CHARGING_ERR = -1,
    P01_CHARGING_NO = 0,
    P01_CHARGING_ONGOING = 1,
    P01_CHARGING_FULL = 2,
};

enum P72_HW_ID{
    P72_ER1_2_HWID = 0,
    P72_ER2_HWID = 1,
    P72_PR_HWID = 2,
    P72_MP_HWID = 3,
    P72_ER1_1_HWID = 98,
    P72_SR_HWID = 99,
};

enum P72G_TS_ID {
    P72G_TS_WINTEK = 1,
    P72G_TS_OFILM  = 2,
    P72G_TS_JTOUCH = 3,
};


/*
*       Check the status of P01 connectness
*       return value: 1: P01 connected
*/

int AX_MicroP_IsP01Connected(void);


/*
*       Check the status of Dock connectness
*       return value: 1: Dock connected and ready
*/
int AX_MicroP_IsDockReady(void);


/*
*       Check the status of Headphone if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err


int AX_MicroP_IsHeadPhoneIn(void);

*/


/*
*       Check the status of AC/USB if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsACUSBIn(void);




/*
*       Check the status of Dock if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsECDockIn(void);



/*
*       Check the status of Dock battery if it is power-bad
*       return value: 1: PowerBad, <0: err
*/

int AX_MicroP_Is_ECBattPowerBad(void);


/*
*       Check the status of Dock Ext. Power if ext power is in
*       return value: 1: PowerBad, <0: err
*/
int AX_MicroP_Is_ECExtPowerCableIn(void);


/*
*   @AX_MicroP_get_ChargingStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*
* return: -1: charge error, 0: no charge, 1: charging normal, 2: charging full, < 0: other error
*/

int AX_MicroP_get_ChargingStatus(int target);




/*
*   @AX_MicroP_get_USBDetectStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*
*  return: 0 for 'no charger/usb', 1 for 'charger', 2 for 'USB', '255' for 'unknown', <0 value means something error
*/

int AX_MicroP_get_USBDetectStatus(int target);




/*
*  GPIO direct control
*  @ AX_MicroP_getGPIOPinLevel
*  input:
	- pinID
*  return: 0 for low, 1 for high, <0 value means something error
*

*  @ AX_MicroP_setGPIOOutputPin
*  input:
*	- pinID
*	- level: 0 for low, 1 for high
*  return: the status of operation. 0 for success, <0 value means something error

*  @ AX_MicroP_getGPIOOutputPinLevel
*  input:
	- pinID
*  return: 0 for low, 1 for high, <0 value means something error
*/

int AX_MicroP_getGPIOPinLevel(int pinID);
int AX_MicroP_setGPIOOutputPin(unsigned int pinID, int level);
int AX_MicroP_getGPIOOutputPinLevel(int pinID);





int AX_MicroP_setPWMValue(uint8_t value);



int AX_MicroP_getPWMValue(void);


#if defined(ASUS_A11_PROJECT) || defined(ASUS_A68M_PROJECT)
int AX_MicroP_Is_resuming(void);
void AX_MicroP_Bus_Suspending(int susp);
#endif

int AX_MicroP_enterSleeping(void);
int AX_MicroP_enterResuming(void);
/*
*  @AX_MicroP_enableInterrupt
*  input:
*            - intrpin: input pin id
*            -  enable: 0 for 'disable', 1 for 'enable'
*  return: 0 for success, <0 value means something error
*/

int AX_MicroP_enablePinInterrupt(unsigned int pinID, int enable);



/*
*  @AX_MicroP_readBattCapacity
*  input: target
*           0: p01 battery
*           1: dock battery
*  return: 0 for success, <0 value means something error
*/


int AX_MicroP_readBattCapacity(int target);

int AX_MicroP_readGaugeAvgCurrent(void);

/*
*  @AX_IsPadUsing_MHL_H
*  return: 1 for MHL_H
*  return: 0 for MHL_L
    else: something err
*/



int AX_MicroP_getOPState(void);

int AX_MicroP_writeKDataOfLightSensor(uint32_t data);
uint32_t AX_MicroP_readKDataOfLightSensor(void);


int AX_MicroP_getTSID(void);
uint8_t AX_MicroP_getHWID(void);
int AX_MicroP_IsMydpNewSKU(void);
int AX_MicroP_getBatterySoc(void *battInfo);
int AX_MicroP_getBatteryInfo(void *battInfo);
int AX_MicroP_setOTGPower(uint16_t value);
int AX_MicroP_getMICROPID(void);

void AX_MicroP_set_VBusPower(int level);
int AX_MicroP_get_VBusPower(void);


int AX_MicroP_set_Proxm_crosstalk(unsigned char *data);
int AX_MicroP_get_Proxm_crosstalk(unsigned char *data);
int AX_MicroP_set_Proxm_threshold(unsigned char *data);
int AX_MicroP_get_Proxm_threshold(unsigned char *data);


int AX_request_gpio_33(void);
void AX_setECPowerOff(void);

int AX_MicroP_setSPK_EN(uint8_t enable);
int AX_MicroP_setRCV_EN(uint8_t enable);
#endif
