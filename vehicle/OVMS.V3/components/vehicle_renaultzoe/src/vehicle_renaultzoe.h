/*
;    Project:       Open Vehicle Monitor System
;    Date:          14th March 2017
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011        Sonny Chen @ EPRO/DX
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#ifndef __VEHICLE_RENAULTZOE_H__
#define __VEHICLE_RENAULTZOE_H__

#include "vehicle.h"

using namespace std;

class OvmsVehicleRenaultZoe : public OvmsVehicle
  {
  public:
    OvmsVehicleRenaultZoe();
    ~OvmsVehicleRenaultZoe();
  

public:
void IncomingFrameCan1(CAN_frame_t* p_frame);
// void Ticker1(uint32_t ticker, OvmsWriter* writer);
void Ticker1(uint32_t ticker);
void IncomingPollReply(canbus* bus, uint16_t type, uint16_t pid, uint8_t* data, uint8_t length, uint16_t mlremain);
void ConfigChanged(OvmsConfigParam* param);



protected:
void vehicle_renaultzoe_car_on(bool isOn);
void UpdateMaxRangeAndSOH(void);
uint16_t calcMinutesRemaining(float target);
bool SendCanMessage_sync(uint16_t id, uint8_t count,
                         uint8_t serviceId, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4,
                         uint8_t b5, uint8_t b6);
bool SendCommandInSessionMode(uint16_t id, uint8_t count,
                              uint8_t serviceId, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4,
                              uint8_t b5, uint8_t b6 );
bool SetTemporarySessionMode(uint16_t id, uint8_t mode);
bool SetDoorLock(bool open, const char* password);
bool OpenTrunk(const char* password);
bool IsPasswordOk(const char *password);
void SetChargeMetrics(float voltage, float current, float climit, bool chademo);

// Renault ZOE specific metrics
OvmsMetricFloat*  m_b_cell_volt_max;           // Battery cell maximum voltage
OvmsMetricFloat*  m_b_cell_volt_min;           // Battery cell minimum voltage
OvmsMetricFloat*  m_b_cell_det_max;               // Battery cell maximum detoriation
OvmsMetricFloat*  m_b_cell_det_min;               // Battery cell minimum detoriation
OvmsMetricFloat*  m_b_bat_avail_energy;               // Battery available kWh
OvmsMetricFloat*  m_b_bat_max_charge_power;    // Max Bat Charge
OvmsMetricFloat*  m_mains_power;                // Mains Power Consumed
OvmsMetricFloat*  m_v_hydraulic_brake_power; // Hydraulic Brake Power Request (Losses)
OvmsMetricFloat* m_bat_heat_sink_temp;
OvmsMetricInt* m_charge_pilot_current;
OvmsMetricInt* m_b_temp1;
OvmsMetricInt* m_b_temp2;
OvmsMetricInt* m_b_temp3;
OvmsMetricInt* m_b_temp4;
OvmsMetricInt* m_b_temp5;
OvmsMetricInt* m_b_temp6;
OvmsMetricInt* m_b_temp7;
OvmsMetricInt* m_b_temp8;
OvmsMetricInt* m_b_temp9;
OvmsMetricInt* m_b_temp10;
OvmsMetricInt* m_b_temp11;
OvmsMetricInt* m_b_temp12;

//float rz_battery_max_detoriation;                 //02 21 05 -> 24 1+2
//float rz_battery_min_detoriation;                 //02 21 05 -> 24 4+5


#define CFG_DEFAULT_MAXRANGE 160
int rz_maxrange = CFG_DEFAULT_MAXRANGE;        // Configured max range at 20 °C

#define CGF_DEFAULT_BATTERY_CAPACITY 27000
float rz_battery_capacity = CGF_DEFAULT_BATTERY_CAPACITY; //TODO Detect battery capacity from VIN or number of batterycells

char m_vin[18];

uint32_t rz_tpms_id[4];
float rz_obc_volt;

float rz_bat_current;
float rz_bat_voltage;
float rz_bat_power;


int8_t rz_battery_module_temp[12];

INT rz_battery_current;                                 //Battery current               02 21 01 -> 21 7+22 1
INT rz_charge_state_local;

uint32_t rz_battery_cum_charge_current;         //Cumulated charge current    02 21 01 -> 24 6+7 & 25 1+2
uint32_t rz_battery_cum_discharge_current;    //Cumulated discharge current 02 21 01 -> 25 3-6
uint32_t rz_battery_cum_charge_power;                         //Cumulated charge power      02 21 01 -> 25 7 + 26 1-3
uint32_t rz_battery_cum_discharge_power;                 //Cumulated discharge power   02 21 01 -> 26 4-7
uint32_t rz_battery_cum_op_time;                     //Cumulated operating time    02 21 01 -> 27 1-4

uint8_t rz_battery_cell_voltage[100];

uint8_t rz_battery_min_temperature;             //02 21 05 -> 21 7
uint8_t rz_battery_inlet_temperature;         //02 21 05 -> 21 6
uint8_t rz_battery_max_temperature;             //02 21 05 -> 22 1
uint8_t rz_battery_heat_1_temperature;     //02 21 05 -> 23 6
uint8_t rz_battery_heat_2_temperature;     //02 21 05 -> 23 7

uint8_t rz_charge_pilot_current;
float rz_heatsink_temperature; //TODO Remove?

struct {
    unsigned char ChargingChademo : 1;
    unsigned char Charging : 1;
    unsigned char : 1;
    unsigned char : 1;
    unsigned char FanStatus : 4;
} rz_charge_bits;

struct {
    uint8_t byte[8];
    uint8_t status;
    uint16_t id;
}  rz_send_can;

const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
};

#define SQR(n) ((n)*(n))
#define ABS(n) (((n) < 0) ? -(n) : (n))
#define MIN(n,m) ((n) < (m) ? (n) : (m))
#define MAX(n,m) ((n) > (m) ? (n) : (m))
#define LIMIT_MIN(n,lim) ((n) < (lim) ? (lim) : (n))
#define LIMIT_MAX(n,lim) ((n) > (lim) ? (lim) : (n))

// CAN buffer access macros: b=byte# 0..7 / n=nibble# 0..15
#define CAN_BYTE(b)     data[b]
#define CAN_UINT(b)     (((UINT)CAN_BYTE(b) << 8) | CAN_BYTE(b+1))
#define CAN_UINT24(b)   (((uint32_t)CAN_BYTE(b) << 16) | ((UINT)CAN_BYTE(b+1) << 8) | CAN_BYTE(b+2))
#define CAN_UINT32(b)   (((uint32_t)CAN_BYTE(b) << 24) | ((uint32_t)CAN_BYTE(b+1) << 16)  | ((UINT)CAN_BYTE(b+2) << 8) | CAN_BYTE(b+3))
#define CAN_NIBL(b)     (can_databuffer[b] & 0x0f)
#define CAN_NIBH(b)     (can_databuffer[b] >> 4)
#define CAN_NIB(n)      (((n)&1) ? CAN_NIBL((n)>>1) : CAN_NIBH((n)>>1))
#define CAN_12NIBL(b)   (((((UINT)CAN_BYTE((int)b/8) << 8) | (CAN_BYTE(((int)b/8)+1))) >> (b-((int)b/8)*8)) & 4095)

#define TO_CELCIUS(n)    ((float)n-40)
#define TO_PSI(n)        ((float)n/4.0)

#define BAT_SOC            StdMetrics.ms_v_bat_soc->AsFloat(100)
#define BATS_CURR          StdMetrics.ms_v_bat_12v_current->AsFloat(100)
#define BATS_VOLT          StdMetrics.ms_v_bat_12v_voltage->AsFloat(100)
#define BAT_CAC            StdMetrics.ms_v_b_cac->AsFloat(100)
#define BAT_CURR           StdMetrics.ms_v_bat_current->AsFloat(100)
#define BAT_VOLTAGE        StdMetrics.ms_v_bat_voltage->AsFloat(100)
#define BAT_ENERG_REC      StdMetrics.ms_v_b_energy_recd->AsFloat(100)
#define BAT_ENERG_USED     StdMetrics.ms_v_b_energy_used->AsFloat(100)
#define BAT_POWER          StdMetrics.ms_v_bat_power->AsFloat(100)
#define BAT_RANGE          StdMetrics.ms_v_bat_range_est->AsFloat(100)
#define BAT_TEMP           StdMetrics.ms_v_bat_temp->AsFloat(100)
#define ENV_TEMP           StdMetrics.ms_v_env_temp->AsFloat(100)

#define TPMS_FL_T          StdMetrics.ms_v_tpms_fl_t->AsFloat(100);
#define TPMS_FR_T          StdMetrics.ms_v_tpms_fr_t->AsFloat(100);
#define TPMS_RR_T          StdMetrics.ms_v_tpms_rr_t->AsFloat(100);
#define TPMS_RL_T          StdMetrics.ms_v_tpms_rl_t->AsFloat(100);
#define TPMS_FL_P          StdMetrics.ms_v_tpms_fl_p->AsFloat(100);
#define TPMS_FR_P          StdMetrics.ms_v_tpms_fr_p->AsFloat(100);
#define TPMS_RR_P          StdMetrics.ms_v_tpms_rr_p->AsFloat(100);
#define TPMS_RL_P          StdMetrics.ms_v_tpms_rl_p->AsFloat(100);

#define BAT_SOH            StdMetrics.ms_v_bat_soh->AsFloat(100)
#define LIMIT_SOC          StdMetrics.ms_v_charge_limit_soc->AsFloat(0)
#define CHARGE_KWH         StdMetrics.ms_v_charge_kwh->AsFloat(0)
#define POS_ODO            StdMetrics.ms_v_pos_odometer->AsFloat(0, Kilometers)
#define POS_SPEED          StdMetrics.ms_v_pos_speed->AsFloat(0, kph)
#define CHARGE_CURRENT     StdMetrics.ms_v_charge_current->AsFloat(0, Amps)
#define CHARGE_VOLTAGE     StdMetrics.ms_v_charge_voltage->AsFloat(0, Volts)
#define SET_CHARGE_STATE(n)   StdMetrics.ms_v_charge_state->SetValue(n)
#define CHARGE_DURATION_FULL  StdMetrics.ms_v_charge_duration_full->AsInt(100)
#define CHARGE_PILOT_PRESENT  StdMetrics.ms_v_charge_pilot->AsBool()

#define VEHICLE_POLL_TYPE_OBDII_IOCTRL_BY_ID 0x2F // InputOutputControlByIdentifier

#define SMART_JUNCTION_BOX 0x771
#define BODY_CONTROL_MODULE  0x7A0


#endif //#ifndef __VEHICLE_RENAULTZOE_H__
