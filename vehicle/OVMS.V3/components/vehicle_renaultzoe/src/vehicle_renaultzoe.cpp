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

#include "ovms_log.h"
static const char *TAG = "v-renaultzoe";

#include <stdio.h>
#include "pcp.h"
#include "vehicle_renaultzoe.h"
#include "ovms_metrics.h"
#include "metrics_standard.h"

#define VERSION "0.1.0"
// Polling types ("modes") supported: see vehicle.h
//
// polltime[state] = poll period in seconds, i.e. 10 = poll every 10 seconds
//
// state: 0..2; set by vehicle_poll_setstate()
//     0=off, 1=on, 2=charging
//
// Use broadcasts to query all ECUs. The request is sent to ID 0x7df (broadcast
// listener) and the poller accepts any response ID from 0x7e8..0x7ef.
// The moduleid is not used for broadcasts, just needs to be != 0.
//
// To poll a specific ECU, send to its specific ID (one of 0x7e0..0x7e7).
// ECUs will respond using their specific ID plus 8 (= range 0x7e8..0x7ef).

static const OvmsVehicle::poll_pid_t vehicle_renaultzoe_polls[] =
{
{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2002, { 10, 10, 10 } },  // SOC
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2006, {  10, 10,  10 } },  // Odometer
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3203, {  10, 10,  10 } },  // Battery Voltage
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3204, {  10, 10,  10 } },  // Battery Current
//{ 0x75a, 0x77e, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3018, {  10, 10,  10 } },  // DCDC Temp
//{ 0x7e4, 0x77e, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x302b, {  10, 10,  10 } },  // Inverter Temp
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3028, {  10, 10,  10 } },  // 12Battery Current
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2005, {  10, 10,  10 } },  // 12Battery Voltage
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3206, {  10, 10,  10 } },  // Battery SOH
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x33dc, {  10, 10,  10 } },  // Domnestic Engergy
//{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x504A, {  10, 10,  10 } },  // Mains active Power
//{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x5063, {  10, 10,  10 } },  // Charging State
//{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x5062, {  10, 10,  10 } },  // Ground Resistance

// -END-
{ 0, 0, 0x00, 0x00, { 0, 0, 0 } }

};
// Start Insertion from Kia Soul

// Init
OvmsVehicleRenaultZoe::OvmsVehicleRenaultZoe()
  {
  ESP_LOGI(TAG, "Renault Zoe vehicle module");
      StdMetrics.ms_v_bat_soc->SetValue(50.5);
      memset( m_vin, 0, sizeof(m_vin));
      memset( rz_battery_cell_voltage ,0, sizeof(rz_battery_cell_voltage));
      memset( rz_battery_module_temp, 0, sizeof(rz_battery_module_temp));
      memset( rz_tpms_id, 0, sizeof(rz_tpms_id));
      
      rz_obc_volt = 230;
      rz_battery_current = 0;
      
      rz_battery_cum_charge_current = 0;
      rz_battery_cum_discharge_current = 0;
      rz_battery_cum_charge_power = 0;
      rz_battery_cum_discharge_power = 0;
      
      rz_charge_bits.Charging = false;
      rz_charge_bits.FanStatus = 0;
      
      rz_battery_min_temperature = 0;
      rz_battery_max_temperature = 0;
      
      rz_heatsink_temperature = 0;
      rz_battery_fan_feedback = 0;
      
      rz_send_can.id = 0;
      rz_send_can.status = 0;
      memset( rz_send_can.byte, 0, sizeof(rz_send_can.byte));
      
      rz_maxrange = CFG_DEFAULT_MAXRANGE;
      
      // C-Bus
      RegisterCanBus(1, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);
      
      MyConfig.RegisterParam("x.rz", "Renault Zoe", true, true);
      ConfigChanged(NULL);
      
      // init metrics:
      m_version = MyMetrics.InitString("x.rz.m.version", 0, VERSION " " __DATE__ " " __TIME__);
      m_b_cell_volt_max = MyMetrics.InitFloat("x.rz.m.b.cell.volt.max", 10, 0, Volts);
      m_b_cell_volt_min = MyMetrics.InitFloat("x.rz.m.b.cell.volt.min", 10, 0, Volts);
      m_c_power = MyMetrics.InitFloat("x.rz.m.c.power", 10, 0, kW);
      
      
      PollSetPidList(m_can1,vehicle_renaultzoe_polls);
      PollSetState(0);
  }

// End Insertion from Kia Soul

OvmsVehicleRenaultZoe::~OvmsVehicleRenaultZoe()
  {
  ESP_LOGI(TAG, "Shutdown Renault Zoe vehicle module");
  }
// Start Insertion from Kia Soul

/**
 * ConfigChanged: reload single/all configuration variables
 */

void OvmsVehicleRenaultZoe::ConfigChanged(OvmsConfigParam* param)
{
    ESP_LOGD(TAG, "Renault Zoe reload configuration");
    
    // Instances:
    // x.rz
    //      batteryCapacity   Battery capacity in wH (Default: 270000)
    //  suffsoc           Sufficient SOC [%] (Default: 0=disabled)
    //  suffrange         Sufficient range [km] (Default: 0=disabled)
    //  maxrange          Maximum ideal range at 20 °C [km] (Default: 160)
    //
    //  canwrite          Bool: CAN write enabled (Default: no)
    //  autoreset         Bool: SEVCON reset on error (Default: yes)
    //  kickdown          Bool: SEVCON automatic kickdown (Default: yes)
    //  autopower         Bool: SEVCON automatic power level adjustment (Default: yes)
    //  console           Bool: SimpleConsole inputs enabled (Default: no)
    //
    
    rz_battery_capacity = (float)MyConfig.GetParamValueInt("x.rz", "batteryCapacity", CGF_DEFAULT_BATTERY_CAPACITY);
    
}

////////////////////////////////////////////////////////////////////////
// vehicle_renaultzoe_car_on()
// Takes care of setting all the state appropriate when the car is on
// or off. Centralized so we can more easily make on and off mirror
// images.
void OvmsVehicleRenaultZoe::vehicle_renaultzoe_car_on(bool isOn)
{
    
    if (isOn && !StdMetrics.ms_v_env_on->AsBool())
    {
        // Car is ON
        StdMetrics.ms_v_env_on->SetValue(isOn);
        StdMetrics.ms_v_env_awake->SetValue(isOn);
        
        PollSetState(1);
        //TODO net_req_notification(NET_NOTIFY_ENV);
    }
    else if(!isOn && StdMetrics.ms_v_env_on->AsBool())
    {
        // Car is OFF
    }
}

/**
 * Handles incoming CAN-frames on bus 1, the C-bus
 */
void OvmsVehicleRenaultZoe::IncomingFrameCan1(CAN_frame_t* p_frame)
{
    //ESP_LOGI(TAG, "Renault Zoe IncomingFrameCan1");
    uint8_t *d = p_frame->data.u8;
    
    switch (p_frame->MsgID)
    {
        case 0x018:
        {
            // Doors status Byte 0:
            StdMetrics.ms_v_door_chargeport->SetValue((d[0] & 0x01) > 0);
            StdMetrics.ms_v_door_fl->SetValue((d[0] & 0x10) > 0);
            StdMetrics.ms_v_door_fr->SetValue((d[0] & 0x80) > 0);
            StdMetrics.ms_v_door_rl->SetValue(false); //TODO
            StdMetrics.ms_v_door_rr->SetValue(false); //TODO
            // Light status Byte 3 & 5
            // Seat belt status Byte 7
        }
            break;
            
            
        case 0x4b0:
        {
            // Motor RPM based on wheel rotation
            int rpm = (d[0]+(d[1]<<8)) * 8.206;
            StdMetrics.ms_v_mot_rpm->SetValue( rpm );
        }
            break;
            
    }
}


/**
 * Incoming poll reply messages
 */
void OvmsVehicleRenaultZoe::IncomingPollReply(canbus* bus, uint16_t type, uint16_t pid, uint8_t* data, uint8_t length, uint16_t mlremain)
{
//    ESP_LOGI(TAG, "Renault Zoe IncomingPollReply");
//    uint8_t bVal;
//    UINT base;
//    uint32_t lVal;

    
    switch (m_poll_moduleid_low) {
            
        case 0x7ec:
            switch (pid) {
                    
                case 0x2002:
                    StdMetrics.ms_v_bat_soc->SetValue( float(CAN_UINT(0)) * 2 / 100);
                    break;

            }
    }
}

/**
 * Ticker1: Called every second
 */
void OvmsVehicleRenaultZoe::Ticker1(uint32_t ticker)
{
}

/**
 *  Sets the charge metrics
 */
void OvmsVehicleRenaultZoe::SetChargeMetrics(float voltage, float current, float climit, bool chademo)
{
    StdMetrics.ms_v_charge_voltage->SetValue( voltage, Volts );
    StdMetrics.ms_v_charge_current->SetValue( current, Amps );
    StdMetrics.ms_v_charge_climit->SetValue( climit, Amps);
}

// End Insertion from Kia Soul
class OvmsVehicleRenaultZoeInit
  {
  public: OvmsVehicleRenaultZoeInit();
} MyOvmsVehicleRenaultZoeInit  __attribute__ ((init_priority (9000)));

OvmsVehicleRenaultZoeInit::OvmsVehicleRenaultZoeInit()
  {
  ESP_LOGI(TAG, "Registering Vehicle: Renault Zoe (9000)");

  MyVehicleFactory.RegisterVehicle<OvmsVehicleRenaultZoe>("RZ","Renault Zoe");
  }

