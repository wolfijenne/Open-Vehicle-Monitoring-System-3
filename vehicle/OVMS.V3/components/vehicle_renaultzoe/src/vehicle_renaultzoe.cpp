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
{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2006, {  10, 10,  10 } },  // Odometer
{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3203, {  10, 10,  10 } },  // Battery Voltage
{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3204, {  10, 10,  10 } },  // Battery Current
{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3028, {  10, 10,  10 } },  // 12Battery Current
// 7ec,24,39,.005,0,0,kwh,22320C,62320C,ff,Available discharge Energy
{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x320C, {  10, 10,  10 } },  // Available discharge Energy
// 7ec,30,31,1,0,0,,223332,623332,ff,Electrical Machine functionning Authorization,0:Not used;1:Inverter Off;2:Inverter On\n" //
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3332, {  10, 10,  10 } },  //  Inverter Off: 1; Inverter On: 2
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2005, {  10, 10,  10 } },  // 12Battery Voltage
//{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3206, {  10, 10,  10 } },  // Battery SOH
{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x33dc, {  10, 10,  10 } },  // Consumed Domnestic Engergy
//{ 0x75a, 0x77e, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3018, {  10, 10,  10 } },  // DCDC Temp
//{ 0x7e4, 0x77e, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x302b, {  10, 10,  10 } },  // Inverter Temp
{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x504A, {  10, 10,  10 } },  // Mains active Power consumed
{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x5063, {  10, 10,  10 } },  // Charging State
// { 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x5062, {  10, 10,  10 } },  // Ground Resistance
{ 0x79b, 0x7bb, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x04, {  10, 10,  10 } },  // Temp Bat Module 1
// -END-
{ 0, 0, 0x00, 0x00, { 0, 0, 0 } }

};

// Init
OvmsVehicleRenaultZoe::OvmsVehicleRenaultZoe()
  {
  ESP_LOGI(TAG, "Renault Zoe vehicle module");
      StdMetrics.ms_v_bat_soc->SetValue(50.5);
      StdMetrics.ms_v_bat_12v_current->SetValue(-99);
      StdMetrics.ms_v_bat_range_est->SetValue(0);
//      StdMetrics.ms_v_bat_cac->SetValue(-99);
      StdMetrics.ms_v_bat_current->SetValue(-99);
      StdMetrics.ms_v_bat_power->SetValue(-99);

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
      m_b_cell_volt_max = MyMetrics.InitFloat("x.rz.m.b.cell.volt.max", 10, 0, Volts);
      m_b_cell_volt_min = MyMetrics.InitFloat("x.rz.m.b.cell.volt.min", 10, 0, Volts);
      m_b_bat_avail_energy = MyMetrics.InitFloat("x.rz.m.b.bat.avail.energy", 10, 0, kWh);
      m_b_bat_max_charge_power = MyMetrics.InitFloat("x.rz.m.b.bat.max.charge.power", 10, 0, kW);
      m_mains_power = MyMetrics.InitFloat("x.rz.m.mains.power",10,0, kW);
      m_b_temp1 = MyMetrics.InitInt("x.rz.m.bat.t1",10,0, Celcius);
      m_b_temp2 = MyMetrics.InitInt("x.rz.m.bat.t2",10,0, Celcius);
      m_b_temp3 = MyMetrics.InitInt("x.rz.m.bat.t3",10,0, Celcius);
      m_b_temp4 = MyMetrics.InitInt("x.rz.m.bat.t4",10,0, Celcius);
      m_b_temp5 = MyMetrics.InitInt("x.rz.m.bat.t5",10,0, Celcius);
      m_b_temp6 = MyMetrics.InitInt("x.rz.m.bat.t6",10,0, Celcius);
      m_b_temp7 = MyMetrics.InitInt("x.rz.m.bat.t7",10,0, Celcius);
      m_b_temp8 = MyMetrics.InitInt("x.rz.m.bat.t8",10,0, Celcius);
      m_b_temp9 = MyMetrics.InitInt("x.rz.m.bat.t9",10,0, Celcius);
      m_b_temp10 = MyMetrics.InitInt("x.rz.m.bat.t10",10,0, Celcius);
      m_b_temp11 = MyMetrics.InitInt("x.rz.m.bat.t11",10,0, Celcius);
      m_b_temp12 = MyMetrics.InitInt("x.rz.m.bat.t12",10,0, Celcius);
      
      
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
        StdMetrics.ms_v_env_on->SetValue( isOn );
        StdMetrics.ms_v_env_awake->SetValue( isOn );
        StdMetrics.ms_v_pos_speed->SetValue( 0 );

    }
}

/**
 * Handles incoming CAN-frames on bus 1, the C-bus
 */
void OvmsVehicleRenaultZoe::IncomingFrameCan1(CAN_frame_t* p_frame)
{
    //ESP_LOGI(TAG, "Renault Zoe IncomingFrameCan1");
//    uint8_t *d = p_frame->data.u8;
    uint8_t *d = p_frame->data.u8;
    #define CAN_7NIBL(b)    (((((UINT)d[(int)b/8] << 8) | (d[((int)b/8)+1])) >> (b-((int)b/8)*8)) & 127)
    #define CAN_8NIBL(b)    (((((UINT)d[(int)b/8] << 8) | (d[((int)b/8)+1])) >> (b-((int)b/8)*8)) & 255)
    #define CAN_9NIBL(b)    (((((UINT)d[(int)b/8] << 8) | (d[((int)b/8)+1])) >> (b-((int)b/8)*8)) & 511)
    switch (p_frame->MsgID)
    {
        case 0x1fd:
        {
            // 1fd,48,55,1,80,0,kW,,,ff Consumption ?
        }
            break;
        case 0x427:
        {
            // 427,40,47,0.3,0,0,kW,,,e2\n"  Available Charging Power
            // 427,49,57,0.1,0,1,kWh,,,e2\n" Available Energy
        }
            break;
        case 0x42a:
        {
            // 42a,30,39,0.1,400,1,°C,,,ff\n" Evaporator Temp Measure
            // 42a,48,50,1,0,0,,,,e2\n" ClimLoopMode
        }
            break;
        case 0x42e:
        {
            // 42e,0,12,0.02,0,2,%,,,e3\n"  State of Charge
            // 42e,20,24,5,0,0,%,,,e2\n"  Engine Fan Speed
            // 42e,38,43,1,0,1,A,,,e3\n"  Charging Pilot Current
            // 42e,44,50,1,40,0,°C,,,e3\n"  HV Battery Temp
//            StdMetrics.ms_v_bat_temp->SetValue((float(CAN_7NIBL(45))-40));
            // 42e,56,63,0.3,0,1,kW,,,ff\n"  Max Charge Power
            m_b_bat_max_charge_power->SetValue((float(d[7])*0.3));
        }
            break;
        case 0x430:
        {
            // 430,24,33,0.5,30,1,°C,,,e2\n"  Comp Temperature Discharge
            // 430,38,39,1,0,0,,,,e2\n"  HV Battery Cooling State
            // 430,40,49,0.1,400,1,°C,,,e2\n"  HV Battery Evaporator Temp
        }
            break;
        case 0x432:
        {
            // 432,36,37,1,0,0,,,,e2\n"  HV Bat Conditionning Mode
        }
            break;
        case 0x534:
        {
            //534,32,40,1,40,0,°C,,,e5\n" // Temp out (Vielleicht dieser statt 656
//            StdMetrics.ms_v_env_temp->SetValue((float(CAN_9NIBL(33))-40));
        }
            break;
            
        case 0x654:
        {
            // Battery Parmeters
            // 654,2,2,1,0,0,,,,ff\n" // Charging Plug Connected
            // 654,25,31,1,0,0,,,,ff\n" // State of Charge
            // 654,32,41,1,0,0,min,,,ff\n" // Time to Full
            StdMetrics.ms_v_charge_duration_full->SetValue((((UINT)d[4] << 8) | d[5]) & 1023);
            // 654,42,51,1,0,0,km,,,ff\n" // Available Distance
        }
            break;
        case 0x656:
        {
            // External Temp
            // 656,48,55,1,40,0,°C,,,e2\n" // External Temp
            StdMetrics.ms_v_env_temp->SetValue((float(d[6])-40));
            
        }
            break;
        case 0x658:
        {
            // SOH
            // 658,33,39,1,0,0,%,,,ff\n" // Battery Health
            StdMetrics.ms_v_bat_soh->SetValue(d[4]);
        }
            break;
        case 0x65b:
        {
            // 65b,41,43,1,0,0,,,,ff\n" // Charging Status Display
            
        }
            break;
        case 0x673:
        {
            // TPMS
            // 673,2,4,1,0,0,,,,ff\n" // Rear right wheel state
            // 673,5,7,1,0,0,,,,ff\n" // Rear left wheel state
            // 673,8,10,1,0,0,,,,ff\n" // Front right wheel state
            // 673,11,13,1,0,0,,,,ff\n" // Front left wheel state
            // 673,16,23,13.725,0,0,mbar,,,ff\n" // Rear right wheel pressure
            StdMetrics.ms_v_tpms_rr_p->SetValue((float(d[2])*13.725));
            // 673,24,31,13.725,0,0,mbar,,,ff\n" // Rear left wheel pressure
            StdMetrics.ms_v_tpms_rl_p->SetValue((float(d[3])*13.725));
            // 673,32,39,13.725,0,0,mbar,,,ff\n" // Front right wheel pressure
            StdMetrics.ms_v_tpms_fr_p->SetValue((float(d[4])*13.725));
            // 673,40,47,13.725,0,0,mbar,,,ff\n" // Front left wheel pressure
            StdMetrics.ms_v_tpms_fl_p->SetValue((float(d[5])*13.725));
            
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
                case 0x2002: {
                    // 7ec,24,39,2.083333333,0,2,%,222002,622002,e5\n" // SOC
                    StdMetrics.ms_v_bat_soc->SetValue( float(CAN_UINT(0)) * 2 / 100);
                }
                    break;
                case 0x2006: {
                    // 7ec,24,47,1,0,0,km,222006,622006,ff\n" // Odometer
                    StdMetrics.ms_v_pos_odometer->SetValue(float(CAN_UINT24(0)));
                }
                    break;
                case 0x3028: {
                    // 7ec,24,31,0.5,0,1,A,223028,623028,ff\n" // 14V current
                    StdMetrics.ms_v_bat_12v_current->SetValue(float(CAN_BYTE(0))*0.1);
                }
                    break;
                case 0x320C: {
                    // 7ec,24,39,.005,0,0,kwh,22320C,62320C,ff,Available discharge Energy
                    m_b_bat_avail_energy->SetValue(float(CAN_UINT(0))*0.005);
                }
                    break;
                case 0x3203: {
                    // 7ec,24,39,0.5,0,2,V,223203,623203,ff\n" // HV Battery voltage
                    rz_bat_voltage = float(CAN_UINT(0)) * 50/100;
                }
                    break;
                case 0x3204: {
                    // 7ec,24,39,0.25,32768,2,A,223204,623204,ff\n" // HV Battery current
                    rz_bat_current = (float(CAN_UINT(0))-32768) * 25/100;
                }
                    break;
                case 0x33dc: {
                    // 7ec,24,47,0.001,1,0,kWh,2233dc,6233dc,ff\n" // Consumed domestic energy
                    StdMetrics.ms_v_charge_kwh->SetValue(CAN_UINT24(0)*0.001);
                }
                    break;
            }
            break;
        case 0x7bb:
            switch (pid) {
                case 0x01:
                {
                    // Todo
                    // 7bb,336,351,0.01,0,2,kW,2101,6101,e2\n" // Maximum battery input power
                }
                    break;
                case 0x03:
                {
                    // Todo
                    // 7bb,56,71,10,0,0,°C,2103,6103,5\n" // Mean battery compartment temp
                    // 7bb,96,111,.01,0,0,V,2103,6103,ff,21_03_#13_Maximum_Cell_voltage\n" //
                    // 7bb,112,127,.01,0,0,V,2103,6103,ff,21_03_#15_Minimum_Cell_voltage\n" //
                    // 7bb,192,207,0.01,0,2,%,2103,6103,e2\n" // Real State of Charge
//                    if (type == VEHICLE_POLL_TYPE_OBDIIGROUP)
//                    {
//                        if (m_poll_ml_frame == 1)
//                        {
//                            m_b_cell_volt_max->SetValue(float( CAN_BYTE(6)*0.01 ),Volts);
//                        }
//                        else if (m_poll_ml_frame == 2)
//                        {
//                            m_b_cell_volt_min->SetValue(float( CAN_BYTE(1)*0.01 ),Volts);
//                        }
//                    }
                }
                    break;
                case 0x04:
                {
                    // Todo
                    // 7bb,32,39,1,40,0,°C,2104,6104,e2\n" // Cell 1 Temperature
                    // 7bb,56,63,1,40,0,°C,2104,6104,e2\n" // Cell 2 Temperature
                    // 7bb,80,87,1,40,0,°C,2104,6104,e2\n" // Cell 3 Temperature
                    // 7bb,104,111,1,40,0,°C,2104,6104,e2\n" // Cell 4 Temperature
                    // 7bb,128,135,1,40,0,°C,2104,6104,e2\n" // Cell 5 Temperature
                    // 7bb,152,159,1,40,0,°C,2104,6104,e2\n" // Cell 6 Temperature
                    // 7bb,176,183,1,40,0,°C,2104,6104,e2\n" // Cell 7 Temperature
                    // 7bb,200,207,1,40,0,°C,2104,6104,e2\n" // Cell 8 Temperature
                    // 7bb,224,231,1,40,0,°C,2104,6104,e2\n" // Cell 9 Temperature
                    // 7bb,248,255,1,40,0,°C,2104,6104,e2\n" // Cell 10 Temperature
                    // 7bb,272,279,1,40,0,°C,2104,6104,e2\n" // Cell 11 Temperature
                    // 7bb,296,303,1,40,0,°C,2104,6104,e2\n" // Cell 12 Temperature
                    if (type == VEHICLE_POLL_TYPE_OBDIIGROUP)
                    {
                        if (m_poll_ml_frame == 0)
                        {
                            rz_battery_module_temp[0]=( (INT)CAN_BYTE(1)-40 );
                        }
                        else if (m_poll_ml_frame == 1)
                        {
                            rz_battery_module_temp[1]=( (INT)CAN_BYTE(1)-40 );
                            rz_battery_module_temp[2]=( (INT)CAN_BYTE(4)-40 );
                        }
                        else if (m_poll_ml_frame == 2)
                        {
                            rz_battery_module_temp[3]=( (INT)CAN_BYTE(0)-40 );
                            rz_battery_module_temp[4]=( (INT)CAN_BYTE(3)-40 );
                            rz_battery_module_temp[5]=( (INT)CAN_BYTE(6)-40 );
                        }
                        else if (m_poll_ml_frame == 3)
                        {
                            rz_battery_module_temp[6]=( (INT)CAN_BYTE(2)-40 );
                            rz_battery_module_temp[7]=( (INT)CAN_BYTE(5)-40 );
                        }
                        else if (m_poll_ml_frame == 4)
                        {
                            rz_battery_module_temp[8]=( (INT)CAN_BYTE(1)-40 );
                            rz_battery_module_temp[9]=( (INT)CAN_BYTE(4)-40 );
                        }
                        else if (m_poll_ml_frame == 5)
                        {
                            rz_battery_module_temp[10]=( (INT)CAN_BYTE(0)-40 );
                            rz_battery_module_temp[11]=( (INT)CAN_BYTE(3)-40 );
                        }
                        int sum=0;
                        for (int i=0; i<12; i++) {
                            sum+=rz_battery_module_temp[i];
                        }
                        StdMetrics.ms_v_bat_temp->SetValue(float(sum)/12); // Calculate Mean Value
                    }
                }
                    break;
            }
            break;
        case 0x77e:
            switch (pid) {
                    //            case 0x3018:
                    //                zoe_dcdc_temp=CAN_UINT(4);
                    //                break;
                case 0x302b: {
                    // 77e,24,31,0.015625,0,2,°C,22302b,62302b,ff\n" // inverter temperature
                    // 77e,24,39,0.015625,0,2,ºC,223018,623018,ff\n" // DCDC converter temperature
                }
                    break;
                    
            }
            break;
        case 0x793:
            switch (pid) {
                case 0x504A: {
                    // 793,24,39,1,20000,0,W,22504A,62504A,ff\n" // Mains active power consumed
                    m_mains_power->SetValue((float(CAN_UINT(0)-20000)/1000),kW);
                }
                    break;
                case 0x5063: {
                    // 793,24,31,1,0,0,,225063,625063,ff\n"
                    // Supervisor state,0:Init;1:Wait;2:ClosingS2;3:InitType;4:InitLkg;5:InitChg;6:Charge;7:ZeroAmpMode;8:EndOfChg;9:OpeningS2;10:ReadyToSleep;11:EmergencyStop;12:InitChargeDF;13:OCPStop;14:WaitS2
                    rz_charge_state_local=CAN_BYTE(0);
                    m_b_temp1->SetValue((INT)rz_charge_state_local);
                    if (rz_charge_state_local==0) {          // Init,Wait,ClosingS2,InitType,InitLkg,InitChg
                        SET_CHARGE_STATE("prepare");
                    } else if (rz_charge_state_local==1) {  // Charge
                        SET_CHARGE_STATE("stopped");
                    } else if (rz_charge_state_local==2) {  // Charge
                        SET_CHARGE_STATE("prepare");
                    } else if (rz_charge_state_local==3) {  // Charge
                        SET_CHARGE_STATE("prepare");
                    } else if (rz_charge_state_local==4) {  // Charge
                        SET_CHARGE_STATE("prepare");
                    } else if (rz_charge_state_local==5) {  // Charge
                        SET_CHARGE_STATE("prepare");
                    } else if (rz_charge_state_local==6) {  // Charge
                        SET_CHARGE_STATE("charging");
                    } else if (rz_charge_state_local==7) {  // ZeroAmpMode
                        SET_CHARGE_STATE("topoff");
                    } else if (rz_charge_state_local==8) {  // EndOfChg
                        SET_CHARGE_STATE("done");
                    } else if (rz_charge_state_local==9) {  // OpeningS2
                        SET_CHARGE_STATE("prepare");
                    } else if (rz_charge_state_local==10) { // ReadyToSleep
                        SET_CHARGE_STATE("stopped");
                    } else if (rz_charge_state_local==11) { //EmergencyStopp
                        SET_CHARGE_STATE("stopped");
                    } else if (rz_charge_state_local==12) { //InitChargeDF
                        SET_CHARGE_STATE("prepare");
                    } else if (rz_charge_state_local==13) { //OCPStop
                        SET_CHARGE_STATE("stopped");
                    } else if (rz_charge_state_local==14) { //WaitS2
                        SET_CHARGE_STATE("prepare");
                    }
                }
                    break;
            }
            break;
    }
}
/**
 * Ticker1: Called every second
 */
void OvmsVehicleRenaultZoe::Ticker1(uint32_t ticker)
{
    StdMetrics.ms_v_bat_voltage->SetValue( rz_bat_voltage );
    StdMetrics.ms_v_bat_current->SetValue( rz_bat_current );
    StdMetrics.ms_v_bat_power->SetValue(rz_bat_voltage * rz_bat_current/1000);
    if (StdMetrics.ms_v_bat_12v_voltage->AsFloat() > 13.5) {
        vehicle_renaultzoe_car_on(true);
    } else {
        vehicle_renaultzoe_car_on(false);
    }
    
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

