/*
    RenogyRover.h - Library for monitoring Renogy Rover 20/40 AMP MPPT controller
    Created by hirschi-dev, November 28, 2020
    Released into the public domain
*/

#ifndef RenogyRover_h
#define RenogyRover_h

#include <Arduino.h>
#include <ModbusMaster.h>

enum ChargingMode {
    UNDEFINED = -1,
    DEACTIVATED = 0,
    ACTIVATED = 1,
    MPPT = 2,
    EQUALIZING = 3,
    BOOST = 4,
    FLOATING = 5,
    OVERPOWER = 6
};

enum FaultCode {
    BAT_OVER_DISCHARGE = 1,
    BAT_OVER_VOLTAGE = 2,
    BAT_UNDER_VOLTAGE_WARNING = 4,
    LOAD_SHORT = 8,
    LOAD_OVERPOWER = 16,
    CONTROLLER_TEMP_HIGH = 32,
    AMBIENT_TEMP_HIGH = 64,
    PV_OVERPOWER = 128,
    PV_SHORT = 256,
    PV_OVER_VOLTAGE = 512,
    PV_COUNTER_CURRENT = 1024,
    PV_WP_OVER_VOLTAGE = 2048,
    PV_REVERSE_CONNECTED = 4096,
    ANTI_REVERSE_MOS_SHORT = 8192,
    CHARGE_MOS_SHORT = 16384
};

struct PanelState {
    float voltage;
    float current;
    float chargingPower;
};

struct BatteryState {
    int stateOfCharge;
    float batteryVoltage;
    float chargingCurrent;
    float controllerTemperature;
    float batteryTemperature;
};

struct DayStatistics {
    float batteryVoltageMinForDay;
    float batteryVoltageMaxForDay;
    float maxChargeCurrentForDay;
    float maxDischargeCurrentForDay;
    float maxChargePowerForDay;
    float maxDischargePowerForDay;
    float chargingAmpHoursForDay;
    float dischargingAmpHoursForDay;
    float powerGenerationForDay;
    float powerConsumptionForDay;
};

struct HistStatistics {
    int operatingDays;
    int batOverDischarges;
    int batFullCharges;
    int batChargingAmpHours;
    int batDischargingAmpHours;
    float powerGenerated;
    float powerConsumed;
};

struct ChargingState {
    int streetLightState;
    int streetLightBrightness;
    ChargingMode chargingMode;
};

class RenogyRover {
    public:
        RenogyRover();
        RenogyRover(int modbusId);
        ModbusMaster getModbusClient();
        void begin(Stream& serial);
        const char* getLastModbusError();

        int getProductModel(char*& productModel);
        int getPanelState(PanelState* state);
        int getBatteryState(BatteryState* state);
        int getDayStatistics(DayStatistics* dayStats);
        int getHistoricalStatistics(HistStatistics* histStats);
        int getChargingState(ChargingState* chargingState);
        int getErrors(int& errors);

        int setStreetLight(int state);
    private:
        ModbusMaster _client;
        int _modbusId;
        uint8_t _lastError;
        int _readHoldingRegisters(int base, int length, uint16_t*& values);
        int8_t _convertSignedMagnitude(uint8_t val);
};

#endif