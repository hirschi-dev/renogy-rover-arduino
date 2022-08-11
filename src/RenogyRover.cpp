/*
    RenogyRover.cpp - Library for monitoring Renogy Rover 20/40 AMP MPPT controller
    Created by hirschi-dev, November 28, 2020
    Released into the public domain
*/

#include <RenogyRover.h>

RenogyRover::RenogyRover() {
    _client = ModbusMaster();
    _modbusId = 1;
}

RenogyRover::RenogyRover(int modbusId) {
    _client = ModbusMaster();
    RenogyRover::_modbusId = modbusId;
}

ModbusMaster RenogyRover::getModbusClient() {
    return _client;
}

void RenogyRover::begin(Stream& serial) {
    _client.begin(_modbusId, serial);
}

const char* RenogyRover::getLastModbusError() {
    switch(_lastError) {
        case _client.ku8MBIllegalDataAddress:
            return "Illegal data address";
        case _client.ku8MBIllegalDataValue:
            return "Illegal data value";
        case _client.ku8MBIllegalFunction:
            return "Illegal function";
        case _client.ku8MBSlaveDeviceFailure:
            return "Slave device failure";
        case _client.ku8MBSuccess:
            return "Success";
        case _client.ku8MBInvalidSlaveID:
            return "Invalid slave ID: The slave ID in the response does not match that of the request.";
        case _client.ku8MBInvalidFunction:
            return "Invalid function: The function code in the response does not match that of the request.";
        case _client.ku8MBResponseTimedOut:
            return "Response timed out";
        case _client.ku8MBInvalidCRC:
            return "InvalidCRC"; 
        default:
            return "Unknown error";
    }
}

int RenogyRover::getProductModel(char*& productModel) {
    int registerBase = 0x000C;
    int registerLength = 8;

    uint16_t* values = new uint16_t[registerLength];
    productModel = new char[registerLength * 2 + 1];

    if (!_readHoldingRegisters(registerBase, registerLength, values)) {
        return 0;
    } else {
        // convert uint16_t to int8_t, 
        // higher and lower byte need to be switched
        int j = 0;
        for (int i = 0; i < registerLength; i++) {
            productModel[j++] = values[i] >> 8;
            productModel[j++] = values[i];
        }
        // append null terminator and slice first two chars as they are spaces
        productModel[16] = '\0';
        productModel = &productModel[2];
    }
    return 1;
}

int RenogyRover::getPanelState(PanelState* state) {
    state->chargingPower = 0;
    state->current = 0;
    state->voltage  = 0;

    int registerBase = 0x0107;
    int registerLength = 3;

    uint16_t* values = new uint16_t[registerLength];

    if (!_readHoldingRegisters(registerBase, registerLength, values)) {
        return 0;
    } else {
        state->voltage = (int16_t) values[0] * 0.1f;
        state->current = (int16_t) values[1] * 0.01f;
        state->chargingPower = (int16_t) values[2];
    }

    return 1;
}

int RenogyRover::getBatteryState(BatteryState* state) {
    state->batteryTemperature = 0;
    state->chargingCurrent = 0;
    state->controllerTemperature = 0;
    state->stateOfCharge = 0;
    state->batteryVoltage = 0;

    int registerBase = 0x0100;
    int registerLength = 4;

    uint16_t* values = new uint16_t[registerLength];

    if (!_readHoldingRegisters(registerBase, registerLength, values)) {
        return 0;
    } else {
        state->stateOfCharge = (int16_t) values[0];
        state->batteryVoltage = (int16_t) values[1] * 0.1f;
        state->chargingCurrent = (int16_t) values[2] * 0.01f;

        // temperatures are in signed magnitude notation
        state->batteryTemperature = _convertSignedMagnitude(values[3]);
        state->controllerTemperature = _convertSignedMagnitude(values[3] >> 8);
    }

    return 1;
}

int RenogyRover::getDayStatistics(DayStatistics* params) {
    params->batteryVoltageMaxForDay = 0;
    params->batteryVoltageMinForDay = 0;
    params->chargingAmpHoursForDay = 0;
    params->maxChargeCurrentForDay = 0;
    params->maxChargePowerForDay = 0;
    params->dischargingAmpHoursForDay = 0;
    params->maxDischargeCurrentForDay = 0;
    params->maxDischargePowerForDay = 0;
    params->powerGenerationForDay = 0;
    params->powerConsumptionForDay = 0;

    int registerBase = 0x010B;
    int registerLength = 10;

    uint16_t* values = new uint16_t[registerLength];

    if (!_readHoldingRegisters(registerBase, registerLength, values)) {
        return 0;
    } else {
        params->batteryVoltageMinForDay = (int16_t) values[0] * 0.1f;
        params->batteryVoltageMaxForDay = (int16_t) values[1] * 0.1f;
        params->maxChargeCurrentForDay = (int16_t) values[2] * 0.01f;
        params->maxDischargeCurrentForDay = (int16_t) values[3] * 0.01f;
        params->maxChargePowerForDay = (int16_t) values[4];
        params->maxDischargePowerForDay = (int16_t) values[5];
        params->chargingAmpHoursForDay= (int16_t) values[6];
        params->dischargingAmpHoursForDay = (int16_t) values[7];
        params->powerGenerationForDay = (int16_t) values[8];
        params->powerConsumptionForDay = (int16_t) values[9];
    }
    
    return 1;
}

int RenogyRover::getHistoricalStatistics(HistStatistics* stats) {
    stats->batChargingAmpHours = 0;
    stats->batDischargingAmpHours = 0;
    stats->batFullCharges = 0;
    stats->batOverDischarges = 0;
    stats->operatingDays = 0;
    stats->powerConsumed = 0;
    stats->powerGenerated = 0;

    int registerBase =  0x0115;
    int registerLength = 3;

    uint16_t* values = new uint16_t[registerLength];

    if (!_readHoldingRegisters(registerBase, registerLength, values)) {
        return 0;
    } else {
        stats->operatingDays = (int16_t) values[0];
        stats->batOverDischarges = (int16_t) values[1];
        stats->batFullCharges = (int16_t) values[2];
    }

    registerBase = 0x118;
    registerLength = 8;

    free(values);
    values = new uint16_t[registerLength];
    uint32_t* integers = new uint32_t[registerLength / 2];

    if (!_readHoldingRegisters(registerBase, registerLength, values)) {
        return 0;
    } else {
        int j = 0;
        for (int i = 0; i < registerLength / 2; i++) {
            integers[i] = ((int32_t) values[j++]) << 8;
            integers[i] = integers[i] | ((int32_t) values[j++]);
        }

        stats->batChargingAmpHours = integers[0];
        stats->batDischargingAmpHours = integers[1];
        stats->powerGenerated =  integers[2] / 10000.0f;
        stats->powerConsumed = integers[3] / 10000.0f;
    }

    free(values);
    free(integers);
    return 1;
};

int RenogyRover::getChargingState(ChargingState* state) {
    state->chargingMode = ChargingMode::UNDEFINED;
    state->streetLightBrightness = 0;
    state->streetLightState = 0;

    int registerBase = 0x0120;
    int registerLength = 1;

    uint16_t* values  = new uint16_t;

    if (!_readHoldingRegisters(registerBase, registerLength, values)) {
        return 0;
    } else {
        state->streetLightState = (*values >> 15) & 1U;
        state->streetLightBrightness = (*values >> 8) & ~(1U << 7);
        state->chargingMode = ChargingMode((uint8_t) *values);
    }

    free(values);
    return 1;
}

int RenogyRover::getErrors(int& errors) {
    int registerBase = 0x0121;
    int registerLength = 2;
    //numErrors = 15;

    uint16_t* values = new uint16_t[registerLength];

    if (!_readHoldingRegisters(registerBase, registerLength, values)) {
        return 0;
    } 

    //int16_t* errVals = new int16_t[numErrors];

    // 16 lower bits are reserved
    // highest bit is reserved
    errors = (values[0] << 1U) >> 1U;

    return 1;
}

int RenogyRover::setStreetLight(int state) {
    if (state > 1 || state < 0) {
        return 0;
    }

    _lastError = _client.writeSingleRegister(0x010A, (uint16_t) state);
    return _lastError == _client.ku8MBSuccess;
}

int RenogyRover::_readHoldingRegisters(int base, int length, uint16_t*& values) {
    _lastError = _client.readHoldingRegisters(base, length);
    if(_lastError != _client.ku8MBSuccess) {
        return 0;
    } else {
        for(uint8_t i = 0x00; i < (uint16_t) length; i++){
            values[i] = _client.getResponseBuffer(i);
        }
    }
    return 1;
}

int* RenogyRover::_filterZeroes(int16_t arr[], int& size) {
    int* result = new int[size];
    int ctr = 0;

    for (int i= 0; i < size; i++) {
        if (arr[i] != 0) {
            result[ctr++] = arr[i];
        }
    }

    if (ctr < size) {
        realloc(result, ctr * sizeof(int));
        size = ctr;
    }

    return result;
}

int8_t RenogyRover::_convertSignedMagnitude(uint8_t val) {
    if (val & 0x80) {
        return -(val & 0x7F);
    }
    return val;
}