/*
 * This file is part of the ZombieVerter project.
 *
 * Orion BMS interface using the Orion "Victron Inverter / MPPT"
 * CANBUS preset @ 500Kbit/s
 *
 * Copyright (C) 2026 Angus Johnson <info@bratindustries.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef ORIONBMS_H
#define ORIONBMS_H

#include "bms.h"
#include "canhardware.h"
#include <stdint.h>

class OrionBMS : public BMS {
public:
  void SetCanInterface(CanHardware *c) override;
  void DecodeCAN(int id, uint8_t *data) override;
  float MaxChargeCurrent() override;
  void Task100Ms() override;

private:
  bool BMSDataValid();

  static uint16_t ReadUInt16LE(const uint8_t *data);
  static int16_t ReadInt16LE(const uint8_t *data);

  int limitsTimeoutCounter = 0;
  int measurementTimeoutCounter = 0;
  int socTimeoutCounter = 0;
  int cellDataTimeoutCounter = 0;

  float chargeVoltageLimit = 0.0f;
  float chargeCurrentLimit = 0.0f;
  float dischargeCurrentLimit = 0.0f;
  float dischargeVoltageLimit = 0.0f;

  float batteryVoltage = 500.0f;
  float batteryCurrent = 0.0f;
  float batteryTemperature = 0.0f;

  float stateOfCharge = 0.0f;
  float stateOfHealth = 0.0f;

  float minCellVoltage = 0.0f;
  float maxCellVoltage = 0.0f;
  float minCellTemperature = 0.0f;
  float maxCellTemperature = 0.0f;

  uint16_t alarmFlags = 0;
  uint16_t warningFlags = 0;

  uint16_t maxInputPower = 0;
  uint16_t maxOutputPower = 0;
};

#endif // ORIONBMS_H

