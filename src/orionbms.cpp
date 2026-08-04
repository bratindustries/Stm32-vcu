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

#include "orionbms.h"

/*
 * Orion BMS Victron preset messages:
 *
 * 0x351:
 *   Bytes 0-1: Charge voltage limit,       0.1 V
 *   Bytes 2-3: Charge current limit,       0.1 A
 *   Bytes 4-5: Discharge current limit,    0.1 A
 *   Bytes 6-7: Discharge voltage limit,    0.1 V
 *
 * 0x355:
 *   Bytes 0-1: State of charge,            1 %
 *   Bytes 2-3: State of health,            1 %
 *   Bytes 4-5: State of charge,            0.01 %
 *
 * 0x356:
 *   Bytes 0-1: Battery voltage,            0.01 V
 *   Bytes 2-3: Battery current, signed,    0.1 A
 *   Bytes 4-5: Battery temperature, signed,0.1 deg C
 *
 * 0x35A:
 *   Alarm and warning flags
 *
 * 0x373:
 *   Bytes 0-1: Minimum cell voltage,       1 mV
 *   Bytes 2-3: Maximum cell voltage,       1 mV
 *   Bytes 4-5: Minimum cell temperature,   1 Kelvin
 *   Bytes 6-7: Maximum cell temperature,   1 Kelvin
 */

uint16_t OrionBMS::ReadUInt16LE(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

int16_t OrionBMS::ReadInt16LE(const uint8_t *data) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(data[0]) |
      (static_cast<uint16_t>(data[1]) << 8));
}

void OrionBMS::SetCanInterface(CanHardware *c) {
  can = c;

  can->RegisterUserMessage(0x351);
  can->RegisterUserMessage(0x355);
  can->RegisterUserMessage(0x356);
  can->RegisterUserMessage(0x35A);
  can->RegisterUserMessage(0x373);
}

bool OrionBMS::BMSDataValid() {
  /*
   * The two safety-critical messages are:
   *
   * 0x351: charge and discharge limits
   * 0x356: measured pack voltage and current
   *
   * SOC and detailed cell information are useful, but are not required
   * for the basic communication-valid decision.
   */
  return limitsTimeoutCounter > 0 &&
         measurementTimeoutCounter > 0;
}

float OrionBMS::MaxChargeCurrent() {
  if (!BMSDataValid())
    return 0.0f;

  return chargeCurrentLimit;
}

void OrionBMS::DecodeCAN(int id, uint8_t *data) {
  const int timeoutValue =
      Param::GetInt(Param::BMS_Timeout) * 10;

  switch (id) {
  case 0x351: {
    chargeVoltageLimit =
        static_cast<float>(ReadUInt16LE(&data[0])) * 0.1f;

    chargeCurrentLimit =
        static_cast<float>(ReadUInt16LE(&data[2])) * 0.1f;

    dischargeCurrentLimit =
        static_cast<float>(ReadUInt16LE(&data[4])) * 0.1f;

    dischargeVoltageLimit =
        static_cast<float>(ReadUInt16LE(&data[6])) * 0.1f;

    limitsTimeoutCounter = timeoutValue;
    break;
  }

  case 0x355: {
    const uint16_t socWhole = ReadUInt16LE(&data[0]);
    const uint16_t sohWhole = ReadUInt16LE(&data[2]);
    const uint16_t socHundredths = ReadUInt16LE(&data[4]);

    /*
     * Prefer the higher-resolution SOC field when it contains a
     * reasonable value. The field is transmitted in 0.01 percent.
     */
    if (socHundredths <= 10000) {
      stateOfCharge =
          static_cast<float>(socHundredths) * 0.01f;
    } else {
      stateOfCharge = static_cast<float>(socWhole);
    }

    stateOfHealth = static_cast<float>(sohWhole);

    socTimeoutCounter = timeoutValue;
    break;
  }

  case 0x356: {
    batteryVoltage =
        static_cast<float>(ReadUInt16LE(&data[0])) * 0.01f;

    batteryCurrent =
        static_cast<float>(ReadInt16LE(&data[2])) * 0.1f;

    batteryTemperature =
        static_cast<float>(ReadInt16LE(&data[4])) * 0.1f;

    measurementTimeoutCounter = timeoutValue;
    break;
  }

  case 0x35A: {
    /*
     * The Orion Victron preset places alarm and warning information
     * in this message. Keep the raw values available inside this class
     * even though the current ZombieVerter parameter set does not have
     * dedicated parameters for every Victron alarm.
     */
    alarmFlags = ReadUInt16LE(&data[0]);
    warningFlags = ReadUInt16LE(&data[4]);
    break;
  }

  case 0x373: {
    const uint16_t minCellMillivolts =
        ReadUInt16LE(&data[0]);

    const uint16_t maxCellMillivolts =
        ReadUInt16LE(&data[2]);

    const uint16_t minTemperatureKelvin =
        ReadUInt16LE(&data[4]);

    const uint16_t maxTemperatureKelvin =
        ReadUInt16LE(&data[6]);

    minCellVoltage =
        static_cast<float>(minCellMillivolts) * 0.001f;

    maxCellVoltage =
        static_cast<float>(maxCellMillivolts) * 0.001f;

    minCellTemperature =
        static_cast<float>(minTemperatureKelvin) - 273.15f;

    maxCellTemperature =
        static_cast<float>(maxTemperatureKelvin) - 273.15f;

    cellDataTimeoutCounter = timeoutValue;
    break;
  }

  default:
    break;
  }
}

void OrionBMS::Task100Ms() {
  if (limitsTimeoutCounter > 0)
    limitsTimeoutCounter--;

  if (measurementTimeoutCounter > 0)
    measurementTimeoutCounter--;

  if (socTimeoutCounter > 0)
    socTimeoutCounter--;

  if (cellDataTimeoutCounter > 0)
    cellDataTimeoutCounter--;

  /*
   * Publish SOC only while the SOC message remains active.
   */
  if (socTimeoutCounter > 0)
    Param::SetFloat(Param::SOC, stateOfCharge);

  /*
   * Publish detailed cell information only while message 0x373
   * remains active.
   */
  if (cellDataTimeoutCounter > 0) {
    Param::SetFloat(Param::BMS_Vmin, minCellVoltage);
    Param::SetFloat(Param::BMS_Vmax, maxCellVoltage);
    Param::SetFloat(Param::BMS_Tmin, minCellTemperature);
    Param::SetFloat(Param::BMS_Tmax, maxCellTemperature);
  } else {
    Param::SetFloat(Param::BMS_Vmin, 0.0f);
    Param::SetFloat(Param::BMS_Vmax, 0.0f);
    Param::SetFloat(Param::BMS_Tmin, 0.0f);
    Param::SetFloat(Param::BMS_Tmax, 0.0f);
  }

  /*
   * 0x356 contains the general BMS/battery temperature. This is used
   * as the average battery temperature parameter.
   */
  if (measurementTimeoutCounter > 0)
    Param::SetInt(Param::BMS_Tavg,
                  static_cast<int>(batteryTemperature));

  if (BMSDataValid()) {
    /*
     * Calculate the power limits in kW. ZombieVerter's BMS_MaxInput
     * and BMS_MaxOutput parameters are used as power limits.
     */
    maxInputPower = static_cast<uint16_t>(
        (chargeCurrentLimit * batteryVoltage) / 1000.0f);

    maxOutputPower = static_cast<uint16_t>(
        (dischargeCurrentLimit * batteryVoltage) / 1000.0f);

    Param::SetFloat(Param::udc2, batteryVoltage);
    Param::SetFloat(Param::idc, batteryCurrent);

    /*
     * Setting udcsw slightly below measured pack voltage allows the
     * existing precharge comparison to operate in the same way as the
     * existing OI BMS implementation.
     */
    if (batteryVoltage > 30.0f)
      Param::SetFloat(Param::udcsw, batteryVoltage - 30.0f);
    else
      Param::SetFloat(Param::udcsw, 0.0f);

    Param::SetInt(Param::BMS_MaxInput, maxInputPower);
    Param::SetInt(Param::BMS_MaxOutput, maxOutputPower);
    Param::SetInt(
        Param::BMS_ChargeLim,
        static_cast<int>(MaxChargeCurrent()));
  } else {
    /*
     * Fail safely when either the limits message or the pack
     * measurement message times out.
     */
    Param::SetFloat(Param::idc, 0.0f);
    Param::SetFloat(Param::udcsw, 500.0f);

    Param::SetInt(Param::BMS_MaxInput, 0);
    Param::SetInt(Param::BMS_MaxOutput, 0);
    Param::SetInt(Param::BMS_ChargeLim, 0);
  }
}

