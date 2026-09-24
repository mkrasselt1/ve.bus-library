#pragma once
// VEBusScaler — converts raw RAM variable values into real units
// (optional, header-only; only compiled into your firmware when included).
//
// Uses built-in defaults measured on a MultiPlus. Optionally (queryDevice =
// true) it asks the Multiplus for the scale/offset of each RAM variable via
// requestRAMVarInfo — off by default: it adds extra bus traffic and the
// replies have not been verified on real hardware yet.
//
//   VEBusScaler scaler(vebus);
//   loop():  scaler.loop();
//            float volts = scaler.value(VEBUS_RAM_UMAINS_RMS, raw);
//
// Call scaler.loop() BEFORE handling hasRAMVarResponse() so info replies are
// consumed here.
#include <Arduino.h>
#include "VEBus.h"

class VEBusScaler {
public:
    static const uint8_t MAX_ID = 20;

    explicit VEBusScaler(VEBus &bus) : _bus(bus)
    {
        // Defaults (MultiPlus, observed): V ×0.01, AC A ×0.01, battery A ×0.1,
        // SoC ×0.5 %, power 1 W. Periods have no safe default.
        _info[VEBUS_RAM_UMAINS_RMS]         = VEBusVarInfo(0.01f, false);
        _info[VEBUS_RAM_IMAINS_RMS]         = VEBusVarInfo(0.01f, true);
        _info[VEBUS_RAM_UINVERTER_RMS]      = VEBusVarInfo(0.01f, false);
        _info[VEBUS_RAM_IINVERTER_RMS]      = VEBusVarInfo(0.01f, true);
        _info[VEBUS_RAM_UBAT]               = VEBusVarInfo(0.01f, false);
        _info[VEBUS_RAM_IBAT]               = VEBusVarInfo(0.1f,  true);
        _info[VEBUS_RAM_UBAT_RMS]           = VEBusVarInfo(0.01f, false);
        _info[VEBUS_RAM_INVERTER_PERIOD]    = VEBusVarInfo(0.0f,  false);
        _info[VEBUS_RAM_MAINS_PERIOD]       = VEBusVarInfo(0.0f,  false);
        _info[VEBUS_RAM_SIGNED_AC_LOAD_CUR] = VEBusVarInfo(0.01f, true);
        _info[VEBUS_RAM_CHARGE_STATE]       = VEBusVarInfo(0.5f,  false);
        for (uint8_t id = VEBUS_RAM_INVERTER_POWER; id <= VEBUS_RAM_OUTPUT_POWER_UF; id++)
            _info[id] = VEBusVarInfo(1.0f, true);
    }

    // Scaled value of a RAM variable (raw as returned by getRAMVarValue()).
    float value(uint8_t id, int16_t raw) const
    {
        return id < MAX_ID ? _info[id].apply(raw) : (float)raw;
    }

    // Periods (RAM 7/8) → Hz; 0 while the scale is unknown.
    float frequency(uint8_t id, int16_t raw) const
    {
        float p = value(id, raw);
        return p > 0 ? 10.0f / p : 0.0f;
    }

    bool queryDevice = false;   // ask the Multiplus for scale/offset (see above)

    bool    fromDevice(uint8_t id) const { return id < MAX_ID && _queried[id]; }
    bool    complete() const             { return _idx >= QUERY_COUNT; }
    const VEBusVarInfo &info(uint8_t id) const { return _info[id < MAX_ID ? id : 0]; }

    void loop()
    {
        if (!queryDevice || complete() || _bus.hasNoSync()) return;
        uint8_t id = _queryId(_idx);

        if (_bus.hasRAMVarInfoResponse()) {
            if (_bus.getRAMVarInfoId() == id) {
                _info[id] = VEBusVarInfo::fromDevice(_bus.getRAMVarInfoScale(),
                                                     _bus.getRAMVarInfoOffset());
                _queried[id] = true;
                _next();
            }
            _bus.clearRAMVarInfoResponse();
            return;
        }
        if (_sentMs && millis() - _sentMs < 1500) return;      // waiting for reply
        if (_tries >= 3) { _next(); return; }                  // keep the default
        _bus.requestRAMVarInfo(id);
        _sentMs = millis();
        _tries++;
    }

private:
    static const uint8_t QUERY_COUNT = 11;
    static uint8_t _queryId(uint8_t i)
    {
        static const uint8_t ids[QUERY_COUNT] = {
            VEBUS_RAM_UMAINS_RMS, VEBUS_RAM_IMAINS_RMS, VEBUS_RAM_UINVERTER_RMS,
            VEBUS_RAM_IINVERTER_RMS, VEBUS_RAM_IBAT, VEBUS_RAM_INVERTER_PERIOD,
            VEBUS_RAM_MAINS_PERIOD, VEBUS_RAM_CHARGE_STATE, VEBUS_RAM_INVERTER_POWER,
            VEBUS_RAM_MAINS_POWER, VEBUS_RAM_OUTPUT_POWER
        };
        return ids[i];
    }

    VEBus       &_bus;
    VEBusVarInfo _info[MAX_ID];
    bool         _queried[MAX_ID] = {false};
    uint8_t      _idx = 0, _tries = 0;
    uint32_t     _sentMs = 0;

    void _next() { _idx++; _tries = 0; _sentMs = 0; }
};

