#pragma once
// VEBusUps — presents a Multiplus as a UPS (optional, header-only).
//
// Derives UPS state (on line / on battery / low battery / overload), a
// runtime estimate and mains-failure statistics from a VEBus instance plus
// the values your sketch reads with readRAMVars() (converted to real units,
// e.g. with VEBusScaler). Used by
// VEBusNutServer (NUT, port 3493) and VEBusApcupsdServer (apcupsd, port 3551).
//
// Only compiled into your firmware when you #include it.
//
//   VEBusUps ups(vebus);
//   ups.lowSocPct = 20;  ups.nominalW = 2400;  ups.batteryWh = 5000;
//   ...after a RAM read:  ups.set(id, scaler.value(id, raw));
//   loop():               ups.loop();
#include <Arduino.h>
#include "VEBus.h"

// Values in real units (V, A, W, %, Hz).
struct VEBusUpsData {
    float mainsV = 0, mainsA = 0, mainsW = 0;           // RAM 0, 1, 15
    float invV = 0, invA = 0, outW = 0;                 // RAM 2, 3, 16
    float batA = 0, soc = 0;                            // RAM 5, 13 (A, %)
    float mainsHz = 0, invHz = 0;                       // from RAM 8, 7
    bool  socValid = false;                             // set once SoC was read
    int   deviceState = -1;                             // VEBusDeviceState, -1 = unknown
    char  firmware[16] = "";                            // VE.Bus firmware, "" = unknown
};

class VEBusUps {
public:
    explicit VEBusUps(VEBus &bus) : _bus(bus) {}

    VEBusUpsData data;

    // Configuration
    uint8_t  lowSocPct = 20;     // LB at or below this SoC (0 = LED only)
    uint16_t nominalW  = 0;      // for load % (0 = not reported)
    uint16_t batteryWh = 0;      // usable capacity for runtime (0 = not reported)
    bool     fsd       = false;  // forced shutdown requested by a NUT primary

    VEBus &bus() { return _bus; }

    // Store a scaled RAM variable (periods 7/8 must already be in Hz).
    void set(uint8_t id, float v)
    {
        switch (id) {
        case VEBUS_RAM_UMAINS_RMS:      data.mainsV = v; break;
        case VEBUS_RAM_IMAINS_RMS:      data.mainsA = v; break;
        case VEBUS_RAM_UINVERTER_RMS:   data.invV   = v; break;
        case VEBUS_RAM_IINVERTER_RMS:   data.invA   = v; break;
        case VEBUS_RAM_IBAT:            data.batA   = v; break;
        case VEBUS_RAM_CHARGE_STATE:    data.soc    = v; data.socValid = true; break;
        case VEBUS_RAM_MAINS_POWER:     data.mainsW = v; break;
        case VEBUS_RAM_OUTPUT_POWER:    data.outW   = v; break;
        case VEBUS_RAM_MAINS_PERIOD:    data.mainsHz = v; break;
        case VEBUS_RAM_INVERTER_PERIOD: data.invHz   = v; break;
        default: break;
        }
    }

    // No VE.Bus sync → every derived value is meaningless.
    bool stale() const { return _bus.hasNoSync(); }

    bool mainsPresent() const
    {
        return (_leds() & VEBUS_LED_MAINS_ON) || data.mainsV > 150;
    }

    bool lowBattery() const
    {
        if (_leds() & VEBUS_LED_LOW_BATTERY) return true;
        return lowSocPct && data.socValid && data.soc <= lowSocPct;
    }

    bool overload() const { return _leds() & VEBUS_LED_OVERLOAD; }

    bool off() const
    {
        return data.deviceState == VEBUS_STATE_OFF || data.deviceState == VEBUS_STATE_DOWN;
    }

    bool charging() const { return mainsPresent() && data.batA > 0; }

    // SoC clamped to 0..100
    float socPct() const { return constrain(data.soc, 0.0f, 100.0f); }

    // Output power as % of nominalW (-1 = unknown)
    float loadPct() const
    {
        if (!nominalW) return -1;
        return constrain(data.outW * 100.0f / nominalW, 0.0f, 200.0f);
    }

    // Remaining runtime in minutes (-1 = unknown)
    float runtimeMinutes() const
    {
        if (!batteryWh || !data.socValid || stale()) return -1;
        float whLeft = batteryWh * socPct() / 100.0f;
        float load   = max(data.outW, 20.0f);           // idle consumption floor
        return min(whLeft / load * 60.0f, 9999.0f);
    }

    float mainsHz()  const { return data.mainsHz; }
    float outputHz() const { return data.invHz; }

    // NUT-style status, e.g. "OL CHRG", "OB DISCHRG LB"; "UNKNOWN" when stale.
    void nutStatus(char *out, size_t n) const
    {
        if (!n) return;
        out[0] = '\0';
        if (stale()) { strlcpy(out, "UNKNOWN", n); return; }
        bool mains = mainsPresent();
        strlcat(out, mains ? "OL" : "OB", n);
        if (charging())   strlcat(out, " CHRG", n);
        if (!mains)       strlcat(out, " DISCHRG", n);
        if (lowBattery()) strlcat(out, " LB", n);
        if (overload())   strlcat(out, " OVER", n);
        if (off())        strlcat(out, " OFF", n);
        if (fsd)          strlcat(out, " FSD", n);
    }

    // Mains-failure statistics
    uint32_t transfers() const        { return _xfers; }
    uint32_t secondsOnBattery() const { return _onBatt ? (millis() - _onBattSince) / 1000 : 0; }
    uint32_t totalSecondsOnBattery() const { return _cumOnBatt + secondsOnBattery(); }

    // Call from loop(): tracks mains failures and clears FSD once power is back.
    void loop()
    {
        if (stale()) return;
        bool onBatt = !mainsPresent();
        if (onBatt && !_onBatt) {
            _onBattSince = millis();
            _xfers++;
        } else if (!onBatt && _onBatt) {
            _cumOnBatt += (millis() - _onBattSince) / 1000;
        }
        _onBatt = onBatt;
        if (fsd && !onBatt && !lowBattery()) fsd = false;
    }

private:
    VEBus   &_bus;
    bool     _onBatt = false;
    uint32_t _onBattSince = 0, _cumOnBatt = 0, _xfers = 0;

    byte _leds() const { return _bus.getLEDon() | _bus.getLEDblink(); }
};
