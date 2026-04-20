#pragma once

#include <Arduino.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ===================================================================
// LED bitmask constants (returned by getLEDon() / getLEDblink())
// ===================================================================
#define VEBUS_LED_MAINS_ON    (1 << 0)
#define VEBUS_LED_ABSORPTION  (1 << 1)
#define VEBUS_LED_BULK        (1 << 2)
#define VEBUS_LED_FLOAT       (1 << 3)
#define VEBUS_LED_INVERTER_ON (1 << 4)
#define VEBUS_LED_OVERLOAD    (1 << 5)
#define VEBUS_LED_LOW_BATTERY (1 << 6)
#define VEBUS_LED_TEMPERATURE (1 << 7)

// ===================================================================
// Switch register bitmask constants (returned by getSwitchRegister())
// ===================================================================
#define VEBUS_SWITCH_REMOTE_CHARGE      (1 << 0)  // Remote interface charger state
#define VEBUS_SWITCH_REMOTE_INVERT      (1 << 1)  // Remote interface inverter state
#define VEBUS_SWITCH_FRONT_UP           (1 << 2)  // Front panel switch UP
#define VEBUS_SWITCH_FRONT_DOWN         (1 << 3)  // Front panel switch DOWN
#define VEBUS_SWITCH_CHARGE             (1 << 4)  // Active: charger enabled
#define VEBUS_SWITCH_INVERT             (1 << 5)  // Active: inverter enabled
#define VEBUS_SWITCH_ONBOARD_REMOTE_INV (1 << 6)  // Onboard remote inverter switch
#define VEBUS_SWITCH_REMOTE_GENERATOR   (1 << 7)  // Remote generator selected

// ===================================================================
// RAM variable IDs (for readRAMVars / CommandReadRAMVar 0x30)
//
// Use CommandGetRAMVarInfo (0x36) to query per-device scale/offset.
// Typical scaling: voltage ×0.01, current ×0.1, power in watts.
// ===================================================================
#define VEBUS_RAM_UMAINS_RMS           0   // Mains voltage RMS (V)
#define VEBUS_RAM_IMAINS_RMS           1   // Mains current RMS (A)
#define VEBUS_RAM_UINVERTER_RMS        2   // Inverter output voltage RMS (V)
#define VEBUS_RAM_IINVERTER_RMS        3   // Inverter output current RMS (A)
#define VEBUS_RAM_UBAT                 4   // Battery voltage (V)
#define VEBUS_RAM_IBAT                 5   // Battery current (A, signed)
#define VEBUS_RAM_UBAT_RMS             6   // Battery ripple voltage RMS
#define VEBUS_RAM_INVERTER_PERIOD      7   // Inverter period (Hz = 10/value)
#define VEBUS_RAM_MAINS_PERIOD         8   // Mains period (Hz = 10/value)
#define VEBUS_RAM_SIGNED_AC_LOAD_CUR   9   // Signed AC load current
#define VEBUS_RAM_VIRTUAL_SWITCH       10  // Virtual switch position (bit 3)
#define VEBUS_RAM_IGNORE_AC_INPUT      11  // Ignore AC input state (bit 0)
#define VEBUS_RAM_RELAY_STATE          12  // Multi-functional relay (bit 5)
#define VEBUS_RAM_CHARGE_STATE         13  // State of charge (~0.5% resolution)
#define VEBUS_RAM_INVERTER_POWER       14  // Inverter power, filtered (W, signed)
#define VEBUS_RAM_MAINS_POWER          15  // Mains power, filtered (W, signed)
#define VEBUS_RAM_OUTPUT_POWER         16  // Output power, filtered (W, signed)
#define VEBUS_RAM_INVERTER_POWER_UF    17  // Inverter power, unfiltered (W)
#define VEBUS_RAM_MAINS_POWER_UF       18  // Mains power, unfiltered (W)
#define VEBUS_RAM_OUTPUT_POWER_UF      19  // Output power, unfiltered (W)

// ===================================================================
// Setting IDs (for readSetting / writeSetting)
//
// Use requestSettingInfo() to query scale/offset/default/min/max.
// Voltage settings are typically scaled ×100 (e.g. 5680 = 56.80 V).
// ===================================================================
#define VEBUS_SETTING_FLAGS0                    0   // Primary flags (16-bit bitmask)
#define VEBUS_SETTING_FLAGS1                    1   // Secondary flags
#define VEBUS_SETTING_UBAT_ABSORPTION           2   // Absorption voltage (raw/100 = V)
#define VEBUS_SETTING_UBAT_FLOAT                3   // Float voltage (raw/100 = V)
#define VEBUS_SETTING_IBAT_BULK                 4   // Bulk charge current (A)
#define VEBUS_SETTING_UINV_SETPOINT             5   // Inverter output voltage setpoint
#define VEBUS_SETTING_IMAINS_LIMIT              6   // AC input 1 current limit
#define VEBUS_SETTING_REPEATED_ABSORPTION_TIME  7
#define VEBUS_SETTING_REPEATED_ABSORPTION_INTERVAL 8
#define VEBUS_SETTING_MAX_ABSORPTION_DURATION   9   // 1 = LiFePO4 fixed
#define VEBUS_SETTING_CHARGE_CHARACTERISTIC     10  // 0=variable, 1=fixed, 2=fixed+storage
#define VEBUS_SETTING_UBAT_LOW_LIMIT            11  // Low battery cutoff (raw/100 = V)
#define VEBUS_SETTING_UBAT_LOW_HYSTERESIS       12
#define VEBUS_SETTING_NUM_SLAVES                13
#define VEBUS_SETTING_THREE_PHASE               14  // 0=3ph, 1=split 180, 2=2-leg 120
#define VEBUS_SETTING_LOWEST_UMAINS             44  // Minimum acceptable mains voltage
#define VEBUS_SETTING_LOWEST_UMAINS_HYST        45
#define VEBUS_SETTING_HIGHEST_UMAINS            46  // Maximum acceptable mains voltage
#define VEBUS_SETTING_HIGHEST_UMAINS_HYST       47
#define VEBUS_SETTING_ASSIST_CURRENT_BOOST      48
#define VEBUS_SETTING_IMAINS_LIMIT_AC2          49  // AC input 2 current limit
#define VEBUS_SETTING_FLAGS2                    60
#define VEBUS_SETTING_FLAGS3                    61
#define VEBUS_SETTING_BATTERY_CAPACITY          64
#define VEBUS_SETTING_GRID_CODE                 81  // 0=none, 1=active

// ===================================================================
// Device states
// ===================================================================
enum VEBusDeviceState : uint8_t {
    VEBUS_STATE_DOWN         = 0,
    VEBUS_STATE_STARTUP      = 1,
    VEBUS_STATE_OFF          = 2,
    VEBUS_STATE_SLAVE        = 3,
    VEBUS_STATE_INVERT_FULL  = 4,
    VEBUS_STATE_INVERT_HALF  = 5,
    VEBUS_STATE_INVERT_AES   = 6,
    VEBUS_STATE_POWER_ASSIST = 7,
    VEBUS_STATE_BYPASS       = 8,
    VEBUS_STATE_CHARGE       = 9,
};

// Charge sub-states (when device state == VEBUS_STATE_CHARGE)
enum VEBusChargeSubState : uint8_t {
    VEBUS_CHARGE_INIT                = 0,
    VEBUS_CHARGE_BULK                = 1,
    VEBUS_CHARGE_ABSORPTION          = 2,
    VEBUS_CHARGE_FLOAT               = 3,
    VEBUS_CHARGE_STORAGE             = 4,
    VEBUS_CHARGE_REPEATED_ABSORPTION = 5,
    VEBUS_CHARGE_FORCED_ABSORPTION   = 6,
    VEBUS_CHARGE_EQUALISE            = 7,
    VEBUS_CHARGE_BULK_STOPPED        = 8,
};

// Force-state actions for forceDeviceState()
enum VEBusForceState : uint8_t {
    VEBUS_FORCE_INQUIRY    = 0,  // Read state only (same as requestDeviceState)
    VEBUS_FORCE_EQUALISE   = 1,
    VEBUS_FORCE_ABSORPTION = 2,
    VEBUS_FORCE_FLOAT      = 3,
};

// ===================================================================
// Switch states for setSwitchState()
// ===================================================================
enum VEBusSwitchState : uint8_t {
    VEBUS_SWITCH_STATE_OFF           = 0x04,  // Both off (= sleep)
    VEBUS_SWITCH_STATE_CHARGER_ONLY  = 0x05,
    VEBUS_SWITCH_STATE_INVERTER_ONLY = 0x06,
    VEBUS_SWITCH_STATE_ON            = 0x07,  // Both on (= wakeup)
};

// ===================================================================
// Winmon command opcodes (internal, for reference)
// ===================================================================
#define VEBUS_WCMD_GET_VERSION_0    0x05  // Software version part 0
#define VEBUS_WCMD_GET_VERSION_1    0x06  // Software version part 1
#define VEBUS_WCMD_GET_SET_STATE    0x0E  // Get/set device state
#define VEBUS_WCMD_READ_RAM         0x30  // Read RAM variables
#define VEBUS_WCMD_READ_SETTING     0x31  // Read a setting
#define VEBUS_WCMD_WRITE_RAM        0x32  // Write RAM (needs 0x34)
#define VEBUS_WCMD_WRITE_SETTING    0x33  // Write setting (needs 0x34)
#define VEBUS_WCMD_WRITE_DATA       0x34  // Data payload for 0x32/0x33
#define VEBUS_WCMD_GET_SETTING_INFO 0x35  // Get setting scale/offset/default/min/max
#define VEBUS_WCMD_GET_RAMVAR_INFO  0x36  // Get RAM var scale/offset
#define VEBUS_WCMD_WRITE_VIA_ID     0x37  // Direct write (preferred)
#define VEBUS_WCMD_READ_SNAPSHOT    0x38  // Read snapshot

// Winmon response codes
#define VEBUS_WRESP_NOT_SUPPORTED   0x80
#define VEBUS_WRESP_VERSION_0       0x82
#define VEBUS_WRESP_VERSION_1       0x83
#define VEBUS_WRESP_RAM_READ_OK     0x85
#define VEBUS_WRESP_SETTING_READ_OK 0x86
#define VEBUS_WRESP_RAM_WRITE_OK    0x87
#define VEBUS_WRESP_SETTING_WRITE_OK 0x88
#define VEBUS_WRESP_SETTING_SCALE   0x89
#define VEBUS_WRESP_SETTING_OFFSET  0x8A
#define VEBUS_WRESP_SETTING_DEFAULT 0x8B
#define VEBUS_WRESP_SETTING_MIN     0x8C
#define VEBUS_WRESP_SETTING_MAX     0x8D
#define VEBUS_WRESP_RAMVAR_SCALE    0x8E
#define VEBUS_WRESP_RAMVAR_OFFSET   0x8F
#define VEBUS_WRESP_VAR_NOT_SUPPORTED  0x90
#define VEBUS_WRESP_SETTING_NOT_SUPPORTED 0x91
#define VEBUS_WRESP_DEVICE_STATE    0x94
#define VEBUS_WRESP_ACCESS_LEVEL    0x9B

// ===================================================================
// Internal command types for the queue
// ===================================================================
enum VEBusCmdType : uint8_t {
    VEBUS_CMD_ESS_POWER       = 1,   // Write ESS power setpoint
    VEBUS_CMD_READ_RAM        = 2,   // Legacy: read UBat + AC power
    VEBUS_CMD_SLEEP           = 3,   // Put Multiplus to sleep
    VEBUS_CMD_WAKEUP          = 4,   // Wake Multiplus from sleep
    VEBUS_CMD_READ_RAM_VARS   = 5,   // Read user-specified RAM variables
    VEBUS_CMD_READ_SETTING    = 6,   // Read a setting
    VEBUS_CMD_WRITE_SETTING   = 7,   // Write a setting (RAM + EEPROM)
    VEBUS_CMD_WRITE_RAM_VAR   = 8,   // Write a RAM variable
    VEBUS_CMD_GET_DEVICE_STATE = 9,  // Query device state
    VEBUS_CMD_SET_DEVICE_STATE = 10, // Force device state
    VEBUS_CMD_GET_VERSION_0   = 11,  // Read firmware version part 0
    VEBUS_CMD_GET_VERSION_1   = 12,  // Read firmware version part 1
    VEBUS_CMD_SET_SWITCH      = 13,  // Set switch state (on/off/charger/inverter)
    VEBUS_CMD_GET_SETTING_INFO = 14, // Get setting info (scale/offset/default/min/max)
    VEBUS_CMD_GET_RAMVAR_INFO  = 15, // Get RAM var info (scale/offset)
};

// Maximum number of RAM variables per read request
#define VEBUS_MAX_RAM_IDS 6

struct VEBusCmd {
    VEBusCmdType type;
    union {
        int16_t  power;                              // ESS_POWER
        struct { uint8_t ids[VEBUS_MAX_RAM_IDS]; uint8_t count; } ram;  // READ_RAM_VARS
        struct { uint8_t id; uint16_t value; } setting;  // READ/WRITE_SETTING, WRITE_RAM_VAR, GET_*_INFO
        uint8_t  state;                              // SET_DEVICE_STATE, SET_SWITCH
    };
};

// ===================================================================
// Setting info response fields (returned by requestSettingInfo)
// ===================================================================
struct VEBusSettingInfo {
    uint8_t  id;
    int16_t  scale;
    int16_t  offset;
    uint16_t defaultValue;
    uint16_t minimum;
    uint16_t maximum;
    uint8_t  receivedFields;  // bitmask: bit0=scale, bit1=offset, bit2=default, bit3=min, bit4=max
};

// ===================================================================
// VEBus
//
// Runs its own FreeRTOS task in a tight loop (no delay) to handle
// RS485 RX/TX with correct sync timing — matching the original
// architecture from PepeTheFroggie's firmware.
//
// Usage:
//   VEBus vebus;
//   void setup() { vebus.begin(RX, TX, DE); }
//   void loop() {
//     vebus.setESSPower(watts);
//     vebus.requestReadRAM();
//     Serial.printf("V=%.2f\n", vebus.getBatVolt());
//   }
// ===================================================================
class VEBus {
public:
    VEBus();

    // Start the RS485 driver and launch the internal task.
    void begin(int rxPin, int txPin, int dePin, int core = 0);

    // ================================================================
    // Thread-safe commands — queued for the next sync slot.
    // Can be called from any core/task at any time.
    // ================================================================

    // --- ESS power control ---
    void setESSPower(int16_t watts);

    // --- Legacy RAM read (battery voltage + AC power) ---
    void requestReadRAM();

    // --- Sleep / wake (kept for backward compatibility) ---
    void requestSleep();
    void requestWakeup();

    // --- Switch state (superset of sleep/wake) ---
    void setSwitchState(VEBusSwitchState state);

    // --- Flexible RAM variable reading (up to 6 per request) ---
    void readRAMVars(const uint8_t *ids, uint8_t count);

    // --- RAM variable writing ---
    void writeRAMVar(uint8_t id, uint16_t value);

    // --- Setting read/write ---
    void readSetting(uint8_t id);
    void writeSetting(uint8_t id, uint16_t value);

    // --- Device state ---
    void requestDeviceState();
    void forceDeviceState(VEBusForceState action);

    // --- Firmware version (auto-chains part 0 → part 1) ---
    void requestVersion();

    // --- Setting/RAM variable info (scale, offset, default, min, max) ---
    void requestSettingInfo(uint8_t id);
    void requestRAMVarInfo(uint8_t id);

    // ================================================================
    // Decoded data — safe to read from any core (atomic on Xtensa)
    // ================================================================

    // --- From legacy requestReadRAM() ---
    float   getBatVolt()     const { return _BatVolt; }
    int16_t getACPower()     const { return _ACPower; }

    // --- From frame 0x80 (charger/inverter condition, broadcast) ---
    float   getDCCurrent()   const { return _multiplusDcCurrent; }
    float   getTemp()        const { return _multiplusTemp; }
    byte    getChargerStatus()  const { return _multiplusStatus80; }
    bool    dcLevelAllowsInverting() const { return _multiplusDcLevelAllowsInverting; }

    // --- From frame 0x41 (MasterMultiLED, broadcast) ---
    byte    getLEDon()       const { return _masterMultiLED_LEDon; }
    byte    getLEDblink()    const { return _masterMultiLED_LEDblink; }
    byte    getLEDstatus()   const { return _masterMultiLED_Status; }
    byte    getAcInputConfiguration() const { return _masterMultiLED_AcInputConfiguration; }
    float   getMinInputCurrentLimit() const { return _masterMultiLED_MinimumInputCurrentLimit; }
    float   getMaxInputCurrentLimit() const { return _masterMultiLED_MaximumInputCurrentLimit; }
    float   getActInputCurrentLimit() const { return _masterMultiLED_ActualInputCurrentLimit; }
    byte    getSwitchRegister() const { return _masterMultiLED_SwitchRegister; }

    // --- From readRAMVars() response ---
    bool    hasRAMVarResponse()  const { return _gotRAMVars; }
    void    clearRAMVarResponse()      { _gotRAMVars = false; }
    uint8_t getRAMVarCount()     const { return _ramVarCount; }
    int16_t getRAMVarValue(uint8_t index) const;  // by response index (0..count-1)

    // --- From readSetting() response ---
    bool     hasSettingResponse()  const { return _gotSetting; }
    void     clearSettingResponse()      { _gotSetting = false; }
    uint8_t  getSettingId()        const { return _settingId; }
    uint16_t getSettingValue()     const { return _settingValue; }

    // --- From writeSetting() / writeRAMVar() ACK ---
    bool    isSettingWriteAcked() const { return _settingWriteAcked; }
    void    clearSettingWriteAcked()    { _settingWriteAcked = false; }

    // --- From requestDeviceState() / forceDeviceState() ---
    bool    hasDeviceStateResponse() const { return _gotDeviceState; }
    void    clearDeviceStateResponse()     { _gotDeviceState = false; }
    uint8_t getDeviceState()    const { return _deviceState; }
    uint8_t getDeviceSubState() const { return _deviceSubState; }

    // --- From requestVersion() ---
    bool     hasVersionResponse() const { return _gotVersion; }
    void     clearVersionResponse()     { _gotVersion = false; }
    uint16_t getVersionLow()  const { return _versionLow; }
    uint16_t getVersionHigh() const { return _versionHigh; }

    // --- From requestSettingInfo() ---
    bool    hasSettingInfoResponse() const { return _gotSettingInfo; }
    void    clearSettingInfoResponse()     { _gotSettingInfo = false; }
    const VEBusSettingInfo& getSettingInfo() const { return _settingInfo; }

    // --- From requestRAMVarInfo() ---
    bool    hasRAMVarInfoResponse() const { return _gotRAMVarInfo; }
    void    clearRAMVarInfoResponse()     { _gotRAMVarInfo = false; }
    uint8_t getRAMVarInfoId()     const { return _ramVarInfoId; }
    int16_t getRAMVarInfoScale()  const { return _ramVarInfoScale; }
    int16_t getRAMVarInfoOffset() const { return _ramVarInfoOffset; }

    // ================================================================
    // Status
    // ================================================================
    bool     hasNewData()  const { return _gotMP2data; }
    void     clearNewData()      { _gotMP2data = false; }
    bool     isAcked()     const { return _acked; }
    void     clearAcked()        { _acked = false; }
    bool     hasNoSync()   const { return _nosync; }
    uint32_t getChecksumFaults() const { return _chksmfault; }

private:
    static void _taskEntry(void *param);
    void        _run();
    void        _processRx();
    void        _sendCommand(const VEBusCmd &cmd);

    // Frame preparation helpers
    int  _prepareESSCommand  (char *out, int16_t power, byte frameNr);
    int  _prepareReadRAM     (char *out, byte frameNr);
    int  _prepareOnOff       (char *out, byte frameNr, bool wakeup);
    int  _prepareReadRAMVars (char *out, const uint8_t *ids, uint8_t count, byte frameNr);
    int  _prepareReadSetting (char *out, uint8_t id, byte frameNr);
    int  _prepareWriteViaID  (char *out, uint8_t flags, uint8_t id, uint16_t value, byte frameNr);
    int  _prepareGetSetState (char *out, uint8_t action, byte frameNr);
    int  _prepareGetVersion  (char *out, uint8_t part, byte frameNr);
    int  _prepareSwitchState (char *out, uint8_t state, byte frameNr);
    int  _prepareGetSettingInfo(char *out, uint8_t id, byte frameNr);
    int  _prepareGetRAMVarInfo(char *out, uint8_t id, byte frameNr);

    // Byte stuffing & checksum
    int  _replaceFAtoFF    (char *out, const char *in, int len);
    int  _destuffFAtoFF    (char *out, const char *in, int len);
    int  _appendChecksum   (char *buf, int len);
    bool _verifyChecksum   (const char *buf, int len);
    void _decodeFrame      (const char *frame, int len);

    // Pin config
    int _rxPin, _txPin, _dePin;

    // FreeRTOS
    TaskHandle_t  _taskHandle;
    QueueHandle_t _cmdQueue;

    // Frame assembly buffers
    char _frbuf1[128];
    char _frbuf2[128];
    char _txbuf1[32];
    char _txbuf2[32];

    byte _frp;
    byte _frlen;
    byte _frameNr;

    // Protocol state
    volatile VEBusCmdType  _lastSentType;
    volatile bool          _syncrxed;
    volatile bool          _nosync;
    volatile bool          _gotMP2data;
    volatile bool          _acked;
    volatile unsigned long _synctime;
    volatile uint32_t      _chksmfault;

    // Decoded device data — frame 0x80 (broadcast)
    volatile float   _multiplusTemp;
    volatile float   _multiplusDcCurrent;
    volatile byte    _multiplusStatus80;
    volatile bool    _multiplusDcLevelAllowsInverting;

    // Decoded device data — frame 0x41 (broadcast)
    volatile byte    _masterMultiLED_LEDon;
    volatile byte    _masterMultiLED_LEDblink;
    volatile byte    _masterMultiLED_Status;
    volatile byte    _masterMultiLED_AcInputConfiguration;
    volatile float   _masterMultiLED_MinimumInputCurrentLimit;
    volatile float   _masterMultiLED_MaximumInputCurrentLimit;
    volatile float   _masterMultiLED_ActualInputCurrentLimit;
    volatile byte    _masterMultiLED_SwitchRegister;

    // Legacy RAM read response (requestReadRAM)
    volatile float   _BatVolt;
    volatile int16_t _ACPower;

    // Flexible RAM read response (readRAMVars)
    volatile bool    _gotRAMVars;
    volatile uint8_t _ramVarCount;
    volatile uint8_t _ramVarIds[VEBUS_MAX_RAM_IDS];
    volatile int16_t _ramVarValues[VEBUS_MAX_RAM_IDS];

    // Setting read response
    volatile bool     _gotSetting;
    volatile uint8_t  _settingId;
    volatile uint16_t _settingValue;

    // Setting/RAM write ACK
    volatile bool _settingWriteAcked;

    // Device state response
    volatile bool    _gotDeviceState;
    volatile uint8_t _deviceState;
    volatile uint8_t _deviceSubState;

    // Firmware version
    volatile bool     _gotVersion;
    volatile uint16_t _versionLow;
    volatile uint16_t _versionHigh;

    // Setting info response
    volatile bool _gotSettingInfo;
    VEBusSettingInfo _settingInfo;

    // RAM var info response
    volatile bool    _gotRAMVarInfo;
    volatile uint8_t _ramVarInfoId;
    volatile int16_t _ramVarInfoScale;
    volatile int16_t _ramVarInfoOffset;
};
