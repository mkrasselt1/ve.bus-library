#include "VEBus.h"

#define TX_DELAY_MS 8  // ms after sync before transmitting (matches original)

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
VEBus::VEBus()
    : _taskHandle(nullptr), _cmdQueue(nullptr),
      _frp(0), _frlen(0), _frameNr(0),
      _lastSentType(VEBUS_CMD_READ_RAM),
      _syncrxed(false), _nosync(false), _gotMP2data(false), _acked(false),
      _synctime(0), _chksmfault(0),
      _multiplusTemp(0.0f), _multiplusDcCurrent(0.0f),
      _multiplusStatus80(0), _multiplusDcLevelAllowsInverting(false),
      _masterMultiLED_LEDon(0), _masterMultiLED_LEDblink(0),
      _masterMultiLED_Status(0), _masterMultiLED_AcInputConfiguration(0),
      _masterMultiLED_MinimumInputCurrentLimit(0.0f),
      _masterMultiLED_MaximumInputCurrentLimit(0.0f),
      _masterMultiLED_ActualInputCurrentLimit(0.0f),
      _masterMultiLED_SwitchRegister(0),
      _BatVolt(0.0f), _ACPower(0),
      _gotRAMVars(false), _ramVarCount(0),
      _gotSetting(false), _settingId(0), _settingValue(0),
      _settingWriteAcked(false),
      _gotDeviceState(false), _deviceState(0), _deviceSubState(0),
      _gotVersion(false), _versionLow(0), _versionHigh(0),
      _gotSettingInfo(false), _settingInfo{},
      _gotRAMVarInfo(false), _ramVarInfoId(0), _ramVarInfoScale(0), _ramVarInfoOffset(0)
{
    memset((void *)_ramVarIds, 0, sizeof(_ramVarIds));
    memset((void *)_ramVarValues, 0, sizeof(_ramVarValues));
}

// -----------------------------------------------------------------------
// begin() — init UART and launch the internal tight-loop task
// -----------------------------------------------------------------------
void VEBus::begin(int rxPin, int txPin, int dePin, int core)
{
    _rxPin = rxPin;
    _txPin = txPin;
    _dePin = dePin;

    _cmdQueue = xQueueCreate(8, sizeof(VEBusCmd));

    xTaskCreatePinnedToCore(
        _taskEntry, "VEBus", 4096, this, 2, &_taskHandle, core);
}

// -----------------------------------------------------------------------
// Thread-safe command API
// -----------------------------------------------------------------------

void VEBus::setESSPower(int16_t watts)
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_ESS_POWER;
    cmd.power = watts;
    xQueueReset(_cmdQueue);
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::requestReadRAM()
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_READ_RAM;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::requestSleep()
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_SLEEP;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::requestWakeup()
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_WAKEUP;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::setSwitchState(VEBusSwitchState state)
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_SET_SWITCH;
    cmd.state = (uint8_t)state;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::readRAMVars(const uint8_t *ids, uint8_t count)
{
    if (count > VEBUS_MAX_RAM_IDS) count = VEBUS_MAX_RAM_IDS;
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_READ_RAM_VARS;
    cmd.ram.count = count;
    memcpy(cmd.ram.ids, ids, count);
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::writeRAMVar(uint8_t id, uint16_t value)
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_WRITE_RAM_VAR;
    cmd.setting.id = id;
    cmd.setting.value = value;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::readSetting(uint8_t id)
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_READ_SETTING;
    cmd.setting.id = id;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::writeSetting(uint8_t id, uint16_t value)
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_WRITE_SETTING;
    cmd.setting.id = id;
    cmd.setting.value = value;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::requestDeviceState()
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_GET_DEVICE_STATE;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::forceDeviceState(VEBusForceState action)
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_SET_DEVICE_STATE;
    cmd.state = (uint8_t)action;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::requestVersion()
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_GET_VERSION_0;
    _gotVersion = false;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::requestSettingInfo(uint8_t id)
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_GET_SETTING_INFO;
    cmd.setting.id = id;
    _gotSettingInfo = false;
    _settingInfo.id = id;
    _settingInfo.receivedFields = 0;
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::requestRAMVarInfo(uint8_t id)
{
    VEBusCmd cmd;
    cmd.type = VEBUS_CMD_GET_RAMVAR_INFO;
    cmd.setting.id = id;
    _gotRAMVarInfo = false;
    _ramVarInfoId = id;
    xQueueSend(_cmdQueue, &cmd, 0);
}

int16_t VEBus::getRAMVarValue(uint8_t index) const
{
    if (index >= _ramVarCount) return 0;
    return _ramVarValues[index];
}

// -----------------------------------------------------------------------
// Task entry point + main tight loop
// -----------------------------------------------------------------------
void VEBus::_taskEntry(void *param)
{
    static_cast<VEBus *>(param)->_run();
}

void VEBus::_run()
{
    Serial1.begin(256000, SERIAL_8N1, _rxPin, _txPin);
    Serial1.setPins(-1, -1, -1, _dePin);
    Serial1.setMode(UART_MODE_RS485_HALF_DUPLEX);

    while (true)
    {
        _processRx();

        if (_syncrxed)
        {
            _nosync = false;
            VEBusCmd cmd;
            if (xQueuePeek(_cmdQueue, &cmd, 0) == pdTRUE)
            {
                if (millis() > _synctime + TX_DELAY_MS)
                {
                    _syncrxed = false;
                    xQueueReceive(_cmdQueue, &cmd, 0);
                    _sendCommand(cmd);
                }
            }
            else
            {
                _syncrxed = false;
            }
        }

        if (millis() > _synctime + 1000)
        {
            _nosync   = true;
            _synctime = millis();
        }

        if (!Serial1.available())
            vTaskDelay(1);
    }
}

// -----------------------------------------------------------------------
// _processRx — exact translation of multiplusCommandHandling()
// -----------------------------------------------------------------------
void VEBus::_processRx()
{
    while (Serial1.available())
    {
        char c = Serial1.read();
        _frbuf1[_frp++] = c;

        if (c == 0x55) {
            if (_frp == 5) _synctime = millis();
        }

        if ((uint8_t)c == 0xFF)
        {
            if ((_frbuf1[2] == (char)0xFD) && (_frbuf1[4] == (char)0x55))
            {
                _frameNr  = (byte)_frbuf1[3];
                _syncrxed = true;
            }
            else if ((_frbuf1[0] == (char)0x83) &&
                     (_frbuf1[1] == (char)0x83) &&
                     (_frbuf1[2] == (char)0xFE))
            {
                if (_verifyChecksum(_frbuf1, _frp))
                {
                    _frlen = _destuffFAtoFF(_frbuf2, _frbuf1, _frp);
                    _decodeFrame(_frbuf2, _frlen);
                }
                else
                {
                    _chksmfault++;
                }
            }
            _frp = 0;
        }
        else
        {
            _syncrxed = false;
        }
    }
}

// -----------------------------------------------------------------------
// _sendCommand — dispatch a queued command
// -----------------------------------------------------------------------
void VEBus::_sendCommand(const VEBusCmd &cmd)
{
    int len;
    byte fn = (_frameNr + 1) & 0x7F;

    _lastSentType = cmd.type;

    switch (cmd.type)
    {
    case VEBUS_CMD_ESS_POWER:
        len = _prepareESSCommand(_txbuf1, cmd.power, fn);
        break;
    case VEBUS_CMD_READ_RAM:
        len = _prepareReadRAM(_txbuf1, fn);
        break;
    case VEBUS_CMD_SLEEP:
        len = _prepareOnOff(_txbuf1, fn, false);
        break;
    case VEBUS_CMD_WAKEUP:
        len = _prepareOnOff(_txbuf1, fn, true);
        break;
    case VEBUS_CMD_SET_SWITCH:
        len = _prepareSwitchState(_txbuf1, cmd.state, fn);
        break;
    case VEBUS_CMD_READ_RAM_VARS:
        // Store which IDs we requested so the response handler can match them
        _ramVarCount = cmd.ram.count;
        memcpy((void *)_ramVarIds, cmd.ram.ids, cmd.ram.count);
        len = _prepareReadRAMVars(_txbuf1, cmd.ram.ids, cmd.ram.count, fn);
        break;
    case VEBUS_CMD_READ_SETTING:
        _settingId = cmd.setting.id;
        len = _prepareReadSetting(_txbuf1, cmd.setting.id, fn);
        break;
    case VEBUS_CMD_WRITE_SETTING:
        // flags: bit0=1 (setting), bit1=0 (write to RAM+EEPROM)
        len = _prepareWriteViaID(_txbuf1, 0x01, cmd.setting.id, cmd.setting.value, fn);
        break;
    case VEBUS_CMD_WRITE_RAM_VAR:
        // flags: bit0=0 (RAM var), bit1=1 (RAM only, no EEPROM)
        len = _prepareWriteViaID(_txbuf1, 0x02, cmd.setting.id, cmd.setting.value, fn);
        break;
    case VEBUS_CMD_GET_DEVICE_STATE:
        len = _prepareGetSetState(_txbuf1, VEBUS_FORCE_INQUIRY, fn);
        break;
    case VEBUS_CMD_SET_DEVICE_STATE:
        len = _prepareGetSetState(_txbuf1, cmd.state, fn);
        break;
    case VEBUS_CMD_GET_VERSION_0:
        len = _prepareGetVersion(_txbuf1, 0, fn);
        break;
    case VEBUS_CMD_GET_VERSION_1:
        len = _prepareGetVersion(_txbuf1, 1, fn);
        break;
    case VEBUS_CMD_GET_SETTING_INFO:
        len = _prepareGetSettingInfo(_txbuf1, cmd.setting.id, fn);
        break;
    case VEBUS_CMD_GET_RAMVAR_INFO:
        len = _prepareGetRAMVarInfo(_txbuf1, cmd.setting.id, fn);
        break;
    default:
        return;
    }

    len = _replaceFAtoFF(_txbuf2, _txbuf1, len);
    len = _appendChecksum(_txbuf2, len);
    Serial1.write((const uint8_t *)_txbuf2, len);
}

// =======================================================================
// Frame building helpers
// =======================================================================

// --- Winmon frame header (shared by all 0xE6 commands) ---
static inline byte _winmonHeader(char *out, byte fn)
{
    byte j = 0;
    out[j++] = 0x98;
    out[j++] = 0xF7;
    out[j++] = 0xFE;
    out[j++] = fn;
    out[j++] = 0x00;
    out[j++] = 0xE6;
    return j;
}

int VEBus::_prepareESSCommand(char *out, int16_t power, byte fn)
{
    byte j = _winmonHeader(out, fn);
    out[j++] = VEBUS_WCMD_WRITE_VIA_ID;
    out[j++] = 0x02;               // Flags: RAMvar, no EEPROM
    out[j++] = 0x83;               // RAM address of ESS power
    out[j++] = (power & 0xFF);
    out[j++] = (power >> 8);
    return j;
}

int VEBus::_prepareReadRAM(char *out, byte fn)
{
    byte j = _winmonHeader(out, fn);
    out[j++] = VEBUS_WCMD_READ_RAM;
    out[j++] = VEBUS_RAM_UBAT;             // RAM id 4 = battery voltage
    out[j++] = VEBUS_RAM_INVERTER_POWER;   // RAM id 14 = AC power
    return j;
}

int VEBus::_prepareOnOff(char *out, byte fn, bool wakeup)
{
    byte j = 0;
    out[j++] = 0x98;
    out[j++] = 0xF7;
    out[j++] = 0xFE;
    out[j++] = fn;
    out[j++] = 0x3F;
    out[j++] = wakeup ? VEBUS_SWITCH_STATE_ON : VEBUS_SWITCH_STATE_OFF;
    out[j++] = 0x00;
    out[j++] = 0x00;
    out[j++] = 0x00;
    return j;
}

int VEBus::_prepareSwitchState(char *out, uint8_t state, byte fn)
{
    byte j = 0;
    out[j++] = 0x98;
    out[j++] = 0xF7;
    out[j++] = 0xFE;
    out[j++] = fn;
    out[j++] = 0x3F;
    out[j++] = state;
    out[j++] = 0x00;
    out[j++] = 0x00;
    out[j++] = 0x00;
    return j;
}

int VEBus::_prepareReadRAMVars(char *out, const uint8_t *ids, uint8_t count, byte fn)
{
    byte j = _winmonHeader(out, fn);
    out[j++] = VEBUS_WCMD_READ_RAM;
    for (uint8_t i = 0; i < count; i++)
        out[j++] = ids[i];
    return j;
}

int VEBus::_prepareReadSetting(char *out, uint8_t id, byte fn)
{
    byte j = _winmonHeader(out, fn);
    out[j++] = VEBUS_WCMD_READ_SETTING;
    out[j++] = id;
    return j;
}

int VEBus::_prepareWriteViaID(char *out, uint8_t flags, uint8_t id, uint16_t value, byte fn)
{
    byte j = _winmonHeader(out, fn);
    out[j++] = VEBUS_WCMD_WRITE_VIA_ID;
    out[j++] = flags;
    out[j++] = id;
    out[j++] = (value & 0xFF);
    out[j++] = (value >> 8);
    return j;
}

int VEBus::_prepareGetSetState(char *out, uint8_t action, byte fn)
{
    byte j = _winmonHeader(out, fn);
    out[j++] = VEBUS_WCMD_GET_SET_STATE;
    out[j++] = action;
    return j;
}

int VEBus::_prepareGetVersion(char *out, uint8_t part, byte fn)
{
    byte j = _winmonHeader(out, fn);
    out[j++] = (part == 0) ? VEBUS_WCMD_GET_VERSION_0 : VEBUS_WCMD_GET_VERSION_1;
    return j;
}

int VEBus::_prepareGetSettingInfo(char *out, uint8_t id, byte fn)
{
    byte j = _winmonHeader(out, fn);
    out[j++] = VEBUS_WCMD_GET_SETTING_INFO;
    out[j++] = id;
    return j;
}

int VEBus::_prepareGetRAMVarInfo(char *out, uint8_t id, byte fn)
{
    byte j = _winmonHeader(out, fn);
    out[j++] = VEBUS_WCMD_GET_RAMVAR_INFO;
    out[j++] = id;
    return j;
}

// =======================================================================
// Byte-stuffing (unchanged from original)
// =======================================================================

int VEBus::_replaceFAtoFF(char *out, const char *in, int len)
{
    int j = 0;
    for (int i = 0; i < 4; i++) out[j++] = in[i];
    for (int i = 4; i < len; i++)
    {
        byte c = (byte)in[i];
        if (c >= 0xFA) {
            out[j++] = 0xFA;
            out[j++] = 0x70 | (c & 0x0F);
        } else {
            out[j++] = in[i];
        }
    }
    return j;
}

int VEBus::_destuffFAtoFF(char *out, const char *in, int len)
{
    int j = 0;
    for (int i = 0; i < 4; i++) out[j++] = in[i];
    for (int i = 4; i < len; i++)
    {
        byte c = (byte)in[i];
        if (c == 0xFA) {
            c = (byte)in[++i];
            if (c == 0xFF) {
                out[j++] = 0xFA;
                out[j++] = (char)c;
            } else {
                out[j++] = (char)(c + 0x80);
            }
        } else {
            out[j++] = in[i];
        }
    }
    return j;
}

bool VEBus::_verifyChecksum(const char *buf, int len)
{
    byte cs = 0;
    for (int i = 2; i < len; i++) cs += (byte)buf[i];
    return (cs == 0);
}

int VEBus::_appendChecksum(char *buf, int len)
{
    byte cs = 1;
    for (int i = 2; i < len; i++) cs -= (byte)buf[i];
    int j = len;
    if (cs >= 0xFB) {
        buf[j++] = (char)0xFA;
        buf[j++] = (char)(cs - 0xFA);
    } else {
        buf[j++] = (char)cs;
    }
    buf[j++] = (char)0xFF;
    return j;
}

// =======================================================================
// Frame decoder
// =======================================================================

void VEBus::_decodeFrame(const char *frame, int len)
{
    switch ((byte)frame[4])
    {
    // -------------------------------------------------------------------
    // Frame 0x80: Charger/Inverter condition (broadcast)
    // -------------------------------------------------------------------
    case 0x80:
        if ((frame[5] == (char)0x80) &&
            ((frame[6] & 0xFE) == 0x12) &&
            (frame[8] == (char)0x80) &&
            ((frame[11] & 0x10) == 0x10))
        {
            _multiplusStatus80 = (byte)frame[7];
            _multiplusDcLevelAllowsInverting = ((byte)frame[6] & 0x01) != 0;
            int16_t t = 256 * (uint8_t)frame[10] + (uint8_t)frame[9];
            _multiplusDcCurrent = 0.1f * (float)t;
            if (((byte)frame[11] & 0xF0) == 0x30)
                _multiplusTemp = 0.1f * (float)(uint8_t)frame[15];
        }
        break;

    // -------------------------------------------------------------------
    // Frame 0x41: Master LED / Multiplus mode (broadcast)
    // -------------------------------------------------------------------
    case 0x41:
        if ((len == 19) && (frame[5] == (char)0x10))
        {
            _masterMultiLED_LEDon    = (byte)frame[6];
            _masterMultiLED_LEDblink = (byte)frame[7];
            _masterMultiLED_Status   = (byte)frame[8];
            _masterMultiLED_AcInputConfiguration = (byte)frame[9];
            int16_t t;
            t = 256 * (uint8_t)frame[11] + (uint8_t)frame[10];
            _masterMultiLED_MinimumInputCurrentLimit = (float)t / 10.0f;
            t = 256 * (uint8_t)frame[13] + (uint8_t)frame[12];
            _masterMultiLED_MaximumInputCurrentLimit = (float)t / 10.0f;
            t = 256 * (uint8_t)frame[15] + (uint8_t)frame[14];
            _masterMultiLED_ActualInputCurrentLimit  = (float)t / 10.0f;
            _masterMultiLED_SwitchRegister = (byte)frame[16];
        }
        break;

    // -------------------------------------------------------------------
    // Frame 0x00: Winmon command responses
    // -------------------------------------------------------------------
    case 0x00:
        if (frame[5] == (char)0xE6)
        {
            byte respCode = (byte)frame[6];

            switch (respCode)
            {
            // --- RAM read OK (0x85) ---
            case VEBUS_WRESP_RAM_READ_OK:
                if (_lastSentType == VEBUS_CMD_READ_RAM)
                {
                    // Legacy: hardcoded battery voltage + AC power
                    _gotMP2data = true;
                    int16_t v = 256 * (uint8_t)frame[8] + (uint8_t)frame[7];
                    _BatVolt = 0.01f * (float)v;
                    _ACPower = 256 * (uint8_t)frame[10] + (uint8_t)frame[9];
                }
                else if (_lastSentType == VEBUS_CMD_READ_RAM_VARS)
                {
                    // Flexible: store raw 16-bit values by index
                    for (uint8_t i = 0; i < _ramVarCount && (7 + i * 2 + 1) < len; i++)
                    {
                        _ramVarValues[i] = (int16_t)(
                            256 * (uint8_t)frame[8 + i * 2] +
                                  (uint8_t)frame[7 + i * 2]);
                    }
                    _gotRAMVars = true;
                }
                break;

            // --- RAM write OK (0x87) ---
            case VEBUS_WRESP_RAM_WRITE_OK:
                _acked = true;
                break;

            // --- Setting read OK (0x86) ---
            case VEBUS_WRESP_SETTING_READ_OK:
                _settingValue = (uint16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                _gotSetting = true;
                break;

            // --- Setting write OK (0x88) ---
            case VEBUS_WRESP_SETTING_WRITE_OK:
                _settingWriteAcked = true;
                break;

            // --- Device state (0x94) ---
            case VEBUS_WRESP_DEVICE_STATE:
                _deviceState    = (byte)frame[7];
                _deviceSubState = (byte)frame[8];
                _gotDeviceState = true;
                break;

            // --- Firmware version part 0 (0x82) ---
            case VEBUS_WRESP_VERSION_0:
                _versionLow = (uint16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                // Auto-chain: request part 1
                {
                    VEBusCmd autoCmd;
                    autoCmd.type = VEBUS_CMD_GET_VERSION_1;
                    xQueueSend(_cmdQueue, &autoCmd, 0);
                }
                break;

            // --- Firmware version part 1 (0x83) ---
            case VEBUS_WRESP_VERSION_1:
                _versionHigh = (uint16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                _gotVersion = true;
                break;

            // --- Setting info responses (0x89-0x8D) ---
            case VEBUS_WRESP_SETTING_SCALE:
                _settingInfo.scale = (int16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                _settingInfo.receivedFields |= 0x01;
                if (_settingInfo.receivedFields == 0x1F) _gotSettingInfo = true;
                break;
            case VEBUS_WRESP_SETTING_OFFSET:
                _settingInfo.offset = (int16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                _settingInfo.receivedFields |= 0x02;
                if (_settingInfo.receivedFields == 0x1F) _gotSettingInfo = true;
                break;
            case VEBUS_WRESP_SETTING_DEFAULT:
                _settingInfo.defaultValue = (uint16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                _settingInfo.receivedFields |= 0x04;
                if (_settingInfo.receivedFields == 0x1F) _gotSettingInfo = true;
                break;
            case VEBUS_WRESP_SETTING_MIN:
                _settingInfo.minimum = (uint16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                _settingInfo.receivedFields |= 0x08;
                if (_settingInfo.receivedFields == 0x1F) _gotSettingInfo = true;
                break;
            case VEBUS_WRESP_SETTING_MAX:
                _settingInfo.maximum = (uint16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                _settingInfo.receivedFields |= 0x10;
                if (_settingInfo.receivedFields == 0x1F) _gotSettingInfo = true;
                break;

            // --- RAM var info responses (0x8E-0x8F) ---
            case VEBUS_WRESP_RAMVAR_SCALE:
                _ramVarInfoScale = (int16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                break;
            case VEBUS_WRESP_RAMVAR_OFFSET:
                _ramVarInfoOffset = (int16_t)(256 * (uint8_t)frame[8] + (uint8_t)frame[7]);
                _gotRAMVarInfo = true;
                break;

            // --- Error responses ---
            case VEBUS_WRESP_NOT_SUPPORTED:
            case VEBUS_WRESP_VAR_NOT_SUPPORTED:
            case VEBUS_WRESP_SETTING_NOT_SUPPORTED:
            case VEBUS_WRESP_ACCESS_LEVEL:
                // Could add error tracking here in the future
                break;

            default:
                break;
            }
        }
        break;

    default:
        break;
    }
}
