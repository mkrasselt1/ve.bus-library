#include "VEBus.h"

#define TX_DELAY_MS 8  // ms after sync before transmitting (matches original)

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
VEBus::VEBus()
    : _taskHandle(nullptr), _cmdQueue(nullptr),
      _frp(0), _frlen(0), _frameNr(0),
      _syncrxed(false), _nosync(false), _gotMP2data(false), _acked(false),
      _synctime(0), _chksmfault(0),
      _masterMultiLED_LEDon(0), _masterMultiLED_LEDblink(0),
      _masterMultiLED_Status(0), _masterMultiLED_AcInputConfiguration(0),
      _masterMultiLED_MinimumInputCurrentLimit(0.0f),
      _masterMultiLED_MaximumInputCurrentLimit(0.0f),
      _masterMultiLED_ActualInputCurrentLimit(0.0f),
      _masterMultiLED_SwitchRegister(0),
      _multiplusTemp(0.0f), _multiplusDcCurrent(0.0f),
      _multiplusStatus80(0), _multiplusDcLevelAllowsInverting(false),
      _BatVolt(0.0f), _ACPower(0)
{
}

// -----------------------------------------------------------------------
// begin() — init UART and launch the internal tight-loop task
// -----------------------------------------------------------------------
void VEBus::begin(int rxPin, int txPin, int dePin, int core)
{
    _rxPin = rxPin;
    _txPin = txPin;
    _dePin = dePin;

    // Command queue: holds up to 4 pending commands
    _cmdQueue = xQueueCreate(4, sizeof(VEBusCmd));

    // Launch the RS485 handler task — UART init happens INSIDE the task
    // so the interrupt handler is registered on the same core that reads.
    xTaskCreatePinnedToCore(
        _taskEntry, "VEBus", 4096, this, 2, &_taskHandle, core);
}

// -----------------------------------------------------------------------
// Thread-safe command API — queue commands from any core
// -----------------------------------------------------------------------
void VEBus::setESSPower(int16_t watts)
{
    VEBusCmd cmd = { VEBUS_CMD_ESS_POWER, watts };
    xQueueReset(_cmdQueue);           // discard stale commands
    xQueueSend(_cmdQueue, &cmd, 0);   // queue latest setpoint
}

void VEBus::requestReadRAM()
{
    VEBusCmd cmd = { VEBUS_CMD_READ_RAM, 0 };
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::requestSleep()
{
    VEBusCmd cmd = { VEBUS_CMD_SLEEP, 0 };
    xQueueSend(_cmdQueue, &cmd, 0);
}

void VEBus::requestWakeup()
{
    VEBusCmd cmd = { VEBUS_CMD_WAKEUP, 0 };
    xQueueSend(_cmdQueue, &cmd, 0);
}

// -----------------------------------------------------------------------
// Task entry point + main tight loop
// Matches the original loop() exactly: process RX, check sync, send.
// NO delay — runs as fast as possible to catch sync timing.
// -----------------------------------------------------------------------
void VEBus::_taskEntry(void *param)
{
    static_cast<VEBus *>(param)->_run();
}

void VEBus::_run()
{
    // Init UART on THIS core so the RX interrupt fires here too
    Serial1.begin(256000, SERIAL_8N1, _rxPin, _txPin);
    Serial1.setPins(-1, -1, -1, _dePin);
    Serial1.setMode(UART_MODE_RS485_HALF_DUPLEX);

    while (true)
    {
        // --- 1. Process all available RX bytes ---
        _processRx();

        // --- 2. If sync received, try to send a queued command ---
        // This is a direct translation of the original loop() logic:
        //   if (syncrxed) {
        //     if (gotmsg && millis() > synctime + txdelay) { sendmsg(1); }
        //     else if (sendnow && millis() > synctime + txdelay) { sendmsg(2); }
        //     else syncrxed = false;
        //   }
        if (_syncrxed)
        {
            _nosync = false;
            VEBusCmd cmd;
            if (xQueuePeek(_cmdQueue, &cmd, 0) == pdTRUE)
            {
                // Wait for TX slot
                if (millis() > _synctime + TX_DELAY_MS)
                {
                    _syncrxed = false;
                    xQueueReceive(_cmdQueue, &cmd, 0);
                    _sendCommand(cmd);
                }
            }
            else
            {
                _syncrxed = false;  // nothing to send, release sync
            }
        }

        // --- 3. No-sync watchdog ---
        if (millis() > _synctime + 1000)
        {
            _nosync   = true;
            _synctime = millis();
        }

        // Yield when no bytes are waiting — feeds the IDLE task watchdog.
        // When bytes ARE available, loop stays tight (no delay).
        // During the TX slot gap (~8 ms after sync, no bytes), this yields
        // for 1 ms per iteration — same effective timing as the original.
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
    default:
        return;
    }

    len = _replaceFAtoFF(_txbuf2, _txbuf1, len);
    len = _appendChecksum(_txbuf2, len);
    Serial1.write((const uint8_t *)_txbuf2, len);
}

// =======================================================================
// Frame building helpers (unchanged from original)
// =======================================================================

int VEBus::_prepareESSCommand(char *out, int16_t power, byte fn)
{
    byte j = 0;
    out[j++] = 0x98;
    out[j++] = 0xf7;
    out[j++] = 0xfe;
    out[j++] = fn;
    out[j++] = 0x00;
    out[j++] = 0xe6;
    out[j++] = 0x37;  // CommandWriteViaID
    out[j++] = 0x02;  // Flags: RAMvar, no EEPROM
    out[j++] = 0x83;  // RAM address of ESS power
    out[j++] = (power & 0xFF);
    out[j++] = (power >> 8);
    return j;
}

int VEBus::_prepareReadRAM(char *out, byte fn)
{
    byte j = 0;
    out[j++] = 0x98;
    out[j++] = 0xf7;
    out[j++] = 0xfe;
    out[j++] = fn;
    out[j++] = 0x00;
    out[j++] = 0xe6;
    out[j++] = 0x30;  // Command: read RAM
    out[j++] = 0x04;  // RAM id 4 = battery voltage
    out[j++] = 0x0E;  // RAM id 14 = AC power
    return j;
}

int VEBus::_prepareOnOff(char *out, byte fn, bool wakeup)
{
    byte j = 0;
    out[j++] = 0x98;
    out[j++] = 0xf7;
    out[j++] = 0xfe;
    out[j++] = fn;
    out[j++] = 0x3F;
    out[j++] = wakeup ? 0x07 : 0x04;
    out[j++] = 0x00;
    out[j++] = 0x00;
    out[j++] = 0x00;
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
// Frame decoder (unchanged from original)
// =======================================================================

void VEBus::_decodeFrame(const char *frame, int len)
{
    switch ((byte)frame[4])
    {
    case 0x80:  // Charger/Inverter condition
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

    case 0x41:  // Master LED / Multiplus mode
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

    case 0x00:  // ACK or RAM-read response
        if (frame[5] == (char)0xE6)
        {
            if (frame[6] == (char)0x87) {
                _acked = true;
            }
            else if (frame[6] == (char)0x85) {
                _gotMP2data = true;
                int16_t v = 256 * (uint8_t)frame[8] + (uint8_t)frame[7];
                _BatVolt = 0.01f * (float)v;
                _ACPower = 256 * (uint8_t)frame[10] + (uint8_t)frame[9];
            }
        }
        break;

    default:
        break;
    }
}
