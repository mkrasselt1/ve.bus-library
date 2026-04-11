#include "VEBus.h"

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
VEBus::VEBus()
    : _frp(0), _frlen(0), _frameNr(0),
      _syncrxed(false), _gotMP2data(false), _acked(false), _nosync(false),
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
// begin() — configure and start the RS485 UART
// -----------------------------------------------------------------------
void VEBus::begin(int rxPin, int txPin, int dePin, uint32_t baud)
{
    Serial1.begin(baud, SERIAL_8N1, rxPin, txPin);
    // Configure the DE (driver-enable) pin for RS485 half-duplex
    Serial1.setPins(-1, -1, -1, dePin);
    Serial1.setMode(UART_MODE_RS485_HALF_DUPLEX);
}

// -----------------------------------------------------------------------
// handle() — process all bytes currently in the UART RX buffer.
// Call this as often as possible (every few ms) from loop() or a task.
// -----------------------------------------------------------------------
void VEBus::handle()
{
    // No-sync watchdog: if no sync frame for > 1 s, set the nosync flag.
    if (millis() > _synctime + 1000) {
        _nosync   = true;
        _synctime = millis(); // reset so we don't spam every iteration
    }

    while (Serial1.available())
    {
        char c = Serial1.read();
        _frbuf1[_frp++] = c;

        // Detect sync pulse position (0x55 at byte 5 of a sync frame)
        if ((uint8_t)c == 0x55) {
            if (_frp == 5) _synctime = millis();
        }

        if ((uint8_t)c == 0xFF) // End-of-frame marker
        {
            if ((_frbuf1[2] == (char)0xFD) && (_frbuf1[4] == (char)0x55))
            {
                // Sync frame  —  extract rolling frame number
                _frameNr  = (byte)_frbuf1[3];
                _syncrxed = true;
                _nosync   = false;
            }
            else if ((_frbuf1[0] == (char)0x83) &&
                     (_frbuf1[1] == (char)0x83) &&
                     (_frbuf1[2] == (char)0xFE))
            {
                // Data frame from Multiplus
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
            _frp = 0; // reset frame pointer for next frame
        }
        else
        {
            // Any non-EOF byte clears the sync flag so the caller cannot
            // miss the narrow window for TX (matches original behaviour).
            _syncrxed = false;
        }
    }
}

// -----------------------------------------------------------------------
// sendESSPower() — write ESS power setpoint
// -----------------------------------------------------------------------
void VEBus::sendESSPower(int16_t watts)
{
    int len = _prepareESSCommand(_txbuf1, watts, (_frameNr + 1) & 0x7F);
    _sendFrame(_txbuf1, len);
}

// -----------------------------------------------------------------------
// sendReadRAM() — request bat voltage (id 0x04) + AC power (id 0x0E)
// -----------------------------------------------------------------------
void VEBus::sendReadRAM()
{
    int len = _prepareReadRAM(_txbuf1, (_frameNr + 1) & 0x7F);
    _sendFrame(_txbuf1, len);
}

// -----------------------------------------------------------------------
// sendSleep() / sendWakeup()
// -----------------------------------------------------------------------
void VEBus::sendSleep()
{
    int len = _prepareOnOff(_txbuf1, (_frameNr + 1) & 0x7F, false);
    _sendFrame(_txbuf1, len);
}

void VEBus::sendWakeup()
{
    int len = _prepareOnOff(_txbuf1, (_frameNr + 1) & 0x7F, true);
    _sendFrame(_txbuf1, len);
}

// =======================================================================
// Private helpers
// =======================================================================

// -----------------------------------------------------------------------
// _prepareESSCommand — build raw ESS power command in outbuf
// Returns number of bytes written.
// -----------------------------------------------------------------------
int VEBus::_prepareESSCommand(char *out, int16_t power, byte fn)
{
    byte j = 0;
    out[j++] = 0x98;           // MK3 interface address
    out[j++] = 0xf7;           // MK3 interface address
    out[j++] = 0xfe;           // data frame type
    out[j++] = fn;             // rolling frame number
    out[j++] = 0x00;           // source ID (our own)
    out[j++] = 0xe6;           // source ID (our own)
    out[j++] = 0x37;           // CommandWriteViaID
    out[j++] = 0x02;           // Flags: RAM var, no EEPROM
    out[j++] = 0x83;           // RAM address of ESS power setpoint
    out[j++] = (char)(power & 0xFF);  // power lo byte
    out[j++] = (char)(power >> 8);    // power hi byte
    return j;
}

// -----------------------------------------------------------------------
// _prepareReadRAM — request battery voltage and AC power
// -----------------------------------------------------------------------
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
    out[j++] = 0x04;  // RAM id 4  = battery voltage
    out[j++] = 0x0E;  // RAM id 14 = AC power
    return j;
}

// -----------------------------------------------------------------------
// _prepareOnOff — sleep (false) or wakeup (true) command
// -----------------------------------------------------------------------
int VEBus::_prepareOnOff(char *out, byte fn, bool wakeup)
{
    byte j = 0;
    out[j++] = 0x98;
    out[j++] = 0xf7;
    out[j++] = 0xfe;
    out[j++] = fn;
    out[j++] = 0x3F;                   // command byte
    out[j++] = wakeup ? 0x07 : 0x04;   // 0x07 = wakeup, 0x04 = sleep
    out[j++] = 0x00;
    out[j++] = 0x00;
    out[j++] = 0x00;
    return j;
}

// -----------------------------------------------------------------------
// _replaceFAtoFF — encode bytes 0xFA–0xFF as two-byte sequences
// (required for all payload bytes from byte 4 onward)
// -----------------------------------------------------------------------
int VEBus::_replaceFAtoFF(char *out, const char *in, int len)
{
    int j = 0;
    for (int i = 0; i < 4; i++) out[j++] = in[i]; // first 4 bytes unchanged
    for (int i = 4; i < len; i++)
    {
        byte c = (byte)in[i];
        if (c >= 0xFA)
        {
            out[j++] = 0xFA;
            out[j++] = (char)(0x70 | (c & 0x0F));
        }
        else
        {
            out[j++] = in[i];
        }
    }
    return j;
}

// -----------------------------------------------------------------------
// _destuffFAtoFF — reverse the encoding above (for received frames)
// -----------------------------------------------------------------------
int VEBus::_destuffFAtoFF(char *out, const char *in, int len)
{
    int j = 0;
    for (int i = 0; i < 4; i++) out[j++] = in[i]; // first 4 bytes unchanged
    for (int i = 4; i < len; i++)
    {
        byte c = (byte)in[i];
        if (c == 0xFA)
        {
            c = (byte)in[++i];
            if (c == 0xFF)
            {
                // 0xFA 0xFF means the checksum itself was 0xFA; keep both.
                out[j++] = 0xFA;
                out[j++] = (char)c;
            }
            else
            {
                out[j++] = (char)(c + 0x80);
            }
        }
        else
        {
            out[j++] = in[i];
        }
    }
    return j;
}

// -----------------------------------------------------------------------
// _verifyChecksum — sum of bytes [2..n-1] must equal 0x00
// -----------------------------------------------------------------------
bool VEBus::_verifyChecksum(const char *buf, int len)
{
    byte cs = 0;
    for (int i = 2; i < len; i++) cs += (byte)buf[i];
    return (cs == 0);
}

// -----------------------------------------------------------------------
// _appendChecksum — append checksum byte (and EOF 0xFF) to buf
// Returns new length.
// -----------------------------------------------------------------------
int VEBus::_appendChecksum(char *buf, int len)
{
    byte cs = 1;
    for (int i = 2; i < len; i++) cs -= (byte)buf[i];
    int j = len;
    if (cs >= 0xFB)
    {
        // Checksum value itself is in the 0xFA–0xFF range — needs escaping
        buf[j++] = (char)0xFA;
        buf[j++] = (char)(cs - 0xFA);
    }
    else
    {
        buf[j++] = (char)cs;
    }
    buf[j++] = (char)0xFF; // End-of-frame
    return j;
}

// -----------------------------------------------------------------------
// _sendFrame — byte-stuff, add checksum, transmit
// -----------------------------------------------------------------------
void VEBus::_sendFrame(char *raw, int rawLen)
{
    int len = _replaceFAtoFF(_txbuf2, raw, rawLen);
    len = _appendChecksum(_txbuf2, len);
    Serial1.write((const uint8_t *)_txbuf2, len);
}

// -----------------------------------------------------------------------
// _decodeFrame — interpret a de-stuffed data frame from the Multiplus
// -----------------------------------------------------------------------
void VEBus::_decodeFrame(const char *frame, int len)
{
    switch ((byte)frame[4])
    {
    // ------------------------------------------------------------------
    // 0x80 — Charger / Inverter condition (temperature + DC current)
    // ------------------------------------------------------------------
    case 0x80:
        if ((frame[5] == (char)0x80) &&
            ((frame[6] & 0xFE) == 0x12) &&
            (frame[8] == (char)0x80) &&
            ((frame[11] & 0x10) == 0x10))
        {
            _multiplusStatus80 = (byte)frame[7];
            _multiplusDcLevelAllowsInverting = ((byte)frame[6] & 0x01) != 0;
            int16_t t = (int16_t)((uint8_t)frame[10] << 8 | (uint8_t)frame[9]);
            _multiplusDcCurrent = 0.1f * (float)t;
            if (((byte)frame[11] & 0xF0) == 0x30)
                _multiplusTemp = 0.1f * (float)(uint8_t)frame[15];
        }
        break;

    // ------------------------------------------------------------------
    // 0x41 — Multiplus mode / master LED status
    // ------------------------------------------------------------------
    case 0x41:
        if ((len == 19) && (frame[5] == (char)0x10))
        {
            _masterMultiLED_LEDon    = (byte)frame[6];
            _masterMultiLED_LEDblink = (byte)frame[7];
            _masterMultiLED_Status   = (byte)frame[8];
            _masterMultiLED_AcInputConfiguration = (byte)frame[9];

            int16_t t;
            t = (int16_t)((uint8_t)frame[11] << 8 | (uint8_t)frame[10]);
            _masterMultiLED_MinimumInputCurrentLimit = (float)t / 10.0f;
            t = (int16_t)((uint8_t)frame[13] << 8 | (uint8_t)frame[12]);
            _masterMultiLED_MaximumInputCurrentLimit = (float)t / 10.0f;
            t = (int16_t)((uint8_t)frame[15] << 8 | (uint8_t)frame[14]);
            _masterMultiLED_ActualInputCurrentLimit  = (float)t / 10.0f;
            _masterMultiLED_SwitchRegister = (byte)frame[16];
        }
        break;

    // ------------------------------------------------------------------
    // 0x00 — ACK or RAM-read response
    // ------------------------------------------------------------------
    case 0x00:
        if (frame[5] == (char)0xE6)
        {
            if (frame[6] == (char)0x87)
            {
                // Write acknowledgement
                _acked = true;
            }
            else if (frame[6] == (char)0x85)
            {
                // RAM read response: battery voltage + AC power
                _gotMP2data = true;
                int16_t v = (int16_t)((uint8_t)frame[8] << 8 | (uint8_t)frame[7]);
                _BatVolt = 0.01f * (float)v;
                _ACPower = (int16_t)((uint8_t)frame[10] << 8 | (uint8_t)frame[9]);
            }
        }
        break;

    // 0xE4 (AC phase), 0x70 (DC capacity counter), 0x38 (unknown) —
    // frames are received but not decoded here; extend if needed.
    default:
        break;
    }
}
