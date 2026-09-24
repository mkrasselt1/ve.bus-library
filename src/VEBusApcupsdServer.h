#pragma once
// VEBusApcupsdServer — apcupsd Network Information Server (NIS) for a
// Multiplus (optional, header-only; only compiled when you #include it).
// TCP port 3551.
//
// Lets apcaccess, apcupsd slaves (UPSCABLE ether / UPSTYPE net), the Home
// Assistant "APC UPS Daemon" integration and other apcupsd-aware tools read
// the Multiplus as if it were an APC UPS.
//
// Wire format (as in apcupsd's net_send/net_recv):
//   every message = 2-byte big-endian length + payload; length 0 = end.
//   Client sends "status" or "events"; the server answers with one message
//   per status line ("KEY      : value\n") followed by the 0-length marker.
#include <Arduino.h>
#include <WiFi.h>
#include <functional>
#include <string>
#include <time.h>
#include "VEBusUps.h"

//   VEBusUps           ups(vebus);
//   VEBusApcupsdServer apc(ups);
//   setup(): configTime(0, 0, "pool.ntp.org");   // optional, for DATE fields
//            apc.begin();                        // after WiFi is up
//   loop():  ups.loop(); apc.loop();

class VEBusApcupsdServer {
public:
    static const uint8_t  MAX_CLIENTS = 4;
    static const uint32_t IDLE_TIMEOUT_MS = 60000;

    using EmitFn = std::function<void(const char *, const char *)>;

    explicit VEBusApcupsdServer(VEBusUps &ups) : _ups(ups) {}

    const char *upsName  = "multiplus";
    const char *hostName = "vebus";
    const char *model    = "Victron MultiPlus";

    void begin(uint16_t port = 3551)
    {
        if (_running) return;
        _server.begin(port);
        _server.setNoDelay(true);
        _running = true;
    }

    void end()
    {
        if (!_running) return;
        for (auto &c : _c) _drop(c);
        _server.end();
        _running = false;
    }

    uint8_t clientCount() const
    {
        uint8_t n = 0;
        for (auto &c : _c) if (c.active) n++;
        return n;
    }

    void loop()
    {
        if (!_running) return;
        if (!_startEpoch && time(nullptr) > 1600000000)
            _startEpoch = time(nullptr) - millis() / 1000;

        WiFiClient nc = _server.accept();
        if (nc) {
            Client *slot = nullptr;
            for (auto &c : _c) if (!c.active) { slot = &c; break; }
            if (slot) {
                *slot = Client();
                slot->sock = nc;
                slot->active = true;
                slot->lastMs = millis();
            } else {
                nc.stop();
            }
        }

        for (auto &c : _c) {
            if (!c.active) continue;
            if (!c.sock.connected()) { _drop(c); continue; }
            while (c.sock.available()) {
                uint8_t b = c.sock.read();
                c.lastMs = millis();
                if (c.hdr < 2) {                       // length prefix
                    c.need = (c.need << 8) | b;
                    if (++c.hdr == 2) {
                        c.n = 0;
                        if (c.need == 0 || c.need >= sizeof(c.buf)) { _drop(c); break; }
                    }
                    continue;
                }
                c.buf[c.n++] = (char)b;
                if (c.n == c.need) {
                    c.buf[c.n] = '\0';
                    _handle(c);
                    c.hdr = 0; c.need = 0; c.n = 0;
                }
            }
            if (c.active && millis() - c.lastMs > IDLE_TIMEOUT_MS) _drop(c);
        }
    }

private:
    struct Client {
        WiFiClient sock;
        bool       active = false;
        uint8_t    hdr = 0;
        uint16_t   need = 0;
        uint8_t    n = 0;
        char       buf[32];
        uint32_t   lastMs = 0;
    };

    VEBusUps  &_ups;
    time_t     _startEpoch = 0;
    WiFiServer _server{3551};
    Client     _c[MAX_CLIENTS];
    bool       _running = false;
    std::string _out;

    void _drop(Client &c)
    {
        c.sock.stop();
        c.sock = WiFiClient();
        c.active = false;
    }

    static void _record(std::string &out, const std::string &line)
    {
        uint16_t len = line.size();
        out.push_back((char)(len >> 8));
        out.push_back((char)(len & 0xFF));
        out += line;
    }

    static std::string _line(const char *key, const char *value)
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "%-9s: %s\n", key, value);
        return std::string(buf);
    }

    void _handle(Client &c)
    {
        _out.clear();
        if (!strcmp(c.buf, "status")) {
            // Collect lines first: the "APC" header carries their count and size.
            std::string body;
            uint16_t count = 1, bytes = 0;
            _fields([&](const char *k, const char *v) {
                std::string l = _line(k, v);
                count++;
                bytes += l.size();
                _record(body, l);
            });
            char hdr[24];
            snprintf(hdr, sizeof(hdr), "001,%03u,%04u", count, bytes);
            std::string h = _line("APC", hdr);
            // header's own length is part of the byte count apcupsd reports
            snprintf(hdr, sizeof(hdr), "001,%03u,%04u", count, bytes + h.size());
            _record(_out, _line("APC", hdr));
            _out += body;
        }
        // "events" (and anything unknown): no lines, just the end marker
        _out.push_back(0);
        _out.push_back(0);
        c.sock.write((const uint8_t *)_out.data(), _out.size());
    }
    static void _date(char *out, size_t n, time_t t)
    {
        struct tm tm;
        gmtime_r(&t, &tm);
        strftime(out, n, "%Y-%m-%d %H:%M:%S +0000", &tm);
    }

    // Status lines in the order apcupsd prints them.
    void _fields(EmitFn emit)
    {
        const VEBusUpsData &d = _ups.data;
        VEBus &bus = _ups.bus();
        char v[48];
        time_t now = time(nullptr);
        bool clock = now > 1600000000;
        bool stale = _ups.stale();

        if (clock) { _date(v, sizeof(v), now); emit("DATE", v); }
        emit("HOSTNAME", hostName);
        emit("VERSION",  "3.14.14 (31 May 2016) vebus-esp32");
        emit("UPSNAME",  upsName);
        emit("CABLE",    "VE.Bus RS485");
        emit("DRIVER",   "VE.Bus ESP32 Driver");
        emit("UPSMODE",  "Stand Alone");
        if (clock && _startEpoch) { _date(v, sizeof(v), _startEpoch); emit("STARTTIME", v); }
        emit("MODEL",    model);

        // STATFLAG bits as defined by apcupsd
        const uint32_t ONLINE = 0x08, ONBATT = 0x10, OVERLOAD = 0x20, BATTLOW = 0x40,
                       COMMLOST = 0x100, SHUTDOWN = 0x200, PLUGGED = 0x1000000,
                       BATTPRESENT = 0x4000000;
        uint32_t flags = PLUGGED | BATTPRESENT;
        char status[40] = "";
        if (stale) {
            flags |= COMMLOST;
            strlcpy(status, "COMMLOST", sizeof(status));
        } else {
            bool mains = _ups.mainsPresent();
            flags |= mains ? ONLINE : ONBATT;
            strlcat(status, mains ? "ONLINE" : "ONBATT", sizeof(status));
            if (_ups.lowBattery()) { flags |= BATTLOW;  strlcat(status, " LOWBATT", sizeof(status)); }
            if (_ups.overload())   { flags |= OVERLOAD; strlcat(status, " OVERLOAD", sizeof(status)); }
            if (_ups.fsd)          { flags |= SHUTDOWN; strlcat(status, " SHUTTING DOWN", sizeof(status)); }
        }
        emit("STATUS", status);

        if (!stale) {
            snprintf(v, sizeof(v), "%.1f Volts", d.mainsV);                 emit("LINEV", v);
            if (_ups.nominalW) {
                snprintf(v, sizeof(v), "%.1f Percent", _ups.loadPct());     emit("LOADPCT", v);
            }
            if (d.socValid) {
                snprintf(v, sizeof(v), "%.1f Percent", _ups.socPct());      emit("BCHARGE", v);
            }
            float rt = _ups.runtimeMinutes();
            if (rt >= 0) { snprintf(v, sizeof(v), "%.1f Minutes", rt);     emit("TIMELEFT", v); }
        }
        snprintf(v, sizeof(v), "%u Percent", _ups.lowSocPct);              emit("MBATTCHG", v);
        if (!stale) {
            snprintf(v, sizeof(v), "%.1f Volts", d.invV);                   emit("OUTPUTV", v);
            snprintf(v, sizeof(v), "%.1f C", bus.getTemp());                 emit("ITEMP", v);
            snprintf(v, sizeof(v), "%.1f Volts", bus.getBatVolt());          emit("BATTV", v);
            snprintf(v, sizeof(v), "%.1f Hz", _ups.mainsHz());               emit("LINEFREQ", v);
        }
        snprintf(v, sizeof(v), "%lu", (unsigned long)_ups.transfers());     emit("NUMXFERS", v);
        snprintf(v, sizeof(v), "%lu Seconds", (unsigned long)_ups.secondsOnBattery()); emit("TONBATT", v);
        snprintf(v, sizeof(v), "%lu Seconds", (unsigned long)_ups.totalSecondsOnBattery()); emit("CUMONBATT", v);
        snprintf(v, sizeof(v), "0x%08lX", (unsigned long)flags);            emit("STATFLAG", v);
        uint8_t mac[6];
        WiFi.macAddress(mac);
        snprintf(v, sizeof(v), "VEBUS%02X%02X%02X%02X%02X%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);           emit("SERIALNO", v);
        if (_ups.nominalW) { snprintf(v, sizeof(v), "%u Watts", _ups.nominalW); emit("NOMPOWER", v); }
        if (d.firmware[0]) emit("FIRMWARE", d.firmware);
        if (clock) { _date(v, sizeof(v), now); emit("END APC", v); }
    }
};
