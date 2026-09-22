#pragma once
// Minimal apcupsd Network Information Server (NIS) — TCP port 3551.
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

class ApcupsdServer {
public:
    static const uint8_t  MAX_CLIENTS = 4;
    static const uint32_t IDLE_TIMEOUT_MS = 60000;

    // Emits the status lines (key, value) in apcupsd order.
    using FieldFn = std::function<void(std::function<void(const char *, const char *)>)>;
    FieldFn fields;

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
            fields([&](const char *k, const char *v) {
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
};
