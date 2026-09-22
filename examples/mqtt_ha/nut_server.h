#pragma once
// Minimal NUT (Network UPS Tools) server — speaks enough of the upsd network
// protocol (port 3493) for upsmon, Synology/QNAP/TrueNAS, Proxmox, WinNUT and
// the Home Assistant NUT integration to monitor the Multiplus as a UPS and
// shut down on low battery.
//
// Read-only: no SET/INSTCMD. Authentication (USERNAME/PASSWORD) is enforced
// for LOGIN / PRIMARY / FSD when credentials are configured.
#include <Arduino.h>
#include <WiFi.h>
#include <functional>

class NutServer {
public:
    static const uint8_t MAX_CLIENTS = 4;
    static const uint32_t IDLE_TIMEOUT_MS = 300000;   // drop silent clients after 5 min

    // Emits one "name value" pair per call; return false for stale data.
    using VarFn = std::function<bool(std::function<void(const char *, const char *)>)>;

    const char *upsName  = "multiplus";
    const char *upsDesc  = "Victron MultiPlus (VE.Bus)";
    const char *user     = "";      // empty = accept any credentials
    const char *pass     = "";
    VarFn       vars;
    bool        fsd      = false;   // forced shutdown flag set by a primary

    void begin(uint16_t port = 3493)
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

    bool    running() const { return _running; }
    uint8_t clientCount() const
    {
        uint8_t n = 0;
        for (auto &c : _c) if (c.active) n++;
        return n;
    }
    uint8_t loginCount() const
    {
        uint8_t n = 0;
        for (auto &c : _c) if (c.active && c.loggedIn) n++;
        return n;
    }

    void loop()
    {
        if (!_running) return;

        // Accept new connections
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
                nc.print("ERR MAX-CLIENTS\n");
                nc.stop();
            }
        }

        for (auto &c : _c) {
            if (!c.active) continue;
            if (!c.sock.connected()) { _drop(c); continue; }
            while (c.sock.available()) {
                char ch = c.sock.read();
                c.lastMs = millis();
                if (ch == '\r') continue;
                if (ch != '\n') {
                    if (c.n < sizeof(c.line) - 1) c.line[c.n++] = ch;
                    continue;
                }
                c.line[c.n] = '\0';
                c.n = 0;
                if (!_handle(c)) { _drop(c); break; }
            }
            if (c.active && millis() - c.lastMs > IDLE_TIMEOUT_MS) _drop(c);
        }
    }

private:
    struct Client {
        WiFiClient sock;
        bool       active = false;
        char       line[160];
        uint8_t    n = 0;
        uint32_t   lastMs = 0;
        bool       hasUser = false, hasPass = false, authOk = false, loggedIn = false;
        char       userBuf[32] = "";
    };

    WiFiServer _server{3493};
    Client     _c[MAX_CLIENTS];
    bool       _running = false;
    String     _out;

    void _drop(Client &c)
    {
        c.sock.stop();
        c.sock = WiFiClient();
        c.loggedIn = false;
        c.active = false;
    }

    // Split a line into tokens; supports "quoted strings" with \" escapes.
    static uint8_t _tokenize(char *s, char **tok, uint8_t max)
    {
        uint8_t n = 0;
        while (*s && n < max) {
            while (*s == ' ' || *s == '\t') s++;
            if (!*s) break;
            if (*s == '"') {
                char *w = ++s;
                tok[n++] = w;
                while (*s && *s != '"') {
                    if (*s == '\\' && s[1]) s++;
                    *w++ = *s++;
                }
                if (*s) s++;
                *w = '\0';
            } else {
                tok[n++] = s;
                while (*s && *s != ' ' && *s != '\t') s++;
                if (*s) *s++ = '\0';
            }
        }
        return n;
    }

    bool _credsOk(const Client &c) const
    {
        if (!user[0]) return true;
        return c.authOk;
    }

    void _send(Client &c, const String &s) { c.sock.write((const uint8_t *)s.c_str(), s.length()); }

    static void _quote(String &out, const char *v)
    {
        out += '"';
        for (; *v; v++) {
            if (*v == '"' || *v == '\\') out += '\\';
            out += *v;
        }
        out += '"';
    }

    bool _knownUps(const char *name) const { return name && !strcmp(name, upsName); }

    // Returns false when the connection should be closed.
    bool _handle(Client &c)
    {
        char *t[6] = {nullptr};
        uint8_t n = _tokenize(c.line, t, 6);
        if (!n) return true;
        const char *cmd = t[0];

        if (!strcasecmp(cmd, "VER")) {
            _send(c, "Network UPS Tools upsd 2.8.1 - VEBus ESP32\n");
        }
        else if (!strcasecmp(cmd, "NETVER") || !strcasecmp(cmd, "PROTVER")) {
            _send(c, "1.3\n");
        }
        else if (!strcasecmp(cmd, "HELP")) {
            _send(c, "Commands: HELP VER GET LIST SET INSTCMD LOGIN LOGOUT USERNAME PASSWORD STARTTLS\n");
        }
        else if (!strcasecmp(cmd, "STARTTLS")) {
            _send(c, "ERR FEATURE-NOT-CONFIGURED\n");
        }
        else if (!strcasecmp(cmd, "USERNAME")) {
            if (n < 2)            _send(c, "ERR INVALID-ARGUMENT\n");
            else if (c.hasUser)   _send(c, "ERR ALREADY-SET-USERNAME\n");
            else {
                strlcpy(c.userBuf, t[1], sizeof(c.userBuf));
                c.hasUser = true;
                _send(c, "OK\n");
            }
        }
        else if (!strcasecmp(cmd, "PASSWORD")) {
            if (n < 2)            _send(c, "ERR INVALID-ARGUMENT\n");
            else if (c.hasPass)   _send(c, "ERR ALREADY-SET-PASSWORD\n");
            else {
                c.hasPass = true;
                c.authOk  = c.hasUser && !strcmp(c.userBuf, user) && !strcmp(t[1], pass);
                _send(c, "OK\n");
            }
        }
        else if (!strcasecmp(cmd, "LOGIN") || !strcasecmp(cmd, "PRIMARY") || !strcasecmp(cmd, "MASTER")) {
            if (n < 2)                   _send(c, "ERR INVALID-ARGUMENT\n");
            else if (!_knownUps(t[1]))   _send(c, "ERR UNKNOWN-UPS\n");
            else if (user[0] && !c.hasUser) _send(c, "ERR USERNAME-REQUIRED\n");
            else if (user[0] && !c.hasPass) _send(c, "ERR PASSWORD-REQUIRED\n");
            else if (!_credsOk(c))       _send(c, "ERR ACCESS-DENIED\n");
            else if (!strcasecmp(cmd, "LOGIN")) {
                if (c.loggedIn) _send(c, "ERR ALREADY-LOGGED-IN\n");
                else { c.loggedIn = true; _send(c, "OK\n"); }
            }
            else if (!strcasecmp(cmd, "PRIMARY")) _send(c, "OK PRIMARY-GRANTED\n");
            else                                  _send(c, "OK MASTER-GRANTED\n");
        }
        else if (!strcasecmp(cmd, "FSD")) {
            if (n < 2 || !_knownUps(t[1])) _send(c, "ERR UNKNOWN-UPS\n");
            else if (!_credsOk(c) || (user[0] && !c.hasPass)) _send(c, "ERR ACCESS-DENIED\n");
            else { fsd = true; _send(c, "OK FSD-SET\n"); }
        }
        else if (!strcasecmp(cmd, "LOGOUT")) {
            _send(c, "OK Goodbye\n");
            return false;
        }
        else if (!strcasecmp(cmd, "GET")) {
            _get(c, t, n);
        }
        else if (!strcasecmp(cmd, "LIST")) {
            _list(c, t, n);
        }
        else if (!strcasecmp(cmd, "SET") || !strcasecmp(cmd, "INSTCMD")) {
            _send(c, "ERR CMD-NOT-SUPPORTED\n");
        }
        else {
            _send(c, "ERR UNKNOWN-COMMAND\n");
        }
        return true;
    }

    void _get(Client &c, char **t, uint8_t n)
    {
        if (n < 3) { _send(c, "ERR INVALID-ARGUMENT\n"); return; }
        const char *what = t[1], *ups = t[2];
        if (!_knownUps(ups)) { _send(c, "ERR UNKNOWN-UPS\n"); return; }

        if (!strcasecmp(what, "UPSDESC")) {
            _out = "UPSDESC "; _out += upsName; _out += ' '; _quote(_out, upsDesc); _out += '\n';
            _send(c, _out);
        }
        else if (!strcasecmp(what, "NUMLOGINS")) {
            _send(c, String("NUMLOGINS ") + upsName + ' ' + loginCount() + '\n');
        }
        else if (!strcasecmp(what, "VAR") || !strcasecmp(what, "TYPE") || !strcasecmp(what, "DESC")) {
            if (n < 4) { _send(c, "ERR INVALID-ARGUMENT\n"); return; }
            const char *want = t[3];
            String val;
            bool found = false;
            bool fresh = vars([&](const char *k, const char *v) {
                if (!found && !strcmp(k, want)) { val = v; found = true; }
            });
            if (!found)       { _send(c, "ERR VAR-NOT-SUPPORTED\n"); return; }
            if (!strcasecmp(what, "TYPE")) {
                bool num = val.length() && strspn(val.c_str(), "-0123456789.") == val.length();
                _send(c, String("TYPE ") + upsName + ' ' + want + (num ? " NUMBER\n" : " STRING:64\n"));
            }
            else if (!strcasecmp(what, "DESC")) {
                _send(c, String("DESC ") + upsName + ' ' + want + " \"Description unavailable\"\n");
            }
            else if (!fresh)  _send(c, "ERR DATA-STALE\n");
            else {
                _out = "VAR "; _out += upsName; _out += ' '; _out += want; _out += ' ';
                _quote(_out, val.c_str()); _out += '\n';
                _send(c, _out);
            }
        }
        else if (!strcasecmp(what, "CMDDESC")) {
            _send(c, "ERR CMD-NOT-SUPPORTED\n");
        }
        else {
            _send(c, "ERR INVALID-ARGUMENT\n");
        }
    }

    void _list(Client &c, char **t, uint8_t n)
    {
        if (n < 2) { _send(c, "ERR INVALID-ARGUMENT\n"); return; }
        const char *what = t[1];

        if (!strcasecmp(what, "UPS")) {
            _out = "BEGIN LIST UPS\nUPS "; _out += upsName; _out += ' ';
            _quote(_out, upsDesc); _out += "\nEND LIST UPS\n";
            _send(c, _out);
            return;
        }
        if (n < 3) { _send(c, "ERR INVALID-ARGUMENT\n"); return; }
        if (!_knownUps(t[2])) { _send(c, "ERR UNKNOWN-UPS\n"); return; }

        if (!strcasecmp(what, "VAR")) {
            _out = "BEGIN LIST VAR "; _out += upsName; _out += '\n';
            // Unlike upsd, LIST VAR is served even when stale (ups.status then
            // reads UNKNOWN) so clients such as the Home Assistant NUT
            // integration can be set up before the inverter is connected.
            // GET VAR — what upsmon/NAS clients poll — still reports DATA-STALE.
            vars([&](const char *k, const char *v) {
                _out += "VAR "; _out += upsName; _out += ' '; _out += k; _out += ' ';
                _quote(_out, v); _out += '\n';
            });
            _out += "END LIST VAR "; _out += upsName; _out += '\n';
            _send(c, _out);
        }
        else if (!strcasecmp(what, "CLIENT")) {
            _out = "BEGIN LIST CLIENT "; _out += upsName; _out += '\n';
            for (auto &x : _c) if (x.active && x.loggedIn) {
                _out += "CLIENT "; _out += upsName; _out += ' ';
                _out += x.sock.remoteIP().toString(); _out += '\n';
            }
            _out += "END LIST CLIENT "; _out += upsName; _out += '\n';
            _send(c, _out);
        }
        else if (!strcasecmp(what, "CMD") || !strcasecmp(what, "RW")) {
            String w(what); w.toUpperCase();
            _send(c, "BEGIN LIST " + w + ' ' + upsName + "\nEND LIST " + w + ' ' + upsName + '\n');
        }
        else if (!strcasecmp(what, "ENUM") || !strcasecmp(what, "RANGE")) {
            if (n < 4) { _send(c, "ERR INVALID-ARGUMENT\n"); return; }
            String w(what); w.toUpperCase();
            String sfx = String(upsName) + ' ' + t[3];
            _send(c, "BEGIN LIST " + w + ' ' + sfx + "\nEND LIST " + w + ' ' + sfx + '\n');
        }
        else {
            _send(c, "ERR INVALID-ARGUMENT\n");
        }
    }
};
