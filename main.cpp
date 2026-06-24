#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <algorithm>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr const char* kDefaultPort = "/dev/ttyUSB0";
constexpr int kBaud = B115200;

constexpr uint8_t ASH_FLAG = 0x7E;
constexpr uint8_t ASH_ESCAPE = 0x7D;
constexpr uint8_t ASH_XON = 0x11;
constexpr uint8_t ASH_XOFF = 0x13;
constexpr uint8_t ASH_SUBSTITUTE = 0x18;
constexpr uint8_t ASH_CANCEL = 0x1A;

constexpr uint16_t CMD_VERSION = 0x0000;
constexpr uint16_t CMD_NETWORK_INIT = 0x0017;
constexpr uint16_t CMD_NETWORK_STATE = 0x0018;
constexpr uint16_t CMD_SET_POLICY = 0x0055;
constexpr uint16_t CMD_PERMIT_JOINING = 0x0022;
constexpr uint16_t CMD_ADD_ENDPOINT = 0x0002;
constexpr uint16_t CMD_IMPORT_TRANSIENT_KEY = 0x0111;
constexpr uint16_t CMD_ADD_TRANSIENT_LINK_KEY = 0x00AF;
constexpr uint16_t CMD_GET_NETWORK_PARAMETERS = 0x0028;
constexpr uint16_t CMD_GET_EUI64 = 0x0026;
constexpr uint16_t CMD_GET_NODE_ID = 0x0027;
constexpr uint16_t CMD_FORM_NETWORK = 0x001E;
constexpr uint16_t CMD_SET_INITIAL_SECURITY_STATE = 0x0068;
constexpr uint16_t CMD_SEND_UNICAST = 0x0034;
constexpr uint16_t CMD_SEND_BROADCAST = 0x0036;
constexpr uint16_t CMD_SET_CONFIGURATION_VALUE = 0x0053;
constexpr uint16_t CMD_LEAVE_NETWORK = 0x001B;

constexpr uint8_t ZCL_CONFIGURE_REPORTING = 0x06;

constexpr uint16_t CB_STACK_STATUS = 0x0019;
constexpr uint16_t CB_INCOMING_MESSAGE = 0x0045;
constexpr uint16_t CB_TRUST_CENTER_JOIN = 0x0024;

constexpr uint16_t ZCL_CLUSTER_ONOFF = 0x0006;
constexpr uint16_t ZCL_CLUSTER_METERING = 0x0702;
constexpr uint16_t ZCL_CLUSTER_ELEC_MEAS = 0x0B04;

constexpr uint8_t ZCL_READ = 0x00;
constexpr uint8_t ZCL_READ_RSP = 0x01;
constexpr uint8_t ZCL_DEFAULT_RSP = 0x0B;
constexpr uint8_t ZCL_REPORT = 0x0A;

constexpr uint16_t ATTR_ONOFF = 0x0000;
constexpr uint16_t ATTR_RMS_V = 0x0505;
constexpr uint16_t ATTR_RMS_A = 0x0508;
constexpr uint16_t ATTR_ACTIVE_W = 0x050B;
constexpr uint16_t ATTR_ENERGY = 0x0000;
constexpr uint16_t ATTR_AC_V_MULT = 0x0600;
constexpr uint16_t ATTR_AC_V_DIV  = 0x0601;
constexpr uint16_t ATTR_AC_I_MULT = 0x0602;
constexpr uint16_t ATTR_AC_I_DIV  = 0x0603;
constexpr uint16_t ATTR_AC_P_MULT = 0x0604;
constexpr uint16_t ATTR_AC_P_DIV  = 0x0605;
constexpr uint16_t ATTR_MTR_MULT  = 0x0301;
constexpr uint16_t ATTR_MTR_DIV   = 0x0302;

constexpr uint16_t EMBER_NETWORK_UP = 0x90;
constexpr uint16_t EMBER_NETWORK_OPENED = 0x9C;
constexpr uint16_t kPreferredPan = 0x2D3F;
constexpr uint8_t kPreferredChannel = 26;
volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

// ---- Helpers ----
uint16_t crcCcitt(const uint8_t* d, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        c ^= static_cast<uint16_t>(d[i]) << 8;
        for (int b = 0; b < 8; ++b)
            c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x1021)
                             : static_cast<uint16_t>(c << 1);
    }
    return c;
}

bool isReserved(uint8_t b) {
    return b == ASH_FLAG || b == ASH_ESCAPE || b == ASH_XON || b == ASH_XOFF ||
           b == ASH_SUBSTITUTE || b == ASH_CANCEL;
}

std::vector<uint8_t> prand(size_t n) {
    std::vector<uint8_t> o(n);
    uint8_t r = 0x42;
    for (size_t i = 0; i < n; ++i) {
        o[i] = r;
        r = (r & 1) ? static_cast<uint8_t>((r >> 1) ^ 0xB8)
                    : static_cast<uint8_t>(r >> 1);
    }
    return o;
}

std::vector<uint8_t> arand(const std::vector<uint8_t>& d) {
    auto r = prand(d.size());
    std::vector<uint8_t> o(d.size());
    for (size_t i = 0; i < d.size(); ++i) o[i] = d[i] ^ r[i];
    return o;
}

void le16(std::vector<uint8_t>& o, uint16_t v) {
    o.push_back(v & 0xFF);
    o.push_back((v >> 8) & 0xFF);
}

uint16_t rle16(const std::vector<uint8_t>& d, size_t o) {
    return (o + 1 < d.size()) ? static_cast<uint16_t>(d[o] | (d[o + 1] << 8)) : 0;
}

uint32_t rle32(const std::vector<uint8_t>& d, size_t o) {
    return (o + 3 < d.size()) ? (static_cast<uint32_t>(d[o]) | (static_cast<uint32_t>(d[o+1]) << 8) |
                                 static_cast<uint32_t>(d[o+2]) << 16 | static_cast<uint32_t>(d[o+3]) << 24) : 0;
}

std::string hex(const std::vector<uint8_t>& d) {
    if (d.empty()) return "(empty)";
    std::ostringstream os;
    for (size_t i = 0; i < d.size(); ++i) {
        if (i) os << ' ';
        os << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(d[i]);
    }
    return os.str();
}

class CsvLogger {
    std::ofstream f_;
public:
    CsvLogger() {
        std::ifstream t("plug_data.csv");
        bool e = t.good(); t.close();
        f_.open("plug_data.csv", std::ios::app);
        if (!e && f_.is_open()) f_ << "time,nwk,V,A,W,kWh,state\n";
    }
    void log(uint16_t n, std::optional<float> v, std::optional<float> a,
             std::optional<float> w, std::optional<float> e,
             std::optional<bool> s) {
        if (!f_.is_open()) return;
        timeval tv{}; gettimeofday(&tv, nullptr);
        char buf[32]; strftime(buf, sizeof(buf), "%H:%M:%S", localtime(&tv.tv_sec));
        auto fmt = [](std::optional<float> val) -> std::string {
            if (!val) return "";
            std::ostringstream os; os << std::fixed << std::setprecision(1) << *val;
            return os.str();
        };
        f_ << buf << ",0x" << std::hex << n << std::dec
           << "," << fmt(v)
           << "," << fmt(a)
           << "," << fmt(w)
           << "," << (e ? [&]{ std::ostringstream os; os << std::fixed << std::setprecision(3) << *e; return os.str(); }() : "")
           << "," << (s ? (*s ? "ON" : "OFF") : "") << "\n";
        f_.flush();
    }
};

class SerialPort {
    int fd_ = -1;
public:
    explicit SerialPort(const std::string& path) {
        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) { std::cerr << "open " << path << ": " << strerror(errno) << "\n"; return; }
        termios tio{}; tcgetattr(fd_, &tio); cfmakeraw(&tio);
        cfsetispeed(&tio, kBaud); cfsetospeed(&tio, kBaud);
        tio.c_cflag |= CLOCAL | CREAD; tio.c_cflag &= ~(CRTSCTS | PARENB | CSTOPB | CSIZE); tio.c_cflag |= CS8;
        tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 0;
        tcsetattr(fd_, TCSANOW, &tio); tcflush(fd_, TCIOFLUSH);
    }
    ~SerialPort() { if (fd_ >= 0) close(fd_); }
    bool ok() const { return fd_ >= 0; }
    bool writeAll(const std::vector<uint8_t>& d) {
        size_t o = 0;
        while (o < d.size()) {
            ssize_t n = ::write(fd_, d.data() + o, d.size() - o);
            if (n < 0) { if (errno == EINTR || errno == EAGAIN) continue; return false; }
            o += static_cast<size_t>(n);
        }
        tcdrain(fd_); return true;
    }
    ssize_t readSome(uint8_t* b, size_t n, int ms) {
        fd_set s; FD_ZERO(&s); FD_SET(fd_, &s);
        timeval tv{}; tv.tv_sec = ms / 1000; tv.tv_usec = (ms % 1000) * 1000;
        return (select(fd_ + 1, &s, nullptr, nullptr, &tv) > 0) ? ::read(fd_, b, n) : 0;
    }
};

enum class AshType { Data, Ack, Nak, Rstack, Error, None };
struct AshFrame {
    AshType type = AshType::None;
    uint8_t ctrl = 0, frm = 0;
    bool reTx = false;
    std::vector<uint8_t> ezsp, raw;
};

class NullBuffer : public std::streambuf {
    int overflow(int c) override { return c; }
};

class AshEzsp {
    NullBuffer nullBuf_;
    std::ostream nullOut_{&nullBuf_};
    SerialPort& sp_;
    CsvLogger& csv_;
    std::vector<uint8_t> rx_;
    bool esc_ = false;
    bool verbose_ = false;
    uint8_t txSeq_ = 0, rxSeq_ = 0, ezspSeq_ = 0;
    uint8_t ezspVer_ = 4;
    bool extHdr_ = false;
    bool netUp_ = false;
    uint16_t myNwk_ = 0;
    std::vector<uint16_t> devices_;
    uint8_t pollSeq_ = 0;
    uint8_t apsSeq_ = 0;
    uint16_t msgTag_ = 0;
    bool setupDone_ = false;
    uint16_t vMult_ = 1, vDiv_ = 10;
    uint16_t iMult_ = 1, iDiv_ = 1000;
    uint16_t pMult_ = 1, pDiv_ = 10;
    uint32_t eMult_ = 1, eDiv_ = 10000;
    
    // ---- Bucle interno de eventos: drena callbacks hasta deadline ----
    // (copia la lógica de v2_zigbee_reader.cpp que SÍ funciona)
    void processEvents(int timeoutMs) {
        int64_t deadline = nowMs() + timeoutMs;
        while (nowMs() < deadline && !g_stop) {
            auto opt = readRaw(100);
            if (!opt) continue;

            if (opt->type == AshType::Data) {
                // ACK automático
                if (opt->frm == rxSeq_) {
                    rxSeq_ = (rxSeq_ + 1) & 0x07;
                    sendAck();
                    // Procesar EZSP
                    processEzsp(opt->ezsp);
                } else {
                    sendAck(); // ACK duplicado
                }
            } else if (opt->type == AshType::Ack) {
                // ACK recibido de un comando nuestro - ignorar
            } else if (opt->type == AshType::Nak) {
                dbg() << "  [NAK received]\n";
            }
        }
    }

    void sendAck() {
        sp_.writeAll(enc({static_cast<uint8_t>(0x80 | rxSeq_)}));
    }

    std::string stackStatusName(uint8_t st) {
        switch (st) {
            case 0x90: return "NETWORK_UP";
            case 0x9C: return "NETWORK_OPENED";
            case 0x91: return "STARTUP";
            case 0x92: return "DOWN";
            default: {
                std::ostringstream os;
                os << "0x" << std::hex << static_cast<int>(st) << std::dec;
                return os.str();
            }
        }
    }

    void processEzsp(const std::vector<uint8_t>& ez) {
        if ((!extHdr_ && ez.size() < 3) || (extHdr_ && ez.size() < 5)) return;

        uint16_t cmd;
        size_t po;
        if (extHdr_) { cmd = rle16(ez, 3); po = 5; }
        else { cmd = ez[2]; po = 3; }

        std::vector<uint8_t> pl(ez.begin() + po, ez.end());

        dbg() << "  [EZSP RX] cmd=0x" << std::hex << cmd << std::dec
              << " seq=" << static_cast<int>(ez[0])
              << " pl=" << hex(pl) << "\n";

        if (cmd == CB_STACK_STATUS && !pl.empty()) {
            uint8_t st = pl[0];
            dbg() << "  [stackStatus=" << stackStatusName(st) << "]\n";
            if (st == EMBER_NETWORK_UP) {
                netUp_ = true;
                std::cout << "  *** NETWORK_UP! ***\n";
            } else if (st == EMBER_NETWORK_OPENED) {
                std::cout << "  *** NETWORK OPEN FOR JOINING ***\n";
            }
        } else if (cmd == CB_TRUST_CENTER_JOIN) {
            if (pl.size() < 2) {
                dbg() << "  [TrustCenterJoin short payload] pl=" << hex(pl) << "\n";
            } else {
                uint16_t nwk = rle16(pl, 0);
                std::cout << "\n*** NUEVO DISPOSITIVO UNIDO ***"
                          << " NWK=0x" << std::hex << nwk << std::dec << "\n";
                bool found = false;
                for (auto d : devices_) if (d == nwk) found = true;
                if (!found) {
                    devices_.push_back(nwk);
                    dbg() << "  [Added to poll list]\n";
                }
            }
        } else if (cmd == CB_INCOMING_MESSAGE) {
            printIncomingMessage(pl);
        } else if (cmd == 0x003F) {  // messageSentHandler
            if (pl.size() >= 21) {
                uint32_t rawStatus = rle32(pl, 0);
                uint16_t dest = rle16(pl, 5);
                uint16_t cluster = rle16(pl, 9);
                uint8_t mlen = pl[20];
                dbg() << "\n  [MSG SENT] dst=0x" << std::hex << dest
                      << " cluster=0x" << cluster
                      << " status=0x" << rawStatus << std::dec;
                if (mlen > 0 && static_cast<size_t>(21) + mlen <= pl.size())
                    dbg() << " msg=" << hex(std::vector<uint8_t>(pl.begin()+21, pl.begin()+21+mlen));
                dbg() << "\n";
            } else if (pl.size() >= 17) {
                uint16_t dest = rle16(pl, 1);
                uint16_t cluster = rle16(pl, 5);
                uint8_t status = pl[15];
                dbg() << "\n  [MSG SENT] dst=0x" << std::hex << dest
                      << " cluster=0x" << cluster
                      << " status=0x" << static_cast<int>(status) << std::dec;
                if (pl.size() > 16) {
                    uint8_t mlen = pl[16];
                    if (mlen > 0) dbg() << " msg=" << hex(std::vector<uint8_t>(pl.begin()+17, pl.begin()+17+mlen));
                }
                dbg() << "\n";
            } else {
                dbg() << "  [messageSentHandler short] " << hex(pl) << "\n";
            }
        } else if (cmd == 0x0059) {
            uint16_t source = (pl.size() >= 2) ? rle16(pl, 0) : 0;
            uint8_t lqi = (pl.size() >= 11) ? pl[10] : 0;
            int8_t rssi = (pl.size() >= 12) ? static_cast<int8_t>(pl[11]) : 0;
            uint8_t rcount = (pl.size() > 12) ? pl[12] : 0;
            dbg() << "  [RouteRecord] src=0x" << std::hex << source << std::dec
                  << " LQI=" << static_cast<int>(lqi)
                  << " RSSI=" << static_cast<int>(rssi)
                  << " relays=" << static_cast<int>(rcount) << "\n";
        } else if (cmd == 0x0062) {
            if (pl.size() >= 8) {
                dbg() << "  [SenderEui64] " << hex(std::vector<uint8_t>(pl.begin(), pl.begin()+8)) << "\n";
            }
        } else {
            dbg() << "\n  [CB cmd=0x" << std::hex << cmd << std::dec 
                  << " len=" << pl.size() << " pl=" << hex(pl) << "]\n";
            
            if (cmd == 0x0023) {
                if (pl.size() >= 5) {
                    uint8_t index = pl[0];
                    bool joining = (pl[1] != 0);
                    uint16_t childId = rle16(pl, 2);
                    dbg() << "    [childJoinHandler] index=" << static_cast<int>(index)
                          << " joining=" << (joining ? "YES" : "NO")
                          << " childId=0x" << std::hex << childId << std::dec << "\n";
                }
            } else if (cmd == 0x0027) {
                if (pl.size() >= 6) {
                    uint16_t old_id = rle16(pl, 0);
                    uint16_t new_id = rle16(pl, 2);
                    uint8_t type = pl[4];
                    dbg() << "    [nodeIdChangedHandler] old=0x" << std::hex << old_id 
                          << " new=0x" << new_id << " type=0x" << static_cast<int>(type) << std::dec << "\n";
                }
            }
        }
    }

    void printIncomingMessage(const std::vector<uint8_t>& p) {
        if (p.size() < 12) {
            dbg() << "  [incomingMessage v13 corto] " << hex(p) << "\n";
            return;
        }
        uint16_t cluster = (p.size() > 4) ? rle16(p, 3) : 0;
        uint8_t lqi, sender_nwk_lo, sender_nwk_hi;
        lqi = (p.size() > 12) ? p[12] : 0;
        sender_nwk_lo = (p.size() > 14) ? p[14] : 0;
        sender_nwk_hi = (p.size() > 15) ? p[15] : 0;
        size_t msgOff = 18;
        int8_t rssi = (p.size() > 13) ? static_cast<int8_t>(p[13]) : 0;
        uint16_t sender = sender_nwk_lo | (static_cast<uint16_t>(sender_nwk_hi) << 8);
        uint8_t len = (msgOff < p.size()) ? p[msgOff] : 0;
        if (msgOff + 1 + len > p.size()) len = static_cast<uint8_t>(p.size() - msgOff - 1);
        std::vector<uint8_t> msg;
        if (msgOff + 1 <= p.size()) msg.assign(p.begin() + msgOff + 1, p.begin() + msgOff + 1 + len);

        dbg() << "\n*** MENSAJE RECIBIDO ***\n"
              << "  Cluster=0x" << std::hex << cluster
              << " Sender=0x" << sender << std::dec
              << " LQI=" << static_cast<int>(lqi)
              << " RSSI=" << static_cast<int>(rssi)
              << " fmt=v4"
              << "\n  Payload: " << hex(msg) << "\n";

        size_t hdr = zclHeaderSize(msg);
        if (hdr > 0 && msg.size() >= hdr) {
            uint8_t cmdId = msg[hdr - 1];
            if (cmdId == ZCL_READ_RSP) decodeReadRsp(cluster, sender, msg);
            else if (cmdId == ZCL_REPORT) decodeReport(cluster, sender, msg);
            else if (cmdId == ZCL_DEFAULT_RSP && msg.size() > hdr + 2)
                dbg() << "  [DefaultRsp status=0x" << std::hex << static_cast<int>(msg[hdr + 1]) << std::dec << "]\n";
        }
    }

    void decodeReadRsp(uint16_t cluster, uint16_t sender, const std::vector<uint8_t>& msg) {
        size_t o = zclHeaderSize(msg);
        std::optional<float> volt, amp, watt; std::optional<float> en; std::optional<bool> s;
        while (o + 3 <= msg.size()) {
            uint16_t id = rle16(msg, o);
            uint8_t st = msg[o + 2]; o += 3;
            if (st != 0x00 || o >= msg.size()) break;
            uint8_t dt = msg[o]; o++;
            size_t vs = zclValueSize(dt);
            if (vs == 0 || o + vs > msg.size()) break;

            if (cluster == ZCL_CLUSTER_ELEC_MEAS) {
                if (id == ATTR_AC_V_MULT && dt == 0x21 && vs >= 2) {
                    vMult_ = rle16(msg, o);
                    dbg() << "  [coeff] ac_voltage_multiplier=" << vMult_ << "\n";
                } else if (id == ATTR_AC_V_DIV && dt == 0x21 && vs >= 2) {
                    vDiv_ = rle16(msg, o);
                    dbg() << "  [coeff] ac_voltage_divisor=" << vDiv_ << "\n";
                } else if (id == ATTR_AC_I_MULT && dt == 0x21 && vs >= 2) {
                    iMult_ = rle16(msg, o);
                    dbg() << "  [coeff] ac_current_multiplier=" << iMult_ << "\n";
                } else if (id == ATTR_AC_I_DIV && dt == 0x21 && vs >= 2) {
                    iDiv_ = rle16(msg, o);
                    dbg() << "  [coeff] ac_current_divisor=" << iDiv_ << "\n";
                } else if (id == ATTR_AC_P_MULT && dt == 0x21 && vs >= 2) {
                    pMult_ = rle16(msg, o);
                    dbg() << "  [coeff] ac_power_multiplier=" << pMult_ << "\n";
                } else if (id == ATTR_AC_P_DIV && dt == 0x21 && vs >= 2) {
                    pDiv_ = rle16(msg, o);
                    dbg() << "  [coeff] ac_power_divisor=" << pDiv_ << "\n";
                } else if (id == ATTR_RMS_V && (dt == 0x21 || dt == 0x19) && vs >= 2) {
                    volt = rle16(msg, o) * vMult_ / static_cast<float>(vDiv_);
                } else if (id == ATTR_RMS_A && (dt == 0x21 || dt == 0x19) && vs >= 2) {
                    amp = rle16(msg, o) * iMult_ / static_cast<float>(iDiv_);
                } else if (id == ATTR_ACTIVE_W && dt == 0x29 && vs >= 2) {
                    watt = static_cast<float>(static_cast<int16_t>(rle16(msg, o))) * pMult_ / static_cast<float>(pDiv_ > 0 ? pDiv_ : 1);
                }
                o += vs;
                continue;
            } else if (cluster == ZCL_CLUSTER_METERING) {
                if (id == ATTR_MTR_MULT && vs >= 2) {
                    eMult_ = 0;
                    for (size_t i = 0; i < vs && i < 4; ++i)
                        eMult_ |= static_cast<uint32_t>(msg[o + i]) << (8 * i);
                    dbg() << "  [coeff] metering_multiplier=" << eMult_ << "\n";
                } else if (id == ATTR_MTR_DIV && vs >= 2) {
                    eDiv_ = 0;
                    for (size_t i = 0; i < vs && i < 4; ++i)
                        eDiv_ |= static_cast<uint32_t>(msg[o + i]) << (8 * i);
                    dbg() << "  [coeff] metering_divisor=" << eDiv_ << "\n";
                } else if (id == ATTR_ENERGY && (dt == 0x25 || dt == 0x23) && vs >= 6) {
                    uint64_t rv = 0;
                    for (size_t i = 0; i < vs; ++i)
                        rv |= static_cast<uint64_t>(msg[o + i]) << (8 * i);
                    en = static_cast<float>(rv) * eMult_ / static_cast<float>(eDiv_);
                }
                o += vs;
                continue;
            } else if (cluster == ZCL_CLUSTER_ONOFF && id == ATTR_ONOFF && dt == 0x10 && vs >= 1) {
                s = (msg[o] != 0);
                o += vs;
                continue;
            }
            break;
        }
        if (volt || amp || watt || en || s) {
            std::cout << "  >>> SMART PLUG 0x" << std::hex << sender << std::dec;
            if (volt) std::cout << " voltaje=" << *volt << "V";
            if (amp)  std::cout << " corriente=" << *amp << "A";
            if (watt) std::cout << " potencia=" << *watt << "W";
            if (en)   std::cout << " energia=" << *en << "kWh";
            if (s)    std::cout << " estado=" << (*s ? "ON" : "OFF");
            std::cout << "\n";
            csv_.log(sender, volt, amp, watt, en, s);
        }
    }

    size_t zclValueSize(uint8_t dt) {
        switch (dt) {
            case 0x10: case 0x18: case 0x20: case 0x28: return 1;
            case 0x19: case 0x21: case 0x29: return 2;
            case 0x1A: case 0x22: case 0x2A: return 3;
            case 0x1B: case 0x23: case 0x2B: case 0x39: return 4;
            case 0x24: case 0x2C: return 5;
            case 0x1D: case 0x25: case 0x2D: return 6;
            default: return 0;
        }
    }

    size_t zclHeaderSize(const std::vector<uint8_t>& msg) {
        if (msg.empty()) return 0;
        return (msg[0] & 0x04) ? 5 : 3;
    }

    void decodeReport(uint16_t cluster, uint16_t sender, const std::vector<uint8_t>& msg) {
        size_t o = zclHeaderSize(msg);
        std::optional<float> volt, amp, watt; std::optional<float> en; std::optional<bool> s;
        dbg() << "  [report cluster=0x" << std::hex << cluster << std::dec << "]\n";
        while (o + 2 <= msg.size()) {
            uint16_t id = rle16(msg, o); o += 2;
            if (o >= msg.size()) break;
            uint8_t dt = msg[o++];
            size_t vs = zclValueSize(dt);
            if (vs == 0 || o + vs > msg.size()) {
                dbg() << "    attr=0x" << std::hex << id
                      << " type=0x" << static_cast<int>(dt) << std::dec;
                if (o < msg.size())
                    dbg() << " value=" << hex(std::vector<uint8_t>(msg.begin() + o, msg.end()));
                dbg() << " [skip: vsize=" << vs << "]\n";
                break;
            }

            if (cluster == ZCL_CLUSTER_ELEC_MEAS) {
                if (id == ATTR_RMS_V && (dt == 0x21 || dt == 0x19) && vs >= 2) {
                    volt = rle16(msg, o) * vMult_ / static_cast<float>(vDiv_);
                } else if (id == ATTR_RMS_A && (dt == 0x21 || dt == 0x19) && vs >= 2) {
                    amp = rle16(msg, o) * iMult_ / static_cast<float>(iDiv_);
                } else if (id == ATTR_ACTIVE_W && dt == 0x29 && vs >= 2) {
                    watt = static_cast<float>(static_cast<int16_t>(rle16(msg, o))) * pMult_ / static_cast<float>(pDiv_ > 0 ? pDiv_ : 1);
                }
            } else if (cluster == ZCL_CLUSTER_ONOFF && id == ATTR_ONOFF && dt == 0x10 && vs >= 1) {
                s = (msg[o] != 0);
            } else if (cluster == ZCL_CLUSTER_METERING && id == ATTR_ENERGY && (dt == 0x25 || dt == 0x23) && vs >= 6) {
                uint64_t rv = 0;
                for (size_t i = 0; i < 6; ++i)
                    rv |= static_cast<uint64_t>(msg[o + i]) << (8 * i);
                en = static_cast<float>(rv) * eMult_ / static_cast<float>(eDiv_);
            }

            dbg() << "    attr=0x" << std::hex << id
                  << " type=0x" << static_cast<int>(dt) << std::dec
                  << " value=" << hex(std::vector<uint8_t>(msg.begin() + o, msg.begin() + o + vs)) << "\n";
            o += vs;
        }
        if (volt || amp || watt || en || s) {
            std::cout << "  >>> SMART PLUG 0x" << std::hex << sender << std::dec;
            if (volt) std::cout << " voltaje=" << *volt << "V";
            if (amp)  std::cout << " corriente=" << *amp << "A";
            if (watt) std::cout << " potencia=" << *watt << "W";
            if (en)   std::cout << " energia=" << *en << "kWh";
            if (s)    std::cout << " estado=" << (*s ? "ON" : "OFF");
            std::cout << "\n";
            csv_.log(sender, volt, amp, watt, en, s);
        }
    }

    bool formNetwork() {
        auto euiRsp = cmd(CMD_GET_EUI64, {}, 3000);
        if (!euiRsp || euiRsp->payload.size() < 8) {
            std::cerr << "formNetwork: no se pudo leer EUI64 del NCP\n";
            return false;
        }

        std::vector<uint8_t> trustCenterEui(euiRsp->payload.begin(), euiRsp->payload.begin() + 8);
        std::vector<uint8_t> sec;
        le16(sec, 0x1047);
        const char* wellKnownKey = "ZigBeeAlliance09";
        sec.insert(sec.end(), wellKnownKey, wellKnownKey + 16);
        auto netKey = prand(16);
        sec.insert(sec.end(), netKey.begin(), netKey.end());
        sec.push_back(0x00);
        sec.insert(sec.end(), trustCenterEui.begin(), trustCenterEui.end());

        auto secRsp = cmd(CMD_SET_INITIAL_SECURITY_STATE, sec, 5000);
        if (!secRsp || secRsp->payload.empty() || secRsp->payload[0] != 0x00) {
            std::cerr << "setInitialSecurityState fallo\n";
            return false;
        }
        dbg() << "setInitialSecurityState status=0x" << std::hex
              << static_cast<int>(secRsp->payload[0]) << std::dec << "\n";

        std::vector<uint8_t> form;
        auto extPan = prand(8);
        form.insert(form.end(), extPan.begin(), extPan.end());
        le16(form, 0x2D3F);
        form.push_back(8);
        form.push_back(26);
        form.push_back(0x00);
        le16(form, 0x0000);
        form.push_back(1);
        form.push_back(0x00);
        form.push_back(0x00);
        form.push_back(0x00);
        form.push_back(0x04);

        auto formRsp = cmd(CMD_FORM_NETWORK, form, 10000);
        if (!formRsp || formRsp->payload.empty() || formRsp->payload[0] != 0x00) {
            std::cerr << "formNetwork fallo\n";
            return false;
        }
        dbg() << "formNetwork status=0x" << std::hex
              << static_cast<int>(formRsp->payload[0]) << std::dec << "\n";

        usleep(500000);
        auto ns = cmd(CMD_NETWORK_STATE, {}, 3000);
        if (ns && !ns->payload.empty() && ns->payload[0] == 0x02) {
            netUp_ = true;
            std::cout << "  Network formada y JOINED_NETWORK.\n";
            return true;
        }

        std::cerr << "formNetwork: la red no entró en JOINED_NETWORK\n";
        return false;
    }

    // ---- EZSP command (copia de v2_zigbee_reader.cpp) ----
    struct EzspRsp { uint8_t seq = 0; uint16_t cmd = 0; std::vector<uint8_t> payload; };

    std::optional<EzspRsp> cmd(uint16_t code, const std::vector<uint8_t>& pl, int ms) {
        uint8_t seq = ezspSeq_++;
        std::vector<uint8_t> ez;
        ez.push_back(seq); ez.push_back(0x00);
        if (extHdr_) { ez.push_back(0x01); le16(ez, code); }
        else ez.push_back(static_cast<uint8_t>(code & 0xFF));
        ez.insert(ez.end(), pl.begin(), pl.end());

        uint8_t f = txSeq_;
        txSeq_ = (txSeq_ + 1) & 0x07;

        // TX con hasta 3 intentos
        for (int a = 0; a < 3 && !g_stop; ++a) {
            // Enviar frame
            auto rnd = arand(ez);
            std::vector<uint8_t> ashPl;
            ashPl.push_back(static_cast<uint8_t>((f << 4) | (a > 0 ? 0x08 : 0x00) | rxSeq_));
            ashPl.insert(ashPl.end(), rnd.begin(), rnd.end());
            sp_.writeAll(enc(ashPl));

            // Esperar respuesta
            int64_t dl = nowMs() + ms;
            while (nowMs() < dl && !g_stop) {
                auto fr = readRaw(100);
                if (!fr) continue;
                if (fr->type == AshType::Ack) {
                    continue;
                }
                if (fr->type == AshType::Nak) break;
                if (fr->type == AshType::Data) {
                    if (fr->frm == rxSeq_) {
                        rxSeq_ = (rxSeq_ + 1) & 0x07;
                        sendAck();
                        // parsear EZSP
                        auto rsp = parseEzsp(fr->ezsp, seq);
                        if (rsp) {
                            if (rsp->cmd == code) return rsp;
                            processEzsp(fr->ezsp);
                        }
                    } else {
                        sendAck(); // duplicado
                    }
                }
            }
        }
        return std::nullopt;
    }

    std::optional<EzspRsp> parseEzsp(const std::vector<uint8_t>& ez, std::optional<uint8_t> ws) {
        if ((!extHdr_ && ez.size() < 3) || (extHdr_ && ez.size() < 5)) return std::nullopt;
        EzspRsp r;
        r.seq = ez[0];
        size_t po;
        if (extHdr_) { r.cmd = rle16(ez, 3); po = 5; }
        else { r.cmd = ez[2]; po = 3; }
        r.payload.assign(ez.begin() + po, ez.end());

        if (ws && r.seq == *ws) {
            dbg() << "[RSP cmd=0x" << std::hex << r.cmd << std::dec
                  << " seq=" << static_cast<int>(r.seq)
                  << " pl=" << hex(r.payload) << "]\n";
            return r;
        }
        // Callback no esperado - procesar igual
        processEzsp(ez);
        return std::nullopt;
    }

    void commandNoFail(uint16_t code, const std::vector<uint8_t>& payload, const std::string& label) {
        auto rsp = cmd(code, payload, 3000);
        if (!rsp) {
            dbg() << label << ": sin respuesta, continuo\n";
            return;
        }
        if (!rsp->payload.empty()) {
            dbg() << label << " status/respuesta: " << hex(rsp->payload) << "\n";
        } else {
            dbg() << label << " OK\n";
        }
    }

    void setPoliciesForJoining() {
        // Configure all three security policies that bellows configures
        // POLICY 0x00 = TRUST_CENTER_POLICY
        // Value 0x03 = ALLOW_JOINS | ALLOW_UNSECURED_REJOINS
        cmd(CMD_SET_POLICY, {0x00, 0x03}, 3000);
        dbg() << "  [TRUST_CENTER_POLICY set to ALLOW_JOINS | ALLOW_UNSECURED_REJOINS]\n";
        cmd(CMD_SET_POLICY, {0x05, 0x51}, 3000);
        dbg() << "  [TC_KEY_REQUEST_POLICY set to ALLOW_TC_KEY_REQUESTS_AND_SEND_CURRENT_KEY]\n";
        cmd(CMD_SET_POLICY, {0x06, 0x60}, 3000);
        dbg() << "  [APP_KEY_REQUEST_POLICY set to DENY_APP_KEY_REQUESTS]\n";
    }

    void addTransientTcLinkKey() {
        std::vector<uint8_t> p(8, 0xFF);  // wildcard IEEE
        const char key[] = "ZigBeeAlliance09";
        p.insert(p.end(), key, key + 16);

        if (ezspVer_ >= 13) {
            p.push_back(0x00);  // SecurityManagerContextFlags.NONE
            commandNoFail(CMD_IMPORT_TRANSIENT_KEY, p, "importTransientKey(wildcard)");
        } else {
            commandNoFail(CMD_ADD_TRANSIENT_LINK_KEY, p, "addTransientLinkKey(wildcard)");
        }
    }

    std::vector<uint8_t> endpoint1Payload() {
        std::vector<uint8_t> p;
        p.push_back(1);          // endpoint
        le16(p, 0x0104);        // ZHA profile
        le16(p, 0x0400);        // IAS_CONTROL
        p.push_back(0);          // device version

        const uint16_t inClusters[] = {0x0000, 0x0006, 0x000A, 0x0019, 0x0501};
        const uint16_t outClusters[] = {0x0001, 0x0020, 0x0500, 0x0502};

        p.push_back(sizeof(inClusters) / sizeof(inClusters[0]));
        p.push_back(sizeof(outClusters) / sizeof(outClusters[0]));
        for (uint16_t c : inClusters) le16(p, c);
        for (uint16_t c : outClusters) le16(p, c);
        return p;
    }

    std::ostream& dbg() { return verbose_ ? std::cout : nullOut_; }

public:
    void setVerbose() { verbose_ = true; }
    AshEzsp(SerialPort& sp, CsvLogger& csv) : sp_(sp), csv_(csv) {}

    // ASH layer ----------------------------------------------------------------
    std::vector<uint8_t> enc(std::vector<uint8_t> p) {
        uint16_t c = crcCcitt(p.data(), p.size());
        p.push_back((c >> 8) & 0xFF); p.push_back(c & 0xFF);
        std::vector<uint8_t> o;
        for (uint8_t b : p) {
            if (isReserved(b)) { o.push_back(ASH_ESCAPE); o.push_back(b ^ 0x20); }
            else o.push_back(b);
        }
        o.push_back(ASH_FLAG);
        return o;
    }

    std::optional<AshFrame> readRaw(int ms) {
        int64_t dl = nowMs() + ms;
        while (nowMs() < dl) {
            uint8_t b[128];
            ssize_t n = sp_.readSome(b, sizeof(b), static_cast<int>(dl - nowMs()));
            if (n <= 0) continue;
            for (ssize_t i = 0; i < n; ++i) {
                uint8_t v = b[i];
                if (v == ASH_CANCEL) { rx_.clear(); esc_ = false; continue; }
                if (v == ASH_FLAG) {
                    if (rx_.empty()) continue;
                    auto f = parseRaw(rx_); rx_.clear(); esc_ = false;
                    if (f) return f;
                    continue;
                }
                if (esc_) { rx_.push_back(v ^ 0x20); esc_ = false; }
                else if (v == ASH_ESCAPE) esc_ = true;
                else if (v == ASH_XON || v == ASH_XOFF) continue;
                else rx_.push_back(v);
            }
        }
        return std::nullopt;
    }

    std::optional<AshFrame> parseRaw(const std::vector<uint8_t>& raw) {
        if (raw.size() < 3) return std::nullopt;
        // Verificar CRC
        uint16_t got = static_cast<uint16_t>((raw[raw.size()-2] << 8) | raw[raw.size()-1]);
        uint16_t want = crcCcitt(raw.data(), raw.size()-2);
        if (got != want) return std::nullopt;

        AshFrame f; f.raw = raw; f.ctrl = raw[0];
        if ((f.ctrl & 0x80) == 0x00) {
            f.type = AshType::Data;
            f.frm = (f.ctrl >> 4) & 0x07;
            f.reTx = (f.ctrl & 0x08) != 0;
            auto rnd = std::vector<uint8_t>(raw.begin() + 1, raw.end() - 2);
            f.ezsp = arand(rnd);
        } else if ((f.ctrl & 0xE0) == 0x80) f.type = AshType::Ack;
        else if ((f.ctrl & 0xE0) == 0xA0) f.type = AshType::Nak;
        else if (f.ctrl == 0xC1) f.type = AshType::Rstack;
        else if (f.ctrl == 0xC2) f.type = AshType::Error;
        return f;
    }

    static int64_t nowMs() {
        timeval tv{}; gettimeofday(&tv, nullptr);
        return static_cast<int64_t>(tv.tv_sec) * 1000 + tv.tv_usec / 1000;
    }

    // NCP Configuration -------------------------------------------------------
    // Equivalente a bellows write_config() - configura valores esenciales del NCP
    void setConfigValue(uint8_t configId, uint16_t value, const std::string& label) {
        auto r = cmd(CMD_SET_CONFIGURATION_VALUE, {configId, static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)}, 3000);
        if (r && !r->payload.empty())
            dbg() << "  [CFG " << label << " id=0x" << std::hex << static_cast<int>(configId) << std::dec << " val=" << value << " status=0x" << std::hex << static_cast<int>(r->payload[0]) << std::dec << "]\n";
        else
            dbg() << "  [CFG " << label << " id=0x" << std::hex << static_cast<int>(configId) << std::dec << " failed]\n";
    }

    void writeNcpConfig() {
        dbg() << "\n  [Configurando NCP (write_config equivalent)]...\n";
        setConfigValue(0x0C, 2, "STACK_PROFILE");       // ZigBee Pro
        setConfigValue(0x0D, 5, "SECURITY_LEVEL");       // Enc+Auth
        setConfigValue(0x11, 32, "MAX_END_DEVICE_CHILDREN");
        setConfigValue(0x12, 7680, "INDIRECT_TX_TIMEOUT");
        setConfigValue(0x13, 8, "END_DEVICE_POLL_TIMEOUT");
        setConfigValue(0x1A, 200, "SOURCE_ROUTE_TABLE_SIZE");
        setConfigValue(0x1E, 4, "KEY_TABLE_SIZE");
        setConfigValue(0x22, 2, "PAN_ID_CONFLICT_REPORT_THRESHOLD");
        setConfigValue(0x2A, 0x03, "APPLICATION_ZDO_FLAGS");  // APP_RECEIVES_SUPPORTED_ZDO_REQUESTS | APP_HANDLES_UNSUPPORTED_ZDO_REQUESTS
        setConfigValue(0x2D, 1, "SUPPORTED_NETWORKS");
        setConfigValue(0x06, 16, "MULTICAST_TABLE_SIZE");
        setConfigValue(0x19, 2, "TC_ADDRESS_CACHE_SIZE");
        setConfigValue(0x05, 16, "ADDRESS_TABLE_SIZE");
        setConfigValue(0x38, 90, "TC_REJOINS_WELL_KNOWN_KEY_TIMEOUT");
        // CONFIG_PACKET_BUFFER_COUNT debe ser el ultimo
        setConfigValue(0x01, 255, "PACKET_BUFFER_COUNT");
    }

    // Init ---------------------------------------------------------------------
    bool init() {
        // Obtener EUI64
        auto e = cmd(CMD_GET_EUI64, {}, 3000);
        if (e && e->payload.size() >= 8) {
            dbg() << "EUI64=";
            for (int i = 7; i >= 0; --i) {
                dbg() << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(e->payload[i]);
                if (i > 0) dbg() << ":";
            }
            dbg() << std::dec << "\n";
        }
        
        commandNoFail(CMD_ADD_ENDPOINT, endpoint1Payload(), "addEndpoint(1)");

        // CONFIGURAR NCP (como bellows write_config)
        writeNcpConfig();

        // INICIALIZAR LA RED primero
        auto ni = cmd(CMD_NETWORK_INIT, {0x00, 0x00}, 5000);
        if (!ni) { std::cerr << "networkInit failed\n"; return false; }
        dbg() << "networkInit status=0x" << std::hex << static_cast<int>(ni->payload.empty() ? 0xFF : ni->payload[0]) << std::dec << "\n";

        dbg() << "  Esperando NETWORK_UP (10 segundos)...\n";
        processEvents(10000);
        
        // Configurar Politicas y llaves DESPUÉS de que la red esté arriba (como bellows)
        addTransientTcLinkKey();
        setPoliciesForJoining();

        // Verificar networkState
        auto ns = cmd(CMD_NETWORK_STATE, {}, 3000);
        if (ns && !ns->payload.empty()) {
            dbg() << "networkState=0x" << std::hex << static_cast<int>(ns->payload[0])
                  << std::dec << " (0x02=JOINED)\n";
            if (ns->payload[0] == 0x02) netUp_ = true;
        }

        if (!netUp_) {
            dbg() << "  [No hay red. Intentando formar nueva red Zigbee...]\n";
            if (!formNetwork()) {
                std::cerr << "  Error: no se pudo formar la red Zigbee.\n";
                return false;
            }
        } else {
            dbg() << "  Network IS UP.\n";
        }

        auto nid = cmd(CMD_GET_NODE_ID, {}, 3000);
        if (nid && nid->payload.size() >= 2) {
            myNwk_ = rle16(nid->payload, 0);
            std::cout << "My NWK=0x" << std::hex << myNwk_ << std::dec << "\n";
        }

        auto np = cmd(CMD_GET_NETWORK_PARAMETERS, {}, 3000);
        if (np && np->payload.size() >= 21) {
            uint16_t pan = rle16(np->payload, 1);
            uint8_t ch = np->payload[13];
            std::cout << "PAN=0x" << std::hex << pan << std::dec << " ch=" << static_cast<int>(ch) << "\n";
        }

        dbg() << "\n  [Applying pre_permit security mode]...\n";
        setPoliciesForJoining();
        sleep(2);

        dbg() << "\n  [ZDO Mgmt_Permit_Joining_req broadcast to 0xFFFC]...\n";
        sendZdoBroadcast(0x0036, {0x00, 180, 0x00}, "Mgmt_Permit_Joining_req");

        dbg() << "\n  [Enviando permitJoining(180)]...\n";
        auto perm = cmd(CMD_PERMIT_JOINING, {180}, 3000);
        if (perm && !perm->payload.empty()) {
            dbg() << "  permitJoining response: " << hex(perm->payload) << "\n";
        }

        std::cout << "\n  Red abierta por 180 segundos.\n";
        std::cout << "  Presiona boton pairing del smart plug (~5s).\n";
        dbg() << "  Esperando callbacks de emparejamiento por 30 segundos...\n";
        dbg() << "  [VERBOSE: logging todos los callbacks]...\n";
        processEvents(30000);

        cmd(CMD_SET_POLICY, {0x00, 0x01}, 3000);
        dbg() << "  [TRUST_CENTER_POLICY restored to ALLOW_PRECONFIGURED_KEY_JOINS]\n";

        return true;
    }

    // Polling -----------------------------------------------------------------
    void setupDevice(uint16_t nwk) {
        if (setupDone_) return;
        dbg() << "  [Setting up device 0x" << std::hex << nwk << std::dec << "]\n";
        // 1. Read EM coefficients first (so measurements are decoded correctly)
        {
            std::vector<uint8_t> z;
            z.push_back(0x00); z.push_back(++pollSeq_); z.push_back(ZCL_READ);
            le16(z, ATTR_AC_V_MULT); le16(z, ATTR_AC_V_DIV);
            le16(z, ATTR_AC_I_MULT); le16(z, ATTR_AC_I_DIV);
            le16(z, ATTR_AC_P_MULT); le16(z, ATTR_AC_P_DIV);
            sendZclEp(nwk, ZCL_CLUSTER_ELEC_MEAS, 1, z, "CoeffEM");
        }
        processEvents(2000);
        // 2. Read EM measurements (coefficients now loaded)
        {
            std::vector<uint8_t> z;
            z.push_back(0x00); z.push_back(++pollSeq_); z.push_back(ZCL_READ);
            le16(z, ATTR_RMS_V); le16(z, ATTR_RMS_A); le16(z, ATTR_ACTIVE_W);
            sendZclEp(nwk, ZCL_CLUSTER_ELEC_MEAS, 1, z, "MeasEM");
        }
        processEvents(2000);
        // 3. Read OnOff state
        {
            std::vector<uint8_t> z;
            z.push_back(0x00); z.push_back(++pollSeq_); z.push_back(ZCL_READ);
            le16(z, ATTR_ONOFF);
            sendZclEp(nwk, ZCL_CLUSTER_ONOFF, 1, z, "OnOff");
        }
        processEvents(1000);
        setupDone_ = true;
    }

    void pollOnOff(uint16_t nwk) {
        std::vector<uint8_t> z;
        z.push_back(0x00); z.push_back(++pollSeq_); z.push_back(ZCL_READ);
        le16(z, ATTR_ONOFF);
        sendZclEp(nwk, ZCL_CLUSTER_ONOFF, 1, z, "OnOff");
    }

    void pollMeasurements(uint16_t nwk) {
        std::vector<uint8_t> z;
        z.push_back(0x00); z.push_back(++pollSeq_); z.push_back(ZCL_READ);
        le16(z, ATTR_RMS_V); le16(z, ATTR_RMS_A); le16(z, ATTR_ACTIVE_W);
        sendZclEp(nwk, ZCL_CLUSTER_ELEC_MEAS, 1, z, "MeasEM");
    }

    void pollAll() {
        for (auto nwk : devices_) {
            dbg() << "\n[Polling 0x" << std::hex << nwk << std::dec << "]\n";
            setupDevice(nwk);
            if (setupDone_) {
                // Refresh coefficients then read measurements
                std::vector<uint8_t> z;
                z.push_back(0x00); z.push_back(++pollSeq_); z.push_back(ZCL_READ);
                le16(z, ATTR_AC_V_MULT); le16(z, ATTR_AC_V_DIV);
                le16(z, ATTR_AC_I_MULT); le16(z, ATTR_AC_I_DIV);
                le16(z, ATTR_AC_P_MULT); le16(z, ATTR_AC_P_DIV);
                sendZclEp(nwk, ZCL_CLUSTER_ELEC_MEAS, 1, z, "CoeffEM");
                processEvents(1500);
                pollMeasurements(nwk);
                processEvents(1500);
                pollOnOff(nwk);
                processEvents(1000);
            }
        }
    }

    void sendZcl(uint16_t nwk, uint16_t cluster, const std::vector<uint8_t>& zcl, const std::string& label) {
        std::vector<uint8_t> pl;
        pl.push_back(0x00); // mode = DIRECT
        le16(pl, nwk);
        le16(pl, 0x0104); // ZHA profile
        le16(pl, cluster);
        pl.push_back(1); // srcEp
        pl.push_back(1); // dstEp
        le16(pl, 0x0140);              // options (ENABLE_ROUTE_DISCOVERY | RETRY)
        le16(pl, 0x0000);              // groupId
        pl.push_back(apsSeq_++);        // sequence
        pl.push_back(static_cast<uint8_t>(msgTag_++)); // message_tag (1 byte, v4 format)
        pl.push_back(static_cast<uint8_t>(zcl.size()));
        pl.insert(pl.end(), zcl.begin(), zcl.end());
        auto r = cmd(CMD_SEND_UNICAST, pl, 2000);
        if (r && !r->payload.empty())
            dbg() << "  " << label << " status=0x" << std::hex << static_cast<int>(r->payload[0]) << std::dec << "\n";
        else if (!r)
            dbg() << "  " << label << " TIMEOUT/NO RESPONSE\n";
    }

    bool sendZclEp(uint16_t nwk, uint16_t cluster, uint8_t dstEp, const std::vector<uint8_t>& zcl, const std::string& label) {
        std::vector<uint8_t> pl;
        pl.push_back(0x00);
        le16(pl, nwk);
        le16(pl, 0x0104);
        le16(pl, cluster);
        pl.push_back(1);
        pl.push_back(dstEp);
        le16(pl, 0x0140);              // options (ENABLE_ROUTE_DISCOVERY | RETRY)
        le16(pl, 0x0000);              // groupId
        pl.push_back(apsSeq_++);        // sequence
        pl.push_back(static_cast<uint8_t>(msgTag_++)); // message_tag (1 byte, v4 format)
        pl.push_back(static_cast<uint8_t>(zcl.size()));
        pl.insert(pl.end(), zcl.begin(), zcl.end());
        auto r = cmd(CMD_SEND_UNICAST, pl, 2000);
        if (r && !r->payload.empty()) {
            dbg() << "  " << label << " (ep" << static_cast<int>(dstEp) << ") status=0x"
                  << std::hex << static_cast<int>(r->payload[0]) << std::dec << "\n";
            return true;
        }
        dbg() << "  " << label << " (ep" << static_cast<int>(dstEp) << ") TIMEOUT/NO RESPONSE\n";
        return false;
    }

    void configureDeviceReporting(uint16_t nwk) {
        dbg() << "  [Configuring reporting for 0x" << std::hex << nwk << std::dec << "]\n";
        uint8_t seq = ++pollSeq_;
        // ElectricalMeasurement reporting (ep1 only)
        {
            std::vector<uint8_t> z;
            z.push_back(0x00); z.push_back(seq); z.push_back(ZCL_CONFIGURE_REPORTING);
            z.push_back(0x00); z.push_back(0x05); z.push_back(0x05); z.push_back(0x21);
            z.push_back(0x0A); z.push_back(0x00); z.push_back(0x3C); z.push_back(0x00);
            z.push_back(0x01); z.push_back(0x00);
            z.push_back(0x00); z.push_back(0x05); z.push_back(0x08); z.push_back(0x21);
            z.push_back(0x0A); z.push_back(0x00); z.push_back(0x3C); z.push_back(0x00);
            z.push_back(0x01); z.push_back(0x00);
            z.push_back(0x00); z.push_back(0x05); z.push_back(0x0B); z.push_back(0x29);
            z.push_back(0x0A); z.push_back(0x00); z.push_back(0x3C); z.push_back(0x00);
            z.push_back(0x01); z.push_back(0x00);
            sendZclEp(nwk, ZCL_CLUSTER_ELEC_MEAS, 1, z, "CfgRepEM");
        }
        usleep(300000);
        // Metering reporting (ep1 only)
        {
            std::vector<uint8_t> z;
            z.push_back(0x00); z.push_back(seq); z.push_back(ZCL_CONFIGURE_REPORTING);
            z.push_back(0x00); z.push_back(0x00); z.push_back(0x00); z.push_back(0x25);
            z.push_back(0x0A); z.push_back(0x00); z.push_back(0x2C); z.push_back(0x01);
            z.push_back(0x01); z.push_back(0x00); z.push_back(0x00); z.push_back(0x00);
            z.push_back(0x00); z.push_back(0x00);
            sendZclEp(nwk, ZCL_CLUSTER_METERING, 1, z, "CfgRepMtr");
        }
        usleep(300000);
        // OnOff reporting (ep1 only)
        {
            std::vector<uint8_t> z;
            z.push_back(0x00); z.push_back(seq); z.push_back(ZCL_CONFIGURE_REPORTING);
            z.push_back(0x00); z.push_back(0x00); z.push_back(0x00); z.push_back(0x10);
            z.push_back(0x0A); z.push_back(0x00); z.push_back(0x3C); z.push_back(0x00);
            z.push_back(0x01);
            sendZclEp(nwk, ZCL_CLUSTER_ONOFF, 1, z, "CfgRepOn");
        }
    }

    void sendZdoBroadcast(uint16_t cluster, const std::vector<uint8_t>& payload, const std::string& label) {
        std::vector<uint8_t> pl;
        if (extHdr_) {
            // EZSP v14: alias(2) + destination(2) + sequence(1) + apsFrame(11) + radius(1) + message_tag(2) + msg
            le16(pl, 0x0000); // alias
            le16(pl, 0xFFFC); // destination = ALL_ROUTERS_AND_COORDINATOR
            pl.push_back(0x00); // sequence
        } else {
            // EZSP v4: destination(2) + apsFrame(11) + radius(1) + messageTag(1) + msg
            le16(pl, 0xFFFC); // destination = ALL_ROUTERS_AND_COORDINATOR
        }
        le16(pl, 0x0000); // apsFrame.profileId = ZDO
        le16(pl, cluster);
        pl.push_back(0x00); // apsFrame.sourceEndpoint = ZDO
        pl.push_back(0x00); // apsFrame.destinationEndpoint = ZDO
        le16(pl, 0x0000); // apsFrame.options
        le16(pl, 0x0000); // apsFrame.groupId
        pl.push_back(0x00); // apsFrame.sequence
        pl.push_back(0x00); // radius (0 = max hops)
        if (extHdr_) {
            le16(pl, 0x0000); // message_tag (2 bytes for EZSP v14)
        } else {
            pl.push_back(0x00); // messageTag (1 byte for EZSP v4)
        }
        pl.push_back(static_cast<uint8_t>(payload.size()));
        pl.insert(pl.end(), payload.begin(), payload.end());
        auto r = cmd(CMD_SEND_BROADCAST, pl, 5000);
        if (r && !r->payload.empty())
            dbg() << "  " << label << " status=0x" << std::hex << static_cast<int>(r->payload[0]) << std::dec << "\n";
    }

    void loop() {
        dbg() << "\n=== ESCUCHANDO MENSAJES ZIGBEE ===\n";
        std::cout << "Ctrl+C para salir.\n\n";
        int64_t lastPoll = 0;
        while (!g_stop) {
            // Procesar eventos entrantes
            processEvents(1000);

            // Polling cada 5s si hay dispositivos
            int64_t now = nowMs();
            if (!devices_.empty() && now - lastPoll >= 5000) {
                pollAll();
                lastPoll = now;
            }
        }
    }
    
    bool reset() {
        std::vector<uint8_t> pre(32, ASH_CANCEL);
        auto r = enc({0xC0});
        pre.insert(pre.end(), r.begin(), r.end());
        sp_.writeAll(pre);
        int64_t dl = nowMs() + 3000;
        while (nowMs() < dl) {
            auto f = readRaw(200);
            if (f && f->type == AshType::Rstack) {
                dbg() << "RSTACK ASH v" << static_cast<int>(f->raw[1])
                      << " reset=0x" << std::hex << static_cast<int>(f->raw[2]) << std::dec << "\n";
                txSeq_ = rxSeq_ = ezspSeq_ = 0;
                return true;
            }
        }
        std::cerr << "No RSTACK\n"; return false;
    }

    bool negotiate() {
        extHdr_ = false;
        auto r = cmd(CMD_VERSION, {4}, 3000);
        if (!r || r->payload.size() < 4) return false;
        ezspVer_ = r->payload[0];
        std::cout << "EZSP v" << static_cast<int>(ezspVer_)
                  << " stack=0x" << std::hex << rle16(r->payload, 2) << std::dec << "\n";
        if (ezspVer_ >= 8) {
            extHdr_ = true;
            auto r2 = cmd(CMD_VERSION, {ezspVer_}, 3000);
            if (!r2) return false;
        }
        return true;
    }
};

} // namespace

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--verbose" || a == "-v") continue;
        if (!a.empty() && a[0] != '-') port = a;
    }
    std::signal(SIGINT, onSignal); std::signal(SIGTERM, onSignal);

    SerialPort sp(port);
    if (!sp.ok()) return 1;
    std::cout << "Port: " << port << " @115200\n";

    CsvLogger csv;
    AshEzsp app(sp, csv);
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--verbose" || a == "-v") app.setVerbose();
    }
    if (!app.reset()) return 1;
    if (!app.negotiate()) return 1;
    if (!app.init()) return 1;
    app.loop();
    return 0;
}
