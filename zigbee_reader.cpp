#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

// Minimal EZSP-over-ASH reader for Silicon Labs EmberZNet NCPs.
// Target: Raspberry Pi + /dev/ttyUSB0 + EFR32MG21 NCP firmware.

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
constexpr uint16_t CMD_ADD_ENDPOINT = 0x0002;
constexpr uint16_t CMD_NETWORK_INIT = 0x0017;
constexpr uint16_t CMD_NETWORK_STATE = 0x0018;
constexpr uint16_t CMD_SET_POLICY = 0x0055;
constexpr uint16_t CMD_PERMIT_JOINING = 0x0022;
constexpr uint16_t CMD_ADD_TRANSIENT_LINK_KEY = 0x00AF;
constexpr uint16_t CMD_IMPORT_TRANSIENT_KEY = 0x0111;
constexpr uint16_t CMD_GET_NETWORK_PARAMETERS = 0x0028;
constexpr uint16_t CMD_FORM_NETWORK = 0x001E;
constexpr uint16_t CMD_SET_INITIAL_SECURITY_STATE = 0x0068;
constexpr uint16_t CMD_GET_EUI64 = 0x0026;
constexpr uint16_t CMD_SEND_UNICAST = 0x0034;

constexpr uint16_t CB_STACK_STATUS = 0x0019;
constexpr uint16_t CB_INCOMING_MESSAGE = 0x0045;
constexpr uint16_t CB_TRUST_CENTER_JOIN = 0x0024;

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) {
    g_stop = 1;
}

uint16_t crcCcitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool isReserved(uint8_t b) {
    return b == ASH_FLAG || b == ASH_ESCAPE || b == ASH_XON || b == ASH_XOFF ||
           b == ASH_SUBSTITUTE || b == ASH_CANCEL;
}

std::vector<uint8_t> pseudoRandom(size_t len) {
    std::vector<uint8_t> out;
    out.reserve(len);
    uint8_t rand = 0x42;
    for (size_t i = 0; i < len; ++i) {
        out.push_back(rand);
        rand = (rand & 0x01) ? static_cast<uint8_t>((rand >> 1) ^ 0xB8)
                             : static_cast<uint8_t>(rand >> 1);
    }
    return out;
}

std::vector<uint8_t> ashRandomize(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> rnd = pseudoRandom(data.size());
    std::vector<uint8_t> out(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        out[i] = data[i] ^ rnd[i];
    }
    return out;
}

void appendLe16(std::vector<uint8_t>& out, uint16_t v) {
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

uint16_t readLe16(const std::vector<uint8_t>& data, size_t off) {
    return static_cast<uint16_t>(data[off] | (data[off + 1] << 8));
}

std::string hex(const std::vector<uint8_t>& data) {
    std::ostringstream os;
    for (size_t i = 0; i < data.size(); ++i) {
        if (i) os << ' ';
        os << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
    }
    return os.str();
}

std::string eui64String(const std::vector<uint8_t>& data, size_t off) {
    std::ostringstream os;
    for (int i = 7; i >= 0; --i) {
        if (i != 7) os << ':';
        os << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[off + i]);
    }
    return os.str();
}

class SerialPort {
public:
    explicit SerialPort(const std::string& path) {
        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "No se pudo abrir " << path << ": " << std::strerror(errno) << "\n";
            return;
        }

        termios tio{};
        if (tcgetattr(fd_, &tio) != 0) {
            std::cerr << "tcgetattr fallo: " << std::strerror(errno) << "\n";
            close(fd_);
            fd_ = -1;
            return;
        }

        cfmakeraw(&tio);
        cfsetispeed(&tio, kBaud);
        cfsetospeed(&tio, kBaud);
        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
            std::cerr << "tcsetattr fallo: " << std::strerror(errno) << "\n";
            close(fd_);
            fd_ = -1;
            return;
        }

        tcflush(fd_, TCIOFLUSH);
    }

    ~SerialPort() {
        if (fd_ >= 0) close(fd_);
    }

    bool ok() const { return fd_ >= 0; }

    bool writeAll(const std::vector<uint8_t>& data) {
        size_t off = 0;
        while (off < data.size()) {
            ssize_t n = ::write(fd_, data.data() + off, data.size() - off);
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN) continue;
                std::cerr << "write fallo: " << std::strerror(errno) << "\n";
                return false;
            }
            off += static_cast<size_t>(n);
        }
        tcdrain(fd_);
        return true;
    }

    ssize_t readSome(uint8_t* buf, size_t len, int timeoutMs) {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd_, &set);
        timeval tv{};
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        int rc = select(fd_ + 1, &set, nullptr, nullptr, &tv);
        if (rc < 0) {
            if (errno == EINTR) return 0;
            std::cerr << "select fallo: " << std::strerror(errno) << "\n";
            return -1;
        }
        if (rc == 0) return 0;
        return ::read(fd_, buf, len);
    }

private:
    int fd_ = -1;
};

enum class AshType { Data, Ack, Nak, Rstack, Error, Unknown };

struct AshFrame {
    AshType type = AshType::Unknown;
    uint8_t control = 0;
    uint8_t frmNum = 0;
    uint8_t ackNum = 0;
    bool reTx = false;
    std::vector<uint8_t> ezsp;
    std::vector<uint8_t> raw;
};

class AshEzsp {
public:
    explicit AshEzsp(SerialPort& serial) : serial_(serial) {}

    bool reset() {
        std::vector<uint8_t> prefix(32, ASH_CANCEL);
        std::vector<uint8_t> rst = encodeAshPayload({0xC0});
        prefix.insert(prefix.end(), rst.begin(), rst.end());
        std::cout << "TX RST\n";
        if (!serial_.writeAll(prefix)) return false;

        auto deadline = nowMs() + 3000;
        while (nowMs() < deadline) {
            auto frame = readFrame(200);
            if (!frame) continue;
            if (frame->type == AshType::Rstack) {
                if (frame->raw.size() >= 3) {
                    std::cout << "RSTACK recibido: version ASH=" << static_cast<int>(frame->raw[1])
                              << " reset=0x" << std::hex << static_cast<int>(frame->raw[2])
                              << std::dec << "\n";
                }
                txSeq_ = 0;
                rxSeq_ = 0;
                ezspSeq_ = 0;
                return true;
            }
            std::cout << "RX durante reset: " << describe(*frame) << "\n";
        }
        std::cerr << "Timeout esperando RSTACK\n";
        return false;
    }

    bool negotiateVersion() {
        // Bellows starts with the legacy v4 frame header after ASH reset.
        useExtendedHeader_ = false;
        auto rsp = command(CMD_VERSION, {4}, 3000);
        if (!rsp) return false;
        if (rsp->payload.size() < 4) {
            std::cerr << "Respuesta version demasiado corta: " << hex(rsp->payload) << "\n";
            return false;
        }

        ezspVersion_ = rsp->payload[0];
        uint8_t stackType = rsp->payload[1];
        uint16_t stackVersion = readLe16(rsp->payload, 2);
        std::cout << "EZSP version reportada=" << static_cast<int>(ezspVersion_)
                  << " stackType=" << static_cast<int>(stackType)
                  << " stackVersion=0x" << std::hex << stackVersion << std::dec << "\n";

        if (ezspVersion_ >= 8) {
            useExtendedHeader_ = true;
            auto rsp2 = command(CMD_VERSION, {ezspVersion_}, 3000);
            if (!rsp2) return false;
            std::cout << "EZSP version confirmada: " << hex(rsp2->payload) << "\n";
        } else {
            std::cout << "EZSP antiguo confirmado: " << static_cast<int>(ezspVersion_) << "\n";
        }

        return true;
    }

    bool initialize(uint8_t permitSeconds) {
        // Obtener EUI64 del NCP para usarlo como CIE address en IAS Zone
        auto euiRsp = command(CMD_GET_EUI64, {}, 3000);
        if (euiRsp && euiRsp->payload.size() >= 8) {
            ncpEui64_.assign(euiRsp->payload.begin(), euiRsp->payload.begin() + 8);
            std::cout << "EUI64 del NCP: " << eui64String(ncpEui64_, 0) << "\n";
        }

        // En EZSP v8+, networkInit() requiere un EmberNetworkInitStruct de 2 bytes
        // (bitmask de opciones). 0x0000 = usar configuracion almacenada en NVM.
        auto init = command(CMD_NETWORK_INIT, {0x00, 0x00}, 5000);
        if (!init || init->payload.empty()) return false;
        uint8_t initStatus = init->payload[0];
        std::cout << "networkInit status=0x" << std::hex << static_cast<int>(initStatus)
                  << std::dec << "\n";

        auto state = command(CMD_NETWORK_STATE, {}, 3000);
        if (!state || state->payload.empty()) return false;
        std::cout << "networkState=0x" << std::hex << static_cast<int>(state->payload[0])
                  << std::dec << " (0x02 suele ser JOINED_NETWORK)\n";

        // Mostrar parametros de red para diagnostico (solo lectura, no afecta estado)
        getNetworkParameters();

        bool joined = (state->payload[0] == 0x02);

        if (!joined) {
            std::cout << "NCP no unido a red. Intentando formar nueva red Zigbee...\n";
            if (!formZigbeeNetwork()) {
                std::cerr << "Fallo al formar red. Ejecutando en modo solo-escucha.\n";
                return true;  // seguimos al loop para escuchar lo que llegue
            }
        }

        // Red lista: configurar endpoint, politicas y activar pairing
        commandNoFail(CMD_ADD_ENDPOINT, endpoint1Payload(), "addEndpoint(1)");

        // Bellows v8+ does this before permitJoining: TRUST_CENTER_POLICY =
        // ALLOW_JOINS | ALLOW_UNSECURED_REJOINS.
        addTransientTcLinkKey();
        commandNoFail(CMD_SET_POLICY, {0x00, 0x03}, "setPolicy(TRUST_CENTER_POLICY, 0x03)");

        auto permit = command(CMD_PERMIT_JOINING, {permitSeconds}, 3000);
        if (!permit || permit->payload.empty()) return false;
        std::cout << "permitJoining(" << static_cast<int>(permitSeconds) << ") status=0x"
                  << std::hex << static_cast<int>(permit->payload[0]) << std::dec << "\n";
        return true;
    }

    void loop() {
        std::cout << "Esperando callbacks. Resetea/activa el PIR para emparejar o enviar movimiento.\n";
        while (!g_stop) {
            auto frame = readFrame(500);
            if (!frame) {
                if (pendingIasConfig_) configurePendingIasDevice();
                continue;
            }
            handleFrame(*frame, std::nullopt);
            if (pendingIasConfig_) configurePendingIasDevice();
        }
        std::cout << "\nSaliendo.\n";
    }

private:
    struct EzspResponse {
        uint8_t seq = 0;
        uint16_t cmd = 0;
        std::vector<uint8_t> payload;
    };

    static int64_t nowMs() {
        timeval tv{};
        gettimeofday(&tv, nullptr);
        return static_cast<int64_t>(tv.tv_sec) * 1000 + tv.tv_usec / 1000;
    }

    std::vector<uint8_t> endpoint1Payload() {
        std::vector<uint8_t> p;
        p.push_back(1);          // endpoint
        appendLe16(p, 0x0104);   // ZHA profile
        appendLe16(p, 0x0400);   // IAS_CONTROL
        p.push_back(0);          // device version

        const uint16_t inClusters[] = {0x0000, 0x0006, 0x000A, 0x0019, 0x0501};
        const uint16_t outClusters[] = {0x0001, 0x0020, 0x0500, 0x0502};

        p.push_back(sizeof(inClusters) / sizeof(inClusters[0]));
        p.push_back(sizeof(outClusters) / sizeof(outClusters[0]));
        for (uint16_t c : inClusters) appendLe16(p, c);
        for (uint16_t c : outClusters) appendLe16(p, c);
        return p;
    }

    void commandNoFail(uint16_t cmd, const std::vector<uint8_t>& payload, const std::string& label) {
        auto rsp = command(cmd, payload, 3000);
        if (!rsp) {
            std::cout << label << ": sin respuesta, continuo\n";
            return;
        }
        if (!rsp->payload.empty()) {
            std::cout << label << " status/respuesta: " << hex(rsp->payload) << "\n";
        } else {
            std::cout << label << " OK\n";
        }
    }

    void addTransientTcLinkKey() {
        std::vector<uint8_t> p(8, 0xFF);  // wildcard IEEE
        const char key[] = "ZigBeeAlliance09";
        p.insert(p.end(), key, key + 16);

        if (ezspVersion_ >= 13) {
            p.push_back(0x00);  // SecurityManagerContextFlags.NONE
            commandNoFail(CMD_IMPORT_TRANSIENT_KEY, p, "importTransientKey(wildcard)");
        } else {
            commandNoFail(CMD_ADD_TRANSIENT_LINK_KEY, p, "addTransientLinkKey(wildcard)");
        }
    }

    void getNetworkParameters() {
        auto rsp = command(CMD_GET_NETWORK_PARAMETERS, {}, 3000);
        if (!rsp || rsp->payload.empty()) {
            std::cout << "getNetworkParameters: sin respuesta\n";
            return;
        }
        // El primer byte es el status EZSP. Si no es SUCCESS (0x00), los
        // parametros no son validos.
        if (rsp->payload.size() < 2 || rsp->payload[0] != 0x00) {
            std::cout << "getNetworkParameters status=0x" << std::hex
                      << static_cast<int>(rsp->payload[0])
                      << std::dec << " (red no disponible)\n";
            return;
        }
        // EmberNetworkParameters a partir del byte 1:
        // panId(2), extendedPanId(8), nwkUpdateId(1), radioTxPower(1),
        // radioChannel(1), joinMethod(1), nwkManagerId(2),
        // nwkUpdateInterval(1), nwkUpdateChannel(1)
        const auto& p = rsp->payload;
        if (p.size() >= 21) {
            uint16_t panId = readLe16(p, 1);
            uint8_t channel = p[13];
            std::string extPan = eui64String(p, 2);
            std::cout << "Parametros de red: PAN=0x" << std::hex << panId
                      << " extPAN=" << extPan
                      << " canal=" << std::dec << static_cast<int>(channel) << "\n";
        } else {
            std::cout << "getNetworkParameters respuesta: " << hex(rsp->payload) << "\n";
        }
    }

    bool formZigbeeNetwork() {
        // Obtener EUI64 del NCP para usarlo como Trust Center EUI
        auto euiRsp = command(CMD_GET_EUI64, {}, 3000);
        if (!euiRsp || euiRsp->payload.size() < 8) {
            std::cerr << "No se pudo obtener EUI64 del NCP\n";
            return false;
        }
        std::vector<uint8_t> eui64(euiRsp->payload.begin(), euiRsp->payload.begin() + 8);
        std::cout << "EUI64 del NCP: " << eui64String(eui64, 0) << "\n";

        // ---- setInitialSecurityState ----
        // bitmask: HAVE_TRUST_CENTER_EUI64 | HAVE_PRECONFIGURED_KEY |
        //          HAVE_NETWORK_KEY | TRUST_CENTER_GLOBAL_LINK_KEY |
        //          HAVE_PRECONFIGURED_TRUST_CENTER_EUI64
        uint16_t secBitmask = 0x0001 | 0x0002 | 0x0004 | 0x0040 | 0x1000;  // 0x1047

        std::vector<uint8_t> secPayload;
        appendLe16(secPayload, secBitmask);

        // preconfiguredKey: "ZigBeeAlliance09" (16 bytes)
        const char* wellKnownKey = "ZigBeeAlliance09";
        secPayload.insert(secPayload.end(), wellKnownKey, wellKnownKey + 16);

        // networkKey: aleatoria (16 bytes)
        auto netKey = pseudoRandom(16);
        secPayload.insert(secPayload.end(), netKey.begin(), netKey.end());

        // networkKeySequenceNumber: 0
        secPayload.push_back(0);

        // trustCenterEui64: nuestro EUI64
        secPayload.insert(secPayload.end(), eui64.begin(), eui64.end());

        auto secRsp = command(CMD_SET_INITIAL_SECURITY_STATE, secPayload, 3000);
        if (!secRsp || secRsp->payload.empty()) {
            std::cerr << "setInitialSecurityState sin respuesta\n";
            return false;
        }
        std::cout << "setInitialSecurityState status=0x" << std::hex
                  << static_cast<int>(secRsp->payload[0]) << std::dec << "\n";
        if (secRsp->payload[0] != 0x00) {
            std::cerr << "setInitialSecurityState fallo (status!=0x00)\n";
            return false;
        }

        // ---- formNetwork ----
        std::vector<uint8_t> netParams;

        // extendedPanId (8 bytes) - aleatorio
        auto extPan = pseudoRandom(8);
        netParams.insert(netParams.end(), extPan.begin(), extPan.end());

        // panId (2 bytes) - 0x2D3F (reutilizar el que usaba Bellows)
        appendLe16(netParams, 0x2D3F);

        // radioTxPower (1 byte) - 8 dBm
        netParams.push_back(8);

        // radioChannel (1 byte) - canal 26 (0x1A)
        netParams.push_back(26);

        // joinMethod (1 byte) - 0x00 = USE_MAC_ASSOCIATION
        netParams.push_back(0x00);

        // nwkManagerId (2 bytes) - 0x0000
        appendLe16(netParams, 0x0000);

        // nwkUpdateId (1 byte) - 1
        netParams.push_back(1);

        // channels (4 bytes, uint32_t LE) - bit 26 = 0x04000000
        netParams.push_back(0x00);
        netParams.push_back(0x00);
        netParams.push_back(0x00);
        netParams.push_back(0x04);

        auto formRsp = command(CMD_FORM_NETWORK, netParams, 10000);
        if (!formRsp || formRsp->payload.empty()) {
            std::cerr << "formNetwork sin respuesta\n";
            return false;
        }
        std::cout << "formNetwork status=0x" << std::hex
                  << static_cast<int>(formRsp->payload[0]) << std::dec << "\n";
        if (formRsp->payload[0] != 0x00) {
            std::cerr << "formNetwork fallo con status=0x" << std::hex
                      << static_cast<int>(formRsp->payload[0]) << std::dec << "\n";
            return false;
        }

        // Verificar networkState despues de formar red
        usleep(500000);  // 500 ms para que la pila se estabilice
        auto state = command(CMD_NETWORK_STATE, {}, 3000);
        if (state && !state->payload.empty()) {
            std::cout << "networkState post-form=0x" << std::hex
                      << static_cast<int>(state->payload[0]) << std::dec
                      << " (0x02 es JOINED_NETWORK)\n";
            if (state->payload[0] == 0x02) {
                std::cout << "Red Zigbee formada exitosamente como Coordinador!\n";
                getNetworkParameters();
                return true;
            }
        }

        std::cerr << "La red no alcanzo estado JOINED_NETWORK tras formar.\n";
        return false;
    }

    std::optional<EzspResponse> command(uint16_t cmd, const std::vector<uint8_t>& payload, int timeoutMs) {
        uint8_t seq = ezspSeq_++;
        std::vector<uint8_t> ezsp;
        ezsp.push_back(seq);
        ezsp.push_back(0x00);

        if (useExtendedHeader_) {
            ezsp.push_back(0x01);
            appendLe16(ezsp, cmd);
        } else {
            ezsp.push_back(static_cast<uint8_t>(cmd & 0xFF));
        }
        ezsp.insert(ezsp.end(), payload.begin(), payload.end());

        uint8_t frm = txSeq_;
        txSeq_ = (txSeq_ + 1) & 0x07;

        for (int attempt = 0; attempt < 3; ++attempt) {
            sendDataFrame(frm, attempt > 0, ezsp);

            bool gotAck = false;
            int64_t deadline = nowMs() + timeoutMs;
            while (nowMs() < deadline) {
                auto frame = readFrame(100);
                if (!frame) continue;

                if (frame->type == AshType::Ack) {
                    if (frame->ackNum == ((frm + 1) & 0x07)) {
                        gotAck = true;
                    }
                    continue;
                }

                auto rsp = handleFrame(*frame, seq);
                if (rsp && rsp->cmd == cmd) {
                    return rsp;
                }
            }

            if (!gotAck) {
                std::cout << "Sin ACK ASH para cmd 0x" << std::hex << cmd << std::dec
                          << ", reintento " << (attempt + 1) << "/3\n";
            }
        }

        std::cerr << "Timeout esperando respuesta EZSP cmd 0x" << std::hex << cmd << std::dec << "\n";
        return std::nullopt;
    }

    void sendDataFrame(uint8_t frmNum, bool reTx, const std::vector<uint8_t>& ezsp) {
        std::vector<uint8_t> randomized = ashRandomize(ezsp);
        std::vector<uint8_t> payload;
        payload.push_back(static_cast<uint8_t>((frmNum << 4) | (reTx ? 0x08 : 0x00) | rxSeq_));
        payload.insert(payload.end(), randomized.begin(), randomized.end());
        std::cout << "TX DATA frm=" << static_cast<int>(frmNum)
                  << " ack=" << static_cast<int>(rxSeq_)
                  << " ezsp=" << hex(ezsp) << "\n";
        serial_.writeAll(encodeAshPayload(payload));
    }

    void sendAck() {
        std::vector<uint8_t> payload{static_cast<uint8_t>(0x80 | rxSeq_)};
        serial_.writeAll(encodeAshPayload(payload));
    }

    std::vector<uint8_t> encodeAshPayload(std::vector<uint8_t> payload) {
        uint16_t crc = crcCcitt(payload.data(), payload.size());
        payload.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
        payload.push_back(static_cast<uint8_t>(crc & 0xFF));

        std::vector<uint8_t> out;
        out.reserve(payload.size() + 4);
        for (uint8_t b : payload) {
            if (isReserved(b)) {
                out.push_back(ASH_ESCAPE);
                out.push_back(static_cast<uint8_t>(b ^ 0x20));
            } else {
                out.push_back(b);
            }
        }
        out.push_back(ASH_FLAG);
        return out;
    }

    std::optional<AshFrame> readFrame(int timeoutMs) {
        int64_t deadline = nowMs() + timeoutMs;
        while (nowMs() < deadline) {
            uint8_t buf[128];
            ssize_t n = serial_.readSome(buf, sizeof(buf), static_cast<int>(deadline - nowMs()));
            if (n < 0) return std::nullopt;
            if (n == 0) continue;

            for (ssize_t i = 0; i < n; ++i) {
                uint8_t b = buf[i];
                if (b == ASH_CANCEL) {
                    rxBuf_.clear();
                    escaped_ = false;
                    continue;
                }
                if (b == ASH_FLAG) {
                    if (rxBuf_.empty()) continue;
                    auto parsed = parseAsh(rxBuf_);
                    rxBuf_.clear();
                    escaped_ = false;
                    if (parsed) return parsed;
                    continue;
                }
                if (escaped_) {
                    rxBuf_.push_back(static_cast<uint8_t>(b ^ 0x20));
                    escaped_ = false;
                } else if (b == ASH_ESCAPE) {
                    escaped_ = true;
                } else if (b == ASH_XON || b == ASH_XOFF) {
                    continue;
                } else {
                    rxBuf_.push_back(b);
                }
            }
        }
        return std::nullopt;
    }

    std::optional<AshFrame> parseAsh(const std::vector<uint8_t>& raw) {
        if (raw.size() < 3) {
            std::cerr << "ASH corto: " << hex(raw) << "\n";
            return std::nullopt;
        }
        uint16_t got = static_cast<uint16_t>((raw[raw.size() - 2] << 8) | raw[raw.size() - 1]);
        uint16_t want = crcCcitt(raw.data(), raw.size() - 2);
        if (got != want) {
            std::cerr << "CRC ASH invalido. got=0x" << std::hex << got << " want=0x" << want
                      << std::dec << " raw=" << hex(raw) << "\n";
            return std::nullopt;
        }

        AshFrame f;
        f.raw = raw;
        f.control = raw[0];
        if ((f.control & 0x80) == 0x00) {
            f.type = AshType::Data;
            f.frmNum = (f.control >> 4) & 0x07;
            f.reTx = (f.control & 0x08) != 0;
            f.ackNum = f.control & 0x07;
            std::vector<uint8_t> randomized(raw.begin() + 1, raw.end() - 2);
            f.ezsp = ashRandomize(randomized);
        } else if ((f.control & 0xE0) == 0x80) {
            f.type = AshType::Ack;
            f.ackNum = f.control & 0x07;
        } else if ((f.control & 0xE0) == 0xA0) {
            f.type = AshType::Nak;
            f.ackNum = f.control & 0x07;
        } else if (f.control == 0xC1) {
            f.type = AshType::Rstack;
        } else if (f.control == 0xC2) {
            f.type = AshType::Error;
        }
        return f;
    }

    std::optional<EzspResponse> handleFrame(const AshFrame& frame, std::optional<uint8_t> waitSeq) {
        if (frame.type == AshType::Data) {
            if (frame.frmNum == rxSeq_) {
                rxSeq_ = (rxSeq_ + 1) & 0x07;
                sendAck();
                return parseEzsp(frame.ezsp, waitSeq);
            } else {
                sendAck();
                std::cout << "DATA duplicado/fuera de orden frm=" << static_cast<int>(frame.frmNum)
                          << " esperado=" << static_cast<int>(rxSeq_) << ", ignorado\n";
                return std::nullopt;
            }
        }

        if (frame.type == AshType::Nak) {
            std::cout << "RX NAK ack=" << static_cast<int>(frame.ackNum) << "\n";
        } else if (frame.type == AshType::Error) {
            std::cout << "RX ERROR ASH: " << hex(frame.raw) << "\n";
        } else if (frame.type != AshType::Ack) {
            std::cout << "RX ASH: " << describe(frame) << "\n";
        }
        return std::nullopt;
    }

    std::optional<EzspResponse> parseEzsp(const std::vector<uint8_t>& ezsp, std::optional<uint8_t> waitSeq) {
        if ((!useExtendedHeader_ && ezsp.size() < 3) || (useExtendedHeader_ && ezsp.size() < 5)) {
            std::cout << "EZSP corto: " << hex(ezsp) << "\n";
            return std::nullopt;
        }

        EzspResponse rsp;
        rsp.seq = ezsp[0];
        size_t payloadOff = 0;
        if (useExtendedHeader_) {
            rsp.cmd = readLe16(ezsp, 3);
            payloadOff = 5;
        } else {
            rsp.cmd = ezsp[2];
            payloadOff = 3;
        }
        rsp.payload.assign(ezsp.begin() + payloadOff, ezsp.end());

        if (waitSeq && rsp.seq == *waitSeq) {
            std::cout << "RX RSP cmd=0x" << std::hex << rsp.cmd << std::dec
                      << " seq=" << static_cast<int>(rsp.seq)
                      << " payload=" << hex(rsp.payload) << "\n";
            return rsp;
        }

        printCallback(rsp);
        return std::nullopt;
    }

    void printCallback(const EzspResponse& rsp) {
        std::cout << "RX CALLBACK cmd=0x" << std::hex << rsp.cmd << std::dec
                  << " seq=" << static_cast<int>(rsp.seq)
                  << " payload=" << hex(rsp.payload) << "\n";

        if (rsp.cmd == CB_STACK_STATUS && !rsp.payload.empty()) {
            std::cout << "  stackStatusHandler status=0x" << std::hex
                      << static_cast<int>(rsp.payload[0]) << std::dec << "\n";
            return;
        }

        if (rsp.cmd == CB_TRUST_CENTER_JOIN) {
            handleTrustCenterJoin(rsp.payload);
            return;
        }

        if (rsp.cmd == CB_INCOMING_MESSAGE) {
            printIncomingMessage(rsp.payload);
        }
    }

    void handleTrustCenterJoin(const std::vector<uint8_t>& p) {
        if (p.size() < 14) {
            std::cout << "  trustCenterJoinHandler: payload corto (" << p.size() << " bytes)\n";
            return;
        }
        uint16_t nwk = readLe16(p, 0);
        std::string ieeeStr = eui64String(p, 2);
        uint8_t status = p[10];
        uint8_t decision = p[11];
        uint16_t parent = readLe16(p, 12);
        std::cout << "  trustCenterJoinHandler: nwk=0x" << std::hex << nwk
                  << " ieee=" << ieeeStr
                  << " status=0x" << static_cast<int>(status)
                  << " decision=0x" << static_cast<int>(decision)
                  << " parent=0x" << parent << std::dec << "\n";

        // Programar configuracion IAS Zone para este dispositivo.
        // No llamamos comandos EZSP desde aqui para evitar interferir con
        // el comando en curso. La configuracion se hara en el loop principal.
        if (status == 0x01 && p.size() >= 10) {  // EMBER_DEVICE_JOINED
            pendingIasConfig_ = true;
            pendingNwk_ = nwk;
            pendingIeee_.assign(p.begin() + 2, p.begin() + 10);
            std::cout << "  Dispositivo encolado para configuracion IAS Zone\n";
        }
    }

    void configurePendingIasDevice() {
        if (!pendingIasConfig_ || ncpEui64_.empty()) {
            pendingIasConfig_ = false;
            return;
        }

        uint16_t nwk = pendingNwk_;
        std::vector<uint8_t> ieee = pendingIeee_;
        pendingIasConfig_ = false;
        pendingZclSeq_++;

        std::cout << "Configurando IAS Zone para nwk=0x" << std::hex << nwk << std::dec << "...\n";

        // Paso 1: Escribir CIE address (atributo 0x0010 del cluster IAS Zone 0x0500).
        // ZCL WriteAttributes: frameCtrl=0x00, seq, cmd=0x02, attrId=0x0010, dataType=0xF0, ieee(8)
        {
            std::vector<uint8_t> zcl;
            zcl.push_back(0x00);  // frame control: profile-wide, server→client
            zcl.push_back(pendingZclSeq_);
            zcl.push_back(0x02);  // WriteAttributes
            appendLe16(zcl, 0x0010);  // CIE Address attribute
            zcl.push_back(0xF0);  // data type: IEEE_ADDRESS
            zcl.insert(zcl.end(), ncpEui64_.begin(), ncpEui64_.end());
            sendZclUnicast(nwk, 0x0500, 1, 1, zcl, "Write CIE address");
        }

        // Pequena pausa entre comandos
        usleep(200000);

        // Paso 2: Enviar ZoneEnrollResponse proactivamente.
        // ZCL ZoneEnrollResponse: frameCtrl=0x00, seq, cmd=0x00, enrollResponseCode=0x00, zoneId=0x00
        {
            std::vector<uint8_t> zcl;
            zcl.push_back(0x00);  // frame control: profile-wide, server→client
            zcl.push_back(pendingZclSeq_);
            zcl.push_back(0x00);  // ZoneEnrollResponse (mismo ID que ZoneStatusChangeNotification)
            zcl.push_back(0x00);  // enrollResponseCode: SUCCESS
            zcl.push_back(0x00);  // zoneId: 0
            sendZclUnicast(nwk, 0x0500, 1, 1, zcl, "ZoneEnrollResponse");
        }
    }

    void sendZclUnicast(uint16_t nwk, uint16_t cluster, uint8_t srcEp, uint8_t dstEp,
                        const std::vector<uint8_t>& zclPayload, const std::string& label) {
        // EZSP sendUnicast (0x0034):
        //   type(1): EMBER_OUTGOING_DIRECT = 0x00
        //   indexOrDestination(2): nodeId LE
        //   apsFrame(12):
        //     profileId(2 LE): 0x0104 ZHA
        //     clusterId(2 LE)
        //     sourceEndpoint(1)
        //     destinationEndpoint(1)
        //     options(2 LE): EMBER_APS_OPTION_RETRY | EMBER_APS_OPTION_ENABLE_ROUTE_DISCOVERY = 0x0060
        //     groupId(2 LE): 0x0000
        //     sequence(1)
        //   messageTag(1): 0x01
        //   message(LVBytes)

        std::vector<uint8_t> pl;
        pl.push_back(0x00);  // EMBER_OUTGOING_DIRECT
        appendLe16(pl, nwk);

        // APS frame
        appendLe16(pl, 0x0104);  // profile: ZHA
        appendLe16(pl, cluster);
        pl.push_back(srcEp);
        pl.push_back(dstEp);
        appendLe16(pl, 0x0060);  // options: RETRY | ENABLE_ROUTE_DISCOVERY
        appendLe16(pl, 0x0000);  // groupId
        pl.push_back(pendingZclSeq_);  // APS sequence

        pl.push_back(0x01);  // messageTag
        pl.push_back(static_cast<uint8_t>(zclPayload.size()));
        pl.insert(pl.end(), zclPayload.begin(), zclPayload.end());

        auto rsp = command(CMD_SEND_UNICAST, pl, 5000);
        if (!rsp || rsp->payload.empty()) {
            std::cout << "  " << label << ": sin respuesta\n";
            return;
        }
        if (rsp->payload.size() >= 2) {
            uint8_t ezspStatus = rsp->payload[0];
            uint8_t apsSeq = (rsp->payload.size() >= 2) ? rsp->payload[1] : 0;
            std::cout << "  " << label << " ezspStatus=0x" << std::hex
                      << static_cast<int>(ezspStatus)
                      << " apsSeq=" << static_cast<int>(apsSeq) << std::dec << "\n";
        } else {
            std::cout << "  " << label << " respuesta: " << hex(rsp->payload) << "\n";
        }
    }

    void printIncomingMessage(const std::vector<uint8_t>& p) {
        if (ezspVersion_ >= 14) {
            // v14+: type(1), aps(12), nwk(2), eui64(8), binding(1), address(1),
            // lqi(1), rssi(1), timestamp(4), LVBytes.
            if (p.size() < 32) {
                std::cout << "  incomingMessageHandler v14+ corto\n";
                return;
            }
            size_t msgOff = 31;
            uint8_t len = p[msgOff];
            if (msgOff + 1 + len > p.size()) len = static_cast<uint8_t>(p.size() - msgOff - 1);
            std::vector<uint8_t> msg(p.begin() + msgOff + 1, p.begin() + msgOff + 1 + len);
            std::cout << "  incomingMessageHandler message(hex): " << hex(msg) << "\n";
            return;
        }

        // v13 and older: type(1), aps(12), lqi(1), rssi(1), sender(2),
        // binding(1), address(1), LVBytes.
        if (p.size() < 20) {
            std::cout << "  incomingMessageHandler v13 corto\n";
            return;
        }
        std::cout << "  incomingMessageHandler(RAW): " << hex(p) << "\n";

        uint8_t type = p[0];
        uint16_t profile = readLe16(p, 1);
        uint16_t cluster = readLe16(p, 3);
        uint8_t srcEp = p[5];
        uint8_t dstEp = p[6];
        uint16_t groupId = readLe16(p, 9);
        uint8_t sequence = p[11];
        uint8_t lqi = p[13];
        int8_t rssi = static_cast<int8_t>(p[14]);
        uint16_t sender = readLe16(p, 15);
        uint8_t binding = p[17];
        uint8_t address = p[18];
        size_t msgOff = 19;
        uint8_t len = p[msgOff];
        if (msgOff + 1 + len > p.size()) len = static_cast<uint8_t>(p.size() - msgOff - 1);
        std::vector<uint8_t> msg(p.begin() + msgOff + 1, p.begin() + msgOff + 1 + len);
        std::cout << "  incomingMessageHandler: type=0x" << std::hex << static_cast<int>(type)
                  << " profile=0x" << profile
                  << " cluster=0x" << cluster << std::dec
                  << " srcEp=" << static_cast<int>(srcEp)
                  << " dstEp=" << static_cast<int>(dstEp)
                  << " group=0x" << std::hex << groupId << std::dec
                  << " seq=" << static_cast<int>(sequence)
                  << " addrIdx=" << static_cast<int>(address)
                  << " sender=0x" << std::hex << sender << std::dec
                  << " lqi=" << static_cast<int>(lqi)
                  << " rssi=" << static_cast<int>(rssi)
                  << " binding=" << static_cast<int>(binding)
                  << "\n  message(hex): " << hex(msg) << "\n";
    }

    std::string describe(const AshFrame& f) {
        std::ostringstream os;
        os << "type=";
        switch (f.type) {
            case AshType::Data: os << "DATA"; break;
            case AshType::Ack: os << "ACK"; break;
            case AshType::Nak: os << "NAK"; break;
            case AshType::Rstack: os << "RSTACK"; break;
            case AshType::Error: os << "ERROR"; break;
            default: os << "UNKNOWN"; break;
        }
        os << " raw=" << hex(f.raw);
        if (!f.ezsp.empty()) os << " ezsp=" << hex(f.ezsp);
        return os.str();
    }

    SerialPort& serial_;
    std::vector<uint8_t> rxBuf_;
    bool escaped_ = false;
    uint8_t txSeq_ = 0;
    uint8_t rxSeq_ = 0;
    uint8_t ezspSeq_ = 0;
    uint8_t ezspVersion_ = 4;
    bool useExtendedHeader_ = false;

    // IAS Zone configuration
    std::vector<uint8_t> ncpEui64_;
    bool pendingIasConfig_ = false;
    uint16_t pendingNwk_ = 0;
    std::vector<uint8_t> pendingIeee_;
    uint8_t pendingZclSeq_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
    std::string port = argc >= 2 ? argv[1] : kDefaultPort;
    int permitSeconds = argc >= 3 ? std::stoi(argv[2]) : 180;
    if (permitSeconds < 0) permitSeconds = 0;
    if (permitSeconds > 254) permitSeconds = 254;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    SerialPort serial(port);
    if (!serial.ok()) return 1;

    std::cout << "Puerto serial abierto: " << port << " @115200\n";

    AshEzsp app(serial);
    if (!app.reset()) return 1;
    if (!app.negotiateVersion()) return 1;
    if (!app.initialize(static_cast<uint8_t>(permitSeconds))) return 1;
    app.loop();
    return 0;
}
