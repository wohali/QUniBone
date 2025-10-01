/*
    delqa.hpp: DELQA QBUS Ethernet adapter

    Copyright JS Enterprises, Inc. 2025 via DECromancer.ca
    Contributed under the BSD 2-clause license.

 */
#ifndef _DELQA_HPP_
#define _DELQA_HPP_

#include <assert.h>
#include <bits/stdc++.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_tun.h>
// #include <linux/virtio_net.h>
#include <netdb.h>
#include <netinet/in.h>
// #include <net/if.h>
#include <linux/if_tun.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "utils.hpp"
#include "qunibusadapter.hpp"
#include "qunibusdevice.hpp"
#include "parameter.hpp"

#define DELQA_BUFLEN 2048                    // default buffer length
#define DELQA_MAXQ 256                       // max rx pkt queue depth

class delqa_c : public qunibusdevice_c
{
public:
    delqa_c();
    ~delqa_c();

    parameter_string_c bridge_name = parameter_string_c(this, "bridge_name", "bn", false,
        "Linux bridge to connect with tap device");

    parameter_string_c mac_addr = parameter_string_c(this, "macaddr", "mac",
            false, "Primary MAC address for adapter in colon-delimited format");

    // bool on_before_install(void) override ;
    void on_after_uninstall(void) override ;
    bool on_param_changed(parameter_c *param) override;
    void on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;

    // Resets all register values on BUS INIT or Control Reset functions
    // and any other relevant local state.
    void reset(void);

    // Flushes any received queued packets
    void flush_packet_queue(void);

    // background worker function
    void worker(unsigned instance) override;

    // called by qunibusadapter on emulated register access
    void on_after_register_access(
        qunibusdevice_register_t *device_reg,
        uint8_t unibus_control,
        DATO_ACCESS access) override;

private:
    // DELQA registers (Figure 3-2 of EK-DELQA-UG-002), BASE+00 to BASE+16:
    // SA 0-5 are read-only and contain the device's Ethernet MAC address
    qunibusdevice_register_t *QSA0_reg; // Station Address (SA) 0
    qunibusdevice_register_t *QSA1_reg; // Station Address (SA) 1
    // Buffer Descriptor List (BDL) registers are write-only
    qunibusdevice_register_t *QARL_reg; // Receive BDL Low Reg / SA 2
    qunibusdevice_register_t *QARH_reg; // Receive BDL High Reg / SA 3
    qunibusdevice_register_t *QATL_reg; // Transmit BDL Low Reg / SA 4
    qunibusdevice_register_t *QATH_reg; // Transmit BDL High Reg / SA 5
    qunibusdevice_register_t *QVAR_reg; // Vector Address Register (R/W)
    qunibusdevice_register_t *QCSR_reg; // Control and Status Register (R/W)

    // DELQA has one INTR and two DMA, tx higher priority
    dma_request_c tx_dma_request = dma_request_c(this);
    dma_request_c rx_dma_request = dma_request_c(this);
    intr_request_c intr_request = intr_request_c(this);


    // Control & Status Register (CSR) bits
    volatile bool _re; // Receiver Enable
    volatile bool _sr; // Software Reset
    volatile bool _nx; // Non-existent-Memory Timeout Interrupt
    volatile bool _bd; // Boot/Diagnostic ROM load
    volatile bool _xl; // Transmit List Invalid/Empty
    volatile bool _rl; // Receive List Invalid/Empty
    volatile bool _ie; // Interrupt Enable
    volatile bool _xi; // Transmit Interrupt Request
    volatile bool _il; // Internal Loopback
    volatile bool _el; // External Loopback
    volatile bool _se; // Sanity Timer Enable
    // bit 11 is reserved: always set to zero
    volatile bool _ok = false; // bit 12: Ethernet Transceiver Power OK: set when tunnel is up
    volatile bool _ca = true;  // Carrier from Receiver Enabled
    volatile bool _pe = false; // Parity Error in Memory
    volatile bool _ri; // Receive Interrupt Request

    // Updates the CSR value based on the above bits
    void update_QCSR(void);

    // Vector Address Register bits
    volatile bool _itb = false;  // Identity Test bit
                // bit 1 is reserved, no access defined
    // Bits <09:02> are the interrupt vector, c.f. intr_vector
    volatile uint8_t _sts; // Bits <12:10> are the self-test status (3 bits)
    volatile bool _rst;  // Request Self-Test bit
    volatile bool _ms = true;       // Mode Select bit
    volatile bool _s4 = true;       // Option Switch (S4) setting bit

    // Updates the VAR value based on the above bits
    void update_QVAR(void);

    struct _bdl
    {
        volatile uint32_t bufaddr; // buffer address
        volatile uint16_t buflen;  // buffer length in words, given as a twos complement,
                                   // excluding the two CRC words
        // address descriptor bits
        // valid    chain     definition
        //   1        0       Valid descriptor
        //   1        1       Descriptor contains the address of another descriptor.
        //   0        0       Invalid descriptor (end of BDL)
        //   0        1       Reserved
        volatile bool valid; // bit 15
        volatile bool chain; // bit 14
        // bits 11:08 reserved
        volatile bool lbt; // low byte termination, bit 07
        volatile bool hbs; // high byte start, bit 06
        // bits 05:00 are the high buffer address bits
        uint16_t get_sw_1(); // overridden in tx/rx structs
        uint16_t get_sw_2();

        // shared status word 1 bits
        // lastnot  errused   definition
        //    1        0      Value initialized by the host.
        //    1        1      Buffer has been used but is not the last segment (chained buffer)
        //    0        0      Buffer has packet last segment and has been transmitted, no errors
        //    0        1      Buffer has packet last segment and has been transmitted, with errors
        volatile bool lastnot; // bit 15
        volatile bool errused; // bit 14
    };

    struct _tx_bdl : _bdl
    {
        // address descriptor bits
        volatile bool eom;   // end of message, TX only, bit 13
        volatile bool setup; // setup buffer, TX only, bit 12

        // tx status word 1 bits
        // bit 13 reserved
        volatile bool loss; // set if loss of carrier during transmission, bit 12
        // bit 11 reserved
        volatile bool ste;   // sanity timer enabled (DEQNA-lock only, S4 mirror), bit 10
        volatile bool abort; // aborted due to excessive collisions, bit 09
        // bit 08 reserved
        // bits 07:04 count
        volatile uint8_t count; // # of collisions before attempt associated with this status word
                                // 0: no collisions, or aborted after 16 collisions
                                // 1: one collision
                                // 2: between 2 and 15 collisions
        // bits 03:00 reserved

        // tx status word 2 bits
        // bits 15:10 reserved
        volatile uint16_t tdr; // Time Domain Reflectometry (unsupported)
    };

    struct _rx_bdl : _bdl
    {
        // rx status word 1 bits
        volatile bool esetup; // setup packet, external LB or internal-extended LB packet, bit 13
        // bit 12 reserved
        volatile bool runt;   // set if internal loopback failure
        volatile uint8_t rbl; // received byte length, all set for setup packet. bits 10:08
                              // appears at both bits 15:08 and 07:00.
        // bits 07:03 reserved
        volatile bool frame;  // framing alignment error + CRC error, bit 02
        volatile bool crcerr; // CRC error detected in current packet; still delivered. bit 01
        volatile bool ovf;    // at least 1 packet was discarded between current & previous packet, bit 00
        // rx status word 2 bits
        //volatile uint8_t rbl; // appears at both bits 15:08 and 07:00.
    };

    // Target address information
    // in Normal mode, only the first is used
    // Reset all 14 MAC address slots to broadcast values
    // will populate mac address from mandatory parameter `p mac 00:01:02:03:04:05`
    // requires c++17
    // uint8_t mac_addrs[14][6] {[0 ... 13] = {[0 ... 5] = 0xFF}};
    uint8_t mac_addrs[14][6];

    // TAP device
    const char *clone_dev_name = "/dev/net/tun";
    int tap_fd = 0;
    std::string tap_name;

    // TODO: Control parameters, currently unused
    bool multicast = false;                  // enables multicast reception
    bool promiscuous = false;                // enables promiscuous mode
    uint32_t sanity_timeout = 4 * 60 * 1000; // default sanity timer is 4 minutes

    // TODO: MOP information

    struct DMARequest
    {
        uint32_t address;
        uint32_t count;
        bool write;
        uint16_t *buffer;
        bool timeout;
        uint32_t next_bdl_address;
        bool rx;
    };
    dma_request_c * _get_dma_req(DMARequest &request);
    void _dma_transfer(DMARequest &request);
    void _dma_wrrr(DMARequest &request);
    void _dma_wwwrrr(DMARequest &request, bool tx, bool rx);

    // Causes an interrupt if enabled in CSR
    void _maybe_interrupt(bool xi, bool ri, bool nx);

    void _worker_rx(void);
    void _worker_tap_rx(void);
    void _worker_tx(void);

    pthread_cond_t tx_cond = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t tx_mutex = PTHREAD_MUTEX_INITIALIZER;
    uint32_t _tx_stash_address = 0;
    char tx_buf[DELQA_BUFLEN];

    pthread_cond_t rx_cond = PTHREAD_COND_INITIALIZER;          // shared for setup_* vars
    pthread_mutex_t rx_mutex = PTHREAD_MUTEX_INITIALIZER;
    uint32_t _rx_stash_address = 0;
    char rx_tap_buf[DELQA_BUFLEN];
    char rx_buf[DELQA_BUFLEN];
    volatile bool setup_loopback = false;
    char setup_buf[DELQA_BUFLEN];
    volatile uint16_t setup_sz = 0;


    typedef std::vector<char> bufvec;
    std::queue<bufvec *> rx_packets;
    pthread_cond_t rx_queue_cond = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t rx_queue_mutex = PTHREAD_MUTEX_INITIALIZER;

    pthread_mutex_t dma_mutex = PTHREAD_MUTEX_INITIALIZER;

    std::string _exec(std::string cmd);
    void _teardown_tap(void);
    void _init_tap(void);

    std::vector<uint8_t> _split(const std::string &s, char delim);
    bool _update_mac(std::string val);

    void _update_QVAR(void);
    void _update_QCSR(void);

    struct timespec sleepreq = {0, 50'000'000}; // at least one 1/50th tick

// DELETE US LATER?

int has_ports(int protocol);
void dump_ports(int protocol, int count, const char* buffer);
void dump_packet_ipv4(int count, char* buffer);
void dump_packet_ipv6(int count, char* buffer);
void dump_packet(int count, char* buffer);
void hex(char* source, char* dest, ssize_t count);
#define BUFFLEN (4*1024)
const char HEX[16] = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
  'a', 'b', 'c', 'd', 'e', 'f',
};


};

#endif
