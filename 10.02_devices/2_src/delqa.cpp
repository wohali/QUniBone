/* 
    delqa.cpp: DELQA QBUS Ethernet adapter

    Copyright Joan Touzet 2025 via JS Enterprises, Inc.
    Contributed under the BSD 2-clause license.

 */

#include "logger.hpp"
#include "qunibus.h"
#include "qunibusadapter.hpp"
#include "qunibusdevice.hpp"
#include "delqa.hpp"

delqa_c::delqa_c() : qunibusdevice_c()
{
    set_workers_count(3);   // tx, TAP rx and rx are on separate threads

    // static config
    name.value = "delqa";
    type_name.value = "DELQA";
    log_label = "delqa";

    // base addr, slot, intr-vector, intr level
    // DELQA 1: 17774440, slot, 120, 4
    // DELQA 2: 17774460, slot, floating (rank 047), 4 
    // DELQA would rank below KXV11 and RXV21
    set_default_bus_params(017774440, 7, 0120, 4);

    // DELQA has 8 registers.
    register_count = 8;

    // Station Address (SA) 0 (read-only)
    QSA0_reg = &(this->registers[0]); // @  base addr
    strcpy(QSA0_reg->name, "QSA0");
    QSA0_reg->active_on_dati = false; // no controller state change on read
    QSA0_reg->active_on_dato = false;
    QSA0_reg->reset_value = 0;
    QSA0_reg->writable_bits = 0x0000; // read only

    // Station Address (SA) 1 (read-only)
    QSA1_reg = &(this->registers[1]); // @  base addr + 2
    strcpy(QSA1_reg->name, "QSA1");
    QSA1_reg->active_on_dati = false; // no controller state change on read
    QSA1_reg->active_on_dato = false;
    QSA1_reg->reset_value = 0;
    QSA1_reg->writable_bits = 0x0000; // read only

    // Receive BDL Low / SA 2 (read/write)
    QARL_reg = &(this->registers[2]); // @  base addr + 4
    strcpy(QARL_reg->name, "QARL");
    QARL_reg->active_on_dati = false; // no controller state change on read
    QARL_reg->active_on_dato = false; // or write
    QARL_reg->reset_value = 0;
    QARL_reg->writable_bits = 0xffff;

    // Receive BDL High / SA 3 (read/write)
    QARH_reg = &(this->registers[3]); // @  base addr + 6
    strcpy(QARH_reg->name, "QARH");
    QARH_reg->active_on_dati = false;
    QARH_reg->active_on_dato = true;  // writing to the high addr starts an RX operation
    QARH_reg->reset_value = 0;
    QARH_reg->writable_bits = 0xffff;
    
    // Transmit BDL Low / SA 4 (read/write)
    QATL_reg = &(this->registers[4]); // @  base addr + 10
    strcpy(QATL_reg->name, "QATL");
    QATL_reg->active_on_dati = false; // no controller state change on read
    QATL_reg->active_on_dato = false; // or write
    QATL_reg->reset_value = 0;
    QATL_reg->writable_bits = 0xffff;

    // Transmit BDL High / SA 5 (read/write)
    QATH_reg = &(this->registers[5]); // @  base addr + 12
    strcpy(QATH_reg->name, "QATH");
    QATH_reg->active_on_dati = false;
    QATH_reg->active_on_dato = true;  // writing to the high addr starts a TX operation
    QATH_reg->reset_value = 0;
    QATH_reg->writable_bits = 0xffff;
    
    // Vector Address Register (read/write)
    QVAR_reg = &(this->registers[6]); // @  base addr + 14
    strcpy(QVAR_reg->name, "QVAR");
    QVAR_reg->active_on_dati = false;
    QVAR_reg->active_on_dato = true;  // writing can change the vector addr
    QVAR_reg->reset_value = 0b1011'1111'1111'1100;
    QVAR_reg->writable_bits = 0b1010'0011'1111'1101;
                                     // bit 1 reserved, bits 12:10 and 14 read only

    // Control and Status Register (read/write)
    QCSR_reg = &(this->registers[7]); // @  base addr + 16
    strcpy(QCSR_reg->name, "QCSR");
    QCSR_reg->active_on_dati = false;
    QCSR_reg->active_on_dato = true;
    QCSR_reg->reset_value = 0b0010'0001'0011'0000;
    QCSR_reg->writable_bits = 0b1000'0111'1100'1011;
                                      // bit 11 reserved, bits 2,4,5,12,13,14 read only
}

delqa_c::~delqa_c()
{
    // TODO: any dynamic allocation needs freeing here
    _teardown_tap();
}

// *** private stuff ***

dma_request_c * delqa_c::_get_dma_req(DMARequest &request)
{
    if (request.rx)
    {
        return(&rx_dma_request);
    }
    else
    {
        return(&tx_dma_request);
    }
}
    

void delqa_c::_dma_transfer(DMARequest &request)
{
    pthread_mutex_lock(&dma_mutex);
    dma_request_c *dma_request = _get_dma_req(request);
INFO("dma_request = 0x%0x", dma_request);
    if (request.write)
    {
        // Write FROM buffer TO qunibus memory
        qunibusadapter->DMA(*dma_request, true,
            QUNIBUS_CYCLE_DATO,
            request.address,
            request.buffer,
            request.count);
    } else {
        // Read FROM qunibus memory TO buffer
        qunibusadapter->DMA(*dma_request, true,
            QUNIBUS_CYCLE_DATI, 
            request.address,
            request.buffer,
            request.count);
    }
    request.timeout = !dma_request->success;
    pthread_mutex_unlock(&dma_mutex);
}

// Write all ones to the first word then read the following three words.
// NOTE: request.buffer should point directly to destination for the read!
void delqa_c::_dma_wrrr(DMARequest &request)
{
    pthread_mutex_lock(&dma_mutex);
    uint16_t ones = 0xFFFF;
    request.timeout = false;                        // these 3 lines don't really matter
    request.count = 0;
    request.write = false;
    dma_request_c *dma_request = _get_dma_req(request);
    qunibusadapter->DMA(*dma_request, true,
        QUNIBUS_CYCLE_DATO,
        request.address,
        &ones,
        1
    );
    request.timeout = !dma_request->success;
    if (request.timeout) 
    {
        pthread_mutex_unlock(&dma_mutex);
        WARNING("_dma_wrrr: DMA timeout");
        return;
    }

    qunibusadapter->DMA(*dma_request, true,
        QUNIBUS_CYCLE_DATI,
        (request.address + 2),
        (request.buffer + 1),
        3
    );
    request.timeout = !dma_request->success;
    pthread_mutex_unlock(&dma_mutex);
}

// Write two status words and then perform wrrr on the subsequent BDL
// request.address should point to the first word to write (status word 1)
// request.buffer should point to the entire buffer
void delqa_c::_dma_wwwrrr(DMARequest &request, bool tx, bool rx)
{
    pthread_mutex_lock(&dma_mutex);
    uint16_t *buf = request.buffer;
    dma_request_c *dma_request = _get_dma_req(request);
INFO("_dma_wwwrrr start, rx = %d", request.rx);

    qunibusadapter->DMA(*dma_request, true,
        QUNIBUS_CYCLE_DATO,
        request.address,
        (buf + 4),
        2
    );
    request.timeout = !dma_request->success;
    if (request.timeout) 
    {
        WARNING("_dma_wwwrrr: DMA timeout");
        pthread_mutex_unlock(&dma_mutex);
        return;
    }

    _maybe_interrupt(tx, rx, false);

    pthread_mutex_unlock(&dma_mutex);

    for (auto ctr = 0; ctr < 6; ctr++) buf[ctr] = 0;
    request.address = request.next_bdl_address; 
    request.buffer = buf;
    _dma_wrrr(request);
INFO("_dma_wwwrrr finish, rx = %d", request.rx);
}

void delqa_c::_maybe_interrupt(bool xi = false, bool ri = false, bool nx = false)
{
    pthread_mutex_lock(&on_after_register_access_mutex);
    if (xi) _xi = true;
    if (ri) _ri = true;
    if (nx)
    {
         _nx = true;
         _xl = true;
         _rl = true;
    }
    _update_QCSR();

    if (_ie)
    {
        INFO("actual interrupt, xi=%d, ri=%d, nx=%d", _xi, _ri, _nx);
        qunibusadapter->INTR(intr_request, NULL, 0); // todo: link to interrupt register?
    }
    pthread_mutex_unlock(&on_after_register_access_mutex);
    //nanosleep(&sleepreq, nullptr);  // arbitrary sleep
}

// Background Ethernet TX worker
void delqa_c::_worker_tx(void)
{
    DMARequest request = { 0 };
    uint16_t buf[6] = { 0 };
    uint16_t dbuf[DELQA_BUFLEN] = { 0 };
    uint32_t bdl_addr = 0;
    uint16_t tx_len = 0;
    bool valid = false;                         // valid BDL
    bool chain = false;                         // chained BDL
    bool eom   = false;                         // end of message
    bool setup = false;                         // setup BDL
    bool eol   =  true;                         // end of BDL

    worker_init_realtime_priority(rt_device);
    pthread_mutex_lock(&tx_mutex);

    while (!workers_terminate)
    {
INFO("worker_tx at top");
        if (eol)
        {
            // Previously hit the end of a BDL, need to be re-poked
            // pthread_cond_wait unlocks the locked mutex and blocks until cond is signaled
            // pthread_cond_wait never returns an error code
            int Res [[gnu::unused]] = pthread_cond_wait(
                    &tx_cond,
                    &tx_mutex
            );

INFO("worker_tx awake");
            // WRRR the first BDL from QATL/QATH
            request.address = _tx_stash_address;
            request.buffer = buf;
            request.write = false;
            _dma_wrrr(request);
            if (request.timeout)
            {
                _maybe_interrupt(false, false, true);
                continue;
            }
            tx_len = 0;
            eol = false;
        }
        // otherwise, buf[] already contains the latest bdl

        bdl_addr = request.address;
        request.address = ((buf[1] & 0b111111) << 16) | buf[2];
        int16_t word_count = -((int16_t) buf[3]) % 1024;
        // EK_DELQA-UG-002 p. 3-18: WORD COUNT = (BYTE COUNT + H + L) / 2, so:
        uint8_t l = (buf[1] >> 7) & 0x1;        // ends on a low byte; throw away last byte
        uint8_t h = (buf[0] >> 6) & 0x1;        // starts on a high byte; throw away first byte
        uint16_t byte_count = (word_count * 2) - h - l;
        valid = (buf[1] >> 15) & 1;
        chain = (buf[1] >> 14) & 1;
        eom   = (buf[1] >> 13) & 1;
        setup = (buf[1] >> 12) & 1;

        if (!valid)
        {
            if (chain)                          // reserved!
            {
                ERROR("tx: BDL valid/chain is reserved value 1?!");
            } 
INFO("tx invalid BDL, setting _xl");
            pthread_mutex_lock(&on_after_register_access_mutex);
            _xl = true;
            _update_QCSR();
            pthread_mutex_unlock(&on_after_register_access_mutex);
            tx_len = 0; 
            eol = true;
            continue;
        }

        if (chain)
        {
INFO("tx chained descriptor");
            // Must update status words for every BDL.
            //buf[4] = 0b1100'0000'0000'0000;     // buffer used but not the last segment of a packet
            buf[4] = 0;
            buf[5] = 0;
            request.next_bdl_address = request.address;
            request.address = bdl_addr + 8;
            request.buffer = buf;
            _dma_wwwrrr(request, false, false);
            if (request.timeout)
            {
                _maybe_interrupt(false, false, true);
                eol = true;                     // start over
            }
            continue;
        }

        // process the valid, non-chained BDL
        DEBUG_FAST("tx handling valid bdl 0x%0x, wc=%d", request.address, word_count);

        // get the packet data
        request.count = word_count;
        request.buffer = dbuf;
        request.timeout = false;
        _dma_transfer(request);
        if (request.timeout)
        {
            _maybe_interrupt(false, false, true);
            continue;
        }

        // dbuf[h] lets us skip the first byte if we are not aligned (start on 0 or 1).
        memcpy((void *) &tx_buf[tx_len], (void *) &dbuf[h], sizeof(uint8_t) * byte_count);
        tx_len += byte_count;

        // we always have to set the status words, unfortunately.
        buf[4] = 0;
        buf[5] = 0;

        if (setup)
        {
INFO("tx setup packet, looping back");
            // TODO: handle setup BDL (list of Ethernet destination addresses & control info)
            //     * setup packet is looped back and must be received by the rx thread
            //     * first 127 bytes always overwrite the target addresses
            //     * setup packets between 128-255 bytes also include control params
            //     * a 256-byte packet instead includes MOP element blocks (MEBs)
            //     * byte count is 2's complement of buf[3] as well as H/L bits
            pthread_mutex_lock(&rx_queue_mutex);
            setup_loopback = 1;
            setup_sz = byte_count;
            memcpy((void *)setup_buf, (void *)tx_buf, sizeof(uint8_t) * byte_count);
            pthread_cond_signal(&rx_queue_cond);
            pthread_mutex_unlock(&rx_queue_mutex);
            
            tx_len = 0;
        }
        else                                    // regular BDL
        {
            if (eom)                            // last segment of a packet
            {
INFO("tx full packet");
                // add CRC
                // crc32(initial_value=0, buf, len)
                uint32_t crc = crc32(0, (unsigned char *) tx_buf, tx_len);
                // if need to chunk, iterate:
                // crc = crc32(crc, buf, len)
                memcpy((void *) &tx_buf[tx_len], (void *)&crc, sizeof(uint32_t));
                tx_len += sizeof(uint32_t);     // should be 4
#if 1
                char dumper[2*DELQA_BUFLEN + 1];
                hex(tx_buf, dumper, tx_len);
                printf("TX sending packet: %s\n", dumper);
#endif
                // send the entire packet
                if (write(tap_fd, (void *)tx_buf, sizeof(char) * tx_len) == -1)
                {
                    buf[4] = 1<<14;             // Chain=1, xmit experienced an error
                }
                tx_len = 0;
            }
            else                                // not the last segment in a packet
            {
INFO("tx partial packet");
                buf[4] = 0b1100'0000'0000'0000; // buffer used but not the last segment of a packet
            }
        }

        // DELQA will continue to fetch and process BDLs until the valid bit is unclear.
        // Must update status words for every BDL.
        request.address = bdl_addr + 8;
        request.buffer = buf;
        request.next_bdl_address = bdl_addr + 12;
        // DELQA interrupts on every finished packet after updating status words
INFO("tx _dma_wwwrrr now");
        _dma_wwwrrr(request, (eom | setup), false);
        if (request.timeout)
        {
            _maybe_interrupt(false, false, true);
            eol = true;
            continue;
        }
    }
}

int delqa_c::has_ports(int protocol)
{
  switch(protocol) {
  case IPPROTO_UDP:
  case IPPROTO_UDPLITE:
  case IPPROTO_TCP:
    return 1;
  default:
    return 0;
  }
}

void delqa_c::dump_ports(int protocol, int count, const char* buffer)
{
  if (!has_ports(protocol))
    return;
  if (count < 4)
    return;
  uint16_t source_port;
  uint16_t dest_port;
  memcpy(&source_port, buffer, 2);
  source_port = htons(source_port);
  memcpy(&dest_port, buffer + 2, 2);
  dest_port = htons(dest_port);
  fprintf(stderr, " sport=%u, dport=%d\n", (unsigned) source_port, (unsigned) dest_port);
}

void delqa_c::dump_packet_ipv4(int count, char* buffer)
{
  if (count < 20) {
    fprintf(stderr, "IPv4 packet too short\n");
    return;
  }

  char buffer2[2*BUFFLEN + 1];
  hex(buffer, buffer2, count);

  int protocol = (unsigned char) buffer[9];
  struct protoent* protocol_entry = getprotobynumber(protocol);

  unsigned ttl = (unsigned char) buffer[8];

  fprintf(stderr, "IPv4: src=%u.%u.%u.%u dst=%u.%u.%u.%u proto=%u(%s) ttl=%u\n",
    (unsigned char) buffer[12], (unsigned char) buffer[13], (unsigned char) buffer[14], (unsigned char) buffer[15],
    (unsigned char) buffer[16], (unsigned char) buffer[17], (unsigned char) buffer[18], (unsigned char) buffer[19],
    (unsigned) protocol,
    protocol_entry == NULL ? "?" : protocol_entry->p_name, ttl
    );
  dump_ports(protocol, count - 20, buffer + 20);
  fprintf(stderr, " HEX: %s\n", buffer2);
}

void delqa_c::dump_packet_ipv6(int count, char* buffer)
{
  if (count < 40) {
    fprintf(stderr, "IPv6 packet too short\n");
    return;
  }

  char buffer2[2*BUFFLEN + 1];
  hex(buffer, buffer2, count);

  int protocol = (unsigned char) buffer[6];
  struct protoent* protocol_entry = getprotobynumber(protocol);

  char source_address[33];
  char destination_address[33];

  hex(buffer + 8, source_address, 16);
  hex(buffer + 24, destination_address, 16);

  int hop_limit = (unsigned char) buffer[7];

  fprintf(stderr, "IPv6: src=%s dst=%s proto=%u(%s) hop_limit=%i\n",
    source_address, destination_address,
    (unsigned) protocol,
    protocol_entry == NULL ? "?" : protocol_entry->p_name,
    hop_limit);
  dump_ports(protocol, count - 40, buffer + 40);
  fprintf(stderr, " HEX: %s\n", buffer2);
}

void delqa_c::dump_packet(int count, char* buffer)
{
  char version = ((unsigned char) buffer[0]) >> 4;
  if (version == 4) {
    dump_packet_ipv4(count, buffer);
  } else if (version == 6) {
    dump_packet_ipv6(count, buffer); 
  } else {
    fprintf(stderr, "Unknown packet version\n");
  }
}

void delqa_c::hex(char* source, char* dest, ssize_t count)
{
  for (ssize_t i = 0; i < count; ++i) {
    char data = source[i];
    dest[2 * i] = HEX[data >> 4];
    dest[2 * i + 1] = HEX[data & 15];
  }
  dest[2 * count] = '\0';
}

// Background Ethernet RX worker
void delqa_c::_worker_rx(void)
{
    uint16_t buf[6] = { 0 };
    bufvec *qbuf = nullptr;
    DMARequest request = { 0 };
    uint32_t bdl_addr = 0;
    uint16_t bdl_words = 0;
    uint16_t remaining = 0;
    int16_t bufsz = 0;
    bool valid = false;                         // valid BDL
    bool chain = false;                         // chained BDL
    bool eol   =  true;                         // end of BDL
    request.rx = true;

    worker_init_realtime_priority(rt_device);
    pthread_mutex_lock(&rx_mutex);

    while (!workers_terminate)
    {
INFO("worker_rx at top");
        while (!_re)                        // need _re = true
        {
            nanosleep(&sleepreq, nullptr);  // arbitrary sleep
            if (workers_terminate) break;
        }
        if (workers_terminate) break;

        if (eol)
        {
            // pthread_cond_wait unlocks the locked mutex and blocks until cond is signaled
            // pthread_cond_wait never returns an error code
            int res [[gnu::unused]] = pthread_cond_wait(
                    &rx_cond,
                    &rx_mutex
            );
            // WRRR the first BDL from QARL/QARH
            request.address = _rx_stash_address;
            request.buffer = buf;
            _dma_wrrr(request);
            if (request.timeout)
            {
                _maybe_interrupt(false, false, true);
                continue;
            }
            remaining = 0;
            eol = false;
        }
        // process BDL
        bdl_addr = request.address;
        request.address = ((buf[1] & 0b111111) << 16) | buf[2];
        bdl_words = -((int16_t) buf[3]);
        valid = (buf[1] >> 15) & 1;
        chain = (buf[1] >> 14) & 1;

        if (!valid)
        {
            if (chain)                          // reserved!
            {   
                ERROR("rx: BDL valid/chain is reserved value 1?!");
            }
INFO("rx: invalid BDL, setting _rl");
            pthread_mutex_lock(&on_after_register_access_mutex);
            _rl = true;
            _update_QCSR();
            pthread_mutex_unlock(&on_after_register_access_mutex);
            if (remaining > 0)
            {
WARNING("rx: dropping remaining %d bytes", remaining);
                remaining = 0;
            }
            eol = true;
            continue;
        }

        if (chain)
        {
INFO("rx: chained descriptor");
            // unchain my heart, baby let me be
            buf[4] = 0b1100'0000'0000'0000;     // buffer used but not last segment
            buf[5] = 0;
            request.next_bdl_address = request.address;
            request.address = bdl_addr + 8;
            request.buffer = buf;
            _dma_wwwrrr(request, false, false);
            if (request.timeout)
            {
                _maybe_interrupt(false, false, true);
                eol = true;
            }
            continue;                           // pretend we just got here
        }
        // BDL now ready
INFO("worker_rx has valid BDL");

        if (remaining == 0)                     // need a new pkt
        {
            pthread_mutex_lock(&rx_queue_mutex);
            if (!setup_loopback || rx_packets.size() == 0)
            {
                // wait for new pkt in the rx queue or setup pkt
                int Res [[gnu::unused]] = pthread_cond_wait(
                        &rx_queue_cond,
                        &rx_queue_mutex
                );
            }
            if (workers_terminate)              // terminate sends all cond signals
            {
                 pthread_mutex_unlock(&rx_queue_mutex);
                 break;
            }
            if (setup_loopback)
            {
INFO("rx: setup pkt");
                bufsz = setup_sz;
                remaining = (uint16_t) std::ceil(bufsz / 2);
                request.buffer = (uint16_t *) setup_buf;
            }
            else
            {
                if (rx_packets.size() == 0)    // probably a reset
                {
                     INFO("rx: signal but empty queue, resetting");
                     eol = true;
                     pthread_mutex_unlock(&rx_queue_mutex);
                     continue;
                }
                qbuf = rx_packets.front();
                bufsz = qbuf->size();
                DEBUG_FAST("rx: transferring bufvec %x, size %d", qbuf, bufsz);
                memcpy((void *) rx_buf, (void *) qbuf->data(), sizeof(uint8_t) * bufsz);
#if 1
                // cannot DEBUG_FAST here because too big for the macro
                char dumper[2*DELQA_BUFLEN + 1];
                hex(rx_buf, dumper, bufsz);
                printf("RX got packet: %s\n", dumper);
#endif
                remaining = (uint16_t) std::ceil(bufsz / 2);
                request.buffer = (uint16_t *) rx_buf;

                // pop buffer out of queue
                rx_packets.pop();                   // only the pointer
                delete qbuf;
            }
            pthread_mutex_unlock(&rx_queue_mutex);
        }
        else
        // What if we have a packet we can't fully deliver to the host because
        //       the provided buffers in the rx ring aren't enough space?
        // > No interrupt is generated if there are not enough valid receive buffers in the Receive BDL
        // > to accommodate a complete packet.
        // TODO: not enough room == record dropped packet
        {
            request.buffer = (uint16_t *) rx_buf +
                    (sizeof (uint16_t) * ((int) std::ceil(bufsz/2) - remaining));
        }

INFO("rx: starting DMA");
        // write as much data as possible to the buffer
        request.count = std::min(remaining, bdl_words);
        request.write = true;
        request.timeout = false;
        _dma_transfer(request);
        if (request.timeout)
        {
            _maybe_interrupt(false, false, true);
            eol = true;
            continue;
        }

        remaining -= request.count;
INFO("rx DMA of %d words done, %d remaining", request.count, remaining);

        // prep to update the status words
        request.address = bdl_addr + 8;
        request.buffer = buf;
        request.next_bdl_address = bdl_addr + 12;
        request.timeout = false;

        // TODO: set buf[4] bit 0 if any packets dropped
        if (remaining == 0)                     // full packet
        {
            if (setup_loopback) 
            {
                buf[4] = 0b0010'0111'0000'0000;
                setup_loopback = false;         // setup pkt processed
            }
            else
            {
                bufsz -= 60;
                buf[4] = (bufsz & 0b111'0000'0000);
            }
            buf[5] = (bufsz & 0b1111'1111) << 8 |
                     (bufsz & 0b1111'1111);
        }
        else
        {
            buf[4] = 0b1100'0000'0000'0000;     // buffer used but not last segment
            // TODO: what does real hardware do here?
            buf[4] |= (request.count*2) & 0b111'0000'0000;      // RBL<10:08>
            buf[5] = ((request.count*2) & 0b1111'1111) << 8 |
                     ((request.count*2) & 0b1111'1111);
        }

        // DELQA interrupts on every finished packet after updating status words
INFO("rx _dma_wwwrrr now");
        _dma_wwwrrr(request, false, true);
        if (request.timeout)
        {   
            _maybe_interrupt(false, false, true);
            eol = true;
            continue;
        }
    }
}

// TAP RX worker
void delqa_c::_worker_tap_rx(void)
{
    ssize_t res;
    // char dumper[2*DELQA_BUFLEN + 1];

    pollfd tappoll[] = { { 0, POLLIN, 0 } };

    worker_init_realtime_priority(rt_device);

    while (!workers_terminate)
    {
        if (tap_fd != tappoll[0].fd) tappoll[0].fd = tap_fd;
        if (tappoll[0].fd == 0)
        {
            sleep(1);                   // check every 1s for enable
        }
        else
        {
            res = poll(tappoll, (nfds_t) 1, 50);        // 50ms timeout
            if (res > 0)                // data ready
            {
                res = read(tap_fd, rx_tap_buf, sizeof(char) * DELQA_BUFLEN);
                if (res <= 0)
                {
                    WARNING("_worker_tap_rx read error: %d", errno);
                    sleep(5);
                }
                if ((rx_tap_buf[12] == 0x86) && (rx_tap_buf[13] == 0xDD))        // filter ipv6
                {
                    continue;
                }
                else if (_re)
                {
                    // copy read packets into a vector and stick on queue
                    // _worker_rx will pull off queue when activated
                    bufvec *buf = new bufvec(rx_tap_buf, rx_tap_buf + res / sizeof(rx_tap_buf[0]));
                    pthread_mutex_lock(&rx_queue_mutex);
INFO("Pushing bufvec %x, size %d, queue now %d", buf, buf->size(), rx_packets.size() + 1);
                    if (rx_packets.size() >= DELQA_MAXQ)
                    {
                        // TODO: set overflow bit and report in next BDL
                        WARNING("Dropping packetqueue is full");
                    }
                    else
                    {
                        rx_packets.push(buf);
                        pthread_cond_signal(&rx_queue_cond);
                    }
                    pthread_mutex_unlock(&rx_queue_mutex);
                }
            }
            else if (res == -1)
            {
                WARNING("_worker_tap_rx poll error: %d", errno);
                sleep(5);
            }
       

        }
    }
    // wake up the rx queue on exit
    pthread_mutex_lock(&rx_queue_mutex);
    pthread_cond_signal(&rx_queue_cond);
    pthread_mutex_unlock(&rx_queue_mutex);
}

std::string delqa_c::_exec(std::string cmd)
{
    // FYI: only grabs stdout, not stderr.
    std::array<char, 1024> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

void delqa_c::_teardown_tap()
{
    if (tap_fd > 0)
    {
        std::string cmd = "ip link set dev " + tap_name + " nomaster";
        static_cast<void>(_exec(cmd));
        close(tap_fd);
        INFO("Ethernet TAP %s closed", tap_name.c_str());
    }
    tap_fd = 0;
    tap_name = "";
    _ok = false;
    _update_QCSR();
}

void delqa_c::_init_tap()
{
    _teardown_tap();

    // First, create the TAP interface.

    // TAP interfaces are fds that act like pipes.
    // From the command line, `ip link show dev tap0` will show:
    //   RX: bytes from the application (us) towards the Linux host
    //   TX: bytes from the Linux host to us.
    // TX dropped packets mean we were too slow to process and Linux dropped data.
    // do not initialise if no proper MAC address has been set
    if (memcmp(mac_addrs[0], (const char []) { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }, 6) == 0)
    {
        return;
    }
    tap_fd = open(clone_dev_name, O_RDWR | O_CLOEXEC);
    if (tap_fd < 0)
    {
        ERROR("cannot open %s (%d)", clone_dev_name, errno);
        return;
    }
    struct ifreq ifr = {};
    // hard coded right now, could use "tap%d" or even '\0' in the future
    strncpy(ifr.ifr_name, "tap0", IFNAMSIZ);
    // TAP interface, no 4-byte tun_pi prefix, use virtio_net_hdr instead
    // ifr.ifr_flags = IFF_TAP | IFF_NO_PI | IFF_VNET_HDR;
    ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
    int res = ioctl(tap_fd, TUNSETIFF, &ifr);
    if (res != 0)
    {
        ERROR("cannot ioctl(TUNSETIFF): %d", errno);
    }
    tap_name = ifr.ifr_name;
    INFO("Ethernet TAP %s opened", tap_name.c_str());

    // ioctl TUNSETLINK if you are sending non-Ethernet packets

    /*
    // set up VNET_HDR size and offloads if possible
    int len = 12;               // room for virtio_net_hdr_v1, see virtio_net.h
    res = ioctl(tap_fd, TUNSETVNETHDRSZ, &(int){len});
    if (res != 0)
    {
        ERROR("cannot ioctl(TUNSETVNETHDRSZ): %d", errno);
    }
    // offload here means *we* have to confirm RX checksums.
    // L4 packet checksum offload, TCP segmentation offload for IPv4 packets
    unsigned off_flags = TUN_F_CSUM | TUN_F_TSO4;
    res = ioctl(tap_fd, TUNSETOFFLOAD, off_flags);
    if (res != 0)
    {
        ERROR("cannot ioctl(TUNSETOFFLOAD): %d", errno);
    }
    */
    // Try to add outbound send buffer of 1MB
    // see: https://bugzilla.redhat.com/show_bug.cgi?id=508861
    int sndbuf = 1024 * 1024;
    res = ioctl(tap_fd, TUNSETSNDBUF, &sndbuf);
    if (res != 0)
    {
        ERROR("cannot ioctl(TUNSETSNDBUF): %d", errno);
    }

    // Second, add it to the bridge.
    // Previously we created the bridge here, but this has since been moved to
    // a new file under /etc/network/interfaces.d/ . This ensures the bridges
    // are set up at boot time, and that they are predictably named:
    //
    // auto lo
    // iface lo inet loopback
    //
    // auto eth0
    // iface eth0 inet manual
    // 
    // auto br0
    // iface br0 inet dhcp
    //     bridge_ports eth0
    //     bridge_waitport 0
    //     bridge_fd 0
    // 
    // # use of 198.19.0.1/24: see RFC 2544
    // auto tap1
    // iface tap1 inet manual
    //     pre-up ip tuntap add tap1 mode tap user root
    //     pre-up ip addr add 198.19.0.1/24 dev tap1
    //     up ip link set dev tap1 up
    //     post-down ip link del dev tap1
    // 
    // iface br1 inet static
    //     bridge_ports tap1
    //     address 198.19.0.1/24
    //     broadcast 198.19.0.255
    //     gateway 198.19.0.1
    //     bridge_stp off
    //     bridge_waitport 0
    //     bridge_fd 0
    //
    // The setup is:
    // br0: Bridge with physical eth0 device on it.
    //      Wi-Fi is not included because APs reject frames that spoof
    //      source addresses.
    //      Adding tap0 to this will directly expose the QBus machine.
    // br1: Bridge with local networking to the QBone only.
    //      This allows protected access between QBone and the
    //      PDP-11 or VAX only. Users can choose to configure NAT
    //      and other services (such as port redirection or the HECnet bridge).
    // 
    // set up a bridge and add the TAP and the physical Ethernet device to it.
    // also, persist `net.ipv4.ip_forward=1` in /etc/sysctl.d/50-ipv4-forward.conf
    // We do this all via iproute2 because it's *painful* via just ioctls.

    // TODO: support multiple MAC addresses via multiple TAP interfaces on the bridge

    if (mac_addr.new_value != mac_addr.value)
    {
        mac_addr.value = mac_addr.new_value;
    }

    // TODO: actually pay attention to these return values rather than assuming success
    std::string cmd = "ip link set dev " + tap_name + " address " + mac_addr.value + " ";
    // IMPORTANT: mac addr of tap0 needs to be different than mac addr given to qhost
    // (so just add 1)
    char onemore = (mac_addrs[0][5] + 1) % 256;
    hex(&onemore, &cmd[cmd.length()-3], 1);
    static_cast<void>(_exec(cmd));

    if (bridge_name.new_value != bridge_name.value)
    {
        bridge_name.value = bridge_name.new_value;
    }

    cmd = "ip link set dev " + tap_name + " up master " + bridge_name.value;
    static_cast<void>(_exec(cmd));

    // optional: support iptables filtering on the bridge? not needed?
    //cmd = "ip link set " + bridge_name.value + " type bridge nf_call_iptables 1";
    //static_cast<void>(_exec(cmd));
    INFO("Ethernet TAP %s bridged with %s", tap_name.c_str(), bridge_name.value.c_str());

    if (bridge_name.value == "br0")
    {
        WARNING("***WARNING***: Your QBus machine is now directly on the network");
        WARNING("               connected to your QBone's Ethernet port, which exposes");
        WARNING("               it to possible attack.");
        WARNING("               DO NOT ALLOW THE INTERNET DIRECT ACCESS TO YOUR QBONE.");
        WARNING("               YOU HAVE BEEN WARNED.\n");
    }

    _ok = true;
    _update_QCSR();
}

std::vector<uint8_t> delqa_c::_split(const std::string &s, char delim) {
    std::vector<uint8_t> result;
    std::stringstream ss (s);
    std::string item;

    while (getline (ss, item, delim)) {
        uint8_t val = std::stoi(item, nullptr, 16);
        result.push_back (val);
    }

    return result;
}

// update our stored Eth PHY MAC and the SA* registers
bool delqa_c::_update_mac(std::string val)
{
    std::vector<uint8_t> v = _split((const std::string) val, ':');
    uint8_t idx = 0;
    for (auto octet : v) {
        mac_addrs[0][idx++] = octet;
    }
    if (handle)                                     // no regs before handle
    {
        set_register_dati_value(QSA0_reg, (uint16_t) mac_addrs[0][0], "update_mac");
        set_register_dati_value(QSA1_reg, (uint16_t) mac_addrs[0][1], "update_mac");
        set_register_dati_value(QARL_reg, (uint16_t) mac_addrs[0][2], "update_mac");
        set_register_dati_value(QARH_reg, (uint16_t) mac_addrs[0][3], "update_mac");
        set_register_dati_value(QATL_reg, (uint16_t) mac_addrs[0][4], "update_mac");
        set_register_dati_value(QATH_reg, (uint16_t) mac_addrs[0][5], "update_mac");
    }
    if (_ok)
    {
        _init_tap();    // will handle teardown if necessary
    }
    return true;
}

void delqa_c::_update_QVAR(void)
{
    uint16_t new_QVAR =
        (_itb ? 0x0001 : 0x0000) |
        intr_vector.value << 2 |
        _sts << 10 |
        (_rst ? 0x2000 : 0x0000) |
        (_s4  ? 0x4000 : 0x0000) |
        (_ms  ? 0x8000 : 0x0000);

    if (handle) {
        set_register_dati_value(
            QVAR_reg,
            new_QVAR,
            "update_QVAR"
        );
    }
}

void delqa_c::_update_QCSR(void)
{
    uint16_t new_QCSR =
        (_re  ? 0x0001 : 0x0000) |
        (_sr  ? 0x0002 : 0x0000) |
        (_nx  ? 0x0004 : 0x0000) |
        (_bd  ? 0x0008 : 0x0000) |
        (_xl  ? 0x0010 : 0x0000) |
        (_rl  ? 0x0020 : 0x0000) |
        (_ie  ? 0x0040 : 0x0000) |
        (_xi  ? 0x0080 : 0x0000) |
        (_il  ? 0x0100 : 0x0000) |
        (_el  ? 0x0200 : 0x0000) |
        (_se  ? 0x0400 : 0x0000) |
        // CSR11 is always zero
        (_ok  ? 0x1000 : 0x0000) |
        (_ca  ? 0x2000 : 0x0000) |
        (_pe  ? 0x4000 : 0x0000) |
        (_ri  ? 0x8000 : 0x0000);

    if (handle) {
        set_register_dati_value(
            QCSR_reg,
            new_QCSR,
            "update_QCSR"
        );
    }
}

// *** public stuff ***

void delqa_c::on_after_uninstall(void)
{
    flush_packet_queue();
    _teardown_tap();
}

// handles configuration parameter changes
bool delqa_c::on_param_changed(parameter_c *param) 
{
    // no unique "enable" logic
    if (param == &bridge_name) {
        _init_tap();    // will handle teardown if necessary
        return true;
    } else if (param == &mac_addr) {
        return _update_mac(mac_addr.new_value);
    } else if (param == &priority_slot) {
        tx_dma_request.set_priority_slot(priority_slot.new_value);
        // prevent stepping on tx DMAs, go 1 higher on priority slot
        rx_dma_request.set_priority_slot(priority_slot.new_value + 1);
        intr_request.set_priority_slot(priority_slot.new_value);
    } else if (param == &intr_level) {
        intr_request.set_level(intr_level.new_value);
    } else if (param == &intr_vector) {
        intr_request.set_vector(intr_vector.new_value);
        _update_QVAR();
    }   
    // return false if illegal parameter value.
    return qunibusdevice_c::on_param_changed(param) ; // more actions (for enable)
}

// after QBUS install, device is reset by DCLO/DCOK cycle
void delqa_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge)
{
    UNUSED(aclo_edge);
    UNUSED(dclo_edge);
    reset();
}

// QBUS INIT: clear all registers
void delqa_c::on_init_changed(void)
{
    if (init_asserted) reset();
}

void delqa_c::reset(void)
{
    INFO("resetting");
    // This will reset the DATI values to their defaults.
    // We then need to reset our copy of the values to correspond.
    reset_unibus_registers();

    // Lower bytes of first six register locations should contain the eth phy MAC octets
    set_register_dati_value(QSA0_reg, (uint16_t) mac_addrs[0][0], "update_mac");
    set_register_dati_value(QSA1_reg, (uint16_t) mac_addrs[0][1], "update_mac");
    set_register_dati_value(QARL_reg, (uint16_t) mac_addrs[0][2], "update_mac");
    set_register_dati_value(QARH_reg, (uint16_t) mac_addrs[0][3], "update_mac");
    set_register_dati_value(QATL_reg, (uint16_t) mac_addrs[0][4], "update_mac");
    set_register_dati_value(QATH_reg, (uint16_t) mac_addrs[0][5], "update_mac");

    // VAR, but with undefined interrupt vector
    _itb = false;
    _sts = 0b111;
    _rst = true;
    _ms = true;
    _s4 = true;
    _update_QVAR();

    // CSR: all the defaults except for _ok which depends on TAP status
    _re = false;
    _sr = false;
    _nx = false;
    _bd = false;
    _xl = true;
    _rl = true;
    _ie = false;
    _xi = false;
    _il = true;
    _el = false;
    _se = false;
    // CSR11 is always false
    _ok = (tap_fd > 0);
    _ca = true;
    _pe = false;
    _ri = false;
    _update_QCSR();

    flush_packet_queue();
}

void delqa_c::flush_packet_queue(void)
{
    pthread_mutex_lock(&rx_queue_mutex);
    while (!rx_packets.empty())
    {
        delete rx_packets.front();
        rx_packets.pop();
    }
    pthread_cond_signal(&rx_queue_cond);
    pthread_mutex_unlock(&rx_queue_mutex);
}
    

void delqa_c::worker(unsigned instance)
{
    if (instance == 0)
    {
        _worker_tx();
    }
    else if (instance == 1)
    {
        _worker_rx();
    }
    else
    {
        _worker_tap_rx();
    }

}

void delqa_c::on_after_register_access(
    qunibusdevice_register_t *device_reg,
    uint8_t unibus_control,
    DATO_ACCESS access)
{
    UNUSED(unibus_control);     // no regs are active on both DATI and DATO
    UNUSED(access);

    pthread_mutex_lock(&on_after_register_access_mutex);

    switch (device_reg->index)
    {
        case 3:                 // QARH
            // TODO: early error if no tap device
            _rl = false;
INFO("clearing _rl");
            _update_QCSR();
            // pull the BDL address and hand it to a worker
            pthread_mutex_lock(&rx_mutex);
            _rx_stash_address = get_register_dato_value(QARL_reg) | (get_register_dato_value(QARH_reg) << 16);
            pthread_cond_signal(&rx_cond);
            pthread_mutex_unlock(&rx_mutex);
            break;
        case 5:                 // QATH
            // TODO: early error if no tap device
            _xl = false;
INFO("clearing _xl");
            _update_QCSR();
            // pull the BDL address and hand it to a worker
            pthread_mutex_lock(&tx_mutex);
            _tx_stash_address = get_register_dato_value(QATL_reg) | (get_register_dato_value(QATH_reg) << 16);
            pthread_cond_signal(&tx_cond);
            pthread_mutex_unlock(&tx_mutex);
            break;
        case 6:                 // QVAR
            // _itb can be changed if requested; only DEQNA would hold VAR00=0 even if a 1 is written
            _itb = (bool) (QVAR_reg->active_dato_flipflops & 0x1);
            // vector is QVAR<09:02> with <01:00>=0
            intr_request.set_vector(QVAR_reg->active_dato_flipflops & 0b11'1111'1100);
            // TODO: _rst unimplemented self-test
            //       easy way out: thread that clears _rst 5 seconds later, should happen on reset and when bit set
            // _ms unimplemented DELQA-lock mode (always normal)
            // _s4 unimplemented (no remote boot)
            _update_QVAR();
            break;
        case 7:                 // QCSR
            // could change any of the control bits
            // A zero value, null pointer value, or null member pointer value is converted to false; any other value is converted to true
            bool new_re = (bool) (QCSR_reg->active_dato_flipflops & 0x1);
            if (new_re != _re)
            {
INFO("_re set to %d", new_re);
                _re = new_re;
            }
            bool old_sr = _sr;
            _sr = (bool) (QCSR_reg->active_dato_flipflops & 0x2);
            // _nx (& 0x4) cleared only by reset or via _xi
            bool old_bd = _bd;
            _bd = (bool) (QCSR_reg->active_dato_flipflops & 0x8);

            // _xl is cleared only by writing the Tx BDL address
            // _rl is cleared only by writing the Rx BDL address
            _ie = (bool) (QCSR_reg->active_dato_flipflops & 0x40);

            bool new_xi = (bool) (QCSR_reg->active_dato_flipflops & 0x80);
            if (new_xi)
            {
                if (_xi) 
                {
                    _xi = false;
INFO("clearing _xi");
                }
                if (_nx) _nx = false;
            }

            _il = (bool) (QCSR_reg->active_dato_flipflops & 0x100);
            // TODO: if setting _el, QSA0/QSA1 become the checksum of the Ethernet physical address
            _el = (bool) (QCSR_reg->active_dato_flipflops & 0x200);
            _se = (bool) (QCSR_reg->active_dato_flipflops & 0x400);
            if (_se) {
                WARNING("SANITY TIMER ENABLE ATTEMPT!");
            }

            // _ok is set when the tunnel is established
            // _ca is always set (loopback modes unimplemented)
            // _pe is always clear (DEQNA-lock unimplemented)
            bool new_ri = (bool) (QCSR_reg->active_dato_flipflops & 0x8000);
            if (new_ri && _ri)
            {
                _ri = false;
INFO("clearing _ri");
            }

            // TODO: inform rx thread of _re changed values ?
            if (old_sr && !_sr)
            {
                reset();
                break;
            }
            if (old_bd && !_bd && !_re && _el)
            {
                // DMA bootstrap to the host
                WARNING("Boot/Diagnostic ROM Load unimplemented!");
                break;
            }
            // unimplemented IL/EL modes
            // unimplemented Read SA ROM (!bd && _el)
            // unimplemented sanity timer _se (no DEQNA-lock mode)
            _update_QCSR();
            break;
    }

    pthread_mutex_unlock(&on_after_register_access_mutex);

}
