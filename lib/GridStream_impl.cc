/* -*- c++ -*- */
/*
 * Copyright 2021 Hash.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "GridStream_impl.h"
#include <gnuradio/io_signature.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace gr {
namespace smart_meters {

GridStream::sptr GridStream::make(bool crcEnable,
                                  uint16_t crcInitialValue,
                                  uint32_t meterMonitorID,
                                  uint8_t packetTypeFilter,
                                  uint16_t packetLengthFilter)
{
    return gnuradio::make_block_sptr<GridStream_impl>(
        crcEnable, crcInitialValue, meterMonitorID, packetTypeFilter, packetLengthFilter);
}


/*
 * The private constructor
 */
GridStream_impl::GridStream_impl(bool crcEnable,
                                 uint16_t crcInitialValue,
                                 uint32_t meterMonitorID,
                                 uint8_t packetTypeFilter,
                                 uint16_t packetLengthFilter)
    : gr::block(
          "GridStream", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)),
      d_crcEnable(crcEnable),
      d_crcInitialValue(crcInitialValue),
      d_meterMonitorID(meterMonitorID),
      d_packetTypeFilter(packetTypeFilter),
      d_packetLengthFilter(packetLengthFilter)
{
    message_port_register_in(PMTCONSTSTR__PDU_IN);
    set_msg_handler(PMTCONSTSTR__PDU_IN,
                    [this](pmt::pmt_t pdu) { this->pdu_handler(pdu); });
    message_port_register_out(PMTCONSTSTR__PDU_OUT);
}

/*
 * Our virtual destructor.
 */
GridStream_impl::~GridStream_impl() {}

template <typename T>
std::string int_to_hex(T i)
{
    std::stringstream stream;
    stream << std::setfill('0') << std::setw(sizeof(T) * 2) << std::hex << std::uppercase
           << i;
    return stream.str();
}

template <typename T>
std::string char_to_hex(T i)
{
    std::stringstream stream;
    stream << std::setfill('0') << std::setw(2) << std::hex << std::uppercase << i;
    return stream.str();
}

uint16_t
GridStream_impl::crc16(uint16_t crc, const std::vector<uint8_t>& data, size_t size)
{
    // Some known CRC's below
    // uint16_t crc = 0x45F8;	// (CoServ CRC)
    // uint16_t crc = 0x5FD6;	// (Oncor CRC)
    // uint16_t crc = 0x62C1;	// (Hydro-Quebec CRC)
    // Hard coded Poly 0x1021
    uint16_t i = 6; // Skip over header/packet length [00,FF,2A,55,xx,xx]
    while (size--) {
        crc ^= data[i] << 8;
        i++;
        for (unsigned k = 0; k < 8; k++)
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

// consolidates 4 byte fields into a single integer value LSB first
uint64_t
GridStream_impl::bytes_to_int(const std::vector<uint8_t> &data, size_t start_index)
{
    uint64_t out = data[start_index];
    for (auto ii = 1; ii < 4; ii++) {
        out *= 256;
        out += data[start_index+ii];
    }
    return out;
}

void GridStream_impl::pdu_handler(pmt::pmt_t pdu)
{
    pmt::pmt_t meta = pmt::car(pdu);
    pmt::pmt_t v_data = pmt::cdr(pdu);

    // make sure PDU data is formed properly
    if (!(pmt::is_pdu(pdu))) {
        GR_LOG_WARN(d_logger, "received unexpected PMT (non-pdu)");
        return;
    }

    size_t vlen = pmt::length(pmt::cdr(pdu));
    const std::vector<uint8_t> data = pmt::u8vector_elements(v_data);

    // output data from block, 10 bytes input per 1 byte output, +5 buffer
    std::vector<uint8_t> out;
    out.reserve((data.size() / 10) + 5);
    // Packet not large enough, probably noise
    if (data.size() < 9)
        return;
    // Read header and packet length
    uint8_t byte = 0;
    long offset = 1;
    for (int ii = 0; ii < 4 + 2; ii++) {
        for (int jj = 0; jj < 8; jj++) {
            // START MSB FIRST PROCESSING
            byte >>= 1;
            if (data[jj + offset])
                byte |= 0x80;
            if ((jj % 8) == 7) {
                out.push_back(byte);
                byte = 0;
            }
        }
        offset += 10;
    }
    // Packet decoded 00,FF,2A,packet_type(xx),packet_len(xxxx)
    uint32_t packet_type = out[3];
    uint32_t packet_len = out[5] | out[4] << 8;
        
    // Loop to decode data based on packet_len
    if (data.size() > packet_len) {
        for (int ii = 0; ii < packet_len; ii++) {
            uint8_t byte = 0;
            for (int jj = 0; jj < 8; jj++) {
                // START MSB FIRST PROCESSING
                byte >>= 1;
                if (data[jj + offset])
                    byte |= 0x80;
                if ((jj % 8) == 7) {
                    out.push_back(byte);
                    byte = 0;
                }
            }
            offset += 10;
        }
    } else {
        return;
    }

    uint32_t packet_subtype = out[6];
    meta = pmt::dict_add(meta, pmt::mp("type"), pmt::from_uint64(packet_type));
    meta = pmt::dict_add(meta, pmt::mp("subtype"), pmt::from_uint64(packet_subtype));
    meta = pmt::dict_add(meta, pmt::mp("length"), pmt::from_uint64(packet_len));

    uint32_t meterID{ 0 };
    uint32_t meterID2{ 0 };
    uint32_t upTime{ 0 };
    uint32_t counter;
    float timing;

    if (packet_type == 0x55 && packet_len == 0x0023) {
        meterID = bytes_to_int(out, 26);
        upTime = bytes_to_int(out, 20);
        counter = out[17];
        timing = out[36] | out[35] << 8;
        meta = pmt::dict_add(meta, pmt::mp("meterID1"), pmt::from_uint64(meterID));
        meta = pmt::dict_add(meta, pmt::mp("uptime"), pmt::from_uint64(upTime));
    } else if (packet_type == 0xD5) {
        meterID2 = bytes_to_int(out, 7);
        meterID = bytes_to_int(out, 11);
        counter = out[13];
        timing = out[13 + packet_len] | out[12 + packet_len] << 8;
        meta = pmt::dict_add(meta, pmt::mp("meterID1"), pmt::from_uint64(meterID));
        meta = pmt::dict_add(meta, pmt::mp("meterID2"), pmt::from_uint64(meterID2));
    } else {
        return;
    }

    int receivedCRC = out[packet_len + 5] | out[packet_len + 4] << 8;
    uint16_t calculatedCRC = GridStream_impl::crc16(
        d_crcInitialValue, out, out.size() - 8); // Strip off header/len (6) and crc (2)

    if (((receivedCRC == calculatedCRC) || !(d_crcEnable)) &&
        ((meterID == d_meterMonitorID) || (meterID2 == d_meterMonitorID) ||
         (d_meterMonitorID == 0)) &&
        ((packet_len == d_packetLengthFilter) || (d_packetLengthFilter == 0)) &&
        ((packet_type == d_packetTypeFilter) || (d_packetTypeFilter == 0))) {
        std::cout << std::setfill('0') << std::hex << std::setw(2) << std::uppercase;
        for (int i = 0; i < packet_len + 4 + 2;
             i++) // +4 to capture leading bytes and -2 to strip off CRC -1 to strip off
                  // end of packet
        {
            std::cout << std::setw(2) << int(out[i]);
        }
        std::cout << std::endl;
        
        meta = pmt::dict_add(meta, pmt::mp("counter"), pmt::from_uint64(counter));
        meta = pmt::dict_add(meta, pmt::mp("timing"), pmt::from_double(timing * 0.01));
        meta = pmt::dict_add(meta, pmt::mp("CRC_OK"), pmt::from_bool(receivedCRC == calculatedCRC));

        message_port_pub(PMTCONSTSTR__PDU_OUT,
                         (pmt::cons(meta, pmt::init_u8vector(out.size(), out))));
        return;
    } else {
        return;
    }
}

} /* namespace smart_meters */
} /* namespace gr */
