/* -*- c++ -*- */
/*
 * Copyright 2021 Hash.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SMART_METERS_GRIDSTREAM_IMPL_H
#define INCLUDED_SMART_METERS_GRIDSTREAM_IMPL_H

#include <smart_meters/GridStream.h>
#include <smart_meters/constants.h>

namespace gr {
namespace smart_meters {

class GridStream_impl : public GridStream
{
private:
    bool d_crcEnable;
    uint16_t d_crcInitialValue;
    uint32_t d_meterMonitorID;
    uint8_t d_packetTypeFilter;
    uint16_t d_packetLengthFilter;
    uint16_t crc16(uint16_t crc, const std::vector<uint8_t>& data, size_t size);

    /* converts a 4 byte field into a single integer value */    
    uint64_t bytes_to_int(const std::vector<uint8_t> &data, size_t start_index);

    /*!
     * \brief Message handler for input messages
     *
     * \param msg Dict PMT or PDU message passed from the scheduler's message handling.
     */
    void pdu_handler(pmt::pmt_t pdu);

public:
    GridStream_impl(bool crcEnable,
                    uint16_t crcInitialValue,
                    uint32_t meterMonitorID,
                    uint8_t packetTypeFilter,
                    uint16_t packetLengthFilter);
    ~GridStream_impl() override;
};

} // namespace smart_meters
} // namespace gr

#endif /* INCLUDED_SMART_METERS_GRIDSTREAM_IMPL_H */
