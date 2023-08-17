/* -*- c++ -*- */
/*
 * Copyright 2013 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_DIGITAL_CRC32_BB_IMPL_H
#define INCLUDED_DIGITAL_CRC32_BB_IMPL_H

#include <gnuradio/digital/crc32_bb.h>
#include <boost/crc.hpp>
#include <chrono>
//#include <psutilcpp/psutilcpp.hpp> 
namespace gr {
namespace digital {

class crc32_bb_impl : public crc32_bb
{
private:
    bool d_check;
    bool d_packed;
    boost::crc_optimal<32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true> d_crc_impl;
    int d_crc_length;
    std::vector<char> d_buffer;
    unsigned int calculate_crc32(const unsigned char* in, size_t packet_length);
    void print_stats();
    uint64_t d_npass;
    uint64_t d_nfail;
    long long total_cpu_runtime = 0;
    long long total_memory_footprint = 0;
    inline bool fix1bit(size_t pkt_len,const uint8_t* bytes_in, uint8_t* bytes_fix);
    inline bool fix2bit(size_t pkt_len,const uint8_t* bytes_in, uint8_t* bytes_fix);
    inline bool fix3bit(size_t pkt_len,const uint8_t* bytes_in, uint8_t* bytes_fix);
    uint16_t fix1bits=0,fix2bits=0,fix3bits=0;

public:
    crc32_bb_impl(bool check, const std::string& lengthtagname, bool packed);
    ~crc32_bb_impl();

    int calculate_output_stream_length(const gr_vector_int& ninput_items);
    int work(int noutput_items,
             gr_vector_int& ninput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace digital
} // namespace gr

#endif /* INCLUDED_DIGITAL_CRC32_BB_IMPL_H */
