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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "crc32_bb_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace digital {

crc32_bb::sptr crc32_bb::make(bool check, const std::string& lengthtagname, bool packed, int grand)
{
    return gnuradio::get_initial_sptr(new crc32_bb_impl(check, lengthtagname, packed, grand));
}

crc32_bb_impl::crc32_bb_impl(bool check, const std::string& lengthtagname, bool packed, int grand)
    : tagged_stream_block("crc32_bb",
                          io_signature::make(1, 1, sizeof(char)),
                          io_signature::make(1, 1, sizeof(char)),
                          lengthtagname),
      d_check(check),
      d_packed(packed),
      d_npass(0),
      d_nfail(0),
      d_grand(grand)
{
    d_crc_length = 4;
    if (!d_packed) {
        d_crc_length = 32;
        d_buffer = std::vector<char>(d_crc_length);
    } else {
        d_buffer = std::vector<char>(4096);
    }
    set_tag_propagation_policy(TPP_DONT);
}

crc32_bb_impl::~crc32_bb_impl() {}

int crc32_bb_impl::calculate_output_stream_length(const gr_vector_int& ninput_items)
{
    if (d_check) {
        return ninput_items[0] - d_crc_length;
    } else {
        return ninput_items[0] + d_crc_length;
    }
}

unsigned int crc32_bb_impl::calculate_crc32(const unsigned char* in, size_t packet_length)
{
    unsigned int crc = 0;
    d_crc_impl.reset();
    if (!d_packed) {
        const size_t n_packed_length = 1 + ((packet_length - 1) / 8);
        if (n_packed_length > d_buffer.size()) {
            d_buffer.resize(n_packed_length);
        }
        std::fill(d_buffer.begin(), d_buffer.begin() + n_packed_length, 0);
        for (size_t bit = 0; bit < packet_length; bit++) {
            d_buffer[bit / 8] |= (in[bit] << (bit % 8));
        }
        d_crc_impl.process_bytes(&d_buffer[0], n_packed_length);
        crc = d_crc_impl();
    } else {
        d_crc_impl.process_bytes(in, packet_length);
        crc = d_crc_impl();
    }
    return crc;
}
inline bool crc32_bb_impl::fix1bit(size_t pkt_len,const uint8_t* bytes_in, uint8_t* bytes_fix){
    size_t i = 0;unsigned int crc;
    for (i = 0; i < pkt_len*8; i++) {
        uint8_t ibyte = i/8; // NOTE max packet length is 8 bytes due to fast correlator
        uint8_t ibit = i%8;
        bytes_fix[ibyte] ^= (unsigned long long)(1 << ibit); // flip bit
        d_crc_impl.reset();
        d_crc_impl.process_bytes(bytes_fix, pkt_len - 4);
        crc = d_crc_impl();
        if (crc == *(unsigned int*)(bytes_fix + pkt_len - 4)) { // CRC re-check
#ifdef GRAND_VERBOSE
            printf("%s %d\n", "grand ", i);
            // print how crc passes
            printf("%s %d\n", "PKT_LEN ", pkt_len);
            for(int i =0; i<pkt_len;i++){
                printf("%02X \n", bytes_in[i]);
            }
            printf("\n");
#endif
            fix1bits++;
            return true;
        }
        bytes_fix[ibyte] ^= (unsigned long long)(1 << ibit); // recover flipped bit
    }
    return false;
}
inline bool crc32_bb_impl::fix2bit(size_t pkt_len,const uint8_t* bytes_in, uint8_t* bytes_fix){
    size_t i = 0,j=0;unsigned int crc;
    for (i = 0; i < pkt_len*8-1; i++) {
        uint8_t ibyte = i/8;
        uint8_t ibit = i%8;
        for (j = i+1; j < pkt_len*8; j++) {
            uint8_t jbyte = j/8;
            uint8_t jbit = j%8;
            bytes_fix[ibyte] ^= (unsigned long long)(1 << ibit); // flip bit
            bytes_fix[jbyte] ^= (unsigned long long)(1 << jbit); // flip bit
            d_crc_impl.reset();
            d_crc_impl.process_bytes(bytes_fix, pkt_len - 4);
            crc = d_crc_impl();
            if (crc == *(unsigned int*)(bytes_fix + pkt_len - 4)) { // CRC re-check
#ifdef GRAND_VERBOSE
                printf("%s %d %d\n", "grand ", i, j);
                // print how crc passes
                printf("%s %d\n", "PKT_LEN ", pkt_len);
                for(int b =0; b<pkt_len;b++){
                    printf("%02X \n", bytes_in[b]);
                }
                printf("\n");
#endif
                fix2bits++;
                return true;
            }
            bytes_fix[ibyte] ^= (unsigned long long)(1 << ibit); // recover flipped bit
            bytes_fix[jbyte] ^= (unsigned long long)(1 << jbit); // recover flipped bit
        }
    }
    return false;
}
inline bool crc32_bb_impl::fix3bit(size_t pkt_len,const uint8_t* bytes_in, uint8_t* bytes_fix){
    size_t i = 0,j=0,k=0;unsigned int crc;
    for (i = 0; i < pkt_len*8-2; i++) {
        uint8_t ibyte = i/8;
        uint8_t ibit = i%8;
        for (j = i+1; j < pkt_len*8-1; j++) {
            uint8_t jbyte = j/8;
            uint8_t jbit = j%8;
            for (k = j+1; k < pkt_len*8; k++) {
                uint8_t kbyte = k/8;
                uint8_t kbit = k%8;
                bytes_fix[ibyte] ^= (unsigned long long)(1 << ibit); // flip bit
                bytes_fix[jbyte] ^= (unsigned long long)(1 << jbit); // flip bit
                bytes_fix[kbyte] ^= (unsigned long long)(1 << kbit); // flip bit
                d_crc_impl.reset();
                d_crc_impl.process_bytes(bytes_fix, pkt_len - 4);
                crc = d_crc_impl();
                if (crc == *(unsigned int*)(bytes_fix + pkt_len - 4)) { // CRC re-check
#ifdef GRAND_VERBOSE
                    printf("%s %d %d %d\n", "grand ", i, j, k);
                    // print how crc passes
                    printf("%s %d\n", "PKT_LEN ", pkt_len);
                    for(int b =0; b<pkt_len;b++){
                        printf("%02X \n", bytes_in[b]);
                    }
                    printf("\n");
#endif
                    fix3bits++;
                    return true;
                }
                bytes_fix[ibyte] ^= (unsigned long long)(1 << ibit); // recover flipped bit
                bytes_fix[jbyte] ^= (unsigned long long)(1 << jbit); // recover flipped bit
                bytes_fix[kbyte] ^= (unsigned long long)(1 << kbit); // recover flipped bit
            }
        }
    }
    return false;
}
#define GRAND 1 // note that GRAND only works in d_packed=true mode
void crc32_bb_impl::print_stats(){
    #ifdef GRAND
    printf("%s %ld %ld %ld %d %d %d\n", "pft", d_npass, d_nfail, d_npass+d_nfail,\
    fix1bits,fix2bits,fix3bits);
    #endif
}
#include <fstream>
#include <sstream>
int getMemoryUsage() {
    std::ifstream statusFile("/proc/self/status");
    std::string line;
    int memoryUsage = -1;

    while (std::getline(statusFile, line)) {
        if (line.compare(0, 6, "VmRSS:") == 0) {
            std::istringstream iss(line);
            std::string label;
            int value;
            iss >> label >> value;
            memoryUsage = value;
            break;
        }
    }

    return memoryUsage;
}
int crc32_bb_impl::work(int noutput_items,
                        gr_vector_int& ninput_items,
                        gr_vector_const_void_star& input_items,
                        gr_vector_void_star& output_items)
{
    const unsigned char* in = (const unsigned char*)input_items[0];
    unsigned char* out = (unsigned char*)output_items[0];
    size_t packet_length = ninput_items[0];
    int packet_size_diff = d_check ? -d_crc_length : d_crc_length;
    unsigned int crc;
    //if (grand>0){
    // make a copy of input for grand
    unsigned char input_copy[packet_length];
#ifdef GRAND
    uint8_t * bytes_fix = (uint8_t*)input_copy; //
    uint8_t fixed = 0;
    for (size_t i = 0; i < packet_length; i++) {
        input_copy[i] = in[i];
    }
auto start_time = std::chrono::high_resolution_clock::now();
#endif
    //}
    if (d_check) {
        if (packet_length <= d_crc_length) {
            return 0;
        }
        d_crc_impl.process_bytes(in, packet_length - d_crc_length);
        crc = calculate_crc32(in, packet_length - d_crc_length);
        /*printf("pkt_len %d\n", packet_length);
	for(int i =0; i<packet_length;i++){
	printf("%02X \n", in[i]);
	}
	for(int i =0; i<d_crc_length;i++){
	printf("%02X \n", crc);
	}
	printf("\n");*/
        if (d_packed) {
#ifdef GRAND
start_time = std::chrono::high_resolution_clock::now();
#endif
            if (crc !=
                *(unsigned int*)(in + packet_length - d_crc_length)) { // Drop package
                // or try to correct it with GRAND here
                //#define GRAND 1
                #ifdef GRAND
            //if (nwrong==1) // to save time testing we can condition grand when applicable
                            // looping too much can cause seg fault with high noises 0.4 and up
            //{ // test this only when applicable to measure ref.latency
                // fix one bit errors
                //if (crc != *(unsigned int*)(bytes_in + pkt_len - 4)){ // only try GRAND if CRC fails
                if (d_grand>=1 && fix1bit(packet_length, in, bytes_fix)){
                    d_npass++;
                    fixed=1;
                } 
                else if (d_grand>=2 && fix2bit(packet_length, in, bytes_fix)){
                    d_npass++;
                    fixed=1;
                } 
                else if (d_grand>=3 && fix3bit(packet_length, in, bytes_fix)){
                    d_npass++;
                    fixed=1;
                } 
                else {
                    d_nfail++;
                    //print_stats();
                    return 0;
                }
		#else
		d_nfail++;
                return 0;
                #endif

                //printf("BadCrc");
            }else{
                d_npass++; // if CRC was fine at start
            }
        } else {
            for (int i = 0; i < d_crc_length; i++) {
                if (((crc >> i) & 0x1) !=
                    *(in + packet_length - d_crc_length + i)) { // Drop package
                    //printf("BadCrc");
                    //return 0;
                }
            }
        }
#ifdef GRAND
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
    total_cpu_runtime += duration.count();
	//psutil::Memory memory_info = psutil::virtual_memory();
	total_memory_footprint += getMemoryUsage(); //memory_info.rss();
    int NUM_RUNS = d_npass+d_nfail;
    long long avg_cpu_runtime = total_cpu_runtime / NUM_RUNS;
    long long avg_memory_footprint = total_memory_footprint / NUM_RUNS;

    printf("CPU: %lld msecs\n", avg_cpu_runtime);
    printf("Mem: %lld bytes\n", avg_memory_footprint);

        //printf("GoodCrc");
        print_stats();
        if (fixed==1){
          memcpy((void*)out, (const void*)bytes_fix, packet_length - d_crc_length);
        }else{
          memcpy((void*)out, (const void*)in, packet_length - d_crc_length);
        }
#else
    memcpy((void*)out, (const void*)in, packet_length - d_crc_length);
#endif
    } else {
        crc = calculate_crc32(in, packet_length);
        memcpy((void*)out, (const void*)in, packet_length);
        if (d_packed) {
            memcpy((void*)(out + packet_length),
                   &crc,
                   d_crc_length); // FIXME big-endian/little-endian, this might be wrong
/*        printf("PKT_LEN %d\n", packet_length);
	for(int i =0; i<packet_length;i++){
	printf("%02X \n", out[i]);
	}
	for(int i =0; i<d_crc_length;i++){
	printf("%02X \n", crc);
	}
	printf("\n");  */
        } else {
            for (int i = 0; i < d_crc_length; i++) { // unpack CRC and store in buffer
                d_buffer[i] = (crc >> i) & 0x1;
            }
            memcpy((void*)(out + packet_length), (void*)&d_buffer[0], d_crc_length);
        }
    }

    //if (!d_check){ // comment out tag business when checking prbs dont care
	    std::vector<tag_t> tags;
	    get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + packet_length);
	    for (size_t i = 0; i < tags.size(); i++) {
		tags[i].offset -= nitems_read(0);
	        if (d_check &&
		    tags[i].offset > (unsigned int)(packet_length + packet_size_diff)) {
		    tags[i].offset = packet_length - d_crc_length - 1;
		}
	        add_item_tag(0, nitems_written(0) + tags[i].offset, tags[i].key, tags[i].value);
	    }
    //}
    return packet_length + packet_size_diff;
}

} /* namespace digital */
} /* namespace gr */
