/*!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 * \file gps_l1_ca_telemetry_decoder_gs.cc
 * \brief Implementation of a NAV message demodulator block based on
 * Kay Borre book MATLAB-based GPS receiver
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
=======
=======
>>>>>>> "Like next"
=======
>>>>>>> set to normal
 * \file gps_l1_ca_dll_pll_tracking_test.cc
 * \brief  This class implements a telemetry decoder test for GPS_L1_CA_Telemetry_Decoder
 *  implementation based on some input parameters.
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> set to normal
=======
>>>>>>> "Like next"
=======
>>>>>>> set to normal
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
#include "gps_l1_ca_telemetry_decoder_gs.h"
#include "gps_ephemeris.h"  // for Gps_Ephemeris
#include "gps_iono.h"       // for Gps_Iono
#include "gps_utc_model.h"  // for Gps_Utc_Model
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <cmath>            // for round
#include <cstring>          // for memcpy
#include <exception>        // for exception
#include <iostream>         // for cout
#include <memory>           // for shared_ptr


#ifndef _rotl
#define _rotl(X, N) (((X) << (N)) ^ ((X) >> (32 - (N))))  // Used in the parity check algorithm
#endif


gps_l1_ca_telemetry_decoder_gs_sptr
gps_l1_ca_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump)
{
    return gps_l1_ca_telemetry_decoder_gs_sptr(new gps_l1_ca_telemetry_decoder_gs(satellite, dump));
}


gps_l1_ca_telemetry_decoder_gs::gps_l1_ca_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump) : gr::block("gps_navigation_gs", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);

    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    d_last_valid_preamble = 0;
    d_sent_tlm_failed_msg = false;

    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Initializing GPS L1 TELEMETRY DECODER";

    d_bits_per_preamble = GPS_CA_PREAMBLE_LENGTH_BITS;
    d_samples_per_preamble = d_bits_per_preamble;
    d_preamble_period_symbols = GPS_SUBFRAME_BITS;
    // set the preamble
    d_required_symbols = GPS_SUBFRAME_BITS;
    // preamble bits to sampled symbols
    d_frame_length_symbols = GPS_SUBFRAME_BITS * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;
    d_max_symbols_without_valid_frame = d_required_symbols * 20;  // rise alarm 120 segs without valid tlm
    int32_t n = 0;
    for (int32_t i = 0; i < d_bits_per_preamble; i++)
        {
            if (GPS_CA_PREAMBLE[i] == '1')
                {
                    d_preamble_samples[n] = 1;
                    n++;
                }
            else
                {
                    d_preamble_samples[n] = -1;
                    n++;
                }
        }
    d_sample_counter = 0ULL;
    d_stat = 0;
    d_preamble_index = 0ULL;

    d_flag_frame_sync = false;

    d_flag_parity = false;
    d_TOW_at_current_symbol_ms = 0;
    d_TOW_at_Preamble_ms = 0;
    d_CRC_error_counter = 0;
    d_flag_preamble = false;
    d_channel = 0;
    flag_TOW_set = false;
    flag_PLL_180_deg_phase_locked = false;
    d_prev_GPS_frame_4bytes = 0;
    d_symbol_history.set_capacity(d_required_symbols);
}


gps_l1_ca_telemetry_decoder_gs::~gps_l1_ca_telemetry_decoder_gs()
{
    if (d_dump_file.is_open() == true)
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
        }
}


bool gps_l1_ca_telemetry_decoder_gs::gps_word_parityCheck(uint32_t gpsword)
{
    uint32_t d1;
    uint32_t d2;
    uint32_t d3;
    uint32_t d4;
    uint32_t d5;
    uint32_t d6;
    uint32_t d7;
    uint32_t t;
    uint32_t parity;
    // XOR as many bits in parallel as possible.  The magic constants pick
    //   up bits which are to be XOR'ed together to implement the GPS parity
    //   check algorithm described in IS-GPS-200E.  This avoids lengthy shift-
    //   and-xor loops.
    d1 = gpsword & 0xFBFFBF00U;
    d2 = _rotl(gpsword, 1U) & 0x07FFBF01U;
    d3 = _rotl(gpsword, 2U) & 0xFC0F8100U;
    d4 = _rotl(gpsword, 3U) & 0xF81FFE02U;
    d5 = _rotl(gpsword, 4U) & 0xFC00000EU;
    d6 = _rotl(gpsword, 5U) & 0x07F00001U;
    d7 = _rotl(gpsword, 6U) & 0x00003000U;
    t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7;
    // Now XOR the 5 6-bit fields together to produce the 6-bit final result.
    parity = t ^ _rotl(t, 6U) ^ _rotl(t, 12U) ^ _rotl(t, 18U) ^ _rotl(t, 24U);
    parity = parity & 0x3FU;
    if (parity == (gpsword & 0x3FU))
        {
            return true;
        }

    return false;
}


void gps_l1_ca_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_nav.reset();
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    d_nav.i_satellite_PRN = d_satellite.get_PRN();
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void gps_l1_ca_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    d_nav.i_channel_ID = channel;
    DLOG(INFO) << "Navigation channel set to " << channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename = "telemetry";
                            d_dump_filename.append(std::to_string(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel
                                      << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                        }
                }
        }
}


bool gps_l1_ca_telemetry_decoder_gs::decode_subframe()
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> set to normal
    std::array<char, GPS_SUBFRAME_LENGTH> subframe{};
=======
<<<<<<< HEAD
    char subframe[GPS_SUBFRAME_LENGTH];
>>>>>>> set to normal
<<<<<<< HEAD
=======
    std::array<char, GPS_SUBFRAME_LENGTH> subframe{};
>>>>>>> From GNSS-SDR
=======
>>>>>>> set to normal
=======
    std::array<char, GPS_SUBFRAME_LENGTH> subframe{};
>>>>>>> From GNSS-SDR
>>>>>>> From GNSS-SDR
=======
    std::array<char, GPS_SUBFRAME_LENGTH> subframe{};
>>>>>>> set to normal
    int32_t frame_bit_index = 0;
    int32_t word_index = 0;
    uint32_t GPS_frame_4bytes = 0;
    bool subframe_synchro_confirmation = true;
    for (float subframe_symbol : d_symbol_history)
        {
            // ******* SYMBOL TO BIT *******
            // symbol to bit
            if (subframe_symbol > 0)
                {
                    GPS_frame_4bytes += 1;  // insert the telemetry bit in LSB
                }

            // ******* bits to words ******
            frame_bit_index++;
            if (frame_bit_index == 30)
                {
                    frame_bit_index = 0;
                    // parity check
                    // Each word in wordbuff is composed of:
                    //      Bits 0 to 29 = the GPS data word
                    //      Bits 30 to 31 = 2 LSBs of the GPS word ahead.
                    // prepare the extended frame [-2 -1 0 ... 30]
                    if (d_prev_GPS_frame_4bytes & 0x00000001U)
                        {
                            GPS_frame_4bytes = GPS_frame_4bytes | 0x40000000U;
                        }
                    if (d_prev_GPS_frame_4bytes & 0x00000002U)
                        {
                            GPS_frame_4bytes = GPS_frame_4bytes | 0x80000000U;
                        }
                    // Check that the 2 most recently logged words pass parity. Have to first
                    // invert the data bits according to bit 30 of the previous word.
                    if (GPS_frame_4bytes & 0x40000000U)
                        {
                            GPS_frame_4bytes ^= 0x3FFFFFC0U;  // invert the data bits (using XOR)
                        }
                    // check parity. If ANY word inside the subframe fails the parity, set subframe_synchro_confirmation = false
                    if (not gps_l1_ca_telemetry_decoder_gs::gps_word_parityCheck(GPS_frame_4bytes))
                        {
                            subframe_synchro_confirmation = false;
                        }
                    // add word to subframe
                    // insert the word in the correct position of the subframe
                    std::memcpy(&subframe[word_index * GPS_WORD_LENGTH], &GPS_frame_4bytes, sizeof(uint32_t));
                    word_index++;
                    d_prev_GPS_frame_4bytes = GPS_frame_4bytes;  // save the actual frame
                    GPS_frame_4bytes = 0;
                }
            else
                {
                    GPS_frame_4bytes <<= 1U;  // shift 1 bit left the telemetry word
                }
        }

    // decode subframe
    // NEW GPS SUBFRAME HAS ARRIVED!
    if (subframe_synchro_confirmation)
        {
            int32_t subframe_ID = d_nav.subframe_decoder(subframe.data());  // decode the subframe
            if (subframe_ID > 0 and subframe_ID < 6)
                {
                    std::cout << "New GPS NAV message received in channel " << this->d_channel << ": "
                              << "subframe "
                              << subframe_ID << " from satellite "
                              << Gnss_Satellite(std::string("GPS"), d_nav.i_satellite_PRN) << std::endl;

                    switch (subframe_ID)
                        {
                        case 3:  // we have a new set of ephemeris data for the current SV
                            if (d_nav.satellite_validation() == true)
                                {
                                    // get ephemeris object for this SV (mandatory)
                                    std::shared_ptr<Gps_Ephemeris> tmp_obj = std::make_shared<Gps_Ephemeris>(d_nav.get_ephemeris());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 4:  // Possible IONOSPHERE and UTC model update (page 18)
                            if (d_nav.flag_iono_valid == true)
                                {
                                    std::shared_ptr<Gps_Iono> tmp_obj = std::make_shared<Gps_Iono>(d_nav.get_iono());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            if (d_nav.flag_utc_model_valid == true)
                                {
                                    std::shared_ptr<Gps_Utc_Model> tmp_obj = std::make_shared<Gps_Utc_Model>(d_nav.get_utc_model());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 5:
                            // get almanac (if available)
                            // TODO: implement almanac reader in navigation_message
                            break;
                        default:
                            break;
                        }
                    return true;
                }
        }
    return false;
}


void gps_l1_ca_telemetry_decoder_gs::reset()
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    d_last_valid_preamble = d_sample_counter;
    d_sent_tlm_failed_msg = false;
    flag_TOW_set = false;
    d_symbol_history.clear();
    d_stat = 0;
    DLOG(INFO) << "Telemetry decoder reset for satellite " << d_satellite;
}


int gps_l1_ca_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    // 1. Copy the current tracking output
    Gnss_Synchro current_symbol = in[0][0];
    // add new symbol to the symbol queue
    d_symbol_history.push_back(current_symbol.Prompt_I);
    d_sample_counter++;  // count for the processed symbols
    consume_each(1);
    d_flag_preamble = false;
    // check if there is a problem with the telemetry of the current satellite
    if (d_stat < 2 and d_sent_tlm_failed_msg == false)
        {
            if ((d_sample_counter - d_last_valid_preamble) > d_max_symbols_without_valid_frame)
                {
                    int message = 1;  // bad telemetry
                    this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(message));
                    d_sent_tlm_failed_msg = true;
                }
        }

    // ******* frame sync ******************
    switch (d_stat)
        {
        case 0:  // no preamble information
            {
                // correlate with preamble
                int32_t corr_value = 0;
                if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_BITS)
                    {
                        // ******* preamble correlation ********
                        for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
                            {
                                if (d_symbol_history[i] < 0.0)  // symbols clipping
                                    {
                                        corr_value -= d_preamble_samples[i];
                                    }
                                else
                                    {
                                        corr_value += d_preamble_samples[i];
                                    }
                            }
                    }
                if (abs(corr_value) >= d_samples_per_preamble)
                    {
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                        DLOG(INFO) << "Preamble detection for GPS L1 satellite " << this->d_satellite;
                        decode_subframe();
                        d_stat = 1;  // enter into frame pre-detection status
                    }
                flag_TOW_set = false;
                break;
            }
        case 1:  // possible preamble lock
            {
                // correlate with preamble
                int32_t corr_value = 0;
                int32_t preamble_diff = 0;
                if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_BITS)
                    {
                        // ******* preamble correlation ********
                        for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
                            {
                                if (d_symbol_history[i] < 0.0)  // symbols clipping
                                    {
                                        corr_value -= d_preamble_samples[i];
                                    }
                                else
                                    {
                                        corr_value += d_preamble_samples[i];
                                    }
                            }
                    }
                if (abs(corr_value) >= d_samples_per_preamble)
                    {
                        // check preamble separation
                        preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                        if (abs(preamble_diff - d_preamble_period_symbols) == 0)
                            {
                                DLOG(INFO) << "Preamble confirmation for SAT " << this->d_satellite;
                                d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                if (corr_value < 0)
                                    {
                                        flag_PLL_180_deg_phase_locked = true;
                                    }
                                else
                                    {
                                        flag_PLL_180_deg_phase_locked = false;
                                    }
                                decode_subframe();
                                d_stat = 2;
                            }
                        else
                            {
                                if (preamble_diff > d_preamble_period_symbols)
                                    {
                                        d_stat = 0;  // start again
                                        flag_TOW_set = false;
                                    }
                            }
                    }
                break;
            }
        case 2:  // preamble acquired
            {
                if (d_sample_counter >= d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                    {
                        DLOG(INFO) << "Preamble received for SAT " << this->d_satellite << "d_sample_counter=" << d_sample_counter << "\n";
                        // call the decoder
                        // 0. fetch the symbols into an array
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)

                        if (decode_subframe())
                            {
                                d_CRC_error_counter = 0;
                                d_flag_preamble = true;  // valid preamble indicator (initialized to false every work())
                                gr::thread::scoped_lock lock(d_setlock);
                                d_last_valid_preamble = d_sample_counter;
                                if (!d_flag_frame_sync)
                                    {
                                        d_flag_frame_sync = true;
                                        DLOG(INFO) << " Frame sync SAT " << this->d_satellite;
                                    }
                            }
                        else
                            {
                                d_CRC_error_counter++;
                                if (d_CRC_error_counter > 2)
                                    {
                                        DLOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                                        d_flag_frame_sync = false;
                                        d_stat = 0;
                                        d_TOW_at_current_symbol_ms = 0;
                                        d_TOW_at_Preamble_ms = 0;
                                        d_CRC_error_counter = 0;
                                        flag_TOW_set = false;
                                    }
                            }
                    }
                break;
            }
        }

    // 2. Add the telemetry decoder information
    if (d_flag_preamble == true)
        {
            if (!(d_nav.d_TOW == 0))
                {
                    d_TOW_at_current_symbol_ms = static_cast<uint32_t>(d_nav.d_TOW * 1000.0);
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.d_TOW * 1000.0);
                    flag_TOW_set = true;
                }
            else
                {
                    DLOG(INFO) << "Received GPS L1 TOW equal to zero at sat " << d_nav.i_satellite_PRN;
                }
        }
    else
        {
            if (flag_TOW_set == true)
                {
                    d_TOW_at_current_symbol_ms += GPS_L1_CA_BIT_PERIOD_MS;
                }
        }

    if (flag_TOW_set == true)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = flag_TOW_set;

            if (flag_PLL_180_deg_phase_locked == true)
                {
                    // correct the accumulated phase for the Costas loop phase shift, if required
                    current_symbol.Carrier_phase_rads += GPS_PI;
                }

            if (d_dump == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                        {
                            double tmp_double;
                            uint64_t tmp_ulong_int;
                            tmp_double = static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                            tmp_ulong_int = current_symbol.Tracking_sample_counter;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(uint64_t));
                            tmp_double = static_cast<double>(d_TOW_at_Preamble_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                        }
                }

            // 3. Make the output (copy the object contents to the GNU Radio reserved memory)
            *out[0] = current_symbol;

            return 1;
        }

    return 0;
=======
=======
>>>>>>> "Like next"
=======
>>>>>>> set to normal
#include <armadillo>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/top_block.h>
#include <chrono>
#include <exception>
#include <string>
#include <unistd.h>
#include <utility>
#ifdef GR_GREATER_38
#include <gnuradio/analog/sig_source.h>
#else
#include <gnuradio/analog/sig_source_c.h>
#endif
#include "GPS_L1_CA.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "gnss_synchro.h"
#include "gps_l1_ca_dll_pll_tracking.h"
#include "gps_l1_ca_telemetry_decoder.h"
#include "in_memory_configuration.h"
#include "signal_generator_flags.h"
#include "telemetry_decoder_interface.h"
#include "tlm_dump_reader.h"
#include "tracking_dump_reader.h"
#include "tracking_interface.h"
#include "tracking_true_obs_reader.h"
#include <gnuradio/blocks/interleaved_char_to_complex.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>


// ######## GNURADIO BLOCK MESSAGE RECEVER FOR TRACKING MESSAGES #########
class GpsL1CADllPllTelemetryDecoderTest_msg_rx;

using GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr = boost::shared_ptr<GpsL1CADllPllTelemetryDecoderTest_msg_rx>;

GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_msg_rx_make();

class GpsL1CADllPllTelemetryDecoderTest_msg_rx : public gr::block
{
private:
    friend GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CADllPllTelemetryDecoderTest_msg_rx();

public:
    int rx_message;
    ~GpsL1CADllPllTelemetryDecoderTest_msg_rx();  //!< Default destructor
};

GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_msg_rx_make()
{
    return GpsL1CADllPllTelemetryDecoderTest_msg_rx_sptr(new GpsL1CADllPllTelemetryDecoderTest_msg_rx());
}

void GpsL1CADllPllTelemetryDecoderTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> "Like next"
=======
>>>>>>> set to normal
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
<<<<<<< HEAD
<<<<<<< HEAD
=======
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);

    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    d_last_valid_preamble = 0;
    d_sent_tlm_failed_msg = false;

    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Initializing GPS L1 TELEMETRY DECODER";

    d_bits_per_preamble = GPS_CA_PREAMBLE_LENGTH_BITS;
    d_samples_per_preamble = d_bits_per_preamble;
    d_preamble_period_symbols = GPS_SUBFRAME_BITS;
    // set the preamble
    d_required_symbols = GPS_SUBFRAME_BITS;
    // preamble bits to sampled symbols
    d_frame_length_symbols = GPS_SUBFRAME_BITS * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;
    d_max_symbols_without_valid_frame = d_required_symbols * 20;  // rise alarm 120 segs without valid tlm
    int32_t n = 0;
    for (int32_t i = 0; i < d_bits_per_preamble; i++)
>>>>>>> From GNSS-SDR
=======
>>>>>>> "Like next"
=======
>>>>>>> set to normal
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}

GpsL1CADllPllTelemetryDecoderTest_msg_rx::GpsL1CADllPllTelemetryDecoderTest_msg_rx() : gr::block("GpsL1CADllPllTelemetryDecoderTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CADllPllTelemetryDecoderTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

GpsL1CADllPllTelemetryDecoderTest_msg_rx::~GpsL1CADllPllTelemetryDecoderTest_msg_rx() = default;


// ###########################################################


// ######## GNURADIO BLOCK MESSAGE RECEVER FOR TLM MESSAGES #########
class GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx;

using GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr = boost::shared_ptr<GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx>;

GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_make();

class GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx : public gr::block
{
private:
    friend GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx();

public:
    int rx_message;
    ~GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx();  //!< Default destructor
};

GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_make()
{
    return GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_sptr(new GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx());
}

void GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}

GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx::GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx() : gr::block("GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}

GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx::~GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx() = default;


// ###########################################################


class GpsL1CATelemetryDecoderTest : public ::testing::Test
{
public:
    std::string generator_binary;
    std::string p1;
    std::string p2;
    std::string p3;
    std::string p4;
    std::string p5;

    const int baseband_sampling_freq = FLAGS_fs_gen_sps;

    std::string filename_rinex_obs = FLAGS_filename_rinex_obs;
    std::string filename_raw_data = FLAGS_filename_raw_data;

    int configure_generator();
    int generate_signal();
    void check_results(arma::vec& true_time_s,
        arma::vec& true_value,
        arma::vec& meas_time_s,
        arma::vec& meas_value);

    GpsL1CATelemetryDecoderTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~GpsL1CATelemetryDecoderTest() = default;

    void configure_receiver();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


int GpsL1CATelemetryDecoderTest::configure_generator()
{
    // Configure signal generator
    generator_binary = FLAGS_generator_binary;

    p1 = std::string("-rinex_nav_file=") + FLAGS_rinex_nav_file;
    if (FLAGS_dynamic_position.empty())
        {
            p2 = std::string("-static_position=") + FLAGS_static_position + std::string(",") + std::to_string(FLAGS_duration * 10);
        }
    else
        {
            p2 = std::string("-obs_pos_file=") + std::string(FLAGS_dynamic_position);
        }
    p3 = std::string("-rinex_obs_file=") + FLAGS_filename_rinex_obs;               // RINEX 2.10 observation file output
    p4 = std::string("-sig_out_file=") + FLAGS_filename_raw_data;                  // Baseband signal output file. Will be stored in int8_t IQ multiplexed samples
    p5 = std::string("-sampling_freq=") + std::to_string(baseband_sampling_freq);  // Baseband sampling frequency [MSps]
    return 0;
}


int GpsL1CATelemetryDecoderTest::generate_signal()
{
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> "Like next"
=======
>>>>>>> set to normal
    int child_status;

    char* const parmList[] = {&generator_binary[0], &generator_binary[0], &p1[0], &p2[0], &p3[0], &p4[0], &p5[0], nullptr};

    int pid;
    if ((pid = fork()) == -1)
        {
            perror("fork err");
<<<<<<< HEAD
<<<<<<< HEAD
=======
<<<<<<< HEAD
    std::array<char, GPS_SUBFRAME_LENGTH> subframe{};
=======
    std::array<char, GPS_SUBFRAME_LENGTH> subframe{};
>>>>>>> From GNSS-SDR
    int32_t frame_bit_index = 0;
    int32_t word_index = 0;
    uint32_t GPS_frame_4bytes = 0;
    bool subframe_synchro_confirmation = true;
    for (float subframe_symbol : d_symbol_history)
        {
            // ******* SYMBOL TO BIT *******
            // symbol to bit
            if (subframe_symbol > 0)
                {
                    GPS_frame_4bytes += 1;  // insert the telemetry bit in LSB
                }

            // ******* bits to words ******
            frame_bit_index++;
            if (frame_bit_index == 30)
                {
                    frame_bit_index = 0;
                    // parity check
                    // Each word in wordbuff is composed of:
                    //      Bits 0 to 29 = the GPS data word
                    //      Bits 30 to 31 = 2 LSBs of the GPS word ahead.
                    // prepare the extended frame [-2 -1 0 ... 30]
                    if (d_prev_GPS_frame_4bytes & 0x00000001U)
                        {
                            GPS_frame_4bytes = GPS_frame_4bytes | 0x40000000U;
                        }
                    if (d_prev_GPS_frame_4bytes & 0x00000002U)
                        {
                            GPS_frame_4bytes = GPS_frame_4bytes | 0x80000000U;
                        }
                    // Check that the 2 most recently logged words pass parity. Have to first
                    // invert the data bits according to bit 30 of the previous word.
                    if (GPS_frame_4bytes & 0x40000000U)
                        {
                            GPS_frame_4bytes ^= 0x3FFFFFC0U;  // invert the data bits (using XOR)
                        }
                    // check parity. If ANY word inside the subframe fails the parity, set subframe_synchro_confirmation = false
                    if (not gps_l1_ca_telemetry_decoder_gs::gps_word_parityCheck(GPS_frame_4bytes))
                        {
                            subframe_synchro_confirmation = false;
                        }
                    // add word to subframe
                    // insert the word in the correct position of the subframe
                    std::memcpy(&subframe[word_index * GPS_WORD_LENGTH], &GPS_frame_4bytes, sizeof(uint32_t));
                    word_index++;
                    d_prev_GPS_frame_4bytes = GPS_frame_4bytes;  // save the actual frame
                    GPS_frame_4bytes = 0;
                }
            else
                {
                    GPS_frame_4bytes <<= 1U;  // shift 1 bit left the telemetry word
                }
>>>>>>> From GNSS-SDR
=======
>>>>>>> "Like next"
=======
>>>>>>> set to normal
        }
    else if (pid == 0)
        {
            execv(&generator_binary[0], parmList);
            std::cout << "Return not expected. Must be an execv err." << std::endl;
            std::terminate();
        }

    waitpid(pid, &child_status, 0);

    std::cout << "Signal and Observables RINEX and RAW files created." << std::endl;
    return 0;
}


void GpsL1CATelemetryDecoderTest::configure_receiver()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'G';
    std::string signal = "1C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = FLAGS_test_satellite_PRN;

    config->set_property("GNSS-SDR.internal_fs_sps", std::to_string(baseband_sampling_freq));

    // Set Tracking
    config->set_property("Tracking_1C.item_type", "gr_complex");
    config->set_property("Tracking_1C.dump", "true");
    config->set_property("Tracking_1C.dump_filename", "./tracking_ch_");
    config->set_property("Tracking_1C.pll_bw_hz", "20.0");
    config->set_property("Tracking_1C.dll_bw_hz", "1.5");
    config->set_property("Tracking_1C.early_late_space_chips", "0.5");
    config->set_property("Tracking_1C.unified", "true");
    config->set_property("TelemetryDecoder_1C.dump", "true");
}


void GpsL1CATelemetryDecoderTest::check_results(arma::vec& true_time_s,
    arma::vec& true_value,
    arma::vec& meas_time_s,
    arma::vec& meas_value)
{
    // 1. True value interpolation to match the measurement times
    arma::vec true_value_interp;
    arma::uvec true_time_s_valid = find(true_time_s > 0);
    true_time_s = true_time_s(true_time_s_valid);
    true_value = true_value(true_time_s_valid);
    arma::uvec meas_time_s_valid = find(meas_time_s > 0);
    meas_time_s = meas_time_s(meas_time_s_valid);
    meas_value = meas_value(meas_time_s_valid);

    arma::interp1(true_time_s, true_value, meas_time_s, true_value_interp);

    // 2. RMSE
    // arma::vec err = meas_value - true_value_interp + 0.001;
    arma::vec err = meas_value - true_value_interp;  // - 0.001;

    arma::vec err2 = arma::square(err);
    double rmse = sqrt(arma::mean(err2));

    // 3. Mean err and variance
    double error_mean = arma::mean(err);
    double error_var = arma::var(err);

    // 4. Peaks
    double max_error = arma::max(err);
    double min_error = arma::min(err);

    // 5. report
    std::streamsize ss = std::cout.precision();
    std::cout << std::setprecision(10) << "TLM TOW RMSE="
              << rmse << ", mean=" << error_mean
              << ", stdev=" << sqrt(error_var)
              << " (max,min)=" << max_error
              << "," << min_error
              << " [Seconds]" << std::endl;
    std::cout.precision(ss);

    ASSERT_LT(rmse, 0.3E-6);
    ASSERT_LT(error_mean, 0.3E-6);
    ASSERT_GT(error_mean, -0.3E-6);
    ASSERT_LT(error_var, 0.3E-6);
    ASSERT_LT(max_error, 0.5E-6);
    ASSERT_GT(min_error, -0.5E-6);
}


TEST_F(GpsL1CATelemetryDecoderTest, ValidationOfResults)
{
    // Configure the signal generator
    configure_generator();

    // Generate signal raw signal samples and observations RINEX file
    if (FLAGS_disable_generator == false)
        {
            generate_signal();
        }

    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);

    configure_receiver();

    // open true observables log file written by the simulator
    Tracking_True_Obs_Reader true_obs_data;
    int test_satellite_PRN = FLAGS_test_satellite_PRN;
    std::cout << "Testing satellite PRN=" << test_satellite_PRN << std::endl;
    std::string true_obs_file = std::string("./gps_l1_ca_obs_prn");
    true_obs_file.append(std::to_string(test_satellite_PRN));
    true_obs_file.append(".dat");
    ASSERT_NO_THROW({
        if (true_obs_data.open_obs_file(true_obs_file) == false)
            {
                throw std::exception();
            };
    }) << "Failure opening true observables file";

    top_block = gr::make_top_block("Telemetry_Decoder test");
    std::shared_ptr<TrackingInterface> tracking = std::make_shared<GpsL1CaDllPllTracking>(config.get(), "Tracking_1C", 1, 1);
    // std::shared_ptr<TrackingInterface> tracking = std::make_shared<GpsL1CaDllPllCAidTracking>(config.get(), "Tracking_1C", 1, 1);

    boost::shared_ptr<GpsL1CADllPllTelemetryDecoderTest_msg_rx> msg_rx = GpsL1CADllPllTelemetryDecoderTest_msg_rx_make();

    // load acquisition data based on the first epoch of the true observations
    ASSERT_NO_THROW({
        if (true_obs_data.read_binary_obs() == false)
            {
                throw std::exception();
            };
    }) << "Failure reading true observables file";

    // restart the epoch counter
    true_obs_data.restart();

    std::cout << "Initial Doppler [Hz]=" << true_obs_data.doppler_l1_hz << " Initial code delay [Chips]=" << true_obs_data.prn_delay_chips << std::endl;
    gnss_synchro.Acq_delay_samples = (GPS_L1_CA_CODE_LENGTH_CHIPS - true_obs_data.prn_delay_chips / GPS_L1_CA_CODE_LENGTH_CHIPS) * baseband_sampling_freq * GPS_L1_CA_CODE_PERIOD_S;
    gnss_synchro.Acq_doppler_hz = true_obs_data.doppler_l1_hz;
    gnss_synchro.Acq_samplestamp_samples = 0;

    std::shared_ptr<TelemetryDecoderInterface> tlm(new GpsL1CaTelemetryDecoder(config.get(), "TelemetryDecoder_1C", 1, 1));
    tlm->set_channel(0);

    boost::shared_ptr<GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx> tlm_msg_rx = GpsL1CADllPllTelemetryDecoderTest_tlm_msg_rx_make();

    ASSERT_NO_THROW({
        tracking->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        tracking->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        tracking->connect(top_block);
    }) << "Failure connecting tracking to the top_block.";

    ASSERT_NO_THROW({
        std::string file = "./" + filename_raw_data;
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(int8_t), file_name, false);
        gr::blocks::interleaved_char_to_complex::sptr gr_interleaved_char_to_complex = gr::blocks::interleaved_char_to_complex::make();
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, gr_interleaved_char_to_complex, 0);
        top_block->connect(gr_interleaved_char_to_complex, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, tlm->get_left_block(), 0);
        top_block->connect(tlm->get_right_block(), 0, sink, 0);
        top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks.";

    tracking->start_tracking();

    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";

    // check results
    // load the true values
    int64_t nepoch = true_obs_data.num_epochs();
    std::cout << "True observation epochs=" << nepoch << std::endl;

    arma::vec true_timestamp_s = arma::zeros(nepoch, 1);
    arma::vec true_acc_carrier_phase_cycles = arma::zeros(nepoch, 1);
    arma::vec true_Doppler_Hz = arma::zeros(nepoch, 1);
    arma::vec true_prn_delay_chips = arma::zeros(nepoch, 1);
    arma::vec true_tow_s = arma::zeros(nepoch, 1);

    int64_t epoch_counter = 0;
    while (true_obs_data.read_binary_obs())
        {
            true_timestamp_s(epoch_counter) = true_obs_data.signal_timestamp_s;
            true_acc_carrier_phase_cycles(epoch_counter) = true_obs_data.acc_carrier_phase_cycles;
            true_Doppler_Hz(epoch_counter) = true_obs_data.doppler_l1_hz;
            true_prn_delay_chips(epoch_counter) = true_obs_data.prn_delay_chips;
            true_tow_s(epoch_counter) = true_obs_data.tow;
            epoch_counter++;
        }

    // load the measured values
    Tlm_Dump_Reader tlm_dump;
    ASSERT_NO_THROW({
        if (tlm_dump.open_obs_file(std::string("./telemetry0.dat")) == false)
            {
                throw std::exception();
            };
    }) << "Failure opening telemetry dump file";

    nepoch = tlm_dump.num_epochs();
    std::cout << "Measured observation epochs=" << nepoch << std::endl;

    arma::vec tlm_timestamp_s = arma::zeros(nepoch, 1);
    arma::vec tlm_TOW_at_Preamble = arma::zeros(nepoch, 1);
    arma::vec tlm_tow_s = arma::zeros(nepoch, 1);

    epoch_counter = 0;
    while (tlm_dump.read_binary_obs())
        {
            tlm_timestamp_s(epoch_counter) = static_cast<double>(tlm_dump.Tracking_sample_counter) / static_cast<double>(baseband_sampling_freq);
            tlm_TOW_at_Preamble(epoch_counter) = tlm_dump.d_TOW_at_Preamble;
            tlm_tow_s(epoch_counter) = tlm_dump.TOW_at_current_symbol;
            epoch_counter++;
        }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> "Like next"
=======
>>>>>>> set to normal
    // Cut measurement initial transitory of the measurements
    arma::uvec initial_meas_point = arma::find(tlm_tow_s >= true_tow_s(0), 1, "first");
    ASSERT_EQ(initial_meas_point.is_empty(), false);
    tlm_timestamp_s = tlm_timestamp_s.subvec(initial_meas_point(0), tlm_timestamp_s.size() - 1);
    tlm_tow_s = tlm_tow_s.subvec(initial_meas_point(0), tlm_tow_s.size() - 1);

    check_results(true_timestamp_s, true_tow_s, tlm_timestamp_s, tlm_tow_s);

    std::cout << "Test completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> set to normal
=======
=======
    return 0;
>>>>>>> From GNSS-SDR
>>>>>>> From GNSS-SDR
=======
>>>>>>> "Like next"
=======
>>>>>>> set to normal
}
