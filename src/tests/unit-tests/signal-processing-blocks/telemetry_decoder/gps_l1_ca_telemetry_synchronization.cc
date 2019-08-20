/*!
 * \file gps_l1_ca_telemetry_decoder_lucas_test.cc
 * \brief  This class implements a telemetry decoder test for GPS_L1_CA_Telemetry_Decoder
 *  implementation based on some input parameters.
 * \author Lucas Ventura, 2015. lucas.ventura.r(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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

#include "gnss_satellite.h"
#include "gps_navigation_message.h"

#include <random>

#define vector_size 6000 	// 20 sub-frames with 300 bits


class GpsL1CATelemetrySynchronizationTest : public ::testing::Test
{
public:
	std::vector<int> initial_vector;
	std::vector<double> synchro_vector;
	
	void preamble_samples();
	void make_vector();
	void fill_gnss_synchro();
	
	// gps_l1_ca_telemetry_decoder_gs.h
	int32_t d_bits_per_preamble = GPS_CA_PREAMBLE_LENGTH_BITS;
	int32_t d_samples_per_preamble = d_bits_per_preamble;
	int32_t d_preamble_period_symbols = GPS_SUBFRAME_BITS;
	
	// set the preamble
	uint32_t d_required_symbols = GPS_SUBFRAME_BITS;
	// preamble bits to sampled symbols
	uint32_t d_frame_length_symbols = GPS_SUBFRAME_BITS * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;
	uint32_t d_max_symbols_without_valid_frame = d_required_symbols * 20;  // rise alarm 120 segs without valid tlm
	
	std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_BITS> d_preamble_samples{};
	

	bool flag_PLL_180_deg_phase_locked;
	
	// navigation message vars
	uint32_t d_prev_GPS_frame_4bytes = 0;

	boost::circular_buffer<float> d_symbol_history;

	uint64_t d_sample_counter = 0ULL;
	uint64_t d_preamble_index = 0ULL;
	uint64_t d_last_valid_preamble = 0;
	
	bool d_sent_tlm_failed_msg = false;
	uint32_t d_stat = 0;
	bool d_flag_frame_sync = false;
	bool d_flag_parity = false;
	bool d_flag_preamble = false;
	int32_t d_CRC_error_counter = 0;

	int32_t d_channel = 0;

	uint32_t d_TOW_at_Preamble_ms = 0;
	uint32_t d_TOW_at_current_symbol_ms = 0;

	bool flag_TOW_set = false;
		//bool d_dump;
		//std::string d_dump_filename = "telemetry";
		// std::ofstream d_dump_file;
};


void GpsL1CATelemetrySynchronizationTest::preamble_samples()
{
	int32_t n = 0;
	
	for (int32_t i = 0; i < d_bits_per_preamble; i++)
		{
			if (GPS_CA_PREAMBLE.at(i) == '1')
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
	
	
	d_symbol_history.set_capacity(d_required_symbols);
}

void GpsL1CATelemetrySynchronizationTest::make_vector()
{
	std::random_device rd;  //Otain a seed for the random number engine
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_int_distribution<> dis(0, 1);
	 
	for (int32_t i = 0; i < vector_size; i++)
	{
		// SW = 1 0 0 0 1 0 1 1
		// d_preamble_period_symbols = 300
		
    	if(i%d_preamble_period_symbols == 0)
            initial_vector.push_back(1);
    	else if((i-1)%d_preamble_period_symbols == 0)
            initial_vector.push_back(-1);
    	else if((i-2)%d_preamble_period_symbols == 0)
    		initial_vector.push_back(-1);
    	else if((i-3)%d_preamble_period_symbols == 0)
    	    initial_vector.push_back(-1);
    	else if((i-4)%d_preamble_period_symbols == 0)
    		initial_vector.push_back(1);
    	else if((i-5)%d_preamble_period_symbols == 0)
    		initial_vector.push_back(-1);
    	else if((i-6)%d_preamble_period_symbols == 0)
    		initial_vector.push_back(1);
    	else if((i-7)%d_preamble_period_symbols == 0)
    		initial_vector.push_back(1);
    	else
    		initial_vector.push_back(dis(gen)*2-1); //Transform the random unsigned int generated by gen into an int in [-1, 1]
	}
}



void GpsL1CATelemetrySynchronizationTest::fill_gnss_synchro()
{
	double Prompt_I;
	
    // Random generator with Gaussian distribution
    const double mean = 0.0;
    const double stddev = 0.0;
    auto dist = std::bind(std::normal_distribution<double>{mean, stddev},
                          std::mt19937(std::random_device{}()));
	
    for (int32_t i = 0; i < vector_size; i++)
    {
    	Prompt_I = initial_vector[i] + dist();
    	    	
    	synchro_vector.push_back(Prompt_I);
    }   
	
}

/*
// TEST check
TEST_F(GpsL1CATelemetrySynchronizationTest, Check)
{
	std::chrono::time_point<std::chrono::system_clock> start, end;
	std::chrono::duration<double> elapsed_seconds(0);
	start = std::chrono::system_clock::now();
	
	preamble_samples();
	make_vector();
	fill_gnss_synchro();
	
	for (int32_t i = 0; i < vector_size; i++)
	{
		std::cout << i << "\t\t" << initial_vector[i] << std::endl;
	}
	
	
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "GPSL1CA Telemetry decoder Test completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
	
}
*/

// TEST ValidationOfResults
TEST_F(GpsL1CATelemetrySynchronizationTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    start = std::chrono::system_clock::now();
	
    preamble_samples();
    make_vector();
    fill_gnss_synchro();
    
    //int32_t n_
    
    for (int32_t i = 0; i < vector_size; i++) 
    {
    	//std::cout << i << "\t" << initial_vector[i] << "\t" << synchro_vector[i] << std::endl;	
    	//std::cout << initial_vector[i] << "\t" << synchro_vector[i] << std::endl;
    	
    	d_symbol_history.push_back(synchro_vector[i]);
    	d_sample_counter++;
    	
    	d_flag_preamble = false;
    	
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
						std::cout << "Preamble detection for GPS L1 satellite " << i << std::endl;
						
						
						//decode_subframe();
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
								std::cout << "Preamble confirmation " << i << std::endl;
								d_preamble_index = d_sample_counter;  // record the preamble sample stamp
								if (corr_value < 0)
									{
										flag_PLL_180_deg_phase_locked = true;
									}
								else
									{
										flag_PLL_180_deg_phase_locked = false;
									}
								// decode_subframe();
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
			
			case 2: // preamble acquired
			{
				 if (d_sample_counter >= d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
					{
					 	 std::cout << "Preamble received. "  << "d_sample_counter= " << d_sample_counter << std::endl;
					 	 // call the decoder
					 	 // 0. fetch the symbols into an array
					 	 d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)
					}
				
				break;
			}
		}
    }
    
    
    
    // std::cout << "Synchro vector " << synchro_vector[0].Prompt_I;
    
    std::cout << "\n";
	
    
    
    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "GPSL1CA Telemetry decoder Test completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}
