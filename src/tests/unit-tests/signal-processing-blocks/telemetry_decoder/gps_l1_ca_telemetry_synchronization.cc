/*!
 * \file gps_l1_ca_telemetry_synchronization.cc
 * \brief  This class implements a telemetry decoder synchronization test for GPS_L1_CA_Telemetry_Decoder with SC
 * \note Code added as part of GSoC 2019 program
 * \author Lucas Ventura, 2019. lucas.ventura.r(at)gmail.com
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

#include "gnss_synchro.h"
#include <gtest/gtest.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <cmath>

#define vector_size 1800     // 6 sub-frames with 300 bits (5 SW)
#define Nw 1000              // Number of Monte-Carlo realizations
#define threshold 0	          // Threshold for SLRT
#define snr 10
#define probability 1       // If probability = 1, probability of detection is computed, if 0, probability of false alarm computed


/*!
 * \ The fixture for testing class GpsL1CATelemetrySynchronizationTest.
 *
 */
class GpsL1CATelemetrySynchronizationTest : public ::testing::Test
{
public:
    // Simulates frames (with the SW) from the navigation message before the noise is added
    std::vector<int> initial_vector_pd;
    std::vector<int> initial_vector_pfa;
    // initial_vector with noise
    std::vector<double> synchro_vector;

    // Initializes d_preamble_samples with the SW
    void preamble_samples();
    // Fills initial_vector with the preamble and random data
    void make_vector();
    // Fills synchro_vector (initial_vector + noise)
    void fill_gnss_synchro();

    // Initializate like in gps_l1_ca_telemetry_decoder_gs
    int32_t d_bits_per_preamble = GPS_CA_PREAMBLE_LENGTH_BITS;
    int32_t d_samples_per_preamble = d_bits_per_preamble;
    int32_t d_preamble_period_symbols = GPS_SUBFRAME_BITS;

    // set the preamble
    uint32_t d_required_symbols = GPS_SUBFRAME_BITS;
    // preamble bits to sampled symbols
    uint32_t d_frame_length_symbols = GPS_SUBFRAME_BITS * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;

    std::array<int32_t, GPS_CA_PREAMBLE_LENGTH_BITS> d_preamble_samples{};

    bool flag_PLL_180_deg_phase_locked;

    boost::circular_buffer<float> d_symbol_history;

    uint64_t d_sample_counter = 0ULL;
    uint64_t d_preamble_index = 0ULL;

    uint32_t d_stat = 0;

    double stddev = 0.0;

    int preamble_offset = 0; // Random data before preamble

    bool d_flag_preamble = false;
};

/*!
 * \ Initializes d_preamble_samples with the SW
 *
 */
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


/*!
 * \ Vector with preambles and random data
 *
 * The first preamble starts in "preamble_offset".
 * Preamble every "d_preamble_period_symbols" (300).
 * The Synchronization Word is "10001011".
 */
void GpsL1CATelemetrySynchronizationTest::make_vector()
{
    std::random_device rd;   //Otain a seed for the random number engine
    std::mt19937 gen(rd());  //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis(0, 1);
    std::uniform_int_distribution<> offset(0, 299);


    preamble_offset = offset(gen);

    for (int32_t i = 0; i < vector_size; i++)
        {
            if ((i - preamble_offset) % d_preamble_period_symbols == 0)
            {
              initial_vector_pd.push_back(1);
              initial_vector_pfa.push_back(dis(gen) * 2 - 1);
            }
            else if ((i - 1 - preamble_offset) % d_preamble_period_symbols == 0)
            {
                initial_vector_pd.push_back(-1);
                initial_vector_pfa.push_back(dis(gen) * 2 - 1);
            }
            else if ((i - 2 - preamble_offset) % d_preamble_period_symbols == 0)
            {
                initial_vector_pd.push_back(-1);
                initial_vector_pfa.push_back(dis(gen) * 2 - 1);
            }
            else if ((i - 3 - preamble_offset) % d_preamble_period_symbols == 0)
            {
              initial_vector_pd.push_back(-1);
              initial_vector_pfa.push_back(dis(gen) * 2 - 1);
            }
            else if ((i - 4 - preamble_offset) % d_preamble_period_symbols == 0)
            {
                initial_vector_pd.push_back(1);
                initial_vector_pfa.push_back(dis(gen) * 2 - 1);
            }
            else if ((i - 5 - preamble_offset) % d_preamble_period_symbols == 0)
            {
                initial_vector_pd.push_back(-1);
                initial_vector_pfa.push_back(dis(gen) * 2 - 1);
            }
            else if ((i - 6 - preamble_offset) % d_preamble_period_symbols == 0)
            {
                initial_vector_pd.push_back(1);
                initial_vector_pfa.push_back(dis(gen) * 2 - 1);
            }
            else if ((i - 7 - preamble_offset) % d_preamble_period_symbols == 0)
            {
                initial_vector_pd.push_back(1);
                initial_vector_pfa.push_back(dis(gen) * 2 - 1);
            }
            else
            {
                initial_vector_pd.push_back(dis(gen) * 2 - 1);  //Transform the random unsigned int generated by gen into an int in [-1, 1
                initial_vector_pfa.push_back(dis(gen) * 2 - 1);
            }
        }
}


/*!
 * \ Simulates Prompt_I in the gnss_synchro vector.
 *
 * Adds noise with a mean and standard deviation to the initial_vector.
 */
void GpsL1CATelemetrySynchronizationTest::fill_gnss_synchro()
{
    // Random generator with Gaussian distribution
    const double mean = 0.0;
    stddev = sqrt(pow(10, -snr/10)/2); // No = 10^(-EsNodB/10), stddev²=No/2
    auto dist = std::bind(std::normal_distribution<double>{mean, stddev},
        std::mt19937(std::random_device{}()));

    ASSERT_TRUE(probability == 0 || probability == 1);


    // Probability of false alarm is computed
    if(probability == 0){
      for (int32_t i = 0; i < vector_size; i++)
          {
              synchro_vector.push_back(initial_vector_pfa[i] + dist());
          }
    }
    // probability of detection is computed
    else{
      for (int32_t i = 0; i < vector_size; i++)
          {
              synchro_vector.push_back(initial_vector_pd[i] + dist());
          }
    }

}

/*!
 * \ Simulates the synchronization in gps_l1_ca_telemetry_decoder_gs and saves metrics (HC)
 *
 * Computes the number of total preambles (missed and detected)
 * Calculates the number of detected, correct and wrong, preambles in the different states
 * Saves probabilities in a file "synchronization_HC_test_x.csv"
 */
TEST_F(GpsL1CATelemetrySynchronizationTest, HardCorrelator)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    start = std::chrono::system_clock::now();

    int32_t n_s2 = 0;  // Number of synchronizations

    // Monte-Carlo realizations
    for (int32_t n = 0; n < Nw; n++)
        {
            initial_vector_pd.clear();
            initial_vector_pfa.clear();
            synchro_vector.clear();

            d_sample_counter = 0ULL;
            d_preamble_index = 0ULL;

            d_stat = 0;

            d_flag_preamble = false;

            preamble_samples();
            make_vector();
            fill_gnss_synchro();

            for (int32_t i = 0; i < vector_size; i++)
                {
                    d_symbol_history.push_back(synchro_vector[i]);
                    d_sample_counter++;

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
                                        // std::cout << "Preamble detection for GPS L1 satellite " << d_preamble_index << std::endl;

                                        d_stat = 1;  // enter into frame pre-detection status
                                    }

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
                                                d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                                // std::cout << "Preamble confirmation " << d_preamble_index << std::endl;

                                                if (corr_value < 0)
                                                    {
                                                        flag_PLL_180_deg_phase_locked = true;
                                                    }
                                                else
                                                    {
                                                        flag_PLL_180_deg_phase_locked = false;
                                                    }
                                                d_stat = 2;
                                            }
                                        else
                                            {
                                                if (preamble_diff > d_preamble_period_symbols)
                                                    {
                                                        // std::cout << "Preamble missed in s1 " << d_sample_counter << std::endl;
                                                        d_stat = 0;  // start again
                                                    }
                                            }
                                    }

                                break;
                            }

                        case 2:  // preamble acquired
                            {
                                if (d_sample_counter >= d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                                    {
                                        // std::cout << "Preamble received. " << "d_sample_counter= " << d_sample_counter << std::endl;
                                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)

                                        d_flag_preamble = true; // // valid preamble indicator (initialized to false every montecarlo iteration)
                                    }

                                break;
                            }
                        }
                }

                if(d_flag_preamble) // If preamble indicator set to true, adds 1 to the number of preambles found.
                {
                    n_s2++;
                }
        }


    // file pointer
    std::fstream fout;

    // opens an existing csv (std::ios::app) file or creates a new file (std::ios::out).
    std::string path = "synchronization_HC_test.csv";
    std::ifstream fin(path);
    if(fin.fail())
    {
      fout.open(path, std::ios::out);
      fout << "correlation"    << ", "
           << "montecarlo"     << ", "
           << "snr"            << ", "
           << "threshold"      << ", "
           << "probability"    << ", "
           << "n_s2"           << "\n";

     fout << "HC"  << ", "
          << Nw                << ", "
          << snr               << ", "
          << threshold         << ", "
          << probability       << ", " // probability = 1, probability of detection is computed, if 0, probability of false alarm computed
          << n_s2              << "\n";

    }
    else
    {
      fout.open(path, std::ios::app);
      fout << "HC"  << ", "
           << Nw                << ", "
           << snr               << ", "
           << threshold         << ", "
           << probability       << ", " // probability = 1, probability of detection is computed, if 0, probability of false alarm computed
           << n_s2              << "\n";
    }

    // Close file
    fout.close();

    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "\nGPSL1CA Telemetry decoder Test (SC) completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}

/*!
 * \ Simulates the synchronization in gps_l1_ca_telemetry_decoder_gs and saves metrics (SLRT)
 *
 * Computes the number of total preambles (missed and detected)
 * Calculates the number of detected, correct and wrong, preambles in the different states
 * Saves probabilities in a file "synchronization_SLRT_test_x.csv"
 */
TEST_F(GpsL1CATelemetrySynchronizationTest, SLRTCorrelator)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    start = std::chrono::system_clock::now();

    int32_t n_s2 = 0;  // Number of synchronizations

    // Monte-Carlo realizations
    for (int32_t n = 0; n < Nw; n++)
        {
            initial_vector_pd.clear();
            initial_vector_pfa.clear();
            synchro_vector.clear();

            d_sample_counter = 0ULL;
            d_preamble_index = 0ULL;

            d_stat = 0;

            d_flag_preamble = false;

            preamble_samples();
            make_vector();
            fill_gnss_synchro();

            for (int32_t i = 0; i < vector_size; i++)
                {
                    d_symbol_history.push_back(synchro_vector[i]);
                    d_sample_counter++;

                    // ******* frame sync ******************
                    switch (d_stat)
                        {
                        case 0:  // no preamble information
                            {
                                // correlate with preamble
                                int32_t corr_value1 = 0;
                                int32_t corr_value2 = 0;
                                if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_BITS)
                                    {
                                        // ******* preamble correlation ********
                                        corr_value1 += d_symbol_history[i] * d_preamble_samples[i];
                                        corr_value2 += abs(d_symbol_history[i]);
                                    }

                                if (abs(corr_value1) - corr_value2 >= threshold)
                                    {
                                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                        // std::cout << "Preamble detection for GPS L1 satellite " << d_preamble_index << std::endl;

                                        d_stat = 1;  // enter into frame pre-detection status
                                    }

                                break;
                            }
                        case 1:  // possible preamble lock
                            {
                                // correlate with preamble
                                int32_t corr_value1 = 0;
                                int32_t corr_value2 = 0;
                                int32_t preamble_diff = 0;
                                if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_BITS)
                                    {
                                        // ******* preamble correlation ********
                                        for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
                                            {
                                                corr_value1 += d_symbol_history[i] * d_preamble_samples[i];
                                                corr_value2 += abs(d_symbol_history[i]);
                                            }
                                    }
                                if (abs(corr_value1) - corr_value2 >= threshold)
                                    {
                                        // check preamble separation
                                        preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                                        if (abs(preamble_diff - d_preamble_period_symbols) == 0)
                                            {
                                                d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                                // std::cout << "Preamble confirmation " << d_preamble_index << std::endl;

                                                if (abs(corr_value1) - corr_value2 < 0)
                                                    {
                                                        flag_PLL_180_deg_phase_locked = true;
                                                    }
                                                else
                                                    {
                                                        flag_PLL_180_deg_phase_locked = false;
                                                    }
                                                d_stat = 2;
                                            }
                                        else
                                            {
                                                if (preamble_diff > d_preamble_period_symbols)
                                                    {
                                                        // std::cout << "Preamble missed in s1 " << d_sample_counter << std::endl;
                                                        d_stat = 0;  // start again
                                                    }
                                            }
                                    }

                                break;
                            }

                        case 2:  // preamble acquired
                            {
                                if (d_sample_counter >= d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                                    {
                                        // std::cout << "Preamble received. " << "d_sample_counter= " << d_sample_counter << std::endl;
                                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)

                                        d_flag_preamble = true; // // valid preamble indicator (initialized to false every montecarlo iteration)
                                    }

                                break;
                            }
                        }
              }

              // Probability of detection +1
              if(probability == 1 && d_flag_preamble && (d_preamble_index - preamble_offset) % d_preamble_period_symbols == 0)
              {
                  n_s2++;
              }
              // Probability of false error +1
              else if(probability == 0 && d_flag_preamble)
              {
                n_s2++;
              }

        }

    // file pointer
    std::fstream fout;

    // opens an existing csv (std::ios::app) file or creates a new file (std::ios::out).
    std::string path = "synchronization_SLRT_test.csv";
    std::ifstream fin(path);
    if(fin.fail())
    {
      fout.open(path, std::ios::out);
      fout << "correlation"    << ", "
           << "montecarlo"     << ", "
           << "snr"            << ", "
           << "threshold"      << ", "
           << "probability"    << ", "
           << "n_s2"           << "\n";

     fout << "SLRT"            << ", "
          << Nw                << ", "
          << snr               << ", "
          << threshold         << ", "
          << probability       << ", " // probability = 1, probability of detection is computed, if 0, probability of false alarm computed
          << n_s2              << "\n";

    }
    else
    {
      fout.open(path, std::ios::app);
      fout << "SLRT"  << ", "
           << Nw                << ", "
           << snr               << ", "
           << threshold         << ", "
           << probability       << ", " // probability = 1, probability of detection is computed, if 0, probability of false alarm computed
           << n_s2              << "\n";
    }

    // Close file
    fout.close();

    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "\nGPSL1CA Telemetry decoder Test (HC) completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}





// ################################## MULTIPLE THRESHOLDS

/*!
 * \ Simulates the synchronization in gps_l1_ca_telemetry_decoder_gs and saves metrics (SLRT)
 *
 * Computes the number of total preambles (missed and detected)
 * Calculates the number of detected, correct and wrong, preambles in the different states
 * Saves probabilities in a file "synchronization_SLRT_test_x.csv"
 */
TEST_F(GpsL1CATelemetrySynchronizationTest, MultipleSLRTCorrelator)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);
    start = std::chrono::system_clock::now();

    double multiple_threshold = 0.0;

    for (int32_t t = 0; t < 100; t++)
    {
      int32_t n_s2 = 0;  // Number of synchronizations

      // Monte-Carlo realizations
      for (int32_t n = 0; n < Nw; n++)
          {
              initial_vector_pd.clear();
              initial_vector_pfa.clear();
              synchro_vector.clear();

              d_sample_counter = 0ULL;
              d_preamble_index = 0ULL;

              d_stat = 0;

              d_flag_preamble = false;

              preamble_samples();
              make_vector();
              fill_gnss_synchro();

              for (int32_t i = 0; i < vector_size; i++)
                  {
                      d_symbol_history.push_back(synchro_vector[i]);
                      d_sample_counter++;

                      // ******* frame sync ******************
                      switch (d_stat)
                          {
                          case 0:  // no preamble information
                              {
                                  // correlate with preamble
                                  int32_t corr_value1 = 0;
                                  int32_t corr_value2 = 0;
                                  if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_BITS)
                                      {
                                          // ******* preamble correlation ********
                                          corr_value1 += d_symbol_history[i] * d_preamble_samples[i];
                                          corr_value2 += abs(d_symbol_history[i]);
                                      }

                                  if (abs(corr_value1) - corr_value2 >= multiple_threshold)
                                      {
                                          d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                          // std::cout << "Preamble detection for GPS L1 satellite " << d_preamble_index << std::endl;

                                          d_stat = 1;  // enter into frame pre-detection status
                                      }

                                  break;
                              }
                          case 1:  // possible preamble lock
                              {
                                  // correlate with preamble
                                  int32_t corr_value1 = 0;
                                  int32_t corr_value2 = 0;
                                  int32_t preamble_diff = 0;
                                  if (d_symbol_history.size() >= GPS_CA_PREAMBLE_LENGTH_BITS)
                                      {
                                          // ******* preamble correlation ********
                                          for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
                                              {
                                                  corr_value1 += d_symbol_history[i] * d_preamble_samples[i];
                                                  corr_value2 += abs(d_symbol_history[i]);
                                              }
                                      }
                                  if (abs(corr_value1) - corr_value2 >= multiple_threshold)
                                      {
                                          // check preamble separation
                                          preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                                          if (abs(preamble_diff - d_preamble_period_symbols) == 0)
                                              {
                                                  d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                                  // std::cout << "Preamble confirmation " << d_preamble_index << std::endl;

                                                  if (abs(corr_value1) - corr_value2 < 0)
                                                      {
                                                          flag_PLL_180_deg_phase_locked = true;
                                                      }
                                                  else
                                                      {
                                                          flag_PLL_180_deg_phase_locked = false;
                                                      }
                                                  d_stat = 2;
                                              }
                                          else
                                              {
                                                  if (preamble_diff > d_preamble_period_symbols)
                                                      {
                                                          // std::cout << "Preamble missed in s1 " << d_sample_counter << std::endl;
                                                          d_stat = 0;  // start again
                                                      }
                                              }
                                      }

                                  break;
                              }

                          case 2:  // preamble acquired
                              {
                                  if (d_sample_counter >= d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                                      {
                                          // std::cout << "Preamble received. " << "d_sample_counter= " << d_sample_counter << std::endl;
                                          d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)

                                          d_flag_preamble = true; // // valid preamble indicator (initialized to false every montecarlo iteration)
                                      }

                                  break;
                              }
                          }
                }

                // Probability of detection +1
                if(probability == 1 && d_flag_preamble && (d_preamble_index - preamble_offset) % d_preamble_period_symbols == 0)
                {
                    n_s2++;
                }
                // Probability of false error +1
                else if(probability == 0 && d_flag_preamble)
                {
                  n_s2++;
                }

          }

      // file pointer
      std::fstream fout;

      // opens an existing csv (std::ios::app) file or creates a new file (std::ios::out).
      std::string path = "synchronization_multipleSLRT_test.csv";
      std::ifstream fin(path);

      if(fin.fail())
      {
        fout.open(path, std::ios::out);
        fout << "correlation"    << ", "
             << "montecarlo"     << ", "
             << "snr"            << ", "
             << "threshold"      << ", "
             << "probability"    << ", "
             << "n_s2"           << "\n";

       fout << "SLRT"            << ", "
            << Nw                << ", "
            << snr               << ", "
            << multiple_threshold << ", "
            << probability       << ", " // probability = 1, probability of detection is computed, if 0, probability of false alarm computed
            << n_s2              << "\n";

      }
      else
      {
        fout.open(path, std::ios::app);
        fout << "SLRT"            << ", "
             << Nw                << ", "
             << snr               << ", "
             << multiple_threshold << ", "
             << probability       << ", " // probability = 1, probability of detection is computed, if 0, probability of false alarm computed
             << n_s2              << "\n";
      }
      // Close file
      fout.close();


      multiple_threshold -= 0.1;
    }

    end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;
    std::cout << "\nGPSL1CA Telemetry decoder Test (HC) completed in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}
