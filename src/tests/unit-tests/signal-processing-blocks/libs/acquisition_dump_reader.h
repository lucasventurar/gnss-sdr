/*!
 * \file acquisition_dump_reader.h
 * \brief Helper file for unit testing
 * \authors Carles Fernandez-Prades, 2017. cfernandez(at)cttc.es
 *                    Antonio Ramos, 2018. antonio.ramos(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_ACQUISITION_DUMP_READER_H
#define GNSS_SDR_ACQUISITION_DUMP_READER_H

#include <cstdint>
#include <string>
#include <vector>

class Acquisition_Dump_Reader
{
public:
    Acquisition_Dump_Reader(const std::string& basename,
        unsigned int sat,
        unsigned int doppler_max,
        unsigned int doppler_step,
        unsigned int samples_per_code,
        int channel = 0,
        int execution = 1);

    Acquisition_Dump_Reader(const std::string& basename,
        int channel = 0,
        int execution = 1);

    ~Acquisition_Dump_Reader() = default;

    Acquisition_Dump_Reader(Acquisition_Dump_Reader&& other) noexcept;             //!< Copy constructor
    Acquisition_Dump_Reader& operator=(const Acquisition_Dump_Reader&);            //!< Copy assignment operator
    Acquisition_Dump_Reader(const Acquisition_Dump_Reader& other) noexcept;        //!< Move constructor
    Acquisition_Dump_Reader& operator=(Acquisition_Dump_Reader&& other) noexcept;  //!< Move assignment operator

    bool read_binary_acq();

    std::vector<int> doppler;
    std::vector<unsigned int> samples;
    std::vector<std::vector<float> > mag;
    float acq_doppler_hz;
    float acq_delay_samples;
    float test_statistic;
    float input_power;
    float threshold;
    int positive_acq;
    unsigned int PRN;
    unsigned int num_dwells;
    uint64_t sample_counter;

private:
    std::string d_basename;
    unsigned int d_sat;
    unsigned int d_doppler_max;
    unsigned int d_doppler_step;
    unsigned int d_samples_per_code;
    unsigned int d_num_doppler_bins;
    std::string d_dump_filename;
};

#endif  // GNSS_SDR_ACQUISITION_DUMP_READER_H
