/*!
 * \file spir_gss6450_file_signal_source.h
 * \brief Implementation of a class that reads signals samples from a SPIR file
 * and adapts it to a SignalSourceInterface.
 * \author Antonio Ramos, 2017 antonio.ramos(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is not part of GNSS-SDR.
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

#ifndef GNSS_SDR_SPIR_GSS6450_FILE_SIGNAL_SOURCE_H_
#define GNSS_SDR_SPIR_GSS6450_FILE_SIGNAL_SOURCE_H_

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include "gnss_sdr_valve.h"
#include "unpack_spir_gss6450_samples.h"
#include <gnuradio/blocks/deinterleave.h>
#include <gnuradio/blocks/endian_swap.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/hier_block2.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>


class ConfigurationInterface;

/*!
 * \brief Class that reads signals samples from a file
 * and adapts it to a SignalSourceInterface
 */
class SpirGSS6450FileSignalSource : public GNSSBlockInterface
{
public:
    SpirGSS6450FileSignalSource(ConfigurationInterface* configuration, const std::string& role,
        uint32_t in_streams, uint32_t out_streams, std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    ~SpirGSS6450FileSignalSource() = default;
    inline std::string role() override
    {
        return role_;
    }

    inline std::string implementation() override
    {
        return "Spir_GSS6450_File_Signal_Source";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block(int RF_channel) override;
    gr::basic_block_sptr get_right_block() override;

    inline std::string filename() const
    {
        return filename_;
    }

    inline std::string item_type() const
    {
        return item_type_;
    }

    inline bool repeat() const
    {
        return repeat_;
    }

    inline int64_t sampling_frequency() const
    {
        return sampling_frequency_;
    }

    inline uint64_t samples() const
    {
        return samples_;
    }

private:
    uint64_t samples_;
    int64_t sampling_frequency_;
    std::string filename_;
    bool repeat_;
    bool dump_;  // Enables dumping the gr_complex sample output
    bool enable_throttle_control_;
    bool endian_swap_;
    std::string dump_filename_;
    std::string role_;
    std::string item_type_;
    uint32_t in_streams_;
    uint32_t out_streams_;
    uint32_t adc_bits_;
    uint32_t n_channels_;
    uint32_t sel_ch_;
    gr::blocks::file_source::sptr file_source_;
    gr::blocks::deinterleave::sptr deint_;
    std::vector<gr::blocks::endian_swap::sptr> endian_vec_;
    std::vector<gr::blocks::null_sink::sptr> null_sinks_;
    std::vector<unpack_spir_gss6450_samples_sptr> unpack_spir_vec_;
    std::vector<boost::shared_ptr<gr::block>> valve_vec_;
    std::vector<gr::blocks::file_sink::sptr> sink_vec_;
    std::vector<gr::blocks::throttle::sptr> throttle_vec_;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue_;
    size_t item_size_;
};

#endif  // GNSS_SDR_SPIR_GSS6450_FILE_SIGNAL_SOURCE_H_
