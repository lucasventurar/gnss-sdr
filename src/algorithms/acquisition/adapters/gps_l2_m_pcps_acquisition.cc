/*!
 * \file gps_l2_m_pcps_acquisition.cc
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L2 M signals
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
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

#include "gps_l2_m_pcps_acquisition.h"
#include "GPS_L2C.h"
#include "acq_conf.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include "gps_l2c_signal.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include <algorithm>


GpsL2MPcpsAcquisition::GpsL2MPcpsAcquisition(
    ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./acquisition.mat";

    LOG(INFO) << "role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);
    int64_t fs_in_deprecated = configuration_->property("GNSS-SDR.internal_fs_hz", 2048000);
    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    acq_parameters_.fs_in = fs_in_;
    dump_ = configuration_->property(role + ".dump", false);
    acq_parameters_.dump = dump_;
    acq_parameters_.dump_channel = configuration_->property(role + ".dump_channel", 0);
    blocking_ = configuration_->property(role + ".blocking", true);
    acq_parameters_.blocking = blocking_;
    doppler_max_ = configuration->property(role + ".doppler_max", 5000);
    if (FLAGS_doppler_max != 0)
        {
            doppler_max_ = FLAGS_doppler_max;
        }
    acq_parameters_.doppler_max = doppler_max_;
    bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);
    acq_parameters_.bit_transition_flag = bit_transition_flag_;
    use_CFAR_algorithm_flag_ = configuration_->property(role + ".use_CFAR_algorithm", true);  // will be false in future versions
    acq_parameters_.use_CFAR_algorithm_flag = use_CFAR_algorithm_flag_;
    max_dwells_ = configuration_->property(role + ".max_dwells", 1);
    acq_parameters_.max_dwells = max_dwells_;
    dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);
    acq_parameters_.dump_filename = dump_filename_;
    acq_parameters_.ms_per_code = 20;
    acq_parameters_.sampled_ms = configuration_->property(role + ".coherent_integration_time_ms", acq_parameters_.ms_per_code);
    if ((acq_parameters_.sampled_ms % acq_parameters_.ms_per_code) != 0)
        {
            LOG(WARNING) << "Parameter coherent_integration_time_ms should be a multiple of 20. Setting it to 20";
            acq_parameters_.sampled_ms = acq_parameters_.ms_per_code;
        }

    acq_parameters_.num_doppler_bins_step2 = configuration_->property(role + ".second_nbins", 4);
    acq_parameters_.doppler_step2 = configuration_->property(role + ".second_doppler_step", 125.0);
    acq_parameters_.make_2_steps = configuration_->property(role + ".make_two_steps", false);
    acq_parameters_.blocking_on_standby = configuration_->property(role + ".blocking_on_standby", false);
    acq_parameters_.use_automatic_resampler = configuration_->property("GNSS-SDR.use_acquisition_resampler", false);
    if (acq_parameters_.use_automatic_resampler == true and item_type_ != "gr_complex")
        {
            LOG(WARNING) << "GPS L2CM acquisition disabled the automatic resampler feature because its item_type is not set to gr_complex";
            acq_parameters_.use_automatic_resampler = false;
        }
    if (acq_parameters_.use_automatic_resampler)
        {
            if (acq_parameters_.fs_in > GPS_L2C_OPT_ACQ_FS_SPS)
                {
                    acq_parameters_.resampler_ratio = floor(static_cast<float>(acq_parameters_.fs_in) / GPS_L2C_OPT_ACQ_FS_SPS);
                    uint32_t decimation = acq_parameters_.fs_in / GPS_L2C_OPT_ACQ_FS_SPS;
                    while (acq_parameters_.fs_in % decimation > 0)
                        {
                            decimation--;
                        };
                    acq_parameters_.resampler_ratio = decimation;
                    acq_parameters_.resampled_fs = acq_parameters_.fs_in / static_cast<int>(acq_parameters_.resampler_ratio);
                }

            //--- Find number of samples per spreading code -------------------------
            code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(acq_parameters_.resampled_fs) / (GPS_L2_M_CODE_RATE_CPS / GPS_L2_M_CODE_LENGTH_CHIPS)));
            acq_parameters_.samples_per_ms = static_cast<float>(acq_parameters_.resampled_fs) * 0.001;
            acq_parameters_.samples_per_chip = static_cast<unsigned int>(ceil((1.0 / GPS_L2_M_CODE_RATE_CPS) * static_cast<float>(acq_parameters_.resampled_fs)));
        }
    else
        {
            acq_parameters_.resampled_fs = fs_in_;
            //--- Find number of samples per spreading code -------------------------
            code_length_ = static_cast<unsigned int>(std::floor(static_cast<double>(fs_in_) / (GPS_L2_M_CODE_RATE_CPS / GPS_L2_M_CODE_LENGTH_CHIPS)));
            acq_parameters_.samples_per_ms = static_cast<float>(fs_in_) * 0.001;
            acq_parameters_.samples_per_chip = static_cast<unsigned int>(ceil((1.0 / GPS_L2_M_CODE_RATE_CPS) * static_cast<float>(acq_parameters_.fs_in)));
        }

    acq_parameters_.samples_per_code = acq_parameters_.samples_per_ms * static_cast<float>(GPS_L2_M_PERIOD_S * 1000.0);
    vector_length_ = acq_parameters_.sampled_ms * acq_parameters_.samples_per_ms * (acq_parameters_.bit_transition_flag ? 2 : 1);
    code_ = std::vector<std::complex<float>>(vector_length_);

    if (item_type_ == "cshort")
        {
            item_size_ = sizeof(lv_16sc_t);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
        }

    acq_parameters_.it_size = item_size_;
    acquisition_ = pcps_make_acquisition(acq_parameters_);
    DLOG(INFO) << "acquisition(" << acquisition_->unique_id() << ")";

    if (item_type_ == "cbyte")
        {
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
            float_to_complex_ = gr::blocks::float_to_complex::make();
        }

    channel_ = 0;
    threshold_ = 0.0;
    doppler_step_ = 0;
    doppler_center_ = 0;
    gnss_synchro_ = nullptr;

    num_codes_ = acq_parameters_.sampled_ms / acq_parameters_.ms_per_code;
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }
}


void GpsL2MPcpsAcquisition::stop_acquisition()
{
}


void GpsL2MPcpsAcquisition::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_ + std::to_string(channel_) + ".pfa", 0.0);

    if (pfa == 0.0)
        {
            pfa = configuration_->property(role_ + ".pfa", 0.0);
        }
    if (pfa == 0.0)
        {
            threshold_ = threshold;
        }
    else
        {
            threshold_ = calculate_threshold(pfa);
        }

    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold_;

    acquisition_->set_threshold(threshold_);
}


void GpsL2MPcpsAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    acquisition_->set_doppler_max(doppler_max_);
}


// Be aware that Doppler step should be set to 2/(3T) Hz, where T is the coherent integration time (GPS L2 period is 0.02s)
// Doppler bin minimum size= 33 Hz
void GpsL2MPcpsAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;

    acquisition_->set_doppler_step(doppler_step_);
}


void GpsL2MPcpsAcquisition::set_doppler_center(int doppler_center)
{
    doppler_center_ = doppler_center;

    acquisition_->set_doppler_center(doppler_center_);
}


void GpsL2MPcpsAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    acquisition_->set_gnss_synchro(gnss_synchro_);
}


signed int GpsL2MPcpsAcquisition::mag()
{
    return acquisition_->mag();
}


void GpsL2MPcpsAcquisition::init()
{
    acquisition_->init();
}


void GpsL2MPcpsAcquisition::set_local_code()
{
    std::vector<std::complex<float>> code(code_length_);

    if (acq_parameters_.use_automatic_resampler)
        {
            gps_l2c_m_code_gen_complex_sampled(code, gnss_synchro_->PRN, acq_parameters_.resampled_fs);
        }
    else
        {
            gps_l2c_m_code_gen_complex_sampled(code, gnss_synchro_->PRN, fs_in_);
        }

    gsl::span<gr_complex> code_span(code_.data(), vector_length_);
    for (unsigned int i = 0; i < num_codes_; i++)
        {
            std::copy_n(code.data(), code_length_, code_span.subspan(i * code_length_, code_length_).data());
        }

    acquisition_->set_local_code(code_.data());
}


void GpsL2MPcpsAcquisition::reset()
{
    acquisition_->set_active(true);
}


void GpsL2MPcpsAcquisition::set_state(int state)
{
    acquisition_->set_state(state);
}


float GpsL2MPcpsAcquisition::calculate_threshold(float pfa)
{
    // Calculate the threshold
    unsigned int frequency_bins = 0;
    for (int doppler = static_cast<int>(-doppler_max_); doppler <= static_cast<int>(doppler_max_); doppler += doppler_step_)
        {
            frequency_bins++;
        }
    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;
    unsigned int ncells = vector_length_ * frequency_bins;
    double exponent = 1.0 / static_cast<double>(ncells);
    double val = pow(1.0 - pfa, exponent);
    auto lambda = static_cast<double>(vector_length_);
    boost::math::exponential_distribution<double> mydist(lambda);
    auto threshold = static_cast<float>(quantile(mydist, val));

    return threshold;
}


void GpsL2MPcpsAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            // nothing to connect
        }
    else if (item_type_ == "cshort")
        {
            // nothing to connect
        }
    else if (item_type_ == "cbyte")
        {
            // Since a byte-based acq implementation is not available,
            // we just convert cshorts to gr_complex
            top_block->connect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->connect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->connect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


void GpsL2MPcpsAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            // nothing to disconnect
        }
    else if (item_type_ == "cshort")
        {
            // nothing to disconnect
        }
    else if (item_type_ == "cbyte")
        {
            top_block->disconnect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->disconnect(float_to_complex_, 0, acquisition_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


gr::basic_block_sptr GpsL2MPcpsAcquisition::get_left_block()
{
    if (item_type_ == "gr_complex")
        {
            return acquisition_;
        }
    if (item_type_ == "cshort")
        {
            return acquisition_;
        }
    if (item_type_ == "cbyte")
        {
            return cbyte_to_float_x2_;
        }

    LOG(WARNING) << item_type_ << " unknown acquisition item type";
    return nullptr;
}


gr::basic_block_sptr GpsL2MPcpsAcquisition::get_right_block()
{
    return acquisition_;
}


void GpsL2MPcpsAcquisition::set_resampler_latency(uint32_t latency_samples)
{
    acquisition_->set_resampler_latency(latency_samples);
}
