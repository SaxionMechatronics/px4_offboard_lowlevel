/****************************************************************************
 *
 *   Copyright (c) 2023, SMART Research Group, Saxion University of 
 *   Applied Sciences.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "../include/px4_offboard_lowlevel/controller.h"
#include <iostream>
#include <algorithm>


controller::controller()
    : env_(ORT_LOGGING_LEVEL_WARNING, "controller"),
      session_options_(),
      session_(nullptr)  // temp placeholder, real initialization in loadPolicy
{
    session_options_.SetIntraOpNumThreads(1);
}

void controller::loadPolicy(std::string policy_file) {
    std::cout << "Policy file: \"" << policy_file << '"' << std::endl;

    std::string package_share_dir = ament_index_cpp::get_package_share_directory("px4_offboard_lowlevel");
    std::string model_path = package_share_dir + "/policy/" + policy_file;

    session_ = Ort::Session(env_, model_path.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;

    // Input Info
    Ort::AllocatedStringPtr input_name_ptr = session_.GetInputNameAllocated(0, allocator);
    input_name_ = input_name_ptr.get();

    // Output Info
    Ort::AllocatedStringPtr output_name_ptr = session_.GetOutputNameAllocated(0, allocator);
    output_name_ = output_name_ptr.get();
}

std::vector<float> controller::getObs() {
    const Eigen::Vector3d e_p = r_position_W_ - position_W_;

    std::vector<float> obs(18, 0.0f);
    obs[0]  = e_p(0);
    obs[1]  = e_p(1);
    obs[2]  = e_p(2);
    obs[3]  = R_B_W_(0,0);
    obs[4]  = R_B_W_(0,1);
    obs[5]  = R_B_W_(0,2);
    obs[6]  = R_B_W_(1,0);
    obs[7]  = R_B_W_(1,1);
    obs[8]  = R_B_W_(1,2);
    obs[9]  = R_B_W_(2,0);
    obs[10] = R_B_W_(2,1);
    obs[11] = R_B_W_(2,2);
    obs[12] = velocity_B_(0);
    obs[13] = velocity_B_(1);
    obs[14] = velocity_B_(2);
    obs[15] = angular_velocity_B_(0);
    obs[16] = angular_velocity_B_(1);
    obs[17] = angular_velocity_B_(2);

    return obs;
}

float* controller::forwardPolicy(std::vector<float> input_data) {
    std::vector<int64_t> input_shape{1, static_cast<int64_t>(input_data.size())};

    std::vector<const char*> input_names = {input_name_.c_str()};
    std::vector<const char*> output_names = {output_name_.c_str()};

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        input_data.data(),
        input_data.size() * sizeof(float),
        input_shape.data(),
        input_shape.size()
    );

    // Run inference
    auto output_tensors = session_.Run(
        Ort::RunOptions{nullptr}, input_names.data(), &input_tensor, 1,
        output_names.data(), 1);

    // Extract output
    return output_tensors.front().GetTensorMutableData<float>();
}

void controller::calculateControllerOutput(Eigen::VectorXd *controller_rates_thrust) {
    assert(controller_rates_thrust);

    // Get policy output
    float* output_data = forwardPolicy(getObs());
    std::cout << "Act: " << output_data[0] << '\t'  << output_data[1] << '\t'  << output_data[2] << std::endl;

    // Clamp output
    output_data[0] = std::clamp(output_data[0], -1.0f, 1.0f);
    output_data[1] = std::clamp(output_data[1], -1.0f, 1.0f);
    output_data[2] = std::clamp(output_data[2], -1.0f, 1.0f);

    // Proportional yaw control
    const float yaw = atan2(R_B_W_(1,0), R_B_W_(0,0));
    const float yaw_target = atan2(r_R_B_W_(1,0), r_R_B_W_(0,0));
    const float yaw_rate = (yaw_target - yaw) * 0.5;

    // Thrust with gravity compensation
    double thrust = output_data[0] * thrust_scale_ + (_uav_mass * _gravity);
    Eigen::Vector3d rates(output_data[1] * rate_scale_, output_data[2] * rate_scale_, yaw_rate);

    // Output the wrench
    controller_rates_thrust->resize(4);
    *controller_rates_thrust << rates, thrust;
}
