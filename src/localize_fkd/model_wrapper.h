#pragma once

#include <torch/script.h>
#include <string>

namespace model{
    class ModelWrapper{
    private:
        torch::jit::script::Module module;
        unsigned input_dim = 90;
        unsigned output_dim = 70; // tries to predict vx, vy, w, x, y, sintheta, costheta
    public:
        ModelWrapper(std::string model_path);
        void evaluate(float *input, float *output);
    };
}