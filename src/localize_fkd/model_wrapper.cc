#include <iostream>
#include <vector>
#include "model_wrapper.h"

namespace model{
    using namespace std;

    ModelWrapper::ModelWrapper(string model_path){
        try{
            module = torch::jit::load(model_path);
        }
        catch (const c10::Error &e) {
            cerr << "error loading the model\n";
        }
    }

    void ModelWrapper::evaluate(float *input, float *output){
        // Prepare input
        vector<torch::jit::IValue> inputs;
        torch::Tensor input_tensor = torch::from_blob(input, {1, input_dim}, torch::kFloat32);
        inputs.push_back(input_tensor);

        // Compute output
        torch::Tensor output_tensor = module.forward(inputs).toTensor();
        for(unsigned i = 0; i<output_dim; i++){
            output[i] = *output_tensor[0][i].data_ptr<float>();
        }
    }
}