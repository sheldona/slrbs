// src/ai/AIModel.cpp
#include "ai/AIModel.h"
#include <iostream>
#include <fstream>
#include <filesystem>

AIModel::AIModel(const std::string& model_path, ModelType type)
    : model_path_(model_path) {

    // Auto-detect model type if not specified
    if (type == ModelType::NONE) {
        model_type_ = detectModelType(model_path);
    } else {
        model_type_ = type;
    }

    // Load appropriate model type
    switch (model_type_) {
#ifdef USE_TORCH
        case ModelType::TORCH:
            try {
                // Load the model
                torch_module_ = torch::jit::load(model_path);
                torch_module_.eval();
                is_loaded_ = true;
                std::cout << "LibTorch model loaded successfully from: " << model_path << std::endl;
            } catch (const c10::Error& e) {
                std::cerr << "Error loading the LibTorch model: " << e.what() << std::endl;
                is_loaded_ = false;
            }
            break;
#endif

#ifdef USE_TCNN
        case ModelType::TCNN:
            is_loaded_ = loadTCNNConfig(model_path);
            break;
#endif

        default:
            std::cerr << "Warning: No compatible AI model backend available for: " << model_path << std::endl;
#if !defined(USE_TORCH) && !defined(USE_TCNN)
            std::cerr << "Enable USE_TORCH or USE_TCNN in CMake configuration to use AI models." << std::endl;
#endif
            is_loaded_ = false;
            break;
    }
}

AIModel::~AIModel() {
#ifdef USE_TCNN
    // Explicitly destroy TCNN model
    if (tcnn_model_) {
        tcnn_model_->network.reset();
        tcnn_model_->trainer.reset();
        tcnn_model_->network_with_encoding.reset();
        tcnn_model_.reset();
    }
#endif
}

AIModel::ModelType AIModel::detectModelType(const std::string& path) {
    std::filesystem::path filepath(path);
    std::string extension = filepath.extension().string();

    if (extension == ".pt" || extension == ".pth") {
#ifdef USE_TORCH
        return ModelType::TORCH;
#else
        std::cerr << "LibTorch model detected but USE_TORCH is not enabled." << std::endl;
        return ModelType::NONE;
#endif
    } else if (extension == ".json") {
#ifdef USE_TCNN
        return ModelType::TCNN;
#else
        std::cerr << "tiny-cuda-nn model detected but USE_TCNN is not enabled." << std::endl;
        return ModelType::NONE;
#endif
    }

    return ModelType::NONE;
}

bool AIModel::isLoaded() const {
    return is_loaded_;
}

AIModel::ModelType AIModel::getModelType() const {
    return model_type_;
}

#ifdef USE_TORCH
torch::Tensor AIModel::predict(const torch::Tensor& input) {
    if (!is_loaded_ || model_type_ != ModelType::TORCH) {
        throw std::runtime_error("Cannot predict: LibTorch model not loaded. Check logs for errors.");
    }

    try {
        // Prepare input
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input);

        // Forward pass
        torch::Tensor output = torch_module_.forward(inputs).toTensor();
        return output;
    } catch (const c10::Error& e) {
        std::cerr << "Error during prediction: " << e.what() << std::endl;
        throw;
    }
}

std::vector<torch::Tensor> AIModel::predictMultiOutput(const torch::Tensor& input) {
    if (!is_loaded_ || model_type_ != ModelType::TORCH) {
        throw std::runtime_error("Cannot predict: LibTorch model not loaded. Check logs for errors.");
    }

    try {
        // Prepare input
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input);

        // Forward pass - get tuple output
        auto output_tuple = torch_module_.forward(inputs).toTuple();

        // Convert to vector of tensors
        std::vector<torch::Tensor> results;
        for (const auto& element : output_tuple->elements()) {
            results.push_back(element.toTensor());
        }

        return results;
    } catch (const c10::Error& e) {
        std::cerr << "Error during multi-output prediction: " << e.what() << std::endl;
        throw;
    }
}

torch::Tensor AIModel::predictMultiInput(const std::vector<torch::Tensor>& inputs) {
    if (!is_loaded_ || model_type_ != ModelType::TORCH) {
        throw std::runtime_error("Cannot predict: LibTorch model not loaded. Check logs for errors.");
    }

    try {
        // Prepare multiple inputs
        std::vector<torch::jit::IValue> model_inputs;
        for (const auto& input : inputs) {
            model_inputs.push_back(input);
        }

        // Forward pass
        torch::Tensor output = torch_module_.forward(model_inputs).toTensor();
        return output;
    } catch (const c10::Error& e) {
        std::cerr << "Error during multi-input prediction: " << e.what() << std::endl;
        throw;
    }
}
#endif

#ifdef USE_TCNN
bool AIModel::loadTCNNConfig(const std::string& config_path) {
    try {
        // Load JSON config
        std::ifstream file(config_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open config file: " << config_path << std::endl;
            return false;
        }

        nlohmann::json config;
        file >> config;
        file.close();

        // Check required fields
        if (!config.contains("n_input_dims") || !config.contains("n_output_dims")) {
            std::cerr << "Config file missing required fields: n_input_dims, n_output_dims" << std::endl;
            return false;
        }

        uint32_t n_input_dims = config["n_input_dims"];
        uint32_t n_output_dims = config["n_output_dims"];

        // Create model from config
        createTCNN(n_input_dims, n_output_dims, config);

        std::cout << "tiny-cuda-nn model loaded successfully from: " << config_path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading the tiny-cuda-nn model: " << e.what() << std::endl;
        return false;
    }
}

bool AIModel::saveTCNNConfig(const std::string& config_path) {
    if (!tcnn_model_ || !is_loaded_) {
        std::cerr << "No tiny-cuda-nn model loaded to save" << std::endl;
        return false;
    }

    try {
        // Add dimensions to config
        nlohmann::json config = tcnn_model_->config;
        config["n_input_dims"] = tcnn_model_->n_input_dims;
        config["n_output_dims"] = tcnn_model_->n_output_dims;

        // Save to file
        std::ofstream file(config_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open file for writing: " << config_path << std::endl;
            return false;
        }

        file << config.dump(4); // Pretty-print with 4-space indentation
        file.close();

        std::cout << "tiny-cuda-nn model config saved to: " << config_path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error saving the tiny-cuda-nn model config: " << e.what() << std::endl;
        return false;
    }
}

void AIModel::createTCNN(uint32_t n_input_dims, uint32_t n_output_dims, const nlohmann::json& config) {
    try {
        // Create our model container
        tcnn_model_ = std::make_unique<TCNNModel>();
        tcnn_model_->n_input_dims = n_input_dims;
        tcnn_model_->n_output_dims = n_output_dims;
        tcnn_model_->config = config;

        // Check if we have separate encoding and network sections
        if (config.contains("encoding") && config.contains("network")) {
            // Create a combined network with encoding
            tcnn_model_->network_with_encoding = std::unique_ptr<tcnn::NetworkWithInputEncoding<float>>(
                tcnn::create_network_with_input_encoding<float>(
                    n_input_dims, n_output_dims,
                    config.at("encoding"), config.at("network")
                )
            );
        } else {
            // Just create a regular network
            tcnn_model_->network = std::unique_ptr<tcnn::Network<float>>(
                tcnn::create_network<float>(n_input_dims, n_output_dims, config.at("network"))
            );
        }

        // Create trainer if loss and optimizer are specified
        if (config.contains("loss") && config.contains("optimizer")) {
            tcnn_model_->trainer = std::unique_ptr<tcnn::Trainer<float, float>>(
                tcnn::create_trainer<float, float>(
                    tcnn_model_->network_with_encoding ?
                        tcnn_model_->network_with_encoding.get() :
                        tcnn_model_->network.get(),
                    config.at("optimizer"), config.at("loss")
                )
            );
        }

        is_loaded_ = true;
    } catch (const std::exception& e) {
        std::cerr << "Error creating tiny-cuda-nn model: " << e.what() << std::endl;
        tcnn_model_.reset();
        is_loaded_ = false;
    }
}

std::vector<float> AIModel::predictTCNN(const std::vector<float>& inputs, uint32_t batch_size) {
    if (!is_loaded_ || !tcnn_model_ || model_type_ != ModelType::TCNN) {
        throw std::runtime_error("Cannot predict: tiny-cuda-nn model not loaded. Check logs for errors.");
    }

    try {
        // Ensure inputs are correctly sized
        const uint32_t n_elements = batch_size * tcnn_model_->n_input_dims;
        if (inputs.size() != n_elements) {
            throw std::runtime_error("Input size mismatch: expected " +
                                    std::to_string(n_elements) + " elements, got " +
                                    std::to_string(inputs.size()));
        }

        // Create GPU matrices for input and output
        tcnn::GPUMatrix<float> input_matrix(tcnn_model_->n_input_dims, batch_size);
        tcnn::GPUMatrix<float> output_matrix(tcnn_model_->n_output_dims, batch_size);

        // Copy inputs to GPU
        CUDA_CHECK_THROW(cudaMemcpy(
            input_matrix.data(),
            inputs.data(),
            n_elements * sizeof(float),
            cudaMemcpyHostToDevice
        ));

        // Run inference
        if (tcnn_model_->network_with_encoding) {
            tcnn_model_->network_with_encoding->inference(input_matrix, output_matrix);
        } else {
            tcnn_model_->network->inference(input_matrix, output_matrix);
        }

        // Copy results back to CPU
        std::vector<float> outputs(batch_size * tcnn_model_->n_output_dims);
        CUDA_CHECK_THROW(cudaMemcpy(
            outputs.data(),
            output_matrix.data(),
            outputs.size() * sizeof(float),
            cudaMemcpyDeviceToHost
        ));

        return outputs;
    } catch (const std::exception& e) {
        std::cerr << "Error during tiny-cuda-nn prediction: " << e.what() << std::endl;
        throw;
    }
}

float AIModel::trainTCNN(const std::vector<float>& inputs, const std::vector<float>& targets, uint32_t batch_size) {
    if (!is_loaded_ || !tcnn_model_ || !tcnn_model_->trainer || model_type_ != ModelType::TCNN) {
        throw std::runtime_error("Cannot train: tiny-cuda-nn trainer not available. Check logs for errors.");
    }

    try {
        // Ensure inputs and targets are correctly sized
        const uint32_t input_elements = batch_size * tcnn_model_->n_input_dims;
        const uint32_t output_elements = batch_size * tcnn_model_->n_output_dims;

        if (inputs.size() != input_elements) {
            throw std::runtime_error("Input size mismatch: expected " +
                                    std::to_string(input_elements) + " elements, got " +
                                    std::to_string(inputs.size()));
        }

        if (targets.size() != output_elements) {
            throw std::runtime_error("Target size mismatch: expected " +
                                    std::to_string(output_elements) + " elements, got " +
                                    std::to_string(targets.size()));
        }

        // Create GPU matrices for input and target
        tcnn::GPUMatrix<float> input_matrix(tcnn_model_->n_input_dims, batch_size);
        tcnn::GPUMatrix<float> target_matrix(tcnn_model_->n_output_dims, batch_size);

        // Copy inputs and targets to GPU
        CUDA_CHECK_THROW(cudaMemcpy(
            input_matrix.data(),
            inputs.data(),
            input_elements * sizeof(float),
            cudaMemcpyHostToDevice
        ));

        CUDA_CHECK_THROW(cudaMemcpy(
            target_matrix.data(),
            targets.data(),
            output_elements * sizeof(float),
            cudaMemcpyHostToDevice
        ));

        // Train
        float loss = 0.0f;
        tcnn_model_->trainer->training_step(input_matrix, target_matrix, &loss);

        return loss;
    } catch (const std::exception& e) {
        std::cerr << "Error during tiny-cuda-nn training: " << e.what() << std::endl;
        throw;
    }
}
#endif