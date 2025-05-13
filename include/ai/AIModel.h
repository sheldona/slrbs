// include/ai/AIModel.h
#pragma once
#include <string>
#include <vector>
#include <memory>

// Only include torch headers if LibTorch is enabled
#ifdef USE_TORCH
#include <torch/script.h>
#endif

// Only include tiny-cuda-nn headers if enabled
#ifdef USE_TCNN
#include <tiny-cuda-nn/common.h>
#include <tiny-cuda-nn/gpu_matrix.h>
#include <tiny-cuda-nn/network.h>
#include <tiny-cuda-nn/encoding.h>
#include <tiny-cuda-nn/trainer.h>
#endif

#include <nlohmann/json.hpp>

/**
 * AIModel - Class for loading and using AI models in the simulation
 * Conditionally compiled based on LibTorch and tiny-cuda-nn availability
 */
class AIModel {
public:
    enum class ModelType {
        NONE,
        TORCH,
        TCNN
    };

    /**
     * Constructor loads a model from the given path
     * @param model_path Path to the model file (.pt for LibTorch, .json for tiny-cuda-nn)
     * @param type Type of model to load (auto-detected by file extension if AUTO)
     */
    AIModel(const std::string& model_path, ModelType type = ModelType::NONE);

    /**
     * Destructor
     */
    ~AIModel();

#ifdef USE_TORCH
    /**
     * Predict with a single tensor input (LibTorch)
     * @param input Input tensor
     * @return Output tensor from the model
     */
    torch::Tensor predict(const torch::Tensor& input);

    /**
     * Predict with multiple output tensors (for models that return tuples)
     * @param input Input tensor
     * @return Vector of output tensors
     */
    std::vector<torch::Tensor> predictMultiOutput(const torch::Tensor& input);

    /**
     * Predict with multiple input tensors
     * @param inputs Vector of input tensors
     * @return Output tensor
     */
    torch::Tensor predictMultiInput(const std::vector<torch::Tensor>& inputs);
#endif

#ifdef USE_TCNN
    /**
     * Predict using tiny-cuda-nn
     * @param inputs Input values as vector of floats
     * @param batch_size Number of samples in the batch
     * @return Output values as vector of floats
     */
    std::vector<float> predictTCNN(const std::vector<float>& inputs, uint32_t batch_size = 1);

    /**
     * Train the tiny-cuda-nn model (if model supports training)
     * @param inputs Input values as vector of floats
     * @param targets Target values as vector of floats
     * @param batch_size Number of samples in the batch
     * @return Loss value
     */
    float trainTCNN(const std::vector<float>& inputs, const std::vector<float>& targets, uint32_t batch_size = 1);

    /**
     * Create a new tiny-cuda-nn model from config
     * @param n_input_dims Number of input dimensions
     * @param n_output_dims Number of output dimensions
     * @param config JSON configuration
     */
    void createTCNN(uint32_t n_input_dims, uint32_t n_output_dims, const nlohmann::json& config);

    /**
     * Load tiny-cuda-nn model from a JSON file
     * @param config_path Path to the JSON config file
     * @return true if successful, false otherwise
     */
    bool loadTCNNConfig(const std::string& config_path);

    /**
     * Save the current model configuration to a file (tiny-cuda-nn)
     * @param config_path Path to save the configuration
     * @return true if successful, false otherwise
     */
    bool saveTCNNConfig(const std::string& config_path);
#endif

    /**
     * Check if the model is loaded successfully
     * @return true if model is loaded, false otherwise
     */
    bool isLoaded() const;

    /**
     * Get the type of model that is loaded
     * @return Model type (NONE, TORCH, or TCNN)
     */
    ModelType getModelType() const;

private:
    // Detect model type from file extension
    ModelType detectModelType(const std::string& path);

#ifdef USE_TORCH
    torch::jit::script::Module torch_module_;
#endif

#ifdef USE_TCNN
    struct TCNNModel {
        std::unique_ptr<tcnn::Network<float>> network;
        std::unique_ptr<tcnn::Trainer<float, float>> trainer;
        std::unique_ptr<tcnn::NetworkWithInputEncoding<float>> network_with_encoding;
        uint32_t n_input_dims = 0;
        uint32_t n_output_dims = 0;
        nlohmann::json config;
    };

    std::unique_ptr<TCNNModel> tcnn_model_;
#endif

    std::string model_path_;
    ModelType model_type_ = ModelType::NONE;
    bool is_loaded_ = false;
};