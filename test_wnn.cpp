#include "wnn_payload_modulator.hpp"
#include <iostream>

int main() {
    hlv_wnn::HLVWNNBridge bridge;
    hlv_wnn::WNNPayloadModulator modulator(bridge);
    hlv_wnn::WNNTransductionEngine::DuffingParams params;
    hlv_wnn::WNNTransductionEngine engine(modulator, params);

    engine.step(0.01L);
    std::cout << "Engine time: " << engine.get_time() << std::endl;
    return 0;
}
