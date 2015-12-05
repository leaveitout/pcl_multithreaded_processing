//
// Created by sean on 20/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_PROCESSORPRODUCER_HPP
#define PCL_MULTI_THREADED_PROCESSING_PROCESSORPRODUCER_HPP

#include <memory>
#include <thread>

#include "Timer.hpp"
#include "Buffer.hpp"
#include "Processor.hpp"

template <typename InputType, typename OutputType>
class ProcessorProducer : public Processor<InputType> {
public:
    ProcessorProducer(const std::shared_ptr<Buffer<InputType>> input_buffer_,
                      size_t id,
                      std::string description,
                      size_t output_buffer_size = 100) :
            Processor<InputType>(input_buffer_, id, description) {
        output_buffer_ = std::make_shared<Buffer<OutputType>>(output_buffer_size);
    }

    std::shared_ptr<Buffer<OutputType>> getOutputBuffer() const {
        return output_buffer_;
    };

protected:
    std::shared_ptr<Buffer<OutputType>> output_buffer_;
};

#endif //PCL_MULTI_THREADED_PROCESSING_PROCESSORPRODUCER_HPP
