//
// Created by sean on 19/11/15.
//

#include "NI2ImageViewer.hpp"
#include "Logger.hpp"

NI2ImageViewer::NI2ImageViewer(const std::shared_ptr<Buffer<NI2ImagePtr>> buffer, size_t id, std::string description)
        : Processor<NI2ImagePtr>(buffer, id, description) {}

void NI2ImageViewer::initializeThreadResources() {
    std::stringstream ss;
    ss << "OpenNI2 Image Viewer : " << this->getDescription();
    image_viewer_.reset(new pcl::visualization::ImageViewer(ss.str()));
    image_viewer_->registerMouseCallback(&NI2ImageViewer::mouseCallback, *this);
    image_viewer_->registerKeyboardCallback(&NI2ImageViewer::keyboardCallback, *this);
    // TODO: Add to constructor
    image_viewer_->setPosition(0, 0);
    image_viewer_->setSize(640, 480);
}

void NI2ImageViewer::processBufferElement(NI2ImagePtr image) {
    if(image->getEncoding() == pcl::io::openni2::Image::RGB)
        image_viewer_->addRGBImage((const unsigned char*) image->getData(),
                                   image->getWidth(),
                                   image->getHeight());
    image_viewer_->spinOnce();
}

void NI2ImageViewer::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *) {
    std::string special_key = event.getKeyCode() ? "" : "special ";
    if (event.keyDown())
        Logger::log(Logger::INFO, "The %skey %c was pressed.", special_key.c_str(), event.getKeyCode());
    else
        Logger::log(Logger::INFO, "The %skey %c was released.", special_key.c_str(), event.getKeyCode());
}

void NI2ImageViewer::mouseCallback(const pcl::visualization::MouseEvent &mouse_event, void *) {
    if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
        mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
        Logger::log(Logger::INFO, "Left button pressed @ (%i, %i).\n", mouse_event.getX(), mouse_event.getY());
    }
}

