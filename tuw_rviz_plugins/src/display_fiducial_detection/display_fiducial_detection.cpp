/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "display_fiducial_detection/visual_fiducial_detection.h"
#include "display_fiducial_detection/display_fiducial_detection.h"

namespace tuw_rviz_plugins {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
DisplayFiducialDetection::DisplayFiducialDetection() {
    color_property_ = new rviz::ColorProperty ( "Color", QColor ( 204, 51, 204 ),
            "Color to draw the fiducials.",
            this, SLOT ( updateColor() ) );

    shape_property_ = new rviz::EnumProperty ( "Shape", QString::fromStdString ( "Cube" ),
            "Shape of the fiducials.",
            this, SLOT ( updateShape() ) );
    shape_property_->addOptionStd ( "Cube", rviz::Shape::Cube );
    shape_property_->addOptionStd ( "Cylinder", rviz::Shape::Cylinder );
    shape_property_->addOptionStd ( "Sphere", rviz::Shape::Sphere );

    scale_property_ = new rviz::FloatProperty ( "Scale", 0.3,
            "Scale of the fiducials.",
            this, SLOT ( updateScale() ) );
    scale_property_->setMin ( 0 );
    scale_property_->setMax ( 1 );

    history_length_property_ = new rviz::IntProperty ( "History Length", 1,
            "Number of prior measurements to display.",
            this, SLOT ( updateHistoryLength() ) );
    history_length_property_->setMin ( 1 );
    history_length_property_->setMax ( 100000 );
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void DisplayFiducialDetection::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
}

DisplayFiducialDetection::~DisplayFiducialDetection() {
}

// Clear the visuals by deleting their objects.
void DisplayFiducialDetection::reset() {
    MFDClass::reset();
    visuals_.clear();
}

// Set the current color values for each visual.
void DisplayFiducialDetection::updateColor() {
    Ogre::ColourValue color = color_property_->getOgreColor();

    for ( size_t i = 0; i < visuals_.size(); i++ ) {
        visuals_[ i ]->setColor ( color );
    }
}

// Set the current shape for each visual.
void DisplayFiducialDetection::updateShape() {
    rviz::Shape::Type shape_type = ( rviz::Shape::Type ) shape_property_->getOptionInt();

    for ( size_t i = 0; i < visuals_.size(); i++ ) {
        visuals_[ i ]->setShape ( shape_type );
    }
}

// Set the current scale for each visual.
void DisplayFiducialDetection::updateScale() {
    float scale = scale_property_->getFloat();

    for ( size_t i = 0; i < visuals_.size(); i++ ) {
        visuals_[ i ]->setScale ( scale );
    }
}

// Set the number of past visuals to show.
void DisplayFiducialDetection::updateHistoryLength() {
    visuals_.rset_capacity ( history_length_property_->getInt() );
}

// This is our callback to handle an incoming message.
void DisplayFiducialDetection::processMessage ( const sensor_msgs::FiducialDetection::ConstPtr& msg ) {
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if ( !context_->getFrameManager()->getTransform ( msg->header.frame_id,
            msg->header.stamp,
            position, orientation ) ) {
        ROS_DEBUG ( "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), qPrintable ( fixed_frame_ ) );
        return;
    }

    // We are keeping a circular buffer of visual pointers.  This gets
    // the next one, or creates and stores it if the buffer is not full
    boost::shared_ptr<VisualFiducialDetection> visual;
    if ( visuals_.full() ) {
        visual = visuals_.front();
    } else {
        visual.reset ( new VisualFiducialDetection ( context_->getSceneManager(), scene_node_ ) );
    }

    // Now set or update the contents of the chosen visual.
    visual->setMessage ( msg );
    visual->setFramePosition ( position );
    visual->setFrameOrientation ( orientation );
    visual->setColor ( color_property_->getOgreColor() );
    visual->setShape ( ( rviz::Shape::Type ) shape_property_->getOptionInt() );
    visual->setScale ( scale_property_->getFloat() );

    // And send it to the end of the circular buffer
    visuals_.push_back ( visual );
}

} // end namespace tuw_rviz_plugins

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS ( tuw_rviz_plugins::DisplayFiducialDetection,rviz::Display )
// END_TUTORIAL
