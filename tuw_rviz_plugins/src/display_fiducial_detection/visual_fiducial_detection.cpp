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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "display_fiducial_detection/visual_fiducial_detection.h"

namespace tuw_rviz_plugins {

VisualFiducialDetection::VisualFiducialDetection ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) {
    scene_manager_ = scene_manager;

    // Ogre::SceneNode s form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    //
    // Here we create a node to store the pose of the FiducialDetection's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();

    // initialize global variables
    color_ = Ogre::ColourValue ( 255, 0, 0 );
    shape_type_ = rviz::Shape::Cube;
    scale_ = 0.0;
}

VisualFiducialDetection::~VisualFiducialDetection() {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode ( frame_node_ );
}

void VisualFiducialDetection::setMessage ( const sensor_msgs::FiducialDetection::ConstPtr& msg ) {
    fiducials_.resize ( msg->fiducials.size() );
    for ( size_t i = 0; i < fiducials_.size(); i++ ) {
        double p_x = msg->fiducials[i].pose.position.x;
        double p_y = msg->fiducials[i].pose.position.y;
        double p_z = msg->fiducials[i].pose.position.z;
        double o_x = msg->fiducials[i].pose.orientation.x;
        double o_y = msg->fiducials[i].pose.orientation.y;
        double o_z = msg->fiducials[i].pose.orientation.z;
        double o_w = msg->fiducials[i].pose.orientation.w;

        fiducials_[i].reset ( new rviz::Shape ( shape_type_, scene_manager_, frame_node_ ) );
        fiducials_[i]->setColor ( color_ );
        fiducials_[i]->setPosition ( Ogre::Vector3 ( p_x, p_y, p_z ) );
        fiducials_[i]->setOrientation ( Ogre::Quaternion ( o_w, o_x, o_y, o_z ) );
        fiducials_[i]->setScale ( Ogre::Vector3 ( scale_, scale_, scale_ ) );
    }
}

// Position is passed through to the SceneNode.
void VisualFiducialDetection::setFramePosition ( const Ogre::Vector3& position ) {
    frame_node_->setPosition ( position );
}

// Orientation is passed through to the SceneNode.
void VisualFiducialDetection::setFrameOrientation ( const Ogre::Quaternion& orientation ) {
    frame_node_->setOrientation ( orientation );
}

// Color is passed through to the Shape object.
void VisualFiducialDetection::setColor ( Ogre::ColourValue color ) {
    for ( size_t i = 0; i < fiducials_.size(); i++ ) {
        fiducials_[i]->setColor ( color );
    }
    color_ = color;
}

// Shape type is passed through to the Shape object.
void VisualFiducialDetection::setShape ( rviz::Shape::Type shape_type ) {
    for ( size_t i = 0; i < fiducials_.size(); i++ ) {
        Ogre::Vector3 position = fiducials_[i]->getPosition();
        Ogre::Quaternion orientation = fiducials_[i]->getOrientation();

        fiducials_[i].reset ( new rviz::Shape ( shape_type, scene_manager_, frame_node_ ) );
        fiducials_[i]->setColor ( color_ );
        fiducials_[i]->setPosition ( position );
        fiducials_[i]->setOrientation ( orientation );
        fiducials_[i]->setScale ( Ogre::Vector3 ( scale_, scale_, scale_ ) );
    }

    shape_type_ = shape_type;
}

// Scale is passed through to the Shape object.
void VisualFiducialDetection::setScale ( float scale ) {
    for ( size_t i = 0; i < fiducials_.size(); i++ ) {
        fiducials_[i]->setScale ( Ogre::Vector3 ( scale_, scale_, scale_ ) );
    }
    scale_ = scale;
}

} // end namespace tuw_rviz_plugins

