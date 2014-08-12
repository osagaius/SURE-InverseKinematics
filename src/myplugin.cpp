/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Sungmoon Joo <sungmoon.joo@gmail.com>
 * Date: Feb 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Humanoid Robotics Lab nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */
 
#include "myplugin.h"
#include <iostream>
#include <qplugin.h>
#include <QtGui>
#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/BodyNode.h>
#include <math.h>
#include <Eigen/Dense>

#define DEG2RAD(x) x*3.1415/180

MyPlugin::MyPlugin(QWidget *parent) : ui(new Ui::MyPluginTab){
    ui->setupUi(this);


    // Button connections
    connect( ui->pushButtonAdd, SIGNAL(released()),
             this, SLOT(addValue()) );
    connect( ui->pushButtonReduce, SIGNAL(released()),
             this, SLOT(reduceValue()) );
    connect( ui->pushButtonSetStartConfiguration, SIGNAL(released()),
             this, SLOT(setKrangStartConfig()) );
    connect( ui->seeRobotInfo, SIGNAL(released()),
             this, SLOT(seeRobotInfo()) );
    connect( ui->pushButtonMoveLeftArm, SIGNAL(released()),
	     this, SLOT(moveLeftArm()) );
}

MyPlugin::~MyPlugin(){}

void MyPlugin::GRIPEventSimulationBeforeTimestep()
{
}

void MyPlugin::GRIPEventSimulationAfterTimestep(){}
void MyPlugin::GRIPEventSimulationStart(){}
void MyPlugin::GRIPEventSimulationStop(){}

/**
 * @function GRIPEventSceneLoaded
 * @brief Run right after an scene has been loaded
 */
void MyPlugin::GRIPEventSceneLoaded() {

    // Get GolemHubo skeleton
    dart::dynamics::Skeleton* skel = _world->getSkeleton("tetrapak");

    if (skel) {
        // Get index of LSP (left shoulder pitch
        std::vector<int> index(1);
        index[0] = skel->getJoint("LSP")->getGenCoord(0)->getSkeletonIndex();

        // Initialize joint value for LSP
        Eigen::VectorXd jointValue(1);

        // Move joint around
        for (size_t i = 0; i < 200; ++i) {
            // Set joint value
            jointValue[0] = float(i) * 2 * M_PI / 200;
            skel->setConfig(index, jointValue);

            // Save world to timeline
            _timeline->push_back(GripTimeslice(*_world));
        }
    } else {
        std::cerr << "No skeleton named lwa4" << std::endl;
    }


}

void MyPlugin::GRIPEventTreeViewSelectionChanged()
{
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");    
    if(!_activeNode) {
        std::cerr << "[MyPlugin] No item selected in TreeView" << std::endl;
        return;
    }

    std::cerr << "[MyPlugin] ActiveNodeType: " << _activeNode->dType << std::endl;
    if(Return_Type_Robot == _activeNode->dType) {
        dart::dynamics::Skeleton* robot = (dart::dynamics::Skeleton*)_activeNode->object;
        std::cerr << "[MyPlugin] Skeleton Selected: " << robot->getName() << std::endl;
    } else if(Return_Type_Node == _activeNode->dType) {
        dart::dynamics::BodyNode* node = (dart::dynamics::BodyNode*)_activeNode->object;
        std::cerr << "[MyPlugin] BodyNode Selected: " << node->getName() << std::endl;
    } else {
        std::cerr << "[MyPlugin] Unknown type selected in TreeView" << std::endl;
    }
}

void MyPlugin::GRIPEventPlaybackBeforeFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
void MyPlugin::GRIPEventPlaybackAfterFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
void MyPlugin::GRIPEventPlaybackStart() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
void MyPlugin::GRIPEventPlaybackStop() {}

void MyPlugin::Refresh() {}

void MyPlugin::addValue() {
    Eigen::VectorXd newValues(1);
    std::vector<int> indices(1);
    indices[0] = 2;
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd altValues = robot->getConfig(indices);
    newValues(0) = altValues(0) +  0.1;
    robot->setConfig(indices, newValues);
}

void MyPlugin::reduceValue() {
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd newValues(1);
    std::vector<int> indices(1);
    indices[0] = 13;
    Eigen::VectorXd altValues = robot->getConfig(indices);
    newValues(0) = altValues(0) -  0.1;
    robot->setConfig(indices, newValues);
}

void MyPlugin::setKrangStartConfig() {
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd cfg = robot->getConfig();
    cfg << 1.57665,  -1.58573, -0.905034,  0.165138,   0.23035,  0.134083,        0 ,        0,  0.736354,         0,         0,         0 ,        0, -0.824843,         0, -0.761487,        0,  -1.14214,         0,         0 ,        0     ,    0 , 0.697957  ,       0  , 1.01526   ,      0 ,  1.14214 ,        0     ,    0    ,     0;
    robot->setConfig(cfg);
}

/**
void MyPlugin::moveLeftArm() {
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd newValues(1);
    std::vector<int> indices(1);
    indices[0] = 13;
    Eigen::VectorXd altValues = robot->getConfig(indices);
    newValues(0) = altValues(0) -  0.1;
    robot->setConfig(indices, newValues);
}
**/

void MyPlugin::moveLeftArm() {
    //Initialize vectors
	Eigen::Vector6d xdot(6);
    xdot[0] = 1.0; // ask for +x velocity
	Eigen::VectorXd qdot;
    Eigen::VectorXd _nullspace;
    std::vector<int> left_arm_indices(7);
    Eigen::VectorXd l_arm_cfg(qdot.size());



    //Get qdot - velocity in joint, given x velocity in workspace
	refJSVelocity(xdot, _nullspace, qdot);

    //Print out xdot and qdot
    std::cout << "xdot: " << xdot.transpose() << std::endl;
    std::cout << "qdot: " << qdot.transpose() << std::endl;

    //Robot confiiguration
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd cfg = robot->getConfig();
    std::cout << "cfg size: " << cfg.size() << std::endl;
    std::cout << "qdot size: " << qdot.size() << std::endl;
    std::cout << "old cfg: " << cfg.transpose() << std::endl;

    // Initialize parameters
    double inc_by = 0.05;
    int l_shoulder_idx = 10;

    //Set left arm indices
    for(int i = 0; i < left_arm_indices.size(); i++)
        left_arm_indices[i] = l_shoulder_idx + i;

    //Set left arm configuration
    for(int i = 0; i < left_arm_indices.size(); i++) 
        //cfg[left_arm_indices[i]] += (inc_by * qdot[i]);
        l_arm_cfg[i] = cfg[left_arm_indices[i]] + (inc_by * qdot[i]);

    //Print out left arm indices and configuration info
    std::cout << "left arm indices: ";
    for (int i=0; i<left_arm_indices.size(); i++) 
        std::cout <<  " " << left_arm_indices[i] << std::endl;
    std::cout << "new cfg: " << cfg.transpose() << std::endl;
    std::cout << "indices len: " << left_arm_indices.size() << std::endl;
    // std::cout<< "cfg slice len: " << cfg.segment(l_shoulder_idx, l_shoulder_idx+6).size() << std::endl;
    // std::cout<< "cfg slice len: " << cfg.segment(10, 20).size() << std::endl;

    //robot->setConfig(left_arm_indices, cfg.segment(l_shoulder_idx, l_shoulder_idx+6));

    //Set new configuration
    robot->setConfig(left_arm_indices, l_arm_cfg);
}


const Eigen::VectorXd& MyPlugin::refJSVelocity(const Eigen::VectorXd& xdot,
	                                     const Eigen::VectorXd& qdot_nullspace, Eigen::VectorXd& qdot) {


        dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
		dart::dynamics::BodyNode* endEffector = robot->getBodyNode("lGripper");

		// Get the Jacobian for the end-effector
		Eigen::MatrixXd Jfull = endEffector->getBodyJacobian();
        Eigen::MatrixXd J = Jfull.topRightCorner<6,7>();
        // Eigen::MatrixXd J = Jfull.topLeftCorner<6,7>(); //???


        // debugging
        std::cout << "DOF from effector: \n";
        dart::dynamics::BodyNode* parent = endEffector;
        while (parent != NULL) {
            std::cout << "Node name: " << parent->getName() << ", idx: " << parent->getSkeletonIndex() << std::endl;
            parent = parent->getParentBodyNode();
        }

        std::cout << "Full skeleton names: \n";
        for (int i=0; i < robot->getNumBodyNodes(); i++) {
            std::cout << "idx: " << robot->getBodyNode(i)->getSkeletonIndex() << ", name: " << robot->getBodyNode(i)->getName() << std::endl;
        }

		// Transposing the Jacobian
		Eigen::MatrixXd Jt = J.transpose();
		Eigen::MatrixXd JJt = J * Jt;
        // std::cout << "j dimensions: " << J.rows() << "x" << J.cols() << std::endl;
        // std::cout << "jt dimensions: " << Jt.rows() << "x" << Jt.cols() << std::endl;
        // std::cout << "jjt dimensions: " << JJt.rows() << "x" << JJt.cols() << std::endl;
        // std::cout << "jjt coefficients: " << JJt.size() << std::endl;

		// // inverting the jacobian
		Eigen::MatrixXd Jinv = Jt * JJt.inverse();
        // std::cout << "right pseudo-inverse: " << Jinv << std::endl;
        qdot = Jinv * xdot;
        std::cout << "final qdot from rpi: " << qdot.transpose() << std::endl;

        // Eigen version:
        Eigen::VectorXd qdot2 = J.colPivHouseholderQr().solve(xdot);
        std::cout << "final qdot from eigen QR: " << qdot2.transpose() << std::endl;
        std::cout << "err: " << (qdot - qdot2).transpose() << std:: endl;

		return qdot;
}



void MyPlugin::seeRobotInfo() {
    dart::dynamics::Skeleton* skel = _world->getSkeleton("Krang");
    dart::dynamics::Skeleton* table = _world->getSkeleton("glassDiningTable");
    dart::dynamics::BodyNode* node = skel->getBodyNode("L2");
    if (skel != NULL) {
	std::cout<<"Robot name: "<<skel->getName()<<std::endl;
        Eigen::VectorXd robotConfig = skel->getConfig();
        std::cout<<"Robot configuration: "<<robotConfig.matrix().transpose()<<std::endl;
	std::cout<<"Left arm position: "<<node->getWorldJacobian().matrix().transpose()<<std::endl;
        
        std::cout<<"printing left joint info: "<<skel->getJoint("LJ2")->getGenCoord(0)->get_q()<<std::endl;
        
        Eigen::VectorXd tableConfig = table->getConfig();
        std::cout<<"printing table configuration: "<<tableConfig.transpose()<<std::endl;
       
	Eigen::Isometry3d tableWorld = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d grasppositionTable = Eigen::Isometry3d::Identity();

        tableWorld = table->getBodyNode(0)->getWorldTransform();
	grasppositionTable.translation()<<0.21, 0, 0.32;

	Eigen::Isometry3d grasppositionWorld;
 	grasppositionWorld = tableWorld * grasppositionTable;
        
             
    std::cout<<"printing table info: \n"<<grasppositionWorld.matrix()<<std::endl; 
        
    Eigen::Isometry3d gripWorld = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d gripperGrip = Eigen::Isometry3d::Identity();
     
        gripWorld = skel->getBodyNode("lGripper")->getWorldTransform();
	gripperGrip.translation()<<0, 0, 0.16;

	Eigen::Isometry3d gripperWorld;
	gripperWorld  = gripWorld * gripperGrip;

        std::cout<<"printing grip info :  \n"<<gripperWorld.matrix()<<std::endl;
        
        Eigen::Vector3d distance;
	distance = gripperWorld.translation() - grasppositionWorld.translation();

	    
        std::cout<<"printing distance info :  \n"<<distance.norm()<<std::endl;     
 }

}


Q_EXPORT_PLUGIN2(MyPlugin, MyPlugin)
