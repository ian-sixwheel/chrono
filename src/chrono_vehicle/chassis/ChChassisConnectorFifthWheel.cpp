// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Ian Rust
// =============================================================================
//
// Template for a fifth wheel chassis connector.  This is a passive connector,
// modeled with a universal joint.
//
// =============================================================================

#include "chrono_vehicle/chassis/ChChassisConnectorFifthWheel.h"

namespace chrono {
namespace vehicle {

ChChassisConnectorFifthWheel::ChChassisConnectorFifthWheel(const std::string& name) : ChChassisConnectorHitch(name) {}

ChChassisConnectorFifthWheel::~ChChassisConnectorFifthWheel() {
    if (!m_initialized)
        return;

    auto sys = m_joint->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_joint);
}

void ChChassisConnectorFifthWheel::Initialize(std::shared_ptr<ChChassis> front, std::shared_ptr<ChChassisRear> rear) {
    std::cout << "?" << std::endl;
    ChChassisConnector::Initialize(front, rear);
    std::cout << "?" << std::endl;

    ChQuaternion<> x2z;
    std::cout << "?" << std::endl;
    x2z.SetFromAngleY(90.0);
    std::cout << "?" << std::endl;

    // Express the connector reference frames in the local coordinate system of the trailer and towing vehicle coordinate system
    //
    ChFrame<> rear_frame(rear->GetLocalPosFrontConnector());
    std::cout << "?" << std::endl;
    rear_frame.ConcatenatePreTransformation(rear->GetBody()->GetFrameRefToCOM());
    std::cout << "?" << std::endl;
    // rotate so that the fixed axis (z) is aligned with the body x direction
    rear_frame.SetRot(x2z);
    std::cout << "?" << std::endl;

    ChFrame<> front_frame(front->GetLocalPosRearConnector());
    std::cout << "?" << std::endl;
    front_frame.ConcatenatePreTransformation(front->GetBody()->GetFrameRefToCOM());
    std::cout << "?" << std::endl;
    // rotate so that the fixed axis (z) is aligned with the body x direction
    front_frame.SetRot(x2z);
    std::cout << "?" << std::endl;

    // Create the universal joint connection
    m_joint = chrono_types::make_shared<ChLinkUniversal>();
    std::cout << "?" << std::endl;
    m_joint->SetName(m_name + " joint");
    std::cout << "?" << std::endl;
    m_joint->Initialize(front->GetBody(), rear->GetBody(), true, front_frame, rear_frame);
    std::cout << "?" << std::endl;
    rear->GetBody()->GetSystem()->AddLink(m_joint);
    std::cout << "?" << std::endl;
}

}  // end namespace vehicle
}  // end namespace chrono
