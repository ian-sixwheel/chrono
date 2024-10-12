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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Semitractor for the long haul vehicle model based on Kraz 64431 data
//
// =============================================================================

#ifndef KRAZ_TRACTOR_H
#define KRAZ_TRACTOR_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/chassis/ChChassisConnectorHitch.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// Revoy system.
class CH_MODELS_API Revoy : public ChWheeledVehicle {
  public:
    Revoy(bool fixed,
          CollisionType chassis_collision_type = CollisionType::NONE,
          ChContactMethod contactMethod = ChContactMethod::NSC);
    Revoy(ChSystem* system, bool fixed, CollisionType chassis_collision_type = CollisionType::NONE);
    ~Revoy() {}

    virtual unsigned int GetNumberAxles() const override { return 1; }

    virtual void Initialize(std::shared_ptr<ChChassis> frontChassis,
                            const ChCoordsys<>& chassisPos,
                            double chassisFwdVel = 0);

    void DebugLog(int what);

  private:
    void Create(bool fixed, CollisionType chassis_collision_type);

    std::shared_ptr<ChChassisConnectorHitch> m_connector;  ///< connector to pulling vehicle
};

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
