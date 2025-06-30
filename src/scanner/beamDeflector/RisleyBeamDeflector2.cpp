#include "RisleyBeamDeflector2.h"

#include <iostream>
#include <sstream>
using namespace std;

#include <glm/glm.hpp>
#define _USE_MATH_DEFINES
#include <logging.hpp>
#include <math.h>

#include "maths/Directions.h"
#include "maths/MathConverter.h"

using Base = std::shared_ptr<AbstractBeamDeflector>;

// Construction/Cloning
Base
RisleyBeamDeflector2::clone()
{
  Base ombd = std::make_shared<RisleyBeamDeflector2>(
    cfg_device_scanAngleMax_rad,
    rotorSpeed_rad_1 * 2.0 * M_PI,
    rotorSpeed_rad_2 * 2.0 * M_PI,
    rotorSpeed_rad_3 * 2.0 * M_PI,
    MathConverter::radiansToDegrees(prism1_angle_rad),
    MathConverter::radiansToDegrees(prism2_angle_rad),
    MathConverter::radiansToDegrees(prism3_angle_rad),
    prism1_thickness,
    prism2_thickness,
    prism3_thickness,
    prism1_radius,
    prism2_radius,
    prism3_radius,
    distance_prism1_2,
    distance_prism2_3,
    distance_to_observation_plane,
    refrIndex_prism1,
    refrIndex_prism2,
    refrIndex_prism3,
    refrIndex_air,
    numberOfBeams,
    beamSpreadLim);

  _clone(ombd);
  return ombd;
}
void
RisleyBeamDeflector2::_clone(std::shared_ptr<AbstractBeamDeflector> abd)
{
  AbstractBeamDeflector::_clone(abd);
  RisleyBeamDeflector2* ombd = (RisleyBeamDeflector2*)abd.get();
  ombd->scanAngle = scanAngle;
  ombd->rotorSpeed_rad_1 = rotorSpeed_rad_1;
  ombd->rotorSpeed_rad_2 = rotorSpeed_rad_2;
  ombd->rotorSpeed_rad_2 = rotorSpeed_rad_2;
  ombd->rotorSpeed_rad_3 = rotorSpeed_rad_3;

  ombd->prism1_angle_rad = prism1_angle_rad;
  ombd->prism2_angle_rad = prism2_angle_rad;
  ombd->prism3_angle_rad = prism3_angle_rad;

  ombd->prism1_thickness = prism1_thickness;
  ombd->prism2_thickness = prism2_thickness;
  ombd->prism3_thickness = prism3_thickness;

  ombd->prism1_radius = prism1_radius;
  ombd->prism2_radius = prism2_radius;
  ombd->prism3_radius = prism3_radius;

  ombd->distance_prism1_2 = distance_prism1_2;
  ombd->distance_prism2_3 = distance_prism2_3;
  ombd->distance_to_observation_plane = distance_to_observation_plane;

  ombd->refrIndex_prism1 = refrIndex_prism1;
  ombd->refrIndex_prism2 = refrIndex_prism2;
  ombd->refrIndex_prism3 = refrIndex_prism3;
  ombd->refrIndex_air = refrIndex_air;

  ombd->numberOfBeams = numberOfBeams;
  ombd->beamSpreadLim = beamSpreadLim;
}

void
RisleyBeamDeflector2::applySettings(std::shared_ptr<ScannerSettings> settings)
{
  AbstractBeamDeflector::applySettings(settings);
  cached_angleBetweenPulses_rad =
    (double)(this->cfg_setting_scanFreq_Hz * this->cfg_setting_scanAngle_rad *
             4) /
    settings->pulseFreq_Hz;
  scanAngle = this->cfg_setting_scanAngle_rad;
  deltaT = 1.0 / settings->pulseFreq_Hz;
}

void
RisleyBeamDeflector2::doSimStep()
{

  // time integration
  time += deltaT;

  // calculate the absolute angle

  double xFOV = cos(time * rotorSpeed_rad_1) + cos(time * rotorSpeed_rad_2);
  double yFOV = -sin(time * rotorSpeed_rad_1) - sin(time * rotorSpeed_rad_2);

  double phi = -xFOV / 2.0 * scanAngle;
  double eta = -yFOV / 2.0 * scanAngle;

  // Rotate to current position:
  this->cached_emitterRelativeAttitude =
    Rotation(RotationOrder::ZXY, phi, eta, 0);
}

void
RisleyBeamDeflector2::setScanAngle_rad(double scanAngle_rad)
{
  double scanAngle_deg = MathConverter::radiansToDegrees(scanAngle_rad);

  // Max. scan angle is limited by scan product:
  /*if (scanAngle_deg * this->cfg_setting_scanFreq_Hz >
  this->cfg_device_scanProduct) { logging::WARN( "ERROR: Requested scan angle
  exceeds device limitations " "as defined by scan product. " "Will set it to
  maximal possible value."
  );
          scanAngle_deg = ((double) this->cfg_device_scanProduct) /
  this->cfg_setting_scanFreq_Hz;
  }*/

  this->cfg_setting_scanAngle_rad =
    MathConverter::degreesToRadians(scanAngle_deg);
  stringstream ss;
  ss << "Scan angle set to " << scanAngle_deg << " degrees.";
  logging::INFO(ss.str());
}

// This setter method should not be used for this scanner.

void
RisleyBeamDeflector2::setScanFreq_Hz(double scanFreq_Hz)
{
  // Max. scan frequency is limited by scan product:
  // if( MathConverter::radiansToDegrees(this->cfg_setting_scanAngle_rad) *
  //    scanFreq_Hz > this->cfg_device_scanProduct
  //   ){
  //	logging::WARN(
  //	    "ERROR: Requested scan frequency exceeds device limitations "
  //           "as defined by scan product. "
  //           "Will set it to maximal possible value."
  //       );
  //	scanFreq_Hz = ((double) this->cfg_device_scanProduct) /
  //	    MathConverter::radiansToDegrees(this->cfg_setting_scanAngle_rad);
  //}
  this->cfg_setting_scanFreq_Hz = scanFreq_Hz;
  stringstream ss;
  ss << "Scan frequency set to " << this->cfg_setting_scanFreq_Hz << " Hz.";
  logging::INFO(ss.str());
}

void
RisleyBeamDeflector2::initializeGeometry()
{
  cachedPrism1NormalVector1 = glm::dvec3(0.0, 0.0, 1.0);
  cachedPrism1NormalVector2 =
    glm::dvec3(0.0, sin(prism1_angle_rad), cos(prism1_angle_rad));
  cachedPrism1NormalVector2Original = cachedPrism1NormalVector2;

  cachedPrism2NormalVector1 =
    glm::dvec3(0.0, -sin(prism2_angle_rad), cos(prism2_angle_rad));
  cachedPrism2NormalVector1Original = cachedPrism2NormalVector1;
  cachedPrism2NormalVector2 = glm::dvec3(0.0, 0.0, 1.0);

  cachedPrism3NormalVector1 =
    glm::dvec3(0.0, -sin(prism3_angle_rad), cos(prism3_angle_rad));
  cachedPrism3NormalVector1Original = cachedPrism3NormalVector1;
  cachedPrism3NormalVector2 = glm::dvec3(0.0, 0.0, 1.0);

  cachedObservationPlaneNormalVector = glm::dvec3(0.0, 0.0, 1.0);

  cachedBeamDirectionVectors.clear();
  for (int i = 0; i < numberOfBeams; ++i) {
    double a = -beamSpreadLim + 2 * beamSpreadLim * i / (numberOfBeams - 1);
    glm::dvec3 dir(a, 0.0, 1.0);
    dir = glm::normalize(dir);
    cachedBeamDirectionVectors.push_back(dir);
  }

  cachedPrism1ThicknessSlopedZAxis = prism1_radius * tan(prism1_angle_rad);
  cachedPrism2ThicknessSlopedZAxis = prism2_radius * tan(prism2_angle_rad);
  cachedPrism3ThicknessSlopedZAxis = prism3_radius * tan(prism3_angle_rad);

  cachedBeamZAxisPoint = glm::dvec3(0.0, 0.0, -1.0);
  cachedPrism1ZAxisPoint1 = glm::dvec3(0.0, 0.0, 0.0);

  double Z = prism1_thickness + cachedPrism1ThicknessSlopedZAxis;
  cachedPrism1ZAxisPoint2 = glm::dvec3(0.0, 0.0, Z);

  Z += distance_prism1_2 + cachedPrism1ThicknessSlopedZAxis +
       cachedPrism2ThicknessSlopedZAxis;
  cachedPrism2ZAxisPoint1 = glm::dvec3(0.0, 0.0, Z);

  Z += prism2_thickness + cachedPrism2ThicknessSlopedZAxis;
  cachedPrism2ZAxisPoint2 = glm::dvec3(0.0, 0.0, Z);

  Z += distance_prism2_3 + cachedPrism3ThicknessSlopedZAxis;
  cachedPrism3ZAxisPoint1 = glm::dvec3(0.0, 0.0, Z);

  Z += prism3_thickness + cachedPrism3ThicknessSlopedZAxis;
  cachedPrism3ZAxisPoint2 = glm::dvec3(0.0, 0.0, Z);

  Z += distance_to_observation_plane;
  cachedObservationPlaneZAxisPoint = glm::dvec3(0.0, 0.0, Z);
}
