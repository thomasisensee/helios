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

  // Rotate the normal vectors of moving prism surfaces
  glm::dvec3 Prism1NormalVector2 = rotateVectorRodrigues(
    this->cachedPrism1NormalVector2, this->rotorSpeed_rad_1 * this->time);
  glm::dvec3 Prism2NormalVector1 = rotateVectorRodrigues(
    this->cachedPrism2NormalVector1, this->rotorSpeed_rad_2 * this->time);
  glm::dvec3 Prism3NormalVector1 = rotateVectorRodrigues(
    this->cachedPrism3NormalVector1, this->rotorSpeed_rad_3 * this->time);

  // === Beam propagation ===

  // Initial beam direction
  glm::dvec3 beam00 = this->cachedBeamDirection;

  // Prism 1: First surface
  glm::dvec3 Pb11;
  bool ok = intersectLinePlane(this->cachedBeamZAxisPoint,
                               beam00,
                               this->cachedPrism1ZAxisPoint1,
                               this->cachedPrism1NormalVector1,
                               Pb11);
  if (!ok) {
    printf("Parallel\n");
    return;
  }

  glm::dvec3 beam11;
  ok = refractBeam(beam00,
                   this->cachedPrism1NormalVector1,
                   this->refrIndex_air,
                   this->refrIndex_prism1,
                   beam11);
  if (!ok) {
    printf("Refraction\n");
    return;
  }

  // Prism 1: Second surface
  glm::dvec3 Pb12;
  ok = intersectLinePlane(
    Pb11, beam11, this->cachedPrism1ZAxisPoint2, Prism1NormalVector2, Pb12);
  if (!ok) {
    printf("Parallel\n");
    return;
  }

  glm::dvec3 beam12;
  ok = refractBeam(beam11,
                   Prism1NormalVector2,
                   this->refrIndex_prism1,
                   this->refrIndex_air,
                   beam12);
  if (!ok) {
    printf("Refraction\n");
    return;
  }

  // Prism 2: First surface
  glm::dvec3 Pb21;
  ok = intersectLinePlane(
    Pb12, beam12, this->cachedPrism2ZAxisPoint1, Prism2NormalVector1, Pb12);
  if (!ok) {
    printf("Parallel\n");
    return;
  }

  glm::dvec3 beam21;
  ok = refractBeam(beam12,
                   Prism2NormalVector1,
                   this->refrIndex_air,
                   this->refrIndex_prism2,
                   beam21);
  if (!ok) {
    printf("Refraction\n");
    return;
  }

  // Prism 2: Second surface
  glm::dvec3 Pb22;
  ok = intersectLinePlane(Pb21,
                          beam21,
                          this->cachedPrism2ZAxisPoint2,
                          this->cachedPrism2NormalVector2,
                          Pb22);
  if (!ok) {
    printf("Parallel\n");
    return;
  }

  glm::dvec3 beam22;
  ok = refractBeam(beam21,
                   this->cachedPrism2NormalVector2,
                   this->refrIndex_prism2,
                   this->refrIndex_air,
                   beam22);
  if (!ok) {
    printf("Refraction\n");
    return;
  }

  // Prism 3: First surface
  glm::dvec3 Pb31;
  ok = intersectLinePlane(
    Pb22, beam22, this->cachedPrism3ZAxisPoint1, Prism3NormalVector1, Pb31);
  if (!ok) {
    printf("Parallel\n");
    return;
  }

  glm::dvec3 beam31;
  ok = refractBeam(beam22,
                   Prism3NormalVector1,
                   this->refrIndex_air,
                   this->refrIndex_prism3,
                   beam31);
  if (!ok) {
    printf("Refraction\n");
    return;
  }

  // Prism 3: Second surface
  glm::dvec3 Pb32;
  ok = intersectLinePlane(Pb31,
                          beam31,
                          this->cachedPrism3ZAxisPoint2,
                          this->cachedPrism3NormalVector2,
                          Pb32);
  if (!ok) {
    printf("Parallel\n");
    return;
  }

  glm::dvec3 beam32;
  ok = refractBeam(beam31,
                   this->cachedPrism3NormalVector2,
                   this->refrIndex_prism3,
                   this->refrIndex_air,
                   beam32);
  if (!ok) {
    printf("Refraction\n");
    return;
  }

  // Final intersection with observation plane
  glm::dvec3 Pbobs;
  ok = intersectLinePlane(Pb32,
                          beam32,
                          this->cachedObservationPlaneZAxisPoint,
                          this->cachedObservationPlaneNormalVector,
                          Pbobs);
  if (!ok) {
    printf("Parallel\n");
    return;
  }

  // Rotate to current position:
  this->cached_emitterRelativeAttitude = Rotation(Directions::forward, Pbobs);
}

bool
RisleyBeamDeflector2::intersectLinePlane(const glm::dvec3& pointOnLine,
                                         const glm::dvec3& lineDirection,
                                         const glm::dvec3& pointOnPlane,
                                         const glm::dvec3& normalPlane,
                                         glm::dvec3& intersection)
{
  double denom = glm::dot(lineDirection, normalPlane);
  if (std::abs(denom) < 1e-12)
    return false; // parallel

  double t = glm::dot(pointOnPlane - pointOnLine, normalPlane) / denom;
  intersection = pointOnPlane + t * lineDirection;
  return true;
}

bool
RisleyBeamDeflector2::refractBeam(const glm::dvec3& incidentBeamDirection,
                                  const glm::dvec3& surfaceNormal,
                                  double refractiveIdxA,
                                  double refractiveIdxB,
                                  glm::dvec3& refracted)
{
  double cos_delta_i = glm::dot(incidentBeamDirection, -surfaceNormal);
  double sin2_delta_r = (refractiveIdxA / refractiveIdxB) *
                        (refractiveIdxA / refractiveIdxB) *
                        (1.0 - cos_delta_i * cos_delta_i);

  if (sin2_delta_r > 1.0)
    return false; // total internal reflection

  double cos_delta_r = sqrt(1.0 - sin2_delta_r);
  refracted = (refractiveIdxA / refractiveIdxB) * incidentBeamDirection +
              (refractiveIdxA / refractiveIdxB * cos_delta_i - cos_delta_r) *
                surfaceNormal;
  return true;
}

glm::dvec3
RisleyBeamDeflector2::rotateVectorRodrigues(const glm::dvec3& vec, double angle)
{
  glm::dvec3 axis(0.0, 0.0, 1.0);

  return vec * cos(angle) + glm::cross(axis, vec) * sin(angle) +
         axis * glm::dot(axis, vec) * (1.0 - cos(angle));
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

  cachedPrism2NormalVector1 =
    glm::dvec3(0.0, -sin(prism2_angle_rad), cos(prism2_angle_rad));
  cachedPrism2NormalVector2 = glm::dvec3(0.0, 0.0, 1.0);

  cachedPrism3NormalVector1 =
    glm::dvec3(0.0, -sin(prism3_angle_rad), cos(prism3_angle_rad));
  cachedPrism3NormalVector2 = glm::dvec3(0.0, 0.0, 1.0);

  cachedObservationPlaneNormalVector = glm::dvec3(0.0, 0.0, 1.0);

  // --- Beam direction setup (single beam case) ---
  glm::dvec3 cachedBeamDirection(0.0, 0.0, 1.0);

  // --- Z-axis reference points ---
  cachedBeamZAxisPoint = glm::dvec3(0.0, 0.0, -1.0);
  cachedPrism1ZAxisPoint1 = glm::dvec3(0.0, 0.0, 0.0);

  cachedPrism1ThicknessSlopedZAxis = prism1_radius * tan(prism1_angle_rad);
  cachedPrism2ThicknessSlopedZAxis = prism2_radius * tan(prism2_angle_rad);
  cachedPrism3ThicknessSlopedZAxis = prism3_radius * tan(prism3_angle_rad);

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
