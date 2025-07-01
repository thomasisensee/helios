#include "RisleyBeamDeflector2.h"

#include <iostream>
#include <sstream>
#include <string>
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
    prism1_thickness_base,
    prism2_thickness_base,
    prism3_thickness_base,
    prism1_radius,
    prism2_radius,
    prism3_radius,
    distance_prism1_2,
    distance_prism2_3,
    refrIndex_prism1,
    refrIndex_prism2,
    refrIndex_prism3,
    refrIndex_air);

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

  ombd->prism1_thickness_base = prism1_thickness_base;
  ombd->prism2_thickness_base = prism2_thickness_base;
  ombd->prism3_thickness_base = prism3_thickness_base;

  ombd->prism1_radius = prism1_radius;
  ombd->prism2_radius = prism2_radius;
  ombd->prism3_radius = prism3_radius;

  ombd->distance_prism1_2 = distance_prism1_2;
  ombd->distance_prism2_3 = distance_prism2_3;

  ombd->refrIndex_prism1 = refrIndex_prism1;
  ombd->refrIndex_prism2 = refrIndex_prism2;
  ombd->refrIndex_prism3 = refrIndex_prism3;
  ombd->refrIndex_air = refrIndex_air;
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

  // Determine the normal vectors of moving prism surfaces
  glm::dvec3 Prism1NormalVector1(
    cos(rotorSpeed_rad_1 * time) * sin(prism1_angle_rad),
    sin(rotorSpeed_rad_1 * time) * sin(prism1_angle_rad),
    -cos(prism1_angle_rad));

  glm::dvec3 Prism2NormalVector2(
    -cos(rotorSpeed_rad_2 * time) * sin(prism2_angle_rad),
    -sin(rotorSpeed_rad_2 * time) * sin(prism2_angle_rad),
    -cos(prism2_angle_rad));

  glm::dvec3 Prism3NormalVector1(
    cos(rotorSpeed_rad_3 * time) * sin(prism3_angle_rad),
    sin(rotorSpeed_rad_3 * time) * sin(prism3_angle_rad),
    -cos(prism3_angle_rad));

  // printf("rotorSpeed_rad_1 = %g\n",rotorSpeed_rad_1);
  // printf("rotorSpeed_rad_2 = %g\n",rotorSpeed_rad_2);
  // printf("rotorSpeed_rad_3 = %g\n",rotorSpeed_rad_3);

  // Prism1NormalVector1 = glm::dvec3(0,0,1); // trash
  // Prism2NormalVector2 = glm::dvec3(0,0,1); // trash
  // Prism3NormalVector1 = glm::dvec3(0,0,1); // trash

  // === Print for debugging ===
  // printf("time = %g\n",time);
  // printVec("Prism1NormalVector1", Prism1NormalVector1);
  // printVec("Prism1NormalVector2", Prism1NormalVector2);
  // printVec("Prism2NormalVector1", Prism2NormalVector1);
  // printVec("Prism2NormalVector2", Prism2NormalVector2);
  // printVec("Prism3NormalVector1", Prism3NormalVector1);
  // printVec("Prism3NormalVector2", Prism3NormalVector2);

  // === Beam propagation ===

  // Prism 1: First surface
  glm::dvec3 Pb11;
  bool ok = intersectLinePlane(pointOnOriginalBeam,
                               originalBeam,
                               Prism1Point1ZAxis,
                               Prism1NormalVector1,
                               Pb11);
  if (!ok) {
    printf("Parallel to prism 1 surface 1.\n");
    return;
  }
  // printVec("Pb11", Pb11);
  // printVec("originalBeam", originalBeam);

  glm::dvec3 beam11;
  ok = refractBeam(
    originalBeam, Prism1NormalVector1, refrIndex_air, refrIndex_prism1, beam11);
  if (!ok) {
    printf("Refraction at prism 1 surface 1.\n");
    return;
  }
  // printVec("beam11", beam11);

  // Prism 1: Second surface
  glm::dvec3 Pb12;
  ok = intersectLinePlane(
    Pb11, beam11, Prism1Point2ZAxis, Prism1NormalVector2, Pb12);
  if (!ok) {
    printf("Parallel to prism 1 surface 2.\n");
    return;
  }
  // printVec("Pb12", Pb12);

  glm::dvec3 beam12;
  ok = refractBeam(
    beam11, Prism1NormalVector2, refrIndex_prism1, refrIndex_air, beam12);
  if (!ok) {
    printf("Refraction at prism 1 surface 2.\n");
    return;
  }
  // printVec("beam12", beam12);

  // Prism 2: First surface
  glm::dvec3 Pb21;
  ok = intersectLinePlane(
    Pb12, beam12, Prism2Point1ZAxis, Prism2NormalVector1, Pb21);
  if (!ok) {
    printf("Parallel to prism 2 surface 1.\n");
    return;
  }
  // printVec("Pb21", Pb21);

  glm::dvec3 beam21;
  ok = refractBeam(
    beam12, Prism2NormalVector1, refrIndex_air, refrIndex_prism2, beam21);
  if (!ok) {
    printf("Refraction at prism 2 surface 1.\n");
    return;
  }
  // printVec("beam21", beam21);

  // Prism 2: Second surface
  glm::dvec3 Pb22;
  ok = intersectLinePlane(
    Pb21, beam21, Prism2Point2ZAxis, Prism2NormalVector2, Pb22);
  if (!ok) {
    printf("Parallel to prism 2 surface 2.\n");
    return;
  }
  // printVec("Pb22", Pb22);

  glm::dvec3 beam22;
  ok = refractBeam(
    beam21, Prism2NormalVector2, refrIndex_prism2, refrIndex_air, beam22);
  if (!ok) {
    printf("Refraction at prism 2 surface 2.\n");
    return;
  }
  // printVec("beam22", beam22);

  // Prism 3: First surface
  glm::dvec3 Pb31;
  ok = intersectLinePlane(
    Pb22, beam22, Prism3Point1ZAxis, Prism3NormalVector1, Pb31);
  if (!ok) {
    printf("Parallel to prism 3 surface 1.\n");
    return;
  }
  // printVec("Pb31", Pb31);

  glm::dvec3 beam31;
  ok = refractBeam(
    beam22, Prism3NormalVector1, refrIndex_air, refrIndex_prism3, beam31);
  if (!ok) {
    printf("Refraction at prism 3 surface 1.\n");
    return;
  }
  // printVec("beam31", beam31);

  // Prism 3: Second surface
  glm::dvec3 Pb32;
  ok = intersectLinePlane(
    Pb31, beam31, Prism3Point2ZAxis, Prism3NormalVector2, Pb32);
  if (!ok) {
    printf("Parallel to prism 3 surface 2.\n");
    return;
  }
  // printVec("Pb32", Pb32);

  glm::dvec3 beam32;
  ok = refractBeam(
    beam31, Prism3NormalVector2, refrIndex_prism3, refrIndex_air, beam32);
  if (!ok) {
    printf("Refraction at prism 3 surface 2.\n");
    return;
  }
  printVec("beam32", beam32);

  // Rotate to current position:
  this->cached_emitterRelativeAttitude = Rotation(Directions::up, beam32);
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
  intersection = pointOnLine + t * lineDirection;
  return true;
}

void
RisleyBeamDeflector2::printVec(const std::string& name, const glm::dvec3& v)
{
  std::cout << name << ": (" << v.x << ", " << v.y << ", " << v.z << ")\n";
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
  // printf("cos_delta_i = %g\n",cos_delta_i);
  // printf("sin2_delta_r = %g\n",sin2_delta_r);
  if (sin2_delta_r > 1.0)
    return false; // total internal reflection

  double cos_delta_r = sqrt(1.0 - sin2_delta_r);
  refracted = glm::normalize(
    (refractiveIdxA / refractiveIdxB) * incidentBeamDirection +
    (refractiveIdxA / refractiveIdxB * cos_delta_i - cos_delta_r) *
      surfaceNormal);
  return true;
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
  printf("rotorSpeed_rad_1 = %g\n", rotorSpeed_rad_1);
  printf("rotorSpeed_rad_2 = %g\n", rotorSpeed_rad_2);
  printf("rotorSpeed_rad_3 = %g\n", rotorSpeed_rad_3);

  // Point on original beam
  pointOnOriginalBeam = glm::dvec3(0.0, 0.0, -1.0);

  // Prism thickness on the z-axis
  double Prism1ThicknessOnZAxis =
    prism1_thickness_base + prism1_radius * tan(prism1_angle_rad);
  double Prism2ThicknessOnZAxis =
    prism2_thickness_base + prism2_radius * tan(prism2_angle_rad);
  double Prism3ThicknessOnZAxis =
    prism3_thickness_base + prism3_radius * tan(prism3_angle_rad);

  printf("tan(prism1_angle_rad): %g\n", tan(prism1_angle_rad));
  printf("tan(prism2_angle_rad): %g\n", tan(prism2_angle_rad));
  printf("tan(prism3_angle_rad): %g\n", tan(prism3_angle_rad));
  printf("Prism1ThicknessOnZAxis: %g\n", Prism1ThicknessOnZAxis);
  printf("Prism2ThicknessOnZAxis: %g\n", Prism2ThicknessOnZAxis);
  printf("Prism3ThicknessOnZAxis: %g\n", Prism3ThicknessOnZAxis);

  double Z = 0.0;

  // Prism 1 surface points on z-axis
  Prism1Point1ZAxis = glm::dvec3(0.0, 0.0, Z);
  printVec("Prism1Point1ZAxis", Prism1Point1ZAxis);
  Z += Prism1ThicknessOnZAxis;
  Prism1Point2ZAxis = glm::dvec3(0.0, 0.0, Z);
  printVec("Prism1Point2ZAxis", Prism1Point2ZAxis);

  // Prism 2 surface points on z-axis
  Z += distance_prism1_2;
  Prism2Point1ZAxis = glm::dvec3(0.0, 0.0, Z);
  printVec("Prism2Point1ZAxis", Prism2Point1ZAxis);
  Z += Prism2ThicknessOnZAxis;
  Prism2Point2ZAxis = glm::dvec3(0.0, 0.0, Z);
  // printVec("Prism2Point2ZAxis", Prism2Point2ZAxis);

  // Prism 3 surface points on z-axis
  Z += distance_prism2_3;
  Prism3Point1ZAxis = glm::dvec3(0.0, 0.0, Z);
  printVec("Prism3Point1ZAxis", Prism3Point1ZAxis);
  Z += Prism3ThicknessOnZAxis;
  Prism3Point2ZAxis = glm::dvec3(0.0, 0.0, Z);
  printVec("Prism3Point2ZAxis", Prism3Point2ZAxis);
}
