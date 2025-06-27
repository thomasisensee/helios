#pragma once

#include "AbstractBeamDeflector.h"

#include "MathConverter.h"

/**
 * @brief Class representing a risley prisms beam deflector
 */
class RisleyBeamDeflector2 : public AbstractBeamDeflector
{

public:
  // ***  ATTRIBUTES  *** //
  // ******************** //

  double deltaT = 0;
  double pulseFreq = 0;
  double time = 0;

  /**
   * @brief Scan Angle (defined as the half angle)
   */
  double scanAngle = 0;
  double rotorSpeed_rad_1 = 0;
  double rotorSpeed_rad_2 = 0;
  double rotorSpeed_rad_3 = 0;
  double prism1_angle = 0;
  double prism2_angle = 0;
  double prism3_angle = 0;
  double prism1_thickness = 0;
  double prism2_thickness = 0;
  double prism3_thickness = 0;
  double prism1_radius = 0;
  double prism2_radius = 0;
  double prism3_radius = 0;
  double distance_prism1_2 = 0;
  double distance_prism2_3 = 0;
  double distance_to_observation_plane = 0;
  double refrIndex_prism1 = 0;
  double refrIndex_prism2 = 0;
  double refrIndex_prism3 = 0;
  double refrIndex_air = 0;
  int numberOfBeams = 0;
  double beamSpreadLim = 0;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  /**
   * @brief Constructor for risley prisms beam deflector
   * @see RisleyBeamDeflector2::cfg_device_scanProduct
   * @see AbstractBeamDeflector::AbstractBeamDeflector(
   *  double, double, double)
   */
  RisleyBeamDeflector2(double scanAngleMax_rad,
                       double rotorFreq_Hz_1,
                       double rotorFreq_Hz_2,
                       double rotorFreq_Hz_3,
                       double prism1_angle_deg,
                       double prism2_angle_deg,
                       double prism3_angle_deg,
                       double prism1_thickness,
                       double prism2_thickness,
                       double prism3_thickness,
                       double prism1_radius,
                       double prism2_radius,
                       double prism3_radius,
                       double distance_prism1_2,
                       double distance_prism2_3,
                       double distance_to_observation_plane,
                       double refrIndex_prism1,
                       double refrIndex_prism2,
                       double refrIndex_prism3,
                       double refrIndex_air,
                       int numberOfBeams,
                       double beamSpreadLim)
    : AbstractBeamDeflector(scanAngleMax_rad, 0, 0)
  {
    this->scanAngle = scanAngleMax_rad;
    this->rotorSpeed_rad_1 = rotorFreq_Hz_1 * 0.5 / M_PI;
    this->rotorSpeed_rad_2 = rotorFreq_Hz_2 * 0.5 / M_PI;
    this->rotorSpeed_rad_3 = rotorFreq_Hz_3 * 0.5 / M_PI;

    this->prism1_angle = prism1_angle_deg;
    this->prism2_angle = prism2_angle_deg;
    this->prism3_angle = prism3_angle_deg;

    this->prism1_thickness = prism1_thickness;
    this->prism2_thickness = prism2_thickness;
    this->prism3_thickness = prism3_thickness;

    this->prism1_radius = prism1_radius;
    this->prism2_radius = prism2_radius;
    this->prism3_radius = prism3_radius;

    this->distance_prism1_2 = distance_prism1_2;
    this->distance_prism2_3 = distance_prism2_3;
    this->distance_to_observation_plane = distance_to_observation_plane;

    this->refrIndex_prism1 = refrIndex_prism1;
    this->refrIndex_prism2 = refrIndex_prism2;
    this->refrIndex_prism3 = refrIndex_prism3;
    this->refrIndex_air = refrIndex_air;

    this->numberOfBeams = numberOfBeams;
    this->beamSpreadLim = beamSpreadLim;
  }
  std::shared_ptr<AbstractBeamDeflector> clone() override;
  void _clone(std::shared_ptr<AbstractBeamDeflector> abd) override;

  // ***  M E T H O D S  *** //
  // *********************** //
  /**
   * @see AbstractBeamDeflector::applySettings
   */
  void applySettings(std::shared_ptr<ScannerSettings> settings) override;
  /**
   * @see AbstractBeamDeflector::doSimStep
   */
  void doSimStep() override;

  /**
   * @see AbstractBeamDeflector::getOpticsType
   */
  std::string getOpticsType() const override { return "RISLEY"; }

  // ***  GETTERS and SETTERS  *** //
  // ***************************** //
  /**
   * @see AbstractBeamDeflector::setScanAngle_rad
   */
  void setScanAngle_rad(double scanAngle_rad) override;
  /**
   * @see AbstractBeamDeflector::setScanFreq_Hz
   */
  void setScanFreq_Hz(double scanFreq_Hz) override;
};
