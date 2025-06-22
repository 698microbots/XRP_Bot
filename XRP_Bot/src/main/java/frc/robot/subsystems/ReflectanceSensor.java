// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.xrp.XRPReflectanceSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReflectanceSensor extends SubsystemBase {
  /** Creates a new ReflectanceSensor. */
  XRPReflectanceSensor rs = new XRPReflectanceSensor();
  public ReflectanceSensor() {}

 /**
   * Returns the reflectance value of the left sensor.
   *
   * @return value between 0.0 (white) and 1.0 (black).
   */
  public double getLeftReflectanceValue() {
    return rs.getLeftReflectanceValue();
  }

    /**
   * Returns the reflectance value of the right sensor.
   *
   * @return value between 0.0 (white) and 1.0 (black).
   */
  public double getRightReflectanceValue() {
    return rs.getRightReflectanceValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
