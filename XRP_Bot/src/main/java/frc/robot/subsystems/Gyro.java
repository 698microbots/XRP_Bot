// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
  XRPGyro gyro = new XRPGyro();
  public Gyro() {}

    /**
   * Return the rate of rotation of the gyro
   *
   * <p>The rate is based on the most recent reading of the gyro.
   *
   * @return the current rate in degrees per second
   */
  public double getRate() {
    return gyro.getRate();
  }

  /**
   * Get the rate of turn in degrees-per-second around the X-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateX() {
    return gyro.getRateX();
  }

  /**
   * Get the rate of turn in degrees-per-second around the Y-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateY() {
    return gyro.getRateY();
  }

    /**
   * Get the rate of turn in degrees-per-second around the Z-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateZ() {
    return gyro.getRateZ();
  }

   /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around X-axis in degrees
   */
  public double getAngleX() {
    return gyro.getAngleX();
  }

    /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around Y-axis in degrees
   */
  public double getAngleY() {
    return gyro.getAngleY();
  }

    /**
   * Get the currently reported angle around the Z-axis.
   *
   * @return current angle around Z-axis in degrees
   */
  public double getAngleZ() {
    return gyro.getAngleZ();
  }

  /**
   * Return the actual angle in degrees that the robot is currently facing.
   *
   * <p>The angle is based on integration of the returned rate form the gyro. The angle is
   * continuous, that is, it will continue from 360->361 degrees. This allows algorithms that
   * wouldn't want to see a discontinuity in the gyro output as it sweeps from 360 to 0 on the
   * second time around.
   *
   * @return the current heading of the robot in degrees.
   */
  public double getAngle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
