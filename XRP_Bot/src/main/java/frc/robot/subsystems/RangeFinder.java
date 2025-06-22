// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Note that creating a new subsystem also creates a new RangeFinding object, so only make 1 of them

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RangeFinder extends SubsystemBase {
  /** Creates a new RangeFinder. */
  XRPRangefinder rf = new XRPRangefinder();
  public RangeFinder() {}

  public double getDistance() {
    return rf.getDistanceInches();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
