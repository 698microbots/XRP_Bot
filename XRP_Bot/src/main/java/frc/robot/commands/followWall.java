// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.XRPDrivetrain;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class followWall extends Command {
  /** Creates a new followWall. */
  RangeFinder rf;
  XRPDrivetrain drivetrain;
  public followWall(RangeFinder rf, XRPDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rf = rf;
    this.drivetrain = drivetrain;
    addRequirements(rf, drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.arcadeDrive(0, 0);
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (rf.getDistance() > 7){
      drivetrain.arcadeDrive(.6, 0);
    }
    System.out.println(rf.getDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
