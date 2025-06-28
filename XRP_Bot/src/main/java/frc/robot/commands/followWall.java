// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RangeFinder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class followWall extends Command {
  RangeFinder rf;
  Drivetrain dt;
  /** Creates a new followWall. */
  public followWall(RangeFinder rf, Drivetrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rf = rf;
    this.dt = dt;
    addRequirements(rf,dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.arcadeDrive(0, 0);
    dt.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (rf.getDistance() > 7) {
      dt.arcadeDrive(-1, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
