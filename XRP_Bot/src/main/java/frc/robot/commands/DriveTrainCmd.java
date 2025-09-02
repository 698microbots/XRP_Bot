// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTrainCmd extends Command {

  private XRPDrivetrain drivetrain;
  private Supplier<Double> xSpeed, zSpeed;

  /** Creates a new DriveTrainCmd. */
  public DriveTrainCmd(XRPDrivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> zSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.zSpeed = zSpeed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double L = xSpeed.get();
    Double R = zSpeed.get();

    drivetrain.arcadeDrive(-L, -R);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
