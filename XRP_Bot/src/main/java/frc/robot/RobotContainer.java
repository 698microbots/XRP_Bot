// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveDistance;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.DriveTrainCmd;
import frc.robot.commands.GetDistance;
import frc.robot.commands.followWall;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.XRPDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XRPDrivetrain drivetrain = new XRPDrivetrain();
  private final CommandXboxController xboxController = new CommandXboxController(0);
  //private final ExampleCommand m_autoCommand = new ExampleCommand(drivetrain);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final RangeFinder rf = new RangeFinder();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(new DriveTrainCmd(drivetrain, () -> xboxController.getLeftY(), () -> xboxController.getRightY()));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //DriveTrainCmd driveTrainCmd = new DriveTrainCmd(drivetrain, .5, .5);
    //m_autoCommand = driveTrainCmd;

    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    //return driveTrainCmd;
    //return new GetDistance(rf);
    return new followWall(rf, drivetrain);
    //return new DriveDistance(.6, 2.0, drivetrain);
    

    //return m_chooser.getSelected();
    
  }
}
