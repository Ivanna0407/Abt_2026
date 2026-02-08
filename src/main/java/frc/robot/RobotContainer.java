// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Cmd_AutoAlign;
import frc.robot.commands.Cmd_Move_Swerve;
import frc.robot.commands.Cmd_Test_Feeder;
import frc.robot.commands.Cmd_Test_Intake;
import frc.robot.commands.Cmd_Test_Shooter;
import frc.robot.commands.Cmd_resetheading;
import frc.robot.subsystems.Sub_Intake;
import frc.robot.subsystems.Sub_Shooter;
import frc.robot.subsystems.Sub_Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Sub_Swerve Swerve= new Sub_Swerve();
  private final Sub_Shooter shooter = new Sub_Shooter();
  private final Sub_Intake Intake = new Sub_Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController joydrive = new CommandXboxController(0);
  private final CommandXboxController subdrive = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    Swerve.setDefaultCommand(new Cmd_Move_Swerve(Swerve, () -> joydrive.getLeftX(), () -> joydrive.getLeftY(), ()-> joydrive.getRightX(),() -> joydrive.rightBumper().getAsBoolean(),() -> joydrive.y().getAsBoolean()));
    Intake.setDefaultCommand(new Cmd_Test_Intake(Intake, () -> joydrive.rightBumper().getAsBoolean(), () -> joydrive.leftBumper().getAsBoolean(), () -> joydrive.a().getAsBoolean(),() -> joydrive.b().getAsBoolean()));
    shooter.setDefaultCommand(new Cmd_Test_Shooter(shooter, () -> subdrive.getRightTriggerAxis() , () -> subdrive.b().getAsBoolean(), () -> subdrive.a().getAsBoolean(), () -> subdrive.x().getAsBoolean(), ()-> subdrive.y().getAsBoolean()));
    
    configureBindings();
  }


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    joydrive.start().whileTrue(new Cmd_resetheading(Swerve));
    joydrive.x().whileTrue(new Cmd_AutoAlign(false, Swerve));    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Auto");
  }
}