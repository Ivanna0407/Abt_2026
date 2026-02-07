// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Intake extends Command {
  /** Creates a new Cmd_Intake. */
  private final Sub_Intake Sub_Intake;
  private final Supplier <Double> RightTrigger;
  private final Supplier <Boolean> RightBumper, LeftBumper,Y,A;

  private double speedWheels; 
  private double speedOmega; 
  
  //private PIDController Omega_PID; 

  public Cmd_Intake(Sub_Intake Intake, Supplier<Double> RightTrigger, Supplier<Boolean> RightBumper,Supplier<Boolean>LeftBumper,Supplier<Boolean>Y,Supplier<Boolean>A) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.A=A;
    this.Y=Y;
    this.RightTrigger=RightTrigger;
    this.RightBumper=RightBumper;
    this.LeftBumper=LeftBumper;
    this.Sub_Intake=Intake;
    addRequirements(Intake);

    //Omega_PID= new PIDController(0.1, 0, 0); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    speedWheels= 0.3; 
    speedOmega= 0.1;

    Sub_Intake.setMotorIntakeWheels(speedWheels);
    Sub_Intake.setMotorIntakeWheels(speedOmega);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Sub_Intake.setMotorIntakeWheels(0);
    Sub_Intake.setMotorOmegaWheels(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
