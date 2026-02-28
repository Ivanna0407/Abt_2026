// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;

public class Cmd_Auto_Intake extends Command {

  private final Sub_Intake intake;
  private Timer timer;  
  private PIDController intakeController; 
  private double intakeSpeed;
  private double setpoint; 

  public Cmd_Auto_Intake(Sub_Intake intake, double setpoint) {
    this.intake= intake;
    this.setpoint= setpoint;  
    addRequirements(intake);

    intakeController = new PIDController(0, 0, 0);

  }

  @Override
  public void initialize() {
    timer.start();
    intakeController.setSetpoint(setpoint);
  }

  @Override
  public void execute() {

    double current= intake.getEncoderMotorOmega();
    double desired= 90;

    intakeSpeed= intakeController.calculate(current, desired); 
    intake.setMotorOmegaWheels(intakeSpeed);

    if(intakeController.atSetpoint()){
      intake.setMotorIntakeWheels(1);
    }


  }

  @Override
  public void end(boolean interrupted) {

    timer.stop(); 
    intake.setMotorOmegaWheels(0);
    intake.setMotorIntakeWheels(0);
  }

  @Override
  public boolean isFinished() {
    
    if(timer.get()==8){
      intake.setMotorOmegaWheels(0);
      intake.setMotorIntakeWheels(0);
      System.out.println("Terminó comando intake");
      return true; 
    }
    else{
      return false; 
    }


  }
}