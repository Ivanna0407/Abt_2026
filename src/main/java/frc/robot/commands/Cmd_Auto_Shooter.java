// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;



public class Cmd_Auto_Shooter extends Command {

private final Sub_Shooter shooter; 
private Timer timer = new Timer(); 
private double timer2;
private double setpoint; 
private double hoodSpeed; 
private PIDController hoodController;  


  public Cmd_Auto_Shooter(Sub_Shooter shooter , double setpoint) {
    this.shooter= shooter;
    this.setpoint= setpoint;  
    addRequirements(shooter);

    hoodController= new PIDController(0, 0, 0); 

  }

  @Override
  public void initialize() {

    timer.reset();
    hoodController.setSetpoint(setpoint);
    timer.start();

  }

  @Override
  public void execute() {
    System.out.println(timer2); 

    shooter.setShooterSpeed(-1);
    

    double current= shooter.getHoodEncoder();
    double desired= 90; 

    hoodSpeed= hoodController.calculate(current, desired);
    shooter.setFeederSpeed(-.3);
    shooter.setIndexerSpeed(-.5);

    shooter.setHoodSpeed(hoodSpeed);
  
  }

  @Override
  public void end(boolean interrupted) {

    timer.stop();
    shooter.setShooterSpeed(0);
    shooter.setHoodSpeed(0);
    shooter.setFeederSpeed(0);

  }

  @Override
  public boolean isFinished() {

    timer2 = timer.get(); 
    
    if(timer.get()>=5.00){
      shooter.setShooterSpeed(0);
      shooter.setHoodSpeed(0);
      shooter.setFeederSpeed(0);
      System.out.println("terminó comando shooter");
      return true;   
    }
    else{
      return false; 
    }

  }
}