// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;

public class Cmd_Auto_Feeder extends Command {

  private final Sub_Shooter shooter; 
  private Timer timer= new Timer(); 
  private double timer2; 

  public Cmd_Auto_Feeder(Sub_Shooter shooter) {
    this.shooter = shooter; 
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start(); 
  }

  @Override
  public void execute() {

    timer2= timer.get(); 
    System.out.println("timer2" + timer2); 

    shooter.setFeederSpeed(-.3);


  }

  @Override
  public void end(boolean interrupted) {

    shooter.setFeederSpeed(0);

  }

  @Override
  public boolean isFinished() {

    if(timer.get()>=5.00){
      shooter.setFeederSpeed(0);
      System.out.println("terminó comando feeder"); 
      return true; 
    }
    else{
      return false; 
    }
    
  }
}