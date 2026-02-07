// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Shooter extends Command {

  private final Sub_Shooter Shooter; 
  
  private double speedShooter; 
  //private double speedHood; 
  //private PIDController Hood_PID; 



  public Cmd_Shooter(Sub_Shooter Shooter) {

    this.Shooter=Shooter; 
    addRequirements(Shooter);

    //Hood_PID = new PIDController(0, 0, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //Hood_PID.setSetpoint(0);
    //Hood_PID.setTolerance(0.01);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    speedShooter= 0.1; 
    //speedHood= Hood_PID.getP(); 

    Shooter.setShooterSpeed(speedShooter);
    //Shooter.setHoodSpeed(speedHood);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Shooter.setShooterSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
