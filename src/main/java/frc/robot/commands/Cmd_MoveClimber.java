// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/////////////////////////////////////// Mover las ruedas del indexer ///////////////////////////////////////////

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;


public class Cmd_MoveClimber extends Command {
  
  private final Sub_Climber climber;
  double coral;

  public Cmd_MoveClimber(Sub_Climber climber,double coral){
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.coral=coral;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      climber.setMotorsClimber(coral);
      
    }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setMotorsClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  /*
    if(Motor.coral==true){
      return true;
    }
    else{return false;}
     */
  }
}