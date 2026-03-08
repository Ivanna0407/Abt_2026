// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Sub_Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_AutoAlignHood extends Command {
  
  private final Sub_Shooter Shooter;
  private double speedHood; 
  private PIDController hood_PID;  
  private double currentAngle;

  private double DesiredAngle1;
  private double DesiredAngle2;
  private double DesiredAngle3;


  /** Creates a new Cmd_AutoAlignHood. */
  public Cmd_AutoAlignHood(Sub_Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Shooter = shooter;
    hood_PID = new PIDController(0.19, 0, 0.0); //luego le pones los puntos

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {// alch ns si va algo aca aparte de los super duper leds
    
    // metes las funciones para el setpoints y asi...  
} 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //Consige la posicion
      // calcula el angulo
      // setea el angulo

      //Positions array (meters, degrees)
      // [0] = tx 
      // [1] = ty 
      // [2] = tz 
      // [3] = pitch 
      // [4] = yaw 
      // [5] = roll 

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-abt"); 
      //el positions ya guarda todas las poses de arriba

      
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
