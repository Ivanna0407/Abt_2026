// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.Constants;
import frc.robot.LimelightHelpers; 
import frc.robot.subsystems.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;


public class Cmd_AutoAlign extends Command {

  private PIDController xController; 
  private PIDController yController; 
  private PIDController rotController; 
  private double yaw; 
  
  private double rotOut; 
  private double xOut; 
  private double yOut; 

  private boolean isRightScore; 
  private Timer dontSeeTagTimer; 
  private Timer stopTimer; 

  private final Sub_Swerve swerve; 
  private double tagID = -1; 




  public Cmd_AutoAlign( boolean isRightScore, Sub_Swerve swerve) {

    xController = new PIDController(0.19, 0, 0.0); //0.25
    yController= new PIDController(0.25, 0, 0.0); //0.3
    rotController= new PIDController(0.01, 0, 0.0); //0.18

    this.isRightScore= isRightScore; 
    this.swerve= swerve; 
    addRequirements(swerve);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.stopTimer= new Timer(); 
    this.stopTimer.start(); 
    this.dontSeeTagTimer= new Timer(); 
    this.dontSeeTagTimer.start(); 

    rotController.setSetpoint(-8);
    rotController.setTolerance(0.8); //1

    xController.setSetpoint(-2);
    xController.setTolerance(0.1); //0.9

    //yController.setSetpoint(isRightScore ? 0.2 : - 0.2);
    yController.setSetpoint(0.16);
    yController.setTolerance(0.03); //0.01

    tagID= LimelightHelpers.getFiducialID("limelight-abt"); 


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        

    ChassisSpeeds speeds; 

    if (LimelightHelpers.getTV("limelight-abt")){

      this.dontSeeTagTimer.reset();

      //Positions array (meters, degrees)
      // [0] = tx 
      // [1] = ty 
      // [2] = tz 
      // [3] = pitch 
      // [4] = yaw 
      // [5] = roll 

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-abt"); 
      SmartDashboard.putNumber("x", positions[2]); 
      SmartDashboard.putNumber("y", positions[0]);

      double xSpeed= xController.calculate(positions[2]); 
      SmartDashboard.putNumber("xSpeed", xSpeed); 

      double ySpeed= -yController.calculate(positions[0]); 
      SmartDashboard.putNumber("ySpeed", ySpeed);
      double rotValue= rotController.calculate(positions[4]);
      SmartDashboard.putNumber("rotSpeed", rotValue);
      
      System.out.println(positions[2]);
      
      yaw= NetworkTableInstance.getDefault().getTable("limelight-abt").getEntry("yaw").getDouble(0.0);
      SmartDashboard.putNumber("yaw", yaw);



      SmartDashboard.putNumber("xVV", positions[2]); 
      SmartDashboard.putNumber("yVV", positions[0]); 
      SmartDashboard.putNumber("rotVV", positions[4]);

      rotOut = rotValue; 
      xOut= xSpeed; 
      yOut= ySpeed; 

      //speeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);

      if( !(positions[4]>=6 && positions[4]<=7) || !(positions[0]>=-1.8 && positions[0]<-2.1) || !(positions[2]>= -0.69 && positions[2]<= -0.73)){

        stopTimer.reset(); 

      }

      /*if(rotController.atSetpoint()){
        rotOut= 0; 
      }
      if(yController.atSetpoint()){
        yOut= 0; 
      }
      if(xController.atSetpoint()){
        xOut= 0; 
      } **/ 

      speeds= new ChassisSpeeds(xOut, yOut, -rotOut);
      
      
  
    }

    else{
      speeds= new ChassisSpeeds(0,0, 0); 
    }
    
    if(rotController.atSetpoint()){
      //System.out.println("giro listo");
    }

    if(yController.atSetpoint()){
      //System.out.println("hori listo");
    }

    if(xController.atSetpoint()){
      //System.out.println("acerca listo");
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());

    SwerveModuleState[] states= Constants.Swerve.swervekinematics.toSwerveModuleStates(speeds); 
    swerve.setModuleStates(states);


  }

  @Override
  public void end(boolean interrupted) {

    swerve.setModuleStates(Constants.Swerve.swervekinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0,0 )));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-abt"); 

    /*if ( yController.atSetpoint() && rotController.atSetpoint() && xController.atSetpoint() ){
      //System.out.println("Se acabó el comando");
      return true;
    }**/

    if(rotController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint()){
      System.out.println("Se acabó el comando");
      return true; 
    }
    else{
      return false; 
    }
  }
}