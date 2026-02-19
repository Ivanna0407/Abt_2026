// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Sub_Swerve;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;



public class Cmd_Move_Swerve_Locon extends Command {
  private final Sub_Swerve sub_Swerve;
  private final Supplier<Double>  Xaxis,Yaxis,giros;
  private final Supplier<Boolean>fieldoriented,slow;
  private final Supplier<Boolean> RT; 

  private PIDController rotationPID; 
  private final Translation2d hubPose= new Translation2d(4.62,4.03);
  private Pose2d robotPose;

  private double dx; 
  private double dy; 
  private double desiredAngle;
  private double currentAngle; 
  private double omega; 

  private double rotOut; 

  public Cmd_Move_Swerve_Locon(Sub_Swerve Sub_Swerve,Supplier<Double> Xaxis,Supplier<Double> Yaxis,Supplier<Double> giros,Supplier<Boolean> fieldoriented,Supplier<Boolean> slow, Supplier<Boolean> RT) {
    this.sub_Swerve=Sub_Swerve;
    this.Xaxis=Xaxis;
    this.Yaxis=Yaxis;
    this.giros=giros;
    this.fieldoriented=fieldoriented;
    this.slow=slow;
    this.RT = RT; 
    addRequirements(Sub_Swerve);

    rotationPID= new PIDController(1.5, 0, 0); 
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

  }

 
  @Override
  public void initialize() {
    sub_Swerve.zeroHeading();
  }
  ChassisSpeeds chassisSpeeds;
  @Override
  public void execute() {
   
    Pose2d robotPose = sub_Swerve.getPose();   
    Translation2d robotTranslation = robotPose.getTranslation(); 

    dx= hubPose.getX() - robotTranslation.getX(); 
    dy= hubPose.getY() - robotTranslation.getY(); 

    desiredAngle= Math.atan2(dy,dx);
    currentAngle= robotPose.getRotation().getRadians(); 

    double velocidadx=Xaxis.get()*-1;
    double velocidady=(Yaxis.get())*-1;
    double velocidad_giros=giros.get()*-1;
    double fium;
    if(Math.abs(velocidadx)>.3){
    }else{
      if (Math.abs(velocidadx)>0.6){
      }
    }
    

    if (Math.abs(Xaxis.get())<0.05){velocidadx=0;}
    if (Math.abs(Yaxis.get())<0.05){velocidady=0;}
    if (Math.abs(giros.get())<0.05){velocidad_giros=0;}

    if(RT.get()){
      fium= 1; 
      omega= rotationPID.calculate(currentAngle, desiredAngle); 
      velocidad_giros= omega; 
      System.out.println(velocidad_giros);

    }
    

    if (slow.get()){
      fium=.5;
    }
    else{
      fium=.3;
    }

    if (fieldoriented.get()){
      
      chassisSpeeds= new ChassisSpeeds(velocidady*fium,velocidadx*fium, velocidad_giros*fium); 
    }
    else{
      chassisSpeeds= ChassisSpeeds.fromFieldRelativeSpeeds(velocidady*fium, velocidadx*fium, velocidad_giros*fium, sub_Swerve.get2Drotation());
    }
    //Manda un arreglo de estados de modulos que pasan por un objeto de Swerve drive kinematics para poder generar las velocidades
    SwerveModuleState[] moduleStates=Swerve.swervekinematics.toSwerveModuleStates(chassisSpeeds);
    sub_Swerve.setModuleStates(moduleStates);

    //System.out.println(desiredAngle);
  } 


  @Override
  public void end(boolean interrupted) {
    sub_Swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}