// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sub_Climber extends SubsystemBase {
  /** Creates a new Sub_Climber. */
  private final SparkMax Motor_Climber_1 = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax Motor_Climber_2 = new SparkMax(22, MotorType.kBrushless);
  private final SparkMaxConfig Config_Climber_1 = new SparkMaxConfig();
  private final SparkMaxConfig Config_Climber_2 = new SparkMaxConfig();
  private final RelativeEncoder Encoder_Climber_1 = Motor_Climber_1.getEncoder();
   private final RelativeEncoder Encoder_Climber_2 = Motor_Climber_1.getEncoder();
  public Sub_Climber() {
    Config_Climber_1.idleMode(IdleMode.kBrake);
    Config_Climber_2.idleMode(IdleMode.kBrake);
    Motor_Climber_1.set(0);
    Motor_Climber_2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorsClimber(double speed){
    Motor_Climber_1.set(speed);
    Motor_Climber_2.set(speed);
  }

  public double getEncoderClimb1(){
   return Encoder_Climber_1.getPosition();
  }
  public double getEncoderClimb2(){
   return Encoder_Climber_2.getPosition();
  }
}
