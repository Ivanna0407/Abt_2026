// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sub_Shooter extends SubsystemBase {
  /** Creates a new Sub_Shooter. */
  private final SparkFlex Motor_Shoot_R= new SparkFlex(14, MotorType.kBrushless);
  private final SparkFlex Motor_Shoot_L = new SparkFlex(15, MotorType.kBrushless);
  private final SparkMax Motor_Hood = new SparkMax(18, MotorType.kBrushless);
  private final SparkMax Motor_Indexer = new SparkMax(16, MotorType.kBrushless);
  private final SparkMax Motor_Feeder = new SparkMax(17, MotorType.kBrushless);
  private final RelativeEncoder Hood_Encoder= Motor_Hood.getEncoder();
  private final SparkFlexConfig Motor_Shoot_R_Config = new SparkFlexConfig();
  private final SparkFlexConfig Motor_Shoot_L_Config = new SparkFlexConfig();
  private final SparkMaxConfig Motor_Hood_Config = new SparkMaxConfig();
  private final SparkMaxConfig Motor_Indexer_Config = new SparkMaxConfig();
  private final SparkMaxConfig Motor_Feeder_Config= new SparkMaxConfig();
  public Sub_Shooter() {
    Motor_Shoot_R_Config.idleMode(IdleMode.kBrake);
    Motor_Shoot_L_Config.idleMode(IdleMode.kBrake);
    Motor_Hood_Config.idleMode(IdleMode.kBrake);
    Motor_Indexer_Config.idleMode(IdleMode.kBrake);
    Motor_Feeder_Config.idleMode(IdleMode.kBrake);
    Motor_Shoot_R.set(0);
    Motor_Shoot_L.set(0);
    Motor_Hood.set(0);
    Motor_Indexer.set(0);
    Motor_Feeder.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed){
    Motor_Shoot_R.set(speed);
    Motor_Shoot_L.set(-speed);
  }

  public double getShooterAmps(){
    return Motor_Shoot_L.getOutputCurrent();
  }
    public void setHoodSpeed(double speed){
    Motor_Hood.set(speed);
  }

    public void setIndexerSpeed(double speed){
    Motor_Indexer.set(speed);
  }

    public void setFeederSpeed(double speed){
    Motor_Feeder.set(speed);
  }

  public double getHoodEncoder(){
    return Hood_Encoder.getPosition();
  }

  public void resetHoodEncoder(){
    Hood_Encoder.setPosition(0);
  }
}
