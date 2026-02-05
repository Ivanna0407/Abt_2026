// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sub_Shooter extends SubsystemBase {
  /** Creates a new Sub_Shooter. */
  private final SparkFlex Shooter_1 = new SparkFlex(14, MotorType.kBrushless);
  private final SparkFlex Shooter_2 = new SparkFlex(15, MotorType.kBrushless);
  private final SparkMax Shooter_3 = new SparkMax(18, MotorType.kBrushless);
  private final SparkMax Shooter_4 = new SparkMax(16, MotorType.kBrushless);
  private final SparkFlexConfig shooter_1Config = new SparkFlexConfig();
  private final SparkFlexConfig shooter_2Config = new SparkFlexConfig();
  private final SparkMaxConfig shooter_3Config = new SparkMaxConfig();
  private final SparkMaxConfig shooter_4Config = new SparkMaxConfig();
  public Sub_Shooter() {
    shooter_1Config.idleMode(IdleMode.kBrake);
    shooter_2Config.idleMode(IdleMode.kBrake);
    shooter_3Config.idleMode(IdleMode.kBrake);
    shooter_4Config.idleMode(IdleMode.kBrake);
    Shooter_1.set(0);
    Shooter_2.set(0);
    Shooter_3.set(0);
    Shooter_4.set(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Speed Shooter 1", Shooter_1.get());
    SmartDashboard.putNumber("Speed Shooter 2", Shooter_2.get());
    SmartDashboard.putNumber("Speed Shooter 3", Shooter_3.get());
    SmartDashboard.putNumber("Amps Shooter 1", Shooter_1.getOutputCurrent());
    SmartDashboard.putNumber("Amps Shooter 2", Shooter_2.getOutputCurrent());
    SmartDashboard.putNumber("Amps Shooter 3", Shooter_3.getOutputCurrent());
  }


  public void setShooter1Speed(double speed){
    Shooter_1.set(speed);
  }

    public void setShooter2Speed(double speed){
    Shooter_2.set(speed);
  }

    public void setShooter3Speed(double speed){
    Shooter_3.set(speed);
  }

    public void setShooter4Speed(double speed){
    Shooter_4.set(speed);
  }
}
