// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sub_Intake extends SubsystemBase {
  /** Creates a new Sub_Intake. */
  private final SparkMax Motor_Intake_Wheels = new SparkMax(19, MotorType.kBrushless);
  private final SparkMax Motor_Intake_Wheels_2 = new SparkMax(23, MotorType.kBrushless);
  private final SparkFlex Motor_Intake_Omega = new SparkFlex(20,MotorType.kBrushless);
  private final RelativeEncoder Encoder_Intake_Omega = Motor_Intake_Omega.getEncoder();
  private final SparkFlexConfig Config_Motor_Intake_Wheels = new SparkFlexConfig();
  private final SparkFlexConfig Config_Motor_Intake_Wheels_2 = new SparkFlexConfig();
  private final SparkMaxConfig Config_Motor_Intake_Omega = new SparkMaxConfig();
  public Sub_Intake() {
    Config_Motor_Intake_Omega.idleMode(IdleMode.kBrake);
    Config_Motor_Intake_Wheels.idleMode(IdleMode.kBrake);
    Config_Motor_Intake_Wheels_2.idleMode(IdleMode.kBrake);
    Motor_Intake_Omega.set(0);
    Motor_Intake_Wheels.set(0);
    Motor_Intake_Wheels_2.set(0);
    Motor_Intake_Omega.configure(Config_Motor_Intake_Omega, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Motor_Intake_Wheels.configure(Config_Motor_Intake_Wheels, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Motor_Intake_Wheels_2.configure(Config_Motor_Intake_Wheels_2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorIntakeWheels(double speed){
    Motor_Intake_Wheels.set(-speed);
    Motor_Intake_Wheels_2.set(speed);
  }

  public void setMotorOmegaWheels(double speed){
    Motor_Intake_Omega.set(speed);
  }

  public double getEncoderMotorOmega(){
    return Encoder_Intake_Omega.getPosition()*(360/54);
  }

  public double getMotorIntakeWAmps(){
    return Motor_Intake_Wheels.getOutputCurrent();
  }

    public void resetEncoder(){
    Encoder_Intake_Omega.setPosition(0);
  }
}
