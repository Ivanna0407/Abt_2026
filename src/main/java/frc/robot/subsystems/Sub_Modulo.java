// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

public class Sub_Modulo extends SubsystemBase {
  //Se crean los objetos 
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    private final PIDController PIDgiro;
    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
     SwerveModulePosition Position = new SwerveModulePosition();
     SwerveModulePosition posicion = new SwerveModulePosition();
    //Tiene que llamarse igual 
    //Se crea un constructor, como si fuera un comando o una función para no tener que hacer 4 subsitemas diferentes (1 por modulo)
    public Sub_Modulo (int Drive_Motor_ID, int Turn_Motor_ID, boolean Inverted_Drive_Motor, boolean Inverted_Turning_Motor,int Encoder_Absoluto_ID,boolean inverted_encoder_abs){
        //Se dan los valores a los objetos que se habían creado antes
        this.absoluteEncoderReversed=inverted_encoder_abs;
        absoluteEncoder = new CANcoder(Encoder_Absoluto_ID);
        turningMotor= new SparkMax(Turn_Motor_ID,MotorType.kBrushless);
        driveMotor= new SparkMax(Drive_Motor_ID, MotorType.kBrushless);
        SparkMaxConfig configdrive= new SparkMaxConfig();
        SparkMaxConfig configturn=new SparkMaxConfig();

        configdrive.inverted(Inverted_Drive_Motor).idleMode(IdleMode.kBrake);
        configturn.inverted(Inverted_Turning_Motor).idleMode(IdleMode.kBrake);

        driveEncoder=driveMotor.getEncoder();
        turningEncoder=turningMotor.getEncoder();

        configdrive.encoder.positionConversionFactor(Swerve.encoder_a_metros);
        configturn.encoder.positionConversionFactor(Swerve.encoder_a_radianes);

        configdrive.encoder.velocityConversionFactor(Swerve.encoder_a_metros_por_segundos);
        configturn.encoder.velocityConversionFactor(Swerve.encoder_a_radianes_por_segundo);
        
        configdrive.openLoopRampRate(.25); 
        
        PIDgiro= new PIDController(.6, .00, 0.00);//.366 cosmo//.26 aire
        PIDgiro.enableContinuousInput(-Math.PI, Math.PI);//Permite trabajar con los valores de 180 a -180 

        driveMotor.configure(configdrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(configturn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        resetEncoders();
        
    }

    public double getcorrientedrive(){
        return driveMotor.getOutputCurrent();
    }
    public double getcorrienteturn(){
        return turningMotor.getOutputCurrent();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
      }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
      }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRadians(){
        //Al ser un analog input se tiene que checar que valores muestra 
        double angulo =(absoluteEncoder.getAbsolutePosition().getValueAsDouble()*2* Math.PI);
        return angulo * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRadians());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getModulePosition(){
        posicion= new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getAbsoluteEncoderRadians()));
        return posicion;
    }

    public void setDesiredState(SwerveModuleState state){
        //Se pide un Swerve module state que permite darle una serie de velocidades y posiciónes a los modulos 
        //Si la velocidad del estado es muy poca no se manda nada 
        if (Math.abs(state.speedMetersPerSecond)<0.001){
            driveMotor.set(0);
            turningMotor.set(0);
            return;
        }

        var enoderrotation=new Rotation2d(turningEncoder.getPosition());
        
        //state=SwerveModuleState.optimize(state, getState().angle);//330 grados y -30 grados es lo mismo, optimize puede hacer ese calculo 
        //y obtener la ruta más rápida 
        state.optimize(enoderrotation);
        
        driveMotor.set(state.speedMetersPerSecond/1.3);//3.5 es la velocidad máxima del sistema, se debe checar, 4.47 teorico
        //https://www.chiefdelphi.com/t/how-to-calculate-the-max-free-speed-of-a-swerve/400741/4
        turningMotor.set(PIDgiro.calculate(getTurningPosition(),state.angle.getRadians()));
        
        Position= new SwerveModulePosition(Position.distanceMeters + (state.speedMetersPerSecond*.02), state.angle);
    }
    
    public void alto(){
      driveMotor.set(0);
      turningMotor.set(0);
    }

    
}
