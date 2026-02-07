// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve{
    // Los ratios se deben checar porque no estoy segura si son correctos 
    public static final double drive_motor_gear_ratio = 1/6.75;
    public static final double turning_motor_gear_ratio = 1/21.4285714286; //Se tiene que poner en decimales no en division  
    public static final double diametro_llanta=0.1016; //Valor esta en metros son 4 pulgadas 
    public static final double encoder_a_metros= drive_motor_gear_ratio*Math.PI*diametro_llanta;
    public static final double encoder_a_metros_por_segundos = encoder_a_metros/60;
    public static final double encoder_a_radianes= turning_motor_gear_ratio*2*Math.PI;
    public static final double encoder_a_radianes_por_segundo=encoder_a_radianes/60;
    //Trackwidth se refiere a el ancho del robot (de los centros a las llantas)
    //public static final double trackwidth=.5277;
    public static final double trackwidth=.6858;
    //Wheelbase es la distancia entre las dos llantas del lado derecho 
    public static final double wheelbaseentre2=trackwidth/2;
    //public static final double wheelbase=.5277;
    public static final double wheelbase=.6858;
    //Para más dudas https://www.chiefdelphi.com/t/measuring-track-width-and-wheel-base-for-swerve-drive-frc/440389
    public static final SwerveDriveKinematics swervekinematics =new SwerveDriveKinematics(
    new Translation2d(wheelbase/2,trackwidth/2), 
    new Translation2d(wheelbase/2,-trackwidth/2),
    new Translation2d(-wheelbase/2,trackwidth/2),
    new Translation2d(-wheelbase/2,-trackwidth/2));
    public static final double giro_p=.18;
    public static final double giro_i=.021;
  }
  public static class Subsitemas{
    //public static final double Conversion_muñeca = 360/81;
    public static final double conversion_elevador =1/9.52;
    public static final double conversion_wrist =360/55.227;
    public static final double conversion_brazo = 1/27.0;
  }

 
}