// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


public class Sub_Swerve extends SubsystemBase {
  //En este subsistema se unen los 4 modulos y el giroscopio 
   
  private final Sub_Modulo Modulo_1 = new Sub_Modulo(3, 4, true, true, 10, false);
  private final Sub_Modulo Modulo_2 = new Sub_Modulo(5, 6, true, true, 11,  false);
  private final Sub_Modulo Modulo_3 = new Sub_Modulo(7, 8, true, true, 12,  false);
  private final Sub_Modulo Modulo_4 = new Sub_Modulo(1, 2, true, true, 9 , false);
  private final Pigeon2 Pigeon = new Pigeon2(13);
  private final StructArrayPublisher<SwerveModuleState> States;
  //private final SwerveDriveOdometry odometry= new SwerveDriveOdometry(Swerve.swervekinematics,gyro.getRotation2d(), getModulePositions());
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Swerve.swervekinematics, Pigeon.getRotation2d(), getModulePositions());
  private Field2d field= new Field2d();
  double[] array;
  RobotConfig config;
  
  
  private final AprilTagFieldLayout apriltag= AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  
  

  public Sub_Swerve() {
    new Thread(()->{try {Thread.sleep(1000); zeroHeading();}catch(Exception e ){}}).start(); 
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    States= NetworkTableInstance.getDefault().getStructArrayTopic("Swerve", SwerveModuleState.struct).publish();
     AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    field.setRobotPose(getPose()); 
    SmartDashboard.putNumber("Yaw", Head());
    SmartDashboard.putNumber("Radianes", getYawRadians());
    odometry.update(get2Drotation(), getModulePositions());
    States.set(getModuleStates());
    SmartDashboard.putNumberArray("X", getBotpose_TargetSpace());
    double[] x= getBotpose_TargetSpace();
    SmartDashboard.putNumber("Lime X", x[2]);
    SmartDashboard.putNumber("Lime Y", x[0]);
    SmartDashboard.putNumber("Lime ROT", x[4]);
    SmartDashboard.putNumber("Speed", Modulo_1.getDriveVelocity());
    SmartDashboard.putNumber("Amp drive", Modulo_1.getcorrientedrive());
    SmartDashboard.putNumber("Amp giro", Modulo_1.getcorrienteturn());
  }

  public void zeroHeading(){
    Pigeon.reset();
  }

  public double getYawRadians(){
    return Head()*(Math.PI*180);
  }

  public double Head(){
    return Math.IEEEremainder(Pigeon.getYaw().getValueAsDouble(), 360);
  }

  public Rotation2d get2Drotation(){
    //Permite cambiar de angulos a un objeto de Rotation 2D
    return Rotation2d.fromDegrees(Head());
  }


  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }


  public void resetPose(Pose2d pose2d){
    odometry.resetPosition(get2Drotation(), getModulePositions(), pose2d);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return Swerve.swervekinematics.toChassisSpeeds(getModuleStates());
  }

  
  public void stopModules(){
    Modulo_1.alto();
    Modulo_2.alto();
    Modulo_3.alto();
    Modulo_4.alto();
  }

  public void setModuleStates(SwerveModuleState[] desiredModuleStates){
    //Se genera un arreglo de swerve module state para poder mandarlos a los diferentes modulos de acuerdo a posición 
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, 4);// velocidad
    Modulo_1.setDesiredState(desiredModuleStates[0]);
    Modulo_2.setDesiredState(desiredModuleStates[1]);
    Modulo_3.setDesiredState(desiredModuleStates[2]);
    Modulo_4.setDesiredState(desiredModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = Swerve.swervekinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public void setSpeeds(ChassisSpeeds Speeds){
    setModuleStates(Swerve.swervekinematics.toSwerveModuleStates(Speeds));
  }
    

  public void resetAllEncoders(){
    Modulo_1.resetEncoders();
    Modulo_2.resetEncoders();
    Modulo_3.resetEncoders();
    Modulo_4.resetEncoders();
  }

  public void setSpecificState(SwerveModuleState specificState,SwerveModuleState specificState_1,SwerveModuleState specificState_2,SwerveModuleState specificState_3){
    Modulo_1.setDesiredState(specificState);
    Modulo_2.setDesiredState(specificState_1);
    Modulo_3.setDesiredState(specificState_2);
    Modulo_4.setDesiredState(specificState_3);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0]=Modulo_1.getModulePosition();
    positions[1]=Modulo_2.getModulePosition();
    positions[2]=Modulo_3.getModulePosition();
    positions[3]=Modulo_4.getModulePosition();
    return positions;
    }

    public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] positions = new SwerveModuleState[4];
    positions[0]=Modulo_1.getState();
    positions[1]=Modulo_2.getState();
    positions[2]=Modulo_3.getState();
    positions[3]=Modulo_4.getState();
    return positions;
    }

    ////////////////////////LIMELIGHT////////////////////////
  public double getTx() {
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("tx").getDouble(0);
  }

  public double getTy() {
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("ty").getDouble(0);
  }

  public double getTa() {
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("ta").getDouble(10);
  }

  public double getTid() {
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("tid").getDouble(0);
  }

  public void SetVisionMode(Double m) {
    NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("pipeline").setNumber(m);
  }

  public double getTxnc(){
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("txnc").getDouble(0);
  }

  public long getTv(){
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("tv").getInteger(0);
  }

  public double[] getBotpose_TargetSpace(){
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("botpose_targetspace").getDoubleArray(new double[6]);
  }

  public Pose2d getAprilTagPose(int ArpiltagID){
    return apriltag.getTagPose(ArpiltagID).get().toPose2d();
  }

  
}