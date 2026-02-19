// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Test_Shooter extends Command {
  /** Creates a new Cmd_Test_Shooter. */
  private final Sub_Shooter shooter;
  private final Supplier <Double> RT;
  private final Supplier <Boolean> B,X,Y,A;
  public Cmd_Test_Shooter(Sub_Shooter shooter, Supplier<Double> RT,Supplier<Boolean> B,Supplier<Boolean> A,Supplier<Boolean>X,Supplier<Boolean>Y) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.A=A;
    this.B=B;
    this.RT=RT;
    this.X=X;
    this.Y=Y;
    this.shooter=shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterSpeed(-RT.get());

    if(A.get()){
      shooter.setHoodSpeed(-.2);
    }
    else{
      if(Y.get()){
        shooter.setHoodSpeed(.2);
      }
      else{shooter.setHoodSpeed(0);}
    }

    if(X.get()){
      shooter.setFeederSpeed(-.3);
      shooter.setIndexerSpeed(-.5);
    }
    else{
      if(B.get()){
        shooter.setFeederSpeed(.3);
        shooter.setIndexerSpeed(.5);
      }
      else{
        shooter.setFeederSpeed(0);
        shooter.setIndexerSpeed(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFeederSpeed(0);
    shooter.setIndexerSpeed(0);
    shooter.setHoodSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
