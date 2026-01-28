// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Shooter;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Shooter extends Command {
  /** Creates a new Cmd_Shooter. */
  private final Sub_Shooter Shooter;
  private final Supplier <Double> RT,LT;
  private final Supplier <Boolean> RB,LB,A,B,X,Y;
  public Cmd_Shooter(Sub_Shooter shooter,Supplier <Double> RT, Supplier<Double> LT,Supplier <Boolean>RB,Supplier <Boolean> LB,Supplier<Boolean> A,Supplier<Boolean>B, Supplier <Boolean> X,Supplier <Boolean> Y) {
    this.Shooter=shooter;
    this.RT=RT;
    this.LT=LT;
    this.RB=RB;
    this.LB=LB;
    this.A=A;
    this.B=B;
    this.X=X;
    this.Y=Y;
    addRequirements(Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Speed_M1=RT.get()-LT.get();
    Shooter.setShooter1Speed(Speed_M1);
    if(RB.get()){
      Shooter.setShooter2Speed(.5);
    }
    else{
      if(LB.get()){
        Shooter.setShooter2Speed(-.5);
      }
      else{
        Shooter.setShooter2Speed(0);
      }
    }

      if(A.get()){
      Shooter.setShooter3Speed(.5);
    }
    else{
      if(B.get()){
        Shooter.setShooter3Speed(-.5);
      }
      else{
        Shooter.setShooter3Speed(0);
      }
    }

    if(X.get()){
      Shooter.setShooter4Speed(.5);
    }
    else{
      if(Y.get()){
        Shooter.setShooter4Speed(-.5);
      }
      else{
        Shooter.setShooter4Speed(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setShooter2Speed(0);
    Shooter.setShooter3Speed(0);
    Shooter.setShooter4Speed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
