// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Intake;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Test_Intake extends Command {
  /** Creates a new Cmd_Test_Intake. */
  private final Sub_Intake Intake;
  private final Supplier <Boolean> RB,LB,A,B;
  public Cmd_Test_Intake(Sub_Intake Intake, Supplier <Boolean> RB, Supplier <Boolean>LB, Supplier <Boolean> A,Supplier <Boolean> B) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.A=A;
    this.Intake=Intake;
    this.LB=LB;
    this.RB=RB;
    this.B=B;
    addRequirements(Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RB.get()){
      Intake.setMotorOmegaWheels(.5);
    }
    else{
      if (LB.get()) {
        Intake.setMotorOmegaWheels(-.5);
      }
      else{Intake.setMotorOmegaWheels(0);}
    }

    if(A.get()){
      Intake.setMotorIntakeWheels(.5);

    }
    else{
      if(B.get()){
        Intake.setMotorIntakeWheels(-.5);
      }
      else{
        Intake.setMotorIntakeWheels(0);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.setMotorIntakeWheels(0);
    Intake.setMotorOmegaWheels(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
