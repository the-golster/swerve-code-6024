// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.shooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class shooterCmd extends Command {
  private final shooterSubsystem shooter;
  private final double speed;
//   double LeftSpeed;

  public shooterCmd(shooterSubsystem shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }


  @Override
  public void initialize() {
}


@Override
public void execute() {
    shooter.setMotors(speed);
  }


  @Override
  public void end(boolean interrupted) {
    shooter.setMotors(0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
