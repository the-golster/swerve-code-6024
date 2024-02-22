// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class shooterSubsystem extends SubsystemBase {
    
    private final TalonSRX leftMotor = new TalonSRX(Constants.Shooter.shooterLeftID);
    private final TalonSRX rightMotor = new TalonSRX(Constants.Shooter.shooterRightID);
  /** Creates a new shooter. */

  public shooterSubsystem() {
    leftMotor.setInverted(Constants.Shooter.leftInvert);
    rightMotor.setInverted(Constants.Shooter.rightInvert);
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotors(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

}
