// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;


import frc.robot.Constants;

public class armSubystem extends SubsystemBase {
  
  final CANSparkMax leftMotor = new CANSparkMax(Constants.Arm.armLeftID, MotorType.kBrushless);
  final CANSparkMax rightMotor = new CANSparkMax(Constants.Arm.armLeftID, MotorType.kBrushless);
  final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  final DutyCycleEncoder ThroughBoreEncoder = new  DutyCycleEncoder(1);
  
  /** Creates a new arm. */

  public armSubystem() {
    leftMotor.setInverted(Constants.Arm.leftInvert);
    rightMotor.setInverted(Constants.Arm.rightInvert);
    leftEncoder.setInverted(Constants.Arm.leftInvert);
    rightEncoder.setInverted(Constants.Arm.rightInvert);

    resetEncoder();
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotors(double leftSpeed, double rightSpeed) {
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  public double getLeftEncoder(){
    return leftEncoder.getPosition();
  }

  public double getRightEncoder(){
    return rightEncoder.getPosition();
  }

  public double getAbsoluteEncoder(){
    return ThroughBoreEncoder.getAbsolutePosition() - Constants.Arm.absoluteOffset;
  }

  public void resetEncoder(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
}
