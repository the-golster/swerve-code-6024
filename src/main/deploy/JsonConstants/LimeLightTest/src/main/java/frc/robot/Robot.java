// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    double[] Botpose = LimelightHelpers.getBotPose("");
    double distance = 50/Math.tan(LimelightHelpers.getTY("") * Math.PI/180);

    SmartDashboard.putNumber("TX", LimelightHelpers.getTX(""));
    SmartDashboard.putNumber("TY", LimelightHelpers.getTY(""));
    SmartDashboard.putNumber("TA", LimelightHelpers.getTA(""));
    SmartDashboard.putNumber("Latency_Pipeline", LimelightHelpers.getLatency_Pipeline(""));
    SmartDashboard.putNumber("Latency_Capture", LimelightHelpers.getLatency_Capture(""));
    SmartDashboard.putNumber("CurrentPipelineIndex", LimelightHelpers.getCurrentPipelineIndex(""));

    SmartDashboard.putNumberArray("Botpose", Botpose);
    
    SmartDashboard.putNumberArray("Botpose_wpiBlue", LimelightHelpers.getBotPose_wpiBlue(""));
    SmartDashboard.putNumberArray("Botpose_wpiRed", LimelightHelpers.getBotPose_wpiRed(""));
    SmartDashboard.putNumberArray("CameraPose_TargetSpace", LimelightHelpers.getCameraPose_TargetSpace(""));
    SmartDashboard.putNumberArray("TargetPose_CameraSpace", LimelightHelpers.getTargetPose_CameraSpace(""));
    
    SmartDashboard.putNumberArray("TargetColor", LimelightHelpers.getTargetColor(""));
    SmartDashboard.putNumber("FiducialID", LimelightHelpers.getFiducialID(""));
    SmartDashboard.putNumber("NeuralClassID", LimelightHelpers.getNeuralClassID(""));

    SmartDashboard.putNumber("Distance", distance);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
