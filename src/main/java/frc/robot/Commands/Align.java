// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.SwerveSubsystem;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

/**
 * An example command that uses an example subsystem.
 */
public class Align extends Command {

    private final SwerveSubsystem swerve;
    private final PIDController PIDang;
    private final PIDController PIDmove;
    final PhotonCamera photonCamera = new PhotonCamera("cam");

    public Align(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.PIDang = new PIDController(1, 0, 2);
        this.PIDmove = new PIDController(1, 0, 2);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        PIDang.reset();
        PIDmove.reset();
        photonCamera.setPipelineIndex(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // double ID = LimelightHelpers.getFiducialID("");
        var result = photonCamera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            PhotonTrackedTarget target = result.getBestTarget();
            
            double TX = target.getYaw();
            double TY = target.getPitch();

            double ID = target.getFiducialId();

            double d1, d2, d, r, a;

            d1 = 0.307975;
            // d2 = 0.1 / (Math.tan(Math.toRadians(TY)));
            d2 = PhotonUtils.calculateDistanceToTargetMeters(
                                0,
                                0.1,
                                0,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

            d = d1 + d2;

            a = d2 * Math.tan(Units.degreesToRadians(TX));

            r = Math.tanh(a / d);

            PIDang.setSetpoint(0);
            PIDmove.setSetpoint(1);

            double x = PIDmove.calculate(d2);
            Double rot = PIDang.calculate(((r / 2 * Math.PI)));
            Double y = 0.0;
 
            // Command AlignSwerve = swerve.driveCommand(
            //         () -> 1,
            //         () -> 1,
            //         () -> 1, true, false);
            
            swerve.drive(new Translation2d(0,y), rot, false);

            if (Constants.smartEnable) {
                SmartDashboard.putNumber("TX", TX);
                SmartDashboard.putNumber("TY", TY);

                SmartDashboard.putNumber("d2", d2);
                SmartDashboard.putNumber("a", a);

                SmartDashboard.putNumber("x", x);
                SmartDashboard.putNumber("y", y);
                SmartDashboard.putNumber("rot", rot);
            }

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Constants.smartEnable) {
            SmartDashboard.putString("Status", "finished");
        }
        return false;
    }

}
