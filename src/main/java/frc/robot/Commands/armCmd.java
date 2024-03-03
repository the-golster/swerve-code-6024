package frc.robot.Commands;

import frc.robot.Subsystems.armSubystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class armCmd extends Command {
  private final armSubystem Arm;
  private final double leftSpeed;
  private final double rightSpeed;
//   double LeftSpeed;

  public armCmd(armSubystem subsystem, double leftspeed, double rightspeed) {
    Arm = subsystem;
    leftSpeed = leftspeed;
    rightSpeed = rightspeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  


@Override
  public void initialize() {
}


@Override
public void execute() {
    // Arm.setMotors(leftSpeed, rightSpeed);
    SmartDashboard.putNumber("rightspeed", rightSpeed);
    SmartDashboard.putNumber("leftspeed", leftSpeed);
  }


  @Override
  public void end(boolean interrupted) {
    Arm.setMotors(0, 0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
