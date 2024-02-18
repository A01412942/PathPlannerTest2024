package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class S_RotateZ extends Command {
  SwerveSubsystem swerveSub;
  PIDController pid;
  double desiredZ;


  public S_RotateZ(SwerveSubsystem swerveSub, double desiredDegrees) {
    this.swerveSub = swerveSub;
    desiredZ = desiredDegrees;
    pid = new PIDController(SwerveConstants.KP_AUTO_TRANSLATION, SwerveConstants.KI_AUTO_TRANSLATION, SwerveConstants.KD_AUTO_TRANSLATION);
    addRequirements(swerveSub);
    pid.setTolerance(1.5);
  }

 @Override
  public void initialize() {
    desiredZ += swerveSub.getPose().getRotation().getDegrees();
  }

  @Override
  public void execute() {
    SwerveModuleState[] states;
    double zSpeed = pid.calculate(swerveSub.getPose().getRotation().getDegrees(), desiredZ);
    swerveSub.drive(0, 0, zSpeed,  true);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSub.stopModules();
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
    
  }
}
