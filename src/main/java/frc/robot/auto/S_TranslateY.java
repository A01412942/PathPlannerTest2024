package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class S_TranslateY extends Command {
  SwerveSubsystem swerveSub;
  PIDController pid;
  double desiredY;


  public S_TranslateY(SwerveSubsystem swerveSub, double desiredY) {
    this.swerveSub = swerveSub;
    this.desiredY = desiredY;
    pid = new PIDController(SwerveConstants.KP_AUTO_TRANSLATION, SwerveConstants.KI_AUTO_TRANSLATION, SwerveConstants.KD_AUTO_TRANSLATION);
    addRequirements(swerveSub);
    pid.setTolerance(0.03);
  }

 @Override
  public void initialize() {
    desiredY += swerveSub.getPose().getY();
  }

  @Override
  public void execute() {
    SwerveModuleState[] states;
    double ySpeed = pid.calculate(swerveSub.getPose().getY(), desiredY);
    swerveSub.drive(0, ySpeed, 0,  true);
    
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
