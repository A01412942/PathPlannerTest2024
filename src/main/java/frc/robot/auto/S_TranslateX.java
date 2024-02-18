package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;


public class S_TranslateX extends Command {
  SwerveSubsystem swerveSub;
  PIDController pid;
  double desiredX;


  public S_TranslateX(SwerveSubsystem swerveSub, double desiredX) {
    this.swerveSub = swerveSub;
    this.desiredX = desiredX;
    pid = new PIDController(SwerveConstants.KP_AUTO_TRANSLATION, SwerveConstants.KI_AUTO_TRANSLATION, SwerveConstants.KD_AUTO_TRANSLATION);
    addRequirements(swerveSub);
    pid.setTolerance(0.01);
  }

 @Override
  public void initialize() {
    desiredX += swerveSub.getPose().getX();
    // swerveSub.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  @Override
  public void execute() {
    SmartDashboard.putString("CURRENT CMD", getName());
    SwerveModuleState[] states;
    double xSpeed = pid.calculate(swerveSub.getPose().getX(), desiredX);
    swerveSub.drive(xSpeed, 0, 0,  true);
    
    SmartDashboard.putNumber("DESIRED X", desiredX);
    SmartDashboard.putNumber("SPEED", xSpeed);
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
