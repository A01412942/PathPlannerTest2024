package frc.robot.auto;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class S_DriveToPosition extends Command {
  SwerveSubsystem swerveSub;
  PIDController xPID, yPID, zPID;
  double desiredX, desiredY, desiredZ;


  public S_DriveToPosition(SwerveSubsystem swerveSub, double desiredX, double desiredY, double desiredZ) {
    this.swerveSub = swerveSub;
    this.desiredX = desiredX;
    this.desiredY = desiredY;
    this.desiredZ = desiredZ;
    xPID = new PIDController(SwerveConstants.KP_AUTO_TRANSLATION, SwerveConstants.KI_AUTO_TRANSLATION, SwerveConstants.KD_AUTO_TRANSLATION);
    yPID = new PIDController(SwerveConstants.KP_AUTO_TRANSLATION, SwerveConstants.KI_AUTO_TRANSLATION, SwerveConstants.KD_AUTO_TRANSLATION);
    zPID = new PIDController(SwerveConstants.KP_AUTO_ROTATION, SwerveConstants.KP_AUTO_ROTATION, SwerveConstants.KD_AUTO_ROTATION);
    xPID.setTolerance(SwerveConstants.TRANSLATION_TOLLERANCE);
    yPID.setTolerance(SwerveConstants.TRANSLATION_TOLLERANCE);
    zPID.setTolerance(SwerveConstants.ROTATION_TOLLERANCE);

    addRequirements(swerveSub);


  }
  @Override
  public void initialize() {
    
  }


  @Override
  public void execute() {
    SmartDashboard.putString("CURRENT COMMAND", getName());

    double xSped = xPID.calculate(swerveSub.getPose().getX(), desiredX);
    double ySpeed = yPID.calculate(swerveSub.getPose().getY(), desiredY);
    double zSpeed = zPID.calculate(swerveSub.getRotation2d().getDegrees(), desiredZ);
    
    swerveSub.drive(xSped, ySpeed, zSpeed, true);
    
    SmartDashboard.putNumber("xSped", xSped);
    SmartDashboard.putNumber("ySped", ySpeed);
    SmartDashboard.putNumber("zSped", zSpeed);

  }

  @Override
  public void end(boolean interrupted) {
    swerveSub.stopModules();
  }

  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint() && zPID.atSetpoint();
  }
}
