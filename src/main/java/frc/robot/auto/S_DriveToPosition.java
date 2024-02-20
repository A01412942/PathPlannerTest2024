package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class S_DriveToPosition extends Command {
  SwerveSubsystem swerveSub;
  PIDController xPID, yPID, zPID;
  double desiredX, desiredY, desiredZ, translationTollerance;
  boolean rotation;


  public S_DriveToPosition(SwerveSubsystem swerveSub, double desiredX, double desiredY, double desiredZ, double translationTollerance, boolean rotation) {
    this.swerveSub = swerveSub;
    this.desiredX = desiredX;
    this.desiredY = desiredY;
    this.desiredZ = desiredZ;
    this.translationTollerance = translationTollerance;
    this.rotation = rotation;
    xPID = new PIDController(SwerveConstants.KP_AUTO_TRANSLATION, SwerveConstants.KI_AUTO_TRANSLATION, SwerveConstants.KD_AUTO_TRANSLATION);
    yPID = new PIDController(SwerveConstants.KP_AUTO_TRANSLATION, SwerveConstants.KI_AUTO_TRANSLATION, SwerveConstants.KD_AUTO_TRANSLATION);
    zPID = new PIDController(SwerveConstants.KP_AUTO_ROTATION, SwerveConstants.KP_AUTO_ROTATION, SwerveConstants.KD_AUTO_ROTATION);
    zPID.enableContinuousInput(0, 360);
    xPID.setTolerance(translationTollerance);
    yPID.setTolerance(translationTollerance);
    zPID.setTolerance(SwerveConstants.ROTATION_TOLLERANCE);

    addRequirements(swerveSub);
  }

  public S_DriveToPosition(SwerveSubsystem swerveSub, double desiredX, double desiredY, double desiredZ, boolean rotation){
    this(swerveSub, desiredX, desiredY, desiredZ, SwerveConstants.TRANSLATION_TOLLERANCE, rotation);
  }
     
  @Override
  public void initialize() {
    SmartDashboard.putString("IS DONE", "NO");
  }

  @Override
  public void execute() {
    SmartDashboard.putString("CURRENT COMMAND", getName());

    double xSped = xPID.calculate(swerveSub.getPose().getX(), desiredX);
    double ySpeed = yPID.calculate(swerveSub.getPose().getY(), desiredY);
    double zSpeed = !rotation ? 0 : zPID.calculate((((swerveSub.getRotation2d().getDegrees() % 360) + 360) % 360), desiredZ);
    // zSpeed = 0.01/zSpeed; 
      swerveSub.drive(xSped, ySpeed, zSpeed, true);
  
    
    SmartDashboard.putNumber("xSped", xSped);
    SmartDashboard.putNumber("ySped", ySpeed);
    SmartDashboard.putNumber("zSped", zSpeed);

    SmartDashboard.putBoolean("x at setpt", xPID.atSetpoint());
    SmartDashboard.putBoolean("y at setpt", yPID.atSetpoint());
    SmartDashboard.putBoolean("z at setpt", zPID.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("IS DONE", "YES");
    swerveSub.stopModules();
  }

  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint() && (zPID.atSetpoint() || !rotation);
  }
}
