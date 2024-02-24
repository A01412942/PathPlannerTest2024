package frc.robot.testing;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightRotationAlignCommand extends Command {
  private SwerveSubsystem swerveSubs; 
  private DoubleSupplier xSupp, ySupp, zSupp; 
  private PIDController rotationPID; 

  public LimelightRotationAlignCommand(SwerveSubsystem swerveSubs, DoubleSupplier xSupp, DoubleSupplier ySupp, DoubleSupplier zSupp) {
    this.swerveSubs = swerveSubs; 
    this.xSupp = xSupp; 
    this.ySupp = ySupp; 
    this.zSupp = zSupp; 
    rotationPID = new PIDController(0.005, 0, 0);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed;

    if(LimelightHelpersClass.getTV("limelight")){
      rotationSpeed = rotationPID.calculate(LimelightHelpersClass.getTX("limelight"), 0);
    }
    else{
      rotationSpeed = deadzone(zSupp.getAsDouble()); 
    }

    swerveSubs.drive(deadzone(xSupp.getAsDouble()), deadzone(ySupp.getAsDouble()), rotationSpeed, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubs.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
}
}
