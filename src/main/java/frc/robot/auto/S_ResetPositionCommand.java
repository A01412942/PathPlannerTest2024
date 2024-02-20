package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class S_ResetPositionCommand extends Command {
  SwerveSubsystem swerveSub;
  public S_ResetPositionCommand(SwerveSubsystem swerveSub) {
    this.swerveSub = swerveSub;
    addRequirements(swerveSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerveSub.resetOdometry(new Pose2d());
    swerveSub.resetNavx();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
