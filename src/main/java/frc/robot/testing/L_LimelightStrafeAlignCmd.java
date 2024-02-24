// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.testing.LimelightHelpersClass;
import frc.robot.subsystems.SwerveSubsystem;

public class L_LimelightStrafeAlignCmd extends Command {
  private SwerveSubsystem swerveSubs;
  private PIDController strafepid;


  public L_LimelightStrafeAlignCmd(SwerveSubsystem swerveSubsystem) {
    swerveSubs = swerveSubsystem;

    strafepid = new PIDController(0.005, 0, 0);
    addRequirements(swerveSubs);
  }

  @Override
  public void initialize(){

  }


  @Override
  public void execute(){
    double strafeSpeed = strafepid.calculate(LimelightHelpersClass.getTX("limelight"), 0);

    if(LimelightHelpersClass.getTV("limelight")){
      swerveSubs.drive(0, 0, strafeSpeed, false);
    }
    else{
      swerveSubs.stopModules();
    }

  }

  @Override
  public void end(boolean interrupted){
    swerveSubs.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}