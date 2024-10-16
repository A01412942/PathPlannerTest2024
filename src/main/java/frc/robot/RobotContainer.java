package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.auto.S_DriveToPosition;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.testing.L_LimelightStrafeAlignCmd;
import frc.robot.testing.LimelightRotationAlignCommand;
import frc.robot.commands.S_DriveCommand;
import edu.wpi.first.wpilibj.PS4Controller;

public class RobotContainer {
  //SUBSYSTEMS 
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 

  //CONTROLLERS  
  //private final XboxController xbox = new XboxController(ControllerConstants.kDriverControllerPort);
  private final PS4Controller control = new PS4Controller(ControllerConstants.kDriverControllerPort);

  //DRIVE BUTTONS 
  //private final JoystickButton resetNavxButton = new JoystickButton(xbox, XboxController.Button.kA.value); 
  private final JoystickButton resetNavxButton = new JoystickButton(control, 10); 

 private final JoystickButton resetPosButton = new JoystickButton(control, 9);
 // private final JoystickButton limelightStrafeAlign = new JoystickButton(xbox, XboxController.Button.kX.value);

  //AXIS 
  //private final int joystickAxis = XboxController.Axis.kRightY.value;
  private final int joystickAxis = PS4Controller.Axis.kRightY.value;


  public RobotContainer() {
    
    //swerveSubs.setDefaultCommand(new S_DriveCommand(swerveSubs, () -> -.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX(), true));
    swerveSubs.setDefaultCommand(new S_DriveCommand(swerveSubs, () -> -control.getLeftY(), () -> -control.getLeftX(), () -> -control.getRightX(), false));//desactivamos fod para pr
    //CAMBIAR SIGNOS OTRA VEZ Y PONER BIEN LOS OFFSETS
    
    // shooter.setDefaultCommand(new Sh_JoystickControlCommand(shooter, () -> xbox.getRawAxis(joystickAxis) * 0.9));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    resetNavxButton.onTrue(new InstantCommand(() -> swerveSubs.resetNavx()));

    resetPosButton.onTrue(new InstantCommand(() -> swerveSubs.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));
  //  limelightStrafeAlign.onTrue(new LimelightRotationAlignCommand(swerveSubs, () -> -xbox.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX()));
    
  }

  public Command getAutonomousCommand() {
    // return null; 
    // An example command will be run in autonomous
    //return new PathPlannerAuto("New New Auto");
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
    // return new S_TranslateX(swerveSubs, 1.0);
    //return new S_TranslateY(swerveSubs, 1.0);
    //return new S_TranslateZ(swerveSubs, 180.0);

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubs.resetNavx()), 

      new InstantCommand(() -> swerveSubs.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))), 

      

      // new S_DriveToPosition(swerveSubs, 0, Units.feetToMeters(5), 0), //new S_TranslateX(swerveSubs, 1.0)

      new S_DriveToPosition(swerveSubs, -Units.feetToMeters(5), Units.feetToMeters(5), 335, true),

       new S_DriveToPosition(swerveSubs, -Units.feetToMeters(5), 0, 0, .05, false),

       new S_DriveToPosition(swerveSubs, -Units.feetToMeters(22), 0, 0, .05, true)
    
    );

    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> swerveSubs.resetNavx()), 

    //   new InstantCommand(() -> swerveSubs.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))), 

    //   new S_DriveToPosition(swerveSubs, 0, Units.feetToMeters(5), 0)
    // );

    // return AutoBuilder.followPath(path);
  }

}
