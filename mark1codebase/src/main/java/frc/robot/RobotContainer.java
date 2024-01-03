// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
 
    
  private final Swerve swerveSubsystem = new Swerve();

  private final XboxController m_joystick = new XboxController(0);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(
      swerveSubsystem,
      () -> m_joystick.getLeftY(),
      () -> -m_joystick.getLeftX(),
      () -> -m_joystick.getRightX(),
      () -> !m_joystick.getAButtonPressed()));

      configureButtonBindings();

    }

   private void configureButtonBindings() {
    if (m_joystick.getBButtonPressed()) {

      swerveSubsystem.zeroHeading();
      
    }

   }
  
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //Trajectory Settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond).setKinematics(DriveConstants.kDriveKinematics); 
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0,0, new Rotation2d(0)),
          List.of(new Translation2d(1, 0), new Translation2d(1, -1)), 
          new Pose2d(2, -1, Rotation2d.fromDegrees(180)), trajectoryConfig);
    
    // PID
    PIDController xController = new PIDController(DriveConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(DriveConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(DriveConstants.kPThetaController, 0, 0, DriveConstants.kThetaControllerConstraints);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModuleState,
            swerveSubsystem);
            
          
          return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetPose(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> swerveSubsystem.stopModules())
          );



  }
}
