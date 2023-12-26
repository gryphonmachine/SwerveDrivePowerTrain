// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

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
    return null;
  }
}
