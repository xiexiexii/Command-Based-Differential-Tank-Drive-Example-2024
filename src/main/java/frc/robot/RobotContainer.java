// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// Lots of important stuff here! This is where the commands and subsystems come together
public class RobotContainer {
  
  // New Command Xbox Controller (Command is for Command Based Stuff)
  CommandXboxController m_driverController = new CommandXboxController(DriverConstants.k_driverControllerPort);
  
  // Subsystems
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  
  // Drive Commands
  private final ArcadeDriveCommand m_arcadeDefault = new ArcadeDriveCommand(m_driveSubsystem, 0.8, 0.6, m_driverController);
  private final TankDriveCommand m_tankDefault = new TankDriveCommand(m_driveSubsystem, 0.8, m_driverController);

  // Constructor
  public RobotContainer() {

    // Sets the drive command as the default command
    m_driveSubsystem.setDefaultCommand(m_arcadeDefault);

    // Binds buttons to commands
    configureButtonBindings();
  }

  // Bind buttons and stuff here!
  private void configureButtonBindings() {
  }

  // This is where you would put the PathPlanner stuff!
  public Command getAutonomousCommand() {
    return null;
  }
}