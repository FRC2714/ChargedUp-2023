// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EstimatePose extends InstantCommand {
  DriveSubsystem m_drivetrain;
  Limelight m_backLimelight;
  Limelight m_frontLimelight;

  public EstimatePose(DriveSubsystem m_drivetrain, Limelight m_backLimelight, Limelight m_frontLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_backLimelight = m_backLimelight;
    this.m_frontLimelight = m_frontLimelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if both limelight estimates are close, reset odometry
    m_drivetrain.resetOdometry(
      m_backLimelight.getBotPose2d());
  }
}
