// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlignX extends ProfiledPIDCommand {
  private DriveSubsystem m_robotDrive;
  private Limelight m_limelight;
  
  /** Creates a new Autoalign. */
  public AutoAlignX(DriveSubsystem m_robotDrive, Limelight m_limelight) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.7,
            0,
            0,
            // The motion profile constraints
            AutoConstants.kAutoControllerConstraints),
        // This should return the measurement
        m_limelight::getDistanceToGoal,
        // This should return the goal (can also be a constant)
        0.35, // distance from limelight to pole TODO
        // This uses the output
        (output, setpoint) -> m_robotDrive.drive(output, 0, 0, true, false)
          // Use the output (and setpoint, if desired) here
        );
        addRequirements(m_robotDrive);
        this.m_limelight = m_limelight;
        this.m_robotDrive = m_robotDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.07,0);

  }

  public void initialize() {
    m_limelight.setLED(true);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_limelight.setLED(false);
    m_robotDrive.drive(0,0,0, true, false);
    System.out.println("Auto Align X finished");
  }
}
