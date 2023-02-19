// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cleanTransfer extends SequentialCommandGroup {
  Arm m_arm;
  /** Creates a new cleanTransfer. */
  public cleanTransfer(Arm m_arm) {
    addRequirements(m_arm);

    addCommands(
      new WaitUntilCommand(() -> m_arm.baseJointAtSetpoint()).deadlineWith(m_arm.swingOut()),
      new WaitUntilCommand(() -> m_arm.baseJointAtSetpoint()).deadlineWith(m_arm.swingOut2()),
      new WaitUntilCommand(() -> m_arm.secondJointAtSetpoint()).deadlineWith(m_arm.transfer())
    );
  }
}
