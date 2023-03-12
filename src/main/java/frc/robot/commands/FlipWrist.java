// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlipWrist extends SequentialCommandGroup {
  private double finalTargetAngleDegrees = 90;
  /** Creates a new FlipWrist. */
  public FlipWrist(Wrist m_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if ((m_wrist.getAngleDegrees() < 90) || (m_wrist.getAngleDegrees() > 270)) { //when wrist is near 0
      finalTargetAngleDegrees = 180;
    } else if (((m_wrist.getAngleDegrees() > 90) || (m_wrist.getAngleDegrees() < 270))) {
      finalTargetAngleDegrees = 0;
    }

    addCommands(
      new PrintCommand("final target degrees" + finalTargetAngleDegrees),
      new WaitCommand(1).raceWith(new TurnWristToAngle(m_wrist, 90)),
      new WaitCommand(2).raceWith(new TurnWristToAngle(m_wrist, finalTargetAngleDegrees))
    );
  }
}
