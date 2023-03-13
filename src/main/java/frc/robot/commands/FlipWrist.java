// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlipWrist extends SequentialCommandGroup {
  private double finalTargetAngleDegrees;
  private double wristAngle;
  /** Creates a new FlipWrist. */
  public FlipWrist(Wrist m_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    wristAngle = SmartDashboard.getNumber("Wrist Angle Degrees", 180);
    if ((wristAngle >= 0 && wristAngle < 90) || (wristAngle <= 360 && wristAngle > 270)) { //when wrist is near 0
      finalTargetAngleDegrees = 90;
    } else {
      finalTargetAngleDegrees = 270;
    }

    addCommands(
      new PrintCommand("wrist angle" + wristAngle),
      new WaitCommand(0.2).raceWith(new TurnWristToAngle(m_wrist, 90)),
      new WaitCommand(1.5).raceWith(new TurnWristToAngle(m_wrist, finalTargetAngleDegrees))
    );
  }
}
