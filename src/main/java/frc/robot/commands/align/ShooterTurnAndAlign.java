// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterTurnAndAlign extends SequentialCommandGroup {
  /** Creates a new ShooterTurnAndAlign. */
  public ShooterTurnAndAlign(DriveSubsystem m_robotDrive, Limelight m_fronLimelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurnToAngle(m_robotDrive, 180),
      new ShooterAlignToCube(m_robotDrive, m_fronLimelight)
    );
  }
}
