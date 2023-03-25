// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCube extends SequentialCommandGroup {
  /** Creates a new CubeTransfer. */
  public IntakeCube(ArmStateMachine m_armStateMachine, Claw m_claw, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(m_armStateMachine, m_claw, m_intake);

    addCommands(
      m_armStateMachine.setCargoTypeCommand(CargoType.CUBE),
      //m_intake.intakeCube(),
      m_armStateMachine.setTargetArmStateCommand(ArmState.TRANSFER),
      m_claw.intakeOpenCommand(),
      new WaitCommand(4)
    );
  }
}
