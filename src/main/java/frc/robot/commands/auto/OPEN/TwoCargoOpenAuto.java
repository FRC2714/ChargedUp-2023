// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.OPEN;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.align.SmoothAlign;
import frc.robot.commands.auto.AutoBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;
import frc.robot.subsystems.Arm.ArmStateMachine.ScoreLevel;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class TwoCargoOpenAuto extends AutoBase {
	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"2CargoOPEN",
			new PathConstraints(
			2.5,
			2.5));

	public TwoCargoOpenAuto(DriveSubsystem m_robotDrive, ArmStateMachine m_armStateMachine, Intake m_intake, Arm m_arm, Claw m_claw, Limelight m_limelight) {
		super(m_robotDrive);

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();

    	AutoConstants.EventMap.put("intake cube", m_intake.deployAndIntake());

		AutoConstants.EventMap.put("retract intake", m_intake.pivotToHold());

		addCommands(
			m_claw.intakeCloseCommand(),

			m_armStateMachine.setCargoTypeCommand(CargoType.CONE),
			m_armStateMachine.setTargetScoreLevelCommand(ScoreLevel.THREE),
			m_armStateMachine.setTargetArmStateCommand(ArmState.BACK),
			new WaitCommand(2).raceWith(new SmoothAlign(m_robotDrive, m_limelight, m_armStateMachine)),
			new WaitCommand(3.8),

			//Score First Cube
			m_claw.scoreCone(),
      		m_armStateMachine.setTargetArmStateCommand(ArmState.TRANSFER),
			new WaitCommand(1.25),

			//Follow Path
			autoBuilder.fullAuto(autoPathGroup),

			//Score Second Cube
			m_intake.pivotThenShoot()
		);
	}
}