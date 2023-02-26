// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Autoalign;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
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
			"MarkerAuto",
			new PathConstraints(
			AutoConstants.kMaxSpeedMetersPerSecond,
			AutoConstants.kMaxAccelerationMetersPerSecondSquared));

	public TwoCargoOpenAuto(DriveSubsystem m_robotDrive, ArmStateMachine m_armStatemachine, Intake m_intake, Arm m_arm, Claw m_claw, Limelight m_limelight) {
		super(m_robotDrive);

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();
        AutoConstants.EventMap.put("auto align", new Autoalign(m_robotDrive, m_limelight));
		AutoConstants.EventMap.put("score cone", m_claw.score());
		AutoConstants.EventMap.put("set arm cone level 3", 
			m_armStatemachine.setTargetScoreLevelCommand(ScoreLevel.THREE).andThen(
			m_armStatemachine.setTargetArmStateCommand(ArmState.BACK)));
		AutoConstants.EventMap.put("arm to transfer", 
			m_armStatemachine.setTargetScoreLevelCommand(ScoreLevel.THREE).andThen(
			m_armStatemachine.setTargetArmStateCommand(ArmState.TRANSFER)));
		AutoConstants.EventMap.put("intake cone", m_intake.intakeCone());
		AutoConstants.EventMap.put("handoff cone", m_claw.intakeConeCommand());

		addCommands(
			autoBuilder.fullAuto(autoPathGroup)
		);

	}
}