// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.align.AutoAlignY;
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

public class TwoL3ConeBalanceOpenAuto extends AutoBase {
	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"2L3ConeBalanceOpen",
			new PathConstraints(
			AutoConstants.kMaxSpeedMetersPerSecond,
			AutoConstants.kMaxAccelerationMetersPerSecondSquared));

	public TwoL3ConeBalanceOpenAuto(DriveSubsystem m_robotDrive, ArmStateMachine m_armStateMachine, Intake m_intake, Arm m_arm, Claw m_claw, Limelight m_limelight) {
		super(m_robotDrive);

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();
		AutoConstants.EventMap.put("arm to tuck", m_armStateMachine.setTargetArmStateCommand(ArmState.STOW));
		AutoConstants.EventMap.put("intake cone", m_intake.intakeCone());
		AutoConstants.EventMap.put("retract and stop intake", m_intake.retractAndStop());
		//AutoConstants.EventMap.put("cone transfer", new ConeTransfer());

        AutoConstants.EventMap.put("auto align", new AutoAlignY(m_robotDrive, m_limelight).raceWith(new WaitCommand(0.4)));
		AutoConstants.EventMap.put("arm to cone level 3", 
			m_armStateMachine.setTargetScoreLevelCommand(ScoreLevel.THREE).andThen(
			m_armStateMachine.setTargetArmStateCommand(ArmState.BACK)).withTimeout(3.0));
		AutoConstants.EventMap.put("score cone", m_claw.scoreCone());
		AutoConstants.EventMap.put("zero heading", new TurnToAngle(m_robotDrive, 0).raceWith(new WaitCommand(0.3)));

		addCommands(
			m_claw.intakeCloseCommand(),
			new AutoAlignY(m_robotDrive, m_limelight).raceWith(new WaitCommand(0.4)),
			m_armStateMachine.setTargetScoreLevelCommand(ScoreLevel.THREE),
			m_armStateMachine.setTargetArmStateCommand(ArmState.BACK).withTimeout(3.0),
			m_claw.scoreCone(),
			autoBuilder.fullAuto(autoPathGroup),
			new AutoBalance(m_robotDrive)
		);

	}
}