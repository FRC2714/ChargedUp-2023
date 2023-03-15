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
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;
import frc.robot.subsystems.Arm.ArmStateMachine.ScoreLevel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class TwoCubeOpenAutoStop extends AutoBase {
	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"2CubeOPENAuto",
			new PathConstraints(
			2.4,
			2.0));

	public TwoCubeOpenAutoStop(DriveSubsystem m_robotDrive, ArmStateMachine m_armStateMachine, Intake m_intake, Arm m_arm, Claw m_claw, Limelight m_limelight) {
		super(m_robotDrive);

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();

    	AutoConstants.EventMap.put("intake cube", m_intake.intakeCube()
			.andThen(m_claw.intakeCubeCommand()));

		AutoConstants.EventMap.put("arm to front level 2", 
			m_armStateMachine.setTargetScoreLevelCommand(ScoreLevel.TWO)
			.andThen(m_armStateMachine.setTargetArmStateCommand(ArmState.FRONT))
			.andThen(new WaitCommand(0.5))
			.andThen(m_intake.retractAndStop()));

		addCommands(
			m_claw.intakeCubeCommand(),
			m_intake.deployCommand(),

			m_armStateMachine.setCargoTypeCommand(CargoType.CUBE),
			m_armStateMachine.setTargetScoreLevelCommand(ScoreLevel.THREE),
			m_armStateMachine.setTargetArmStateCommand(ArmState.BACK),
      		new WaitCommand(3.9),

			//Score First Cube
			m_claw.shootCube(),
      		new WaitCommand(0.2),
      		m_claw.stopOpen(),

			m_intake.deployCommand(),
      		m_armStateMachine.setTargetArmStateCommand(ArmState.TRANSFER),

			//Follow Path
			autoBuilder.fullAuto(autoPathGroup),

			//Score Second Cube
			new WaitCommand(2),
			m_claw.shootCube(),
      		new WaitCommand(0.5),
      		m_claw.stopOpen()
		);
	}
}