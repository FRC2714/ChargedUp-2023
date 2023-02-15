// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class PPTestAuto3 extends AutoBase {
	DriveSubsystem drivetrain;

	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"SCurve",
			new PathConstraints(
			AutoConstants.kMaxSpeedMetersPerSecond,
			AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    
	SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drivetrain::getPose, // Pose2d supplier
        drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        drivetrain::setModuleStates,
        AutoConstants.EventMap,
        true,
        drivetrain
    );

	/** Creates a new TestAuto. */
	public PPTestAuto3(DriveSubsystem drivetrain) {
		super(drivetrain);
		AutoConstants.EventMap.put("setX", new InstantCommand(drivetrain::setX, drivetrain));

		addCommands(
			autoBuilder.fullAuto(autoPathGroup)
		);

	}
}