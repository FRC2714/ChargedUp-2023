package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;


public class Limelight extends SubsystemBase {

	private String limelightName = "limelight";
	private double kCameraHeight, kMountingAngle, GoalHeight = 0; //inches, deg

	public Limelight(String limelightName, Pose2d limelightPose) {
		this.limelightName = limelightName;
		this.kCameraHeight = limelightPose.getY();
		this.kMountingAngle = limelightPose.getRotation().getDegrees();
	}

	public double getDistanceToGoalInches() {
		return (GoalHeight - kCameraHeight) 
			/ Math.tan(
				Units.degreesToRadians(kMountingAngle + getYAngleOffsetDegrees()));
	}

	public void setGoalHeight(double GoalHeight) {
		this.GoalHeight = GoalHeight;
	}

	public double getGoalHeight() {
		return GoalHeight;
	}

	public double getDistanceToGoalMeters() {
		return Units.inchesToMeters(getDistanceToGoalInches());
	}

	public double getYAngleOffsetDegrees() {
		return LimelightHelpers.getTY(limelightName);
	}

	public double getXAngleOffsetDegrees() {
		return -1 * LimelightHelpers.getTX(limelightName); //must be negative
	}

	public double getXOffsetRadians() {
		return Units.degreesToRadians(getXAngleOffsetDegrees());
	}

	public boolean isTargetVisible() {
		return LimelightHelpers.getTV(limelightName);
	}

	public void setLED(boolean lightOn) {
        if (lightOn) LimelightHelpers.setLEDMode_ForceOn(limelightName); // LED force on
        else LimelightHelpers.setLEDMode_ForceOff(limelightName); // LED force off
    }

	//Back Limelight
	public void setRetroPipeline() {
		setGoalHeight(LimelightConstants.kMiddleRetroTapeHeight);
		LimelightHelpers.setPipelineIndex(limelightName, 0);
	}

	public void setAprilTagPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 1);
	}

	public void setAprilTagFarPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 2);
	}

	//Front Limelight
	public void setHighCubePipeline() {
		setGoalHeight(LimelightConstants.kCubeHighHeight);
		LimelightHelpers.setPipelineIndex(limelightName, 0);
	}

	public void setMiddleCubePipeline() {
		setGoalHeight(LimelightConstants.kCubeMiddleHeight);
		LimelightHelpers.setPipelineIndex(limelightName, 1);
	}

	public void setLowCubePipeline() {
		setGoalHeight(LimelightConstants.kCubeLowHeight);
		LimelightHelpers.setPipelineIndex(limelightName, 2);
	}

	public Command setLEDCommand(boolean lightOn) {
		return new InstantCommand(() -> setLED(lightOn));
	}

	public Pose2d getBotPose2d() {
		return LimelightHelpers.getBotPose2d(limelightName);
	}

	@Override
	public void periodic() {
	}
}