package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.LimelightHelpers;


public class Limelight extends SubsystemBase {
	private String limelightName = "limelight";

	public Limelight() {}

	public double getDistanceToGoalMeters() {
		return (FieldConstants.kGoalHeight - CameraConstants.kCameraHeight)/Math.tan(Units.degreesToRadians(CameraConstants.kMountingAngle + getYAngleOffsetDegrees()));
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

	public void setRetroPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 0);
	}

	public void setAprilTagPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 1);
	}

	public void setAprilTagFarPipeline() {
		LimelightHelpers.setPipelineIndex(limelightName, 2);
	}

	public Command setLEDCommand(boolean lightOn) {
		return new InstantCommand(() -> setLED(lightOn));
	}

	public Pose2d getBotPose() {
		return LimelightHelpers.getBotPose2d(limelightName);
	}

	

	@Override
	public void periodic() {
		SmartDashboard.putNumber("distance to goal", getDistanceToGoalMeters());
	}
}