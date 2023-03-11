package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;


public class Limelight extends SubsystemBase {

	private static final double kCameraToGoalHeight = Constants.FieldConstants.kGoalHeight - Constants.CameraConstants.kCameraHeight;

	private String defaultPipeline = "";
	public Limelight() {
	}

	private double internalGetDistanceToGoal() {
		if (getYAngleOffsetDegrees() == -1) return -1;
		return kCameraToGoalHeight / Math.tan(Math.toRadians(Constants.CameraConstants.kMountingAngle + getYAngleOffsetDegrees()));
	}

	public double getDistanceToGoal() {
		return internalGetDistanceToGoal();
	}

	public double getYAngleOffsetDegrees() {
		return LimelightHelpers.getTY("");
	}

	public double getXAngleOffsetDegrees() {
		return -1 * LimelightHelpers.getTX(""); //must be negative
	}

	public double getXOffsetRadians() {
		return Units.degreesToRadians(getXAngleOffsetDegrees());
	}

	public boolean targetVisible() {
		return LimelightHelpers.getTV("");
	}

	public void setPipeline(String pipelineName) {
		LimelightHelpers.setPipelineIndex("", 0);
	}

	public void setLED(boolean lightOn) {
        if (lightOn) LimelightHelpers.setLEDMode_ForceOn(""); // LED force on
        else LimelightHelpers.setLEDMode_ForceOff(""); // LED force off
    }

	public Command setLEDCommand(boolean lightOn) {
		return new InstantCommand(() -> setLED(lightOn));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("distance to goal", getDistanceToGoal());
		SmartDashboard.putNumber("x offset degrees", getXAngleOffsetDegrees());
	}
}