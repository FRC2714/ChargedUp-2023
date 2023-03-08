package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Limelight extends SubsystemBase {

	private NetworkTable limelight;
	private static final double kCameraToGoalHeight = Constants.FieldConstants.kGoalHeight - Constants.CameraConstants.kCameraHeight;

	public Limelight() {
		limelight = NetworkTableInstance.getDefault().getTable("limelight");
	}

	private double internalGetDistanceToGoal() {
		if (getYAngleOffsetDegrees() == -1) return -1;
		return kCameraToGoalHeight / Math.tan(Math.toRadians(Constants.CameraConstants.kMountingAngle + getYAngleOffsetDegrees()));
	}

	public double getDistanceToGoal() {
		return internalGetDistanceToGoal();
	}

	public double getYAngleOffsetDegrees() {
		return limelight.getEntry("ty").getDouble(-1);
	}

	public double getXAngleOffsetDegrees() {
		return -1 * limelight.getEntry("tx").getDouble(-1); //must be negative
	}

	public double getXOffsetRadians() {
		return Units.degreesToRadians(getXAngleOffsetDegrees());
	}

	public boolean targetVisible() {
		return limelight.getEntry("tv").getDouble(0.0) != 0.0;
	}

	public void setPipeline(double numPipeline) {
		limelight.getEntry("pipeline").setDouble(numPipeline);
	}

	public void setLED(boolean lightOn) {
        if (lightOn) limelight.getEntry("ledMode").setDouble(3); // LED force on
        else limelight.getEntry("ledMode").setDouble(1); // LED force off
    }

	public Command setLEDCommand(boolean lightOn) {
		return new InstantCommand(() -> setLED(lightOn));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("distance to goal", getDistanceToGoal());
	}
}