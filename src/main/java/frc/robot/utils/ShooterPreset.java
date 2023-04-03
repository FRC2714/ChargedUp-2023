// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class ShooterPreset {
    private final double pivotAngleDegrees;
    private final double VelocityRPM;

    public ShooterPreset(double pivotAngleDegrees, double VelocityRPM) {
      this.pivotAngleDegrees = pivotAngleDegrees;
      this.VelocityRPM = VelocityRPM;
    }

    public double getPivotAngleDegrees() {
        return pivotAngleDegrees;
    }

    public double getVelocityRPM() {
        return VelocityRPM;
    }
}
