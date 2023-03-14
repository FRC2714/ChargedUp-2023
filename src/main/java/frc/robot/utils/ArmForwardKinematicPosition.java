// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmForwardKinematicPosition {
    private final double baseAngleRadians;
    private final double secondAngleRadians;

    public ArmForwardKinematicPosition(double baseAngleDegrees, double secondAngleDegrees) {
      this.baseAngleRadians = Units.degreesToRadians(baseAngleDegrees);
      this.secondAngleRadians = Units.degreesToRadians(secondAngleDegrees);
    }

    public double getBaseAngleDegrees() {
        return Units.radiansToDegrees(baseAngleRadians);
    }

    public double getSecondAngleDegrees() {
        return Units.radiansToDegrees(secondAngleRadians);
    }

    public double getBaseAngleRadians() {
        return baseAngleRadians;
    }

    public double getSecondAngleRadians() {
        return secondAngleRadians;
    }
}
