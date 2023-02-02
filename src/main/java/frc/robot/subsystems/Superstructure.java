// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class Superstructure extends SubsystemBase {
  private BaseJoint basejoint = new BaseJoint();
  private SecondJoint secondjoint = new SecondJoint();

  private double a1 = ArmConstants.kBaseJointLength;//meters
  private double a2 = ArmConstants.kSecondJointLength;

  private double q1; //units are wrong, convert to radians?? need to make them relative to ground
  private double q2;

  private double targetX;
  private double targetY;

  private double estimatedX;
  private double estimatedY;

  /** Creates a new Superstructure. */
  public Superstructure() {

  }

  public void setFowardKinematics(double baseAngle, double secondAngle) {
    basejoint.setTarget(baseAngle);
    secondjoint.setTarget(secondAngle);
  }

  public void setTargetPosition(double X, double Y) {
    targetX = X;
    targetY = Y;
  }

  public void calculateQ2() {
    q2 = -Math.abs(Math.acos(
        ((targetX*targetX)+(targetY*targetY)-(a1*a1)-(a2*a2))
        /
        (2*a1*a2)
      ));
  }

  public void calculateQ1() {
    q1 = Math.abs(
      Math.atan(targetX/targetY) +
      Math.atan(
        (a2*Math.sin(q2))
        /
        (a1+a2*Math.sin(q2))
      ));
  }

  public void calculateInverseKinematics() {
    calculateQ1();
    calculateQ2();
  }

  public void setInverseKinematics() {
    basejoint.setTarget(q1);
    secondjoint.setTarget(q2);
  }

  public void estimateCurrentXY() {
    estimatedX = a1*Math.cos(basejoint.getAngle()) + a2*Math.cos(basejoint.getAngle()+(secondjoint.getAngle()));
    estimatedY = a2*Math.sin(secondjoint.getAngle()) + a2*Math.sin(basejoint.getAngle()+(secondjoint.getAngle()));
    SmartDashboard.putNumber("Estimated X", Units.metersToInches(estimatedX));
    SmartDashboard.putNumber("Estimated Y", Units.metersToInches(estimatedY));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("BaseJoint Target Angle", Units.radiansToDegrees(q1));
    SmartDashboard.putNumber("SecondJoint Target Angle", Units.radiansToDegrees(q2));
    calculateInverseKinematics();
    estimateCurrentXY();
  }
}
