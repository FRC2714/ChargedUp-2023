// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  private BaseJoint basejoint = new BaseJoint();
  private BaseJoint secondjoint = new BaseJoint();//should be SecondJoint Class

  private double a1 = 31;//inches
  private double a2 = 31;

  private double q1; //units are wrong, convert to radians?? need to make them relative to ground
  private double q2;

  private double targetX;
  private double targetY;

  /** Creates a new Superstructure. */
  public Superstructure(double targetX, double targetY) {
    this.targetX = targetX;
    this.targetY = targetY;
  }

  public void setTargetPosition(double X, double Y) {
    targetX = X;
    targetY = Y;
  }

  //calcuate second joint angle 
  public void calculateQ2() {
    q2 = -Math.abs(Math.acos(
        ((targetX*targetX)+(targetY*targetY)-(a1*a1)-(a2*a2))
        /
        (2*a1*a2)
      )
    );
  }

  //calculate base joint angle
  public void calculateQ1() {
    q1 = Math.abs(
      Math.atan(targetX/targetY) +
      Math.atan(
        (a2*Math.sin(q2))
        /
        (a1+a2*Math.sin(q2))
      )
    );
  }

  public void calclulateAngles() {
    calculateQ1();
    calculateQ2();
  }

  public void moveJoints() {
    basejoint.setGoal(q1);
    secondjoint.setGoal(q2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("a1", a1);
    SmartDashboard.putNumber("a2", a2);


  }
}
