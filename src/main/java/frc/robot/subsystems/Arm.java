// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
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
  public Arm() {

  }

  public void setFowardKinematics(double baseAngle, double secondAngle) {
    basejoint.setTargetKinematicAngle(baseAngle);
    secondjoint.setTargetKinematicAngle(secondAngle);
  }

  public void setTargetPosition(double X, double Y) {
    targetX = X;
    targetY = Y;
  }

  public void calculateQ2() {
    q2 = -Math.abs(Math.acos(
        ((targetX*targetX)+(targetY*targetY)-(a1*a1)-(a2*a2)) /
        (2*a1*a2)));
  }

  public void calculateQ1() {
    q1 = Math.abs(
      Math.atan(targetX/targetY) +
      Math.atan((a2*Math.sin(q2)) / (a1+a2*Math.sin(q2))));
  }

  public void calculateInverseKinematics() {
    calculateQ1();
    calculateQ2();
  }

  public void setInverseKinematics() {
    basejoint.setTargetKinematicAngle(q1);
    secondjoint.setTargetKinematicAngle(q2);
  }

  public void estimateCurrentXY() {
    estimatedX = a1*Math.cos(basejoint.getKinematicAngle()) + a2*Math.cos(basejoint.getKinematicAngle()+secondjoint.getKinematicAngle());
    estimatedY = a2*Math.sin(secondjoint.getKinematicAngle()) + a2*Math.sin(basejoint.getKinematicAngle()+secondjoint.getKinematicAngle());
    SmartDashboard.putNumber("Estimated X", Units.metersToInches(estimatedX));
    SmartDashboard.putNumber("Estimated Y", Units.metersToInches(estimatedY));
  }

  public boolean baseJointAtSetpoint() {
    return basejoint.atSetpoint();
  }

  public boolean secondJointAtSetpoint() {
    return secondjoint.atSetpoint();
  }

  public Command swingOut() {
    return new InstantCommand(() -> setFowardKinematics(Units.degreesToRadians(53), Units.degreesToRadians(150)));
  }

  public Command swingOut2() {
    return new InstantCommand(() -> setFowardKinematics(Units.degreesToRadians(75), Units.degreesToRadians(145)));
  }

  public Command transfer() {
    return new InstantCommand(() -> setFowardKinematics(Units.degreesToRadians(93), Units.degreesToRadians(150)));
  }

  public Command intermediatePosition() {
    return new InstantCommand(() -> setFowardKinematics(Units.degreesToRadians(120), Units.degreesToRadians(-90)));
  }

  public Command scoreConeLevelTwo() {
    return new InstantCommand(() -> setFowardKinematics(Units.degreesToRadians(90), Units.degreesToRadians(-102)));
  }

  public Command scoreConeLevelThree() {
    return new InstantCommand(() -> setFowardKinematics(Units.degreesToRadians(52), Units.degreesToRadians(-31)));
  }

  public Command swingOutLevelTwo() {
    return 
      Commands.waitUntil(() -> basejoint.atSetpoint())
      .deadlineWith(swingOut())
      .andThen(scoreConeLevelTwo());
  }
  
  public Command scoreToTransfer() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJointAtSetpoint() && secondJointAtSetpoint()).deadlineWith(swingOut()),
      new WaitUntilCommand(() -> baseJointAtSetpoint()).deadlineWith(swingOut2()),
      new WaitUntilCommand(() -> secondJointAtSetpoint()).deadlineWith(transfer())
    );
  }
  
  public Command transferToLevelThree() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJointAtSetpoint() && secondJointAtSetpoint()).deadlineWith(transfer()),
      new WaitUntilCommand(() -> baseJointAtSetpoint() && secondJointAtSetpoint()).deadlineWith(swingOut()),
      new WaitUntilCommand(() -> baseJointAtSetpoint()).deadlineWith(intermediatePosition()),
      new WaitUntilCommand(() -> baseJointAtSetpoint()).deadlineWith(scoreConeLevelThree())
    );
  }

  public Command transferToLevelTwo() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJointAtSetpoint() && secondJointAtSetpoint()).deadlineWith(transfer()),
      new WaitUntilCommand(() -> baseJointAtSetpoint() && secondJointAtSetpoint()).deadlineWith(swingOut()),
      new WaitUntilCommand(() -> baseJointAtSetpoint()).deadlineWith(intermediatePosition()),
      new WaitUntilCommand(() -> baseJointAtSetpoint()).deadlineWith(scoreConeLevelTwo())
    );
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculateInverseKinematics();
    estimateCurrentXY();
  }
}
