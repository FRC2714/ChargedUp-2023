// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.utils.ArmForwardKinematicPosition;

public class Arm extends SubsystemBase {
  private BaseJoint baseJoint = new BaseJoint();
  private SecondJoint secondJoint = new SecondJoint();

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
    baseJoint.setTargetKinematicAngle(baseAngle);
    secondJoint.setTargetKinematicAngle(secondAngle);
  }

  public void setFowardKinematics(ArmForwardKinematicPosition forwardKinematicsPosition) {
    baseJoint.setTargetKinematicAngle(forwardKinematicsPosition.getBaseAngleRadians());
    secondJoint.setTargetKinematicAngle(forwardKinematicsPosition.getSecondAngleRadians());
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
    baseJoint.setTargetKinematicAngle(q1);
    secondJoint.setTargetKinematicAngle(q2);
  }

  public void estimateCurrentXY() {
    estimatedX = a1*Math.cos(baseJoint.getKinematicAngle()) + a2*Math.cos(baseJoint.getKinematicAngle()+secondJoint.getKinematicAngle());
    estimatedY = a2*Math.sin(secondJoint.getKinematicAngle()) + a2*Math.sin(baseJoint.getKinematicAngle()+secondJoint.getKinematicAngle());
    SmartDashboard.putNumber("Estimated X", Units.metersToInches(estimatedX));
    SmartDashboard.putNumber("Estimated Y", Units.metersToInches(estimatedY));
  }

  public Command swingOut() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kSwingOutPosition));
  }

  public Command swingOut2() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kSwingOut2Position));
  }

  public Command transfer() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferPosition));
  }

  public Command intermediatePosition() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kIntermediatePosition));
  }

  public Command scoreConeLevelTwo() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kScoreConeLevelTwoPosition));
  }

  public Command scoreConeLevelThree() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kScoreConeLevelThreePosition));
  }
  
  public Command scoreToTransfer() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(swingOut()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(swingOut2()),
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(transfer())
    );
  }
  
  public Command transferToLevelThree() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(transfer()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(swingOut()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(intermediatePosition()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(scoreConeLevelThree())
    );
  }

  public Command transferToLevelTwo() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(transfer()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(swingOut()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(intermediatePosition()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(scoreConeLevelTwo())
    );
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculateInverseKinematics();
    estimateCurrentXY();
  }
}
