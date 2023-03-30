// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Superstructure.CargoType;
import frc.robot.utils.ArmForwardKinematicPosition;

public class ArmStateMachine {
  private Arm m_arm;
  
  public enum ArmState {
    BACK, TRANSFER, FRONT, STOW
  }

  public enum ArmScoreLevel {
    THREE, TWO, ONE, INTAKE
  }

  public ArmState currentArmState = ArmState.BACK; //will default to TRANSFER 
  public ArmState targetArmState = ArmState.BACK; // default to TRANSFER 
  public ArmScoreLevel armScoreLevel = ArmScoreLevel.INTAKE;

  /** Creates a new StateMachine. */
  public ArmStateMachine(Arm m_arm) {
    this.m_arm = m_arm;
  }

  //Score level
  public InstantCommand setArmScoreLevelCommand(ArmScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> setArmScoreLevel(targetScoreLevel));
  }

  public void setArmScoreLevel(ArmScoreLevel scoreLevel) {
    this.armScoreLevel = scoreLevel;
  }

  public ArmScoreLevel getArmScoreLevel() {
    return this.armScoreLevel;
  }

  public Command setTargetArmStateCommand(ArmState targetArmState, CargoType cargoType) {
    return new InstantCommand(() -> {
      if (this.targetArmState != targetArmState || targetArmState != ArmState.STOW) {
        currentArmState = this.targetArmState;
        this.targetArmState = targetArmState;
        getArmCommand(armScoreLevel, cargoType).withInterruptBehavior(InterruptionBehavior.kCancelSelf).schedule();
      }
    });
  }

  private Command nothingCommand() {
    return new InstantCommand();
  }

  private ArmForwardKinematicPosition getBackScoreLevelPosition(ArmScoreLevel armScoreLevel, CargoType cargoType) {
    switch (cargoType) { 
      case CONE: {
        switch (armScoreLevel) {
          case THREE: return ArmConstants.kBackConeL3Position;
          case TWO: return ArmConstants.kBackConeL2Position;
          case ONE: return ArmConstants.kBackConeL1Position;
          case INTAKE: return ArmConstants.kBackIntakePosition;
        }
      }
      case CUBE: {
        switch (armScoreLevel) {
          case THREE: return ArmConstants.kBackCubeL3Position;
          case TWO: return ArmConstants.kBackCubeL2Position;
          case ONE: return ArmConstants.kBackCubeL1Position;
          case INTAKE: return ArmConstants.kBackIntakePosition;
        }
      }
    }
    return ArmConstants.kBackIntakePosition;
  }

  private ArmForwardKinematicPosition getFrontScoreLevelPosition(ArmScoreLevel armScoreLevel, CargoType cargoType) {
    switch (cargoType) { 
      case CONE: {
        switch (armScoreLevel) {
          case THREE: return ArmConstants.kFrontConeL2Position;
          case TWO: return ArmConstants.kFrontConeL2Position;
          case ONE: return ArmConstants.kFrontConeL2Position;
          case INTAKE: return ArmConstants.kFrontIntakePosition;
        }
      }
      case CUBE: {
        switch (armScoreLevel) {
          case THREE: return ArmConstants.kFrontCubeL3Position;
          case TWO: return ArmConstants.kFrontCubeL2Position;
          case ONE: return ArmConstants.kFrontCubeL2Position;
          case INTAKE: return ArmConstants.kFrontIntakePosition;
        }
      }
    }
    return ArmConstants.kFrontIntakePosition;
  }

  public Command getArmCommand(ArmScoreLevel armScoreLevel, CargoType cargoType) {
    switch (targetArmState) {
      case BACK: {
        switch (currentArmState) { // when target arm state = BACK
          case BACK: {
            switch (cargoType) { 
              // when current is back
              case CONE: {
                switch (armScoreLevel) {
                  case THREE: return m_arm.BackToBack(ArmConstants.kBackConeL3Position);
                  case TWO: return m_arm.setForwardKinematicsCommand(ArmConstants.kBackConeL2Position);
                  case ONE: return m_arm.BackToBack(ArmConstants.kBackConeL1Position);
                  case INTAKE: return m_arm.setForwardKinematicsCommand(ArmConstants.kBackIntakePosition);
                }
              }
              case CUBE: {
                switch (armScoreLevel) {
                  case THREE: return m_arm.BackToBack(ArmConstants.kBackCubeL3Position);
                  case TWO: return m_arm.BackToBack(ArmConstants.kBackCubeL2Position);
                  case ONE: return m_arm.BackToBack(ArmConstants.kBackCubeL1Position);
                  case INTAKE: return m_arm.BackToBack(ArmConstants.kBackIntakePosition);
                }
              }
            }
          }
          case TRANSFER: return m_arm.TransferToBack(getBackScoreLevelPosition(armScoreLevel, cargoType)); // When current is TRANSFER
          case FRONT: return m_arm.FrontToBack(getBackScoreLevelPosition(armScoreLevel, cargoType)); // when current is FRONT
          case STOW: return m_arm.StowToBack(getBackScoreLevelPosition(armScoreLevel, cargoType)); // when current is STOW
        }
      }
      case TRANSFER: { // when target arm state = TRANSFER
        switch (currentArmState) { 
          case BACK: return m_arm.BackToTransfer(ArmConstants.kTransferPosition); //when current arm state = BACK
          case TRANSFER: return nothingCommand(); //when current arm state = TRANSFER
          case FRONT: return m_arm.FrontToTransfer(ArmConstants.kTransferPosition); //when current arm state = FRONT
          case STOW: return m_arm.StowToTransfer(ArmConstants.kTransferPosition); //when current arm state = STOW
        }
      }
      case FRONT: { // when target arm state = FRONT
        switch (currentArmState) { 
          case BACK: return m_arm.BackToFront(getFrontScoreLevelPosition(armScoreLevel, cargoType)); // when current state is back
          case TRANSFER: return m_arm.TransferToFront(getFrontScoreLevelPosition(armScoreLevel, cargoType)); // when current state is transfer
          case FRONT: return m_arm.FrontToFront(getFrontScoreLevelPosition(armScoreLevel, cargoType));
          case STOW: return m_arm.StowToFront(getFrontScoreLevelPosition(armScoreLevel, cargoType));
        }
      }
      case STOW: {
        switch (currentArmState) {
          case BACK: return m_arm.BackToStow();
          case TRANSFER: return m_arm.TransferToStow();
          case FRONT: return m_arm.FrontToStow();
          case STOW: return nothingCommand();
        }
      }
    }
    return nothingCommand();
  }
  
  public void updateTelemetry() {
    //This method will be called once per scheduler run
    SmartDashboard.putString("Target Arm State", targetArmState.toString());
    SmartDashboard.putString("Current Arm State", currentArmState.toString());
  }
}
