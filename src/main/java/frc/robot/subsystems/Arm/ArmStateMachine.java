// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Superstructure.CargoType;
import frc.robot.utils.ArmPreset;

public class ArmStateMachine {
  private Arm m_arm;
  
  public enum ArmState {
    BACK, TRANSFER, FRONT, STOW
  }

  public enum ArmScoreLevel {
    HIGH, MIDDLE, LOW, INTAKE
  }

  private ArmState currentArmState = ArmState.BACK; //will default to TRANSFER 
  private ArmState targetArmState = ArmState.BACK; // default to TRANSFER 
  private ArmScoreLevel armScoreLevel = ArmScoreLevel.INTAKE;

  Map<ArmScoreLevel, ArmPreset> BackConeMap = Map.ofEntries(
      Map.entry(ArmScoreLevel.HIGH, ArmConstants.kBackConeHighPosition),
      Map.entry(ArmScoreLevel.MIDDLE, ArmConstants.kBackConeMiddlePosition),
      Map.entry(ArmScoreLevel.LOW, ArmConstants.kBackConeLowPosition),
      Map.entry(ArmScoreLevel.INTAKE, ArmConstants.kBackIntakePosition)
  );

  Map<ArmScoreLevel, ArmPreset> BackCubeMap = Map.ofEntries(
      Map.entry(ArmScoreLevel.HIGH, ArmConstants.kBackCubeHighPosition),
      Map.entry(ArmScoreLevel.MIDDLE, ArmConstants.kBackCubeMiddlePosition),
      Map.entry(ArmScoreLevel.LOW, ArmConstants.kBackCubeLowPosition),
      Map.entry(ArmScoreLevel.INTAKE, ArmConstants.kBackIntakePosition)
  );

  Map<ArmScoreLevel, ArmPreset> FrontConeMap = Map.ofEntries(
      Map.entry(ArmScoreLevel.HIGH, ArmConstants.kFrontConeMiddlePosition),
      Map.entry(ArmScoreLevel.MIDDLE, ArmConstants.kFrontConeMiddlePosition),
      Map.entry(ArmScoreLevel.LOW, ArmConstants.kFrontConeMiddlePosition),
      Map.entry(ArmScoreLevel.INTAKE, ArmConstants.kFrontIntakePosition)
  );

  Map<ArmScoreLevel, ArmPreset> FrontCubeMap = Map.ofEntries(
      Map.entry(ArmScoreLevel.HIGH, ArmConstants.kFrontCubeHighPosition),
      Map.entry(ArmScoreLevel.MIDDLE, ArmConstants.kFrontCubeMiddlePosition),
      Map.entry(ArmScoreLevel.LOW, ArmConstants.kFrontCubeMiddlePosition),
      Map.entry(ArmScoreLevel.INTAKE, ArmConstants.kFrontIntakePosition)
  );

  /** Creates a new StateMachine. */
  public ArmStateMachine(Arm m_arm) {
    this.m_arm = m_arm;
  }

  //Score level
  public InstantCommand setArmScoreLevelCommand(ArmScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> this.armScoreLevel = targetScoreLevel);
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

  private ArmPreset getBackScoreLevelPosition(ArmScoreLevel armScoreLevel, CargoType cargoType) {
    return Map.ofEntries(
      Map.entry(CargoType.CONE, BackConeMap.get(armScoreLevel)),
      Map.entry(CargoType.CUBE, BackCubeMap.get(armScoreLevel))
    ).get(cargoType);
  }

  private ArmPreset getFrontScoreLevelPosition(ArmScoreLevel armScoreLevel, CargoType cargoType) {
    return Map.ofEntries(
      Map.entry(CargoType.CONE, FrontConeMap.get(armScoreLevel)),
      Map.entry(CargoType.CUBE, FrontCubeMap.get(armScoreLevel))
    ).get(cargoType);
  }

  public Command getArmCommand(ArmScoreLevel armScoreLevel, CargoType cargoType) {
    switch (currentArmState) {
      case BACK: switch (targetArmState) {
        case BACK: switch (cargoType) { 
          // when current is back
          case CONE: {
            switch (armScoreLevel) {
              case HIGH: return m_arm.BackToBack(ArmConstants.kBackConeHighPosition);
              case MIDDLE: return m_arm.setPresetCommand(ArmConstants.kBackConeMiddlePosition);
              case LOW: return m_arm.BackToBack(ArmConstants.kBackConeLowPosition);
              case INTAKE: return m_arm.setPresetCommand(ArmConstants.kBackIntakePosition);
            }
          }
          case CUBE: {
            switch (armScoreLevel) {
              case HIGH: return m_arm.BackToBack(ArmConstants.kBackCubeHighPosition);
              case MIDDLE: return m_arm.BackToBack(ArmConstants.kBackCubeMiddlePosition);
              case LOW: return m_arm.BackToBack(ArmConstants.kBackCubeLowPosition);
              case INTAKE: return m_arm.BackToBack(ArmConstants.kBackIntakePosition);
            }
          }
        }
        case TRANSFER: return m_arm.BackToTransfer(ArmConstants.kTransferPosition);
        case FRONT: return m_arm.BackToFront(getFrontScoreLevelPosition(armScoreLevel, cargoType));
        case STOW: return m_arm.BackToStow();
      }
      case TRANSFER: switch (targetArmState) {
        case BACK: return m_arm.TransferToBack(getBackScoreLevelPosition(armScoreLevel, cargoType));
        case TRANSFER: return new InstantCommand();
        case FRONT: return m_arm.TransferToFront(getFrontScoreLevelPosition(armScoreLevel, cargoType));
        case STOW: return m_arm.TransferToStow();
      }
      case FRONT: switch (targetArmState) {
        case BACK: return m_arm.FrontToBack(getBackScoreLevelPosition(armScoreLevel, cargoType));
        case TRANSFER: return m_arm.FrontToTransfer(ArmConstants.kTransferPosition);
        case FRONT: return m_arm.FrontToFront(getFrontScoreLevelPosition(armScoreLevel, cargoType));
        case STOW: return m_arm.FrontToStow();
      }
      case STOW: switch (targetArmState) {
        case BACK: return m_arm.StowToBack(getBackScoreLevelPosition(armScoreLevel, cargoType));
        case TRANSFER: return m_arm.StowToTransfer(ArmConstants.kTransferPosition);
        case FRONT: return m_arm.StowToFront(getFrontScoreLevelPosition(armScoreLevel, cargoType));
        case STOW: return new InstantCommand();
      }
    }
    return new InstantCommand();
  }
  
  public void updateTelemetry() {
    //This method will be called once per scheduler run
    SmartDashboard.putString("Target Arm State", targetArmState.toString());
    SmartDashboard.putString("Current Arm State", currentArmState.toString());
  }
}
