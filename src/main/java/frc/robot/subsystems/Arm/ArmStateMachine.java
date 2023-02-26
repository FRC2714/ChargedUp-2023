// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.LEDs;

public class ArmStateMachine extends SubsystemBase {
  private Arm m_arm;
  private LEDs m_leds;
  
  public enum ArmState {
    BACK, TRANSFER, FRONT
  }

  public enum ScoreLevel {
    THREE, TWO, ONE, INTAKE
  }

  public enum CargoType {
    CONE, CUBE
  }

  public enum IntakeMode {
    FLOOR, HP
  }

  public ArmState currentArmState = ArmState.BACK; //will default to TRANSFER 
  //m_arm.estimateCurrentArmState()??
  public ScoreLevel currentScoreLevel = ScoreLevel.THREE; //default to THREE

  public ArmState targetArmState = ArmState.TRANSFER; // default to TRANSFER 
  public ScoreLevel targetScoreLevel = ScoreLevel.THREE; // default to THREE

  public CargoType cargoType = CargoType.CUBE; // default of cube

  public IntakeMode intakeMode = IntakeMode.FLOOR; // default of FLOOR

  private boolean armStateChanges = false; //default to false
  private boolean scoreLevelChanges = false; //default to false

  /** Creates a new StateMachine. */
  public ArmStateMachine(Arm m_arm, LEDs m_leds) {
    this.m_arm = m_arm;
    this.m_leds = m_leds;
  }

  private void setTargetArmState(ArmState targetArmState) {
    armStateChanges = this.targetArmState != targetArmState;
    currentArmState = this.targetArmState;
    this.targetArmState = targetArmState;

    currentScoreLevel = targetScoreLevel;
    callArmCommand();
  }

  private void setScoreLevel(ScoreLevel targetScoreLevel) {
    scoreLevelChanges = this.targetScoreLevel != targetScoreLevel;
    currentScoreLevel = this.targetScoreLevel;
    this.targetScoreLevel = targetScoreLevel;

    currentArmState = targetArmState;
    callArmCommand();
  }

  private void callArmCommand() {
    getArmCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
  }

  private void setCargoType(CargoType cargoType) {
    this.cargoType = cargoType;
    this.m_leds.updateColor(cargoType, intakeMode);
  }

  private void setIntakeMode(IntakeMode intakeMode) {
    this.intakeMode = intakeMode;
    this.m_leds.updateColor(cargoType, intakeMode);
  }

  public Command setTargetArmStateCommand(ArmState targetArmState) {
    return new InstantCommand(() -> setTargetArmState(targetArmState));
  }

  public Command setTargetScoreLevelCommand(ScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> setScoreLevel(targetScoreLevel));
  }

  public Command setCargoTypeCommand(CargoType cargoType) {
    return new InstantCommand(() -> setCargoType(cargoType));
  }

  public Command setIntakeModeCommand(IntakeMode intakeMode) {
    return new InstantCommand(() -> setIntakeMode(intakeMode));
  }

  public Command nothingCommand() {
    return new WaitCommand(0);
  }

  public Command getArmCommand() {
    switch (intakeMode) {
      case FLOOR: { // floor mode
        switch (targetArmState) {
          case TRANSFER: { // when target arm state = TRANSFER
            switch (currentArmState) {
              case BACK:
                return m_arm.BackToTransfer();
              case FRONT:
                return m_arm.FrontToTransfer();
              case TRANSFER:
                return nothingCommand();
            }
          }
          case BACK: // when target arm state = BACK
            switch (currentArmState) {
              case TRANSFER: { // When current is TRANSFER
                switch (cargoType) {
                  case CONE:
                    switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.TransferToBack(m_arm.BackConeL3Command());
                      case TWO:
                        return m_arm.TransferToBack(m_arm.BackConeL2Command());
                      case ONE:
                        return m_arm.TransferToBack(m_arm.BackConeL1Command());
                      case INTAKE:
                        return m_arm.TransferToBack(m_arm.BackIntakeCommand());
                    }
                  case CUBE:
                    switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.TransferToBack(m_arm.BackCubeL3Command());
                      case TWO:
                        return m_arm.TransferToBack(m_arm.BackCubeL2Command());
                      case ONE:
                        return m_arm.TransferToBack(m_arm.BackCubeL1Command());
                      case INTAKE:
                        return m_arm.TransferToBack(m_arm.BackIntakeCommand());
                    }
                }
              }
              case FRONT: { // when current is FRONT
                switch (cargoType) {
                  case CONE:
                    switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.FrontToBack(m_arm.BackConeL3Command());
                      case TWO:
                        return m_arm.FrontToBack(m_arm.BackConeL2Command());
                      case ONE:
                        return m_arm.FrontToBack(m_arm.BackConeL1Command());
                      case INTAKE:
                        return m_arm.FrontToBack(m_arm.BackIntakeCommand());
                    }
                  case CUBE:
                    switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.FrontToBack(m_arm.BackCubeL3Command());
                      case TWO:
                        return m_arm.FrontToBack(m_arm.BackCubeL2Command());
                      case ONE:
                        return m_arm.FrontToBack(m_arm.BackCubeL1Command());
                      case INTAKE:
                        return m_arm.FrontToBack(m_arm.BackIntakeCommand());
                    }
                }
              }
              case BACK: {
                switch (cargoType) {
                  case CONE: {
                    switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.BackToBack(m_arm.BackConeL3Command());
                      case TWO:
                        return m_arm.BackToBack(m_arm.BackConeL2Command());
                      case ONE:
                        return m_arm.BackToBack(m_arm.BackConeL1Command());
                      case INTAKE:
                        return m_arm.BackToBack(m_arm.BackIntakeCommand());
                    }
                  }
                  case CUBE: {
                    switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.BackToBack(m_arm.BackCubeL3Command());
                      case TWO:
                        return m_arm.BackToBack(m_arm.BackCubeL2Command());
                      case ONE:
                        return m_arm.BackToBack(m_arm.BackCubeL1Command());
                      case INTAKE:
                        return m_arm.BackToBack(m_arm.BackIntakeCommand());
                    }
                  }
                }
              }
            }
          case FRONT: { // when target arm state = FRONT
            switch (currentArmState) {
              case BACK: {
                switch (cargoType) {
                  case CONE: {
                    switch (targetScoreLevel) {
                      case TWO:
                        return m_arm.BackToFront(m_arm.FrontConeL2Command());
                      case INTAKE:
                        return m_arm.BackToFront(m_arm.FrontIntakeCommand());
                    }
                  }
                  case CUBE: {
                    switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.BackToFront(m_arm.FrontCubeL3Command());
                      case TWO:
                        return m_arm.BackToFront(m_arm.FrontCubeL2Command());
                      case INTAKE:
                        return m_arm.BackToFront(m_arm.FrontIntakeCommand());
                    }
                  }
                }
              }
              case TRANSFER:
                return m_arm.TransferToFront();
              case FRONT:
                return nothingCommand();
            }
          }
        }
      }
      case HP: { // human player mode
        switch (targetArmState) {
          case TRANSFER: {
            return m_arm.TransferCommand();
          }
          case BACK: {
            switch (cargoType) {
              case CONE: {
                switch (targetScoreLevel) {
                  case THREE:
                    return m_arm.BackConeL3Command();
                  case TWO:
                    return m_arm.BackConeL2Command();
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.BackIntakeCommand();
                }
              }
              case CUBE:
                switch (targetScoreLevel) {
                  case THREE:
                    return m_arm.BackCubeL3Command();
                  case TWO:
                    return m_arm.BackCubeL2Command();
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.BackIntakeCommand();
                }
            }
          }
          case FRONT: {// when target arm state = FRONT
            switch (cargoType) {
              case CONE: {
                switch (targetScoreLevel) {
                  case THREE:
                    return nothingCommand();
                  case TWO:
                    return m_arm.FrontConeL2Command();
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.FrontIntakeCommand();
                }
              }
              case CUBE: {
                switch (targetScoreLevel) {
                  case THREE:
                    return m_arm.FrontCubeL3Command();
                  case TWO:
                    return m_arm.FrontCubeL2Command();
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.FrontIntakeCommand();
                }
              }
            }
          }
        }
      }
    }
    return nothingCommand();
  }

  
  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    SmartDashboard.putString("Target Arm State", targetArmState.toString());
    SmartDashboard.putString("Current Arm State", currentArmState.toString());
    
    SmartDashboard.putString("Target Score Level", targetScoreLevel.toString());
    SmartDashboard.putString("Current Score Level", currentScoreLevel.toString());

    SmartDashboard.putString("Cargo Type", cargoType.toString());
    SmartDashboard.putString("Arm Intake Mode", intakeMode.toString());

    SmartDashboard.putBoolean("Arm State Changes?", armStateChanges);
    SmartDashboard.putBoolean("Score Level Changes?", scoreLevelChanges);
  }
}
