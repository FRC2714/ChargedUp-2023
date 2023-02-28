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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class ArmStateMachine extends SubsystemBase {
  private Arm m_arm;
  private LEDs m_leds;
  private Intake m_intake;
  
  public enum ArmState {
    BACK, TRANSFER, FRONT, TUCK
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

  public ArmState targetArmState = ArmState.BACK; // default to TRANSFER 
  public ScoreLevel targetScoreLevel = ScoreLevel.THREE; // default to THREE

  public CargoType cargoType = CargoType.CUBE; // default of cube

  public IntakeMode intakeMode = IntakeMode.FLOOR; // default of FLOOR

  private boolean armStateChanges = false; //default to false
  private boolean scoreLevelChanges = false; //default to false

  /** Creates a new StateMachine. */
  public ArmStateMachine(Arm m_arm, LEDs m_leds, Intake m_intake) {
    this.m_arm = m_arm;
    this.m_leds = m_leds;
    this.m_intake = m_intake;
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

  public void setCargoType(CargoType cargoType) {
    this.cargoType = cargoType;
    updateLEDs();
  }

  public void setIntakeMode(IntakeMode intakeMode) {
    this.intakeMode = intakeMode;
    updateLEDs();
  }

  private void updateLEDs() {
    switch (intakeMode) {
      case HP: {
        switch(cargoType) {
          case CUBE: {m_leds.setPurpleWave();}
          case CONE: {m_leds.setYellowWave();}
        }
      }
      case FLOOR: {
        switch(cargoType) {
          case CUBE: {m_leds.setPurple();}
          case CONE: {m_leds.setYellow();}
        }
      }
    }
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
          case BACK: switch (currentArmState) { // when target arm state = BACK
              case BACK: switch (cargoType) { //when current is back
                  case CONE: switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.BackToBack(m_arm.BackConeL3Command());
                      case TWO:
                        return m_arm.BackConeL2Command();
                      case ONE:
                        return m_arm.BackToBack(m_arm.BackConeL1Command());
                      case INTAKE:
                        return m_arm.BackIntakeCommand();
                    }
                  case CUBE:  switch (targetScoreLevel) {
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
              case TRANSFER: switch (cargoType) { // When current is TRANSFER
                  case CONE: switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.TransferToBack(m_arm.BackConeL3Command());
                      case TWO:
                        return m_arm.TransferToBack(m_arm.BackConeL2Command());
                      case ONE:
                        return m_arm.TransferToBack(m_arm.BackConeL1Command());
                      case INTAKE:
                        return m_arm.TransferToBack(m_arm.BackIntakeCommand());
                    }
                  case CUBE: switch (targetScoreLevel) {
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
              case FRONT: switch (cargoType) { // when current is FRONT
                  case CONE: switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.FrontToBack(m_arm.BackConeL3Command());
                      case TWO:
                        return m_arm.FrontToBack(m_arm.BackConeL2Command());
                      case ONE:
                        return m_arm.FrontToBack(m_arm.BackConeL1Command());
                      case INTAKE:
                        return m_arm.FrontToBack(m_arm.BackIntakeCommand());
                    }
                  case CUBE: switch (targetScoreLevel) {
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
              case TUCK: switch (cargoType) { // when current is FRONT
                case CONE: switch (targetScoreLevel) {
                    case THREE:
                      return m_arm.TuckToBack(m_arm.BackConeL3Command());
                    case TWO:
                      return m_arm.TuckToBack(m_arm.BackConeL2Command());
                    case ONE:
                      return m_arm.TuckToBack(m_arm.BackConeL1Command());
                    case INTAKE:
                      return m_arm.TuckToBack(m_arm.BackIntakeCommand());
                  }
                case CUBE: switch (targetScoreLevel) {
                    case THREE:
                      return m_arm.TuckToBack(m_arm.BackCubeL3Command());
                    case TWO:
                      return m_arm.TuckToBack(m_arm.BackCubeL2Command());
                    case ONE:
                      return m_arm.TuckToBack(m_arm.BackCubeL1Command());
                    case INTAKE:
                      return m_arm.TuckToBack(m_arm.BackIntakeCommand());
                  }
              }
            }
          case TRANSFER: switch (currentArmState) { // when target arm state = BACK
              case BACK: switch (cargoType) {
                case CONE: m_arm.BackToTransfer(m_arm.TransferConeIntakeCommand()); 
                case CUBE: m_arm.BackToTransfer(m_arm.TransferCubeIntakeCommand()).andThen(m_intake.intakeCube());
                }
              case TRANSFER: switch (cargoType) {
                case CONE: m_arm.TransferToTransfer(m_arm.TransferConeIntakeCommand()); 
                case CUBE: m_arm.TransferToTransfer(m_arm.TransferCubeIntakeCommand()).andThen(m_intake.intakeCube());
                }; 
              case FRONT: switch (cargoType) {
                case CONE: m_arm.FrontToTransfer(m_arm.TransferConeIntakeCommand()); 
                case CUBE: m_arm.FrontToTransfer(m_arm.TransferCubeIntakeCommand()).andThen(m_intake.intakeCube());
                }
              case TUCK: switch (cargoType) {
                case CONE: m_arm.TuckToTransfer(m_arm.TransferConeIntakeCommand()); 
                case CUBE: m_arm.TuckToTransfer(m_arm.TransferCubeIntakeCommand()).andThen(m_intake.intakeCube());
                }
            }
          case FRONT:  switch (currentArmState) { // when target arm state = FRONT
              case BACK: switch (cargoType) { //when current state is back
                  case CONE: switch (targetScoreLevel) {
                      case THREE:
                        return nothingCommand();
                      case TWO:
                        return m_arm.BackToFront(m_arm.FrontConeL2Command());
                      case ONE:
                        return nothingCommand();
                      case INTAKE:
                        return m_arm.BackToFront(m_arm.FrontIntakeCommand());
                    }
                  case CUBE: switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.BackToFront(m_arm.FrontCubeL3Command());
                      case TWO:
                        return m_arm.BackToFront(m_arm.FrontCubeL2Command());
                      case ONE:
                        return nothingCommand();
                      case INTAKE:
                        return m_arm.BackToFront(m_arm.FrontIntakeCommand());
                    }
                }
              case TRANSFER: switch (cargoType) { //when current state is transfer
                  case CONE: switch (targetScoreLevel) {
                      case THREE:
                        return nothingCommand();
                      case TWO:
                        return m_arm.TransferToFront(m_arm.FrontConeL2Command());
                      case ONE:
                        return nothingCommand();
                      case INTAKE:
                        return m_arm.TransferToFront(m_arm.FrontIntakeCommand());
                    }
                  case CUBE: switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.TransferToFront(m_arm.FrontCubeL3Command());
                      case TWO:
                        return m_arm.TransferToFront(m_arm.FrontCubeL2Command());
                      case ONE:
                        return nothingCommand();
                      case INTAKE:
                        return m_arm.TransferToFront(m_arm.FrontIntakeCommand());
                    }
                }
              case FRONT: switch (cargoType) {
                  case CONE: switch (targetScoreLevel) {
                      case THREE:
                        return nothingCommand();
                      case TWO:
                        return m_arm.FrontToFront(m_arm.FrontConeL2Command());
                      case ONE:
                        return nothingCommand();
                      case INTAKE:
                        return m_arm.FrontToFront(m_arm.FrontIntakeCommand());
                    }
                  case CUBE: switch (targetScoreLevel) {
                      case THREE:
                        return m_arm.FrontToFront(m_arm.FrontCubeL3Command());
                      case TWO:
                        return m_arm.FrontToFront(m_arm.FrontCubeL2Command());
                      case ONE:
                        return nothingCommand();
                      case INTAKE:
                        return m_arm.FrontToFront(m_arm.FrontIntakeCommand());
                    }
                }
              case TUCK: switch (cargoType) {
                case CONE: switch (targetScoreLevel) {
                    case THREE:
                      return nothingCommand();
                    case TWO:
                      return m_arm.TuckToFront(m_arm.FrontConeL2Command());
                    case ONE:
                      return nothingCommand();
                    case INTAKE:
                      return m_arm.TuckToFront(m_arm.FrontIntakeCommand());
                  }
                case CUBE: switch (targetScoreLevel) {
                    case THREE:
                      return m_arm.TuckToFront(m_arm.FrontCubeL3Command());
                    case TWO:
                      return m_arm.TuckToFront(m_arm.FrontCubeL2Command());
                    case ONE:
                      return nothingCommand();
                    case INTAKE:
                      return m_arm.TuckToFront(m_arm.FrontIntakeCommand());
                  }
              }
            }
          case TUCK: switch (currentArmState) {
              case BACK: m_arm.BackToTuck();
              case TRANSFER: m_arm.TransferToTuck();
              case FRONT: m_arm.FrontToTuck();
              case TUCK: return nothingCommand();
            }
        }
      }
      case HP: { switch (targetArmState) {
          case TRANSFER: {
            return m_arm.TransferConeIntakeCommand();
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
