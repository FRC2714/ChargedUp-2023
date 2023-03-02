// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class ArmStateMachine extends SubsystemBase {
  private Arm m_arm;
  private LEDs m_leds;
  private Intake m_intake;
  private Claw m_claw;
  
  public enum ArmState {
    BACK, TRANSFER, FRONT, STOW
  }

  public enum ScoreLevel {
    THREE, TWO, ONE, INTAKE
  }

  public enum CargoType {
    CONE, CUBE
  }

  public ArmState currentArmState = ArmState.BACK; //will default to TRANSFER 
  //m_arm.estimateCurrentArmState()??
  public ScoreLevel currentScoreLevel = ScoreLevel.THREE; //default to THREE

  public ArmState targetArmState = ArmState.BACK; // default to TRANSFER 
  public ScoreLevel targetScoreLevel = ScoreLevel.THREE; // default to THREE

  public CargoType cargoType = CargoType.CONE; // default of cube

  /** Creates a new StateMachine. */
  public ArmStateMachine(Arm m_arm, LEDs m_leds, Intake m_intake, Claw m_claw) {
    this.m_arm = m_arm;
    this.m_leds = m_leds;
    this.m_intake = m_intake;
    this.m_claw = m_claw;
  }

  private void setTargetArmState(ArmState targetArmState) {
    if(this.targetArmState != targetArmState || targetArmState == ArmState.TRANSFER) {
      currentArmState = this.targetArmState;
      this.targetArmState = targetArmState;

      currentScoreLevel = targetScoreLevel;

      callArmCommand();
    }
  }

  private void setScoreLevel(ScoreLevel targetScoreLevel) {
    if(this.targetScoreLevel != targetScoreLevel) {
      currentScoreLevel = this.targetScoreLevel;
      this.targetScoreLevel = targetScoreLevel;

      currentArmState = targetArmState;
    }
  }

  private void callArmCommand() {
    getArmCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
  }

  public void setCargoType(CargoType cargoType) {
    this.cargoType = cargoType;
  }

  public Command setTargetArmStateCommand(ArmState targetArmState) {
    return new InstantCommand(() -> setTargetArmState(targetArmState));
  }

  public Command setTargetScoreLevelCommand(ScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> setScoreLevel(targetScoreLevel));
  }

  public Command setCargoTypeCommand(CargoType cargoType) {
    return new InstantCommand(() -> setCargoType(cargoType))
      .andThen(m_leds.setColorCargoType(cargoType));
  }

  public Command scoreCommand() {
    if (cargoType == CargoType.CONE) {
      return m_claw.scoreCone();
    } else {
      return m_claw.shootCube();
    }
  }

  public Command nothingCommand() {
    return new WaitCommand(0);
  }

  public Command getArmCommand() {
    switch (targetArmState) {
      case BACK: {
        switch (currentArmState) { // when target arm state = BACK
          case BACK: {
            switch (cargoType) { 
              // when current is back
              case CONE: {
                switch (targetScoreLevel) {
                  case THREE:
                    return m_arm.BackToBack(m_arm.BackConeL3Command());
                  case TWO:
                    return m_arm.BackConeL2Command();
                  case ONE:
                    return m_arm.BackToBack(m_arm.BackConeL1Command());
                  case INTAKE:
                    return m_arm.BackIntakeCommand();
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
          case TRANSFER: { // When current is TRANSFER
            switch (cargoType) { 
              case CONE: {
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
              }
              case CUBE: {
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
          }
          case FRONT: { // when current is FRONT
            switch (cargoType) { 
              case CONE: {
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
              }
              case CUBE: {
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
          }
          case STOW: { // when current is TUCK
            switch (cargoType) { 
              case CONE: {
                switch (targetScoreLevel) {
                  case THREE:
                    return m_arm.TuckToBack(m_arm.BackConeL3Command());
                  case TWO:
                    return m_arm.TuckToBack(m_arm.BackConeL2Command());
                  case ONE:
                    return m_arm.TuckToBack(m_arm.BackConeL1Command());
                  case INTAKE:
                    return m_arm.TuckToBack(m_arm.BackIntakeCommand());
                }
              }
              case CUBE: {
                switch (targetScoreLevel) {
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
          }
        }
      }
      case TRANSFER: { // when target arm state = TRANSFER
        switch (currentArmState) { 
          case BACK: { //when current arm state = BACK
            switch (cargoType) {
              case CONE:
                return m_arm.BackToTransfer(m_arm.TransferConeIntakeCommand());
              case CUBE:
                return m_intake.deployCommand().andThen(m_arm.BackToTransfer(m_arm.TransferCubeIntakeCommand()));
            }
          }
          case TRANSFER: { //when current arm state = TRANSFER
            switch (cargoType) {
              case CONE:
                return m_arm.TransferToTransfer(m_arm.TransferConeIntakeCommand());
              case CUBE:
                return m_intake.deployCommand().andThen(m_arm.TransferToTransfer(m_arm.TransferCubeIntakeCommand()));
            }
          }
          case FRONT: { //when current arm state = FRONT
            switch (cargoType) {
              case CONE:
                return m_arm.FrontToTransfer(m_arm.TransferConeIntakeCommand());
              case CUBE:
                return m_intake.deployCommand().andThen(m_arm.FrontToTransfer(m_arm.TransferCubeIntakeCommand()));
            }
          }
          case STOW: { //when current arm state = TUCK
            switch (cargoType) {
              case CONE:
                return m_arm.TuckToTransfer(m_arm.TransferConeIntakeCommand());
              case CUBE:
                return m_intake.deployCommand().andThen(m_arm.TuckToTransfer(m_arm.TransferCubeIntakeCommand()));
            }
          }
        }
      }
      case FRONT: { // when target arm state = FRONT
        switch (currentArmState) { 
          case BACK: { // when current state is back
            switch (cargoType) { 
              case CONE: {
                switch (targetScoreLevel) {
                  case THREE:
                    return nothingCommand();
                  case TWO:
                    return m_arm.BackToFront(m_arm.FrontConeL2Command());
                  case ONE:
                    return nothingCommand();
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
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.BackToFront(m_arm.FrontIntakeCommand());
                }
              }
            }
          }
          case TRANSFER: { // when current state is transfer
            switch (cargoType) { 
              case CONE: {
                switch (targetScoreLevel) {
                  case THREE:
                    return nothingCommand();
                  case TWO:
                    return m_arm.TransferToFront(m_arm.FrontConeL2Command());
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.TransferToFront(m_arm.FrontIntakeCommand());
                }
              }
              case CUBE: {
                switch (targetScoreLevel) {
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
            }
          }
          case FRONT: {
            switch (cargoType) {
              case CONE: {
                switch (targetScoreLevel) {
                  case THREE:
                    return nothingCommand();
                  case TWO:
                    return m_arm.FrontToFront(m_arm.FrontConeL2Command());
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.FrontToFront(m_arm.FrontIntakeCommand());
                }
              }
              case CUBE: {
                switch (targetScoreLevel) {
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
            }
          }
          case STOW: {
            switch (cargoType) {
              case CONE: {
                switch (targetScoreLevel) {
                  case THREE:
                    return nothingCommand();
                  case TWO:
                    return m_arm.TuckToFront(m_arm.FrontConeL2Command());
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.TuckToFront(m_arm.FrontIntakeCommand());
                }
              }
              case CUBE: {
                switch (targetScoreLevel) {
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
          }
        }
      }
      case STOW: {
        switch (currentArmState) {
          case BACK:
            return m_arm.BackToTuck();
          case TRANSFER:
           return m_arm.TransferToTuck();
          case FRONT:
            return m_arm.FrontToTuck();
          case STOW:
            return nothingCommand();
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
  }
}
