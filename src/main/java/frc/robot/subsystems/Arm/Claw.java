// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor;

  private DoubleSolenoid clawSolenoid;

  public enum ClawState {
    INTAKING, OUTTAKING, STOPPED
  }

  private static ClawState clawState = ClawState.STOPPED;
  private Timer clawRunningTimer = new Timer();

  /** Creates a new Claw. */
  public Claw() {
    clawMotor = new CANSparkMax(ClawConstants.kClawMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
    clawMotor.setInverted(true);
    clawMotor.enableVoltageCompensation(ClawConstants.kNominalVoltage);
    clawMotor.burnFlash();

    clawSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH, 
      ClawConstants.kClawSolenoidForwardChannel, 
      ClawConstants.kClawSolenoidReverseChannel);
  }

  public void setClawIntake() {
    clawMotor.set(ClawConstants.kIntakeMotorSpeed * ClawConstants.kNominalVoltage);
    if (clawState != ClawState.INTAKING) {
      clawRunningTimer.reset();
      clawRunningTimer.start();
    }
    clawState = ClawState.INTAKING;
  }

  private void setClawOuttake() {
    clawMotor.set(ClawConstants.kOuttakeMotorSpeed);
    clawState = ClawState.OUTTAKING;
  }

  public void setClawStop() {
    clawMotor.set(0);
  }

  public void setClawOpen() {
    clawSolenoid.set(Value.kForward);
    clawState = ClawState.OUTTAKING;
  }

  public void setClawClose() {
    clawSolenoid.set(Value.kReverse);
  }

  public ClawState getClawState() {
    return clawState;
  }

  public boolean isCurrentSpikeDetected() {
    return (clawRunningTimer.get() > 0.5) && //excludes current spike when motor first starts
      (clawMotor.getOutputCurrent() > 20) && //cone intake current threshold
      (clawState == ClawState.INTAKING);
  }

  public Command intakeCube() {
    return new InstantCommand(() -> {
      setClawOpen();
      setClawIntake();
    });
  }

  public Command intakeCone() {
    return new InstantCommand(() -> {
      setClawClose();
      setClawIntake();
    });
  }

  public Command scoreCone() {
    return new InstantCommand(() -> {
      setClawStop();
      setClawOpen();
    });
  }

  public Command scoreCube() {
    return new InstantCommand(() -> {
      setClawOuttake();
      setClawOpen();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("claw current", clawMotor.getOutputCurrent());
  }
}