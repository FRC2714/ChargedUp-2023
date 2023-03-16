// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.PivotIntake;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private CANSparkMax pivotMotor;

  private AbsoluteEncoder pivotEncoder;

  private double pivotGearRatio = 50;
  private double deployAngleDegrees = 230;
  private double holdAngleDegrees = 114;
  private double retractAngleDegrees = 35;
  private double shootAngleDegrees = 150;
  private double outtakeAngleDegrees = 200;

  private IntakeState intakeState = IntakeState.STOPPED;

  private Timer intakeRunningTimer = new Timer();

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    pivotMotor = new CANSparkMax(IntakeConstants.kPivotMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    intakeMotor.setInverted(true);
    pivotMotor.setInverted(false);

    intakeMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
    pivotMotor.setSmartCurrentLimit(IntakeConstants.kPivotMotorCurrentLimit);

    intakeMotor.enableVoltageCompensation(IntakeConstants.kNominalVoltage);
    pivotMotor.enableVoltageCompensation(IntakeConstants.kNominalVoltage);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(2*Math.PI*pivotGearRatio);
    pivotEncoder.setInverted(true);
  }

  public enum IntakeState {
    INTAKING, OUTTAKING, STOPPED
  }

  private void intake() {
    intakeMotor.setVoltage(IntakeConstants.kIntakeMotorSpeed*IntakeConstants.kNominalVoltage);
    if (intakeState != IntakeState.INTAKING) {
      intakeRunningTimer.reset();
      intakeRunningTimer.start();
      intakeState = IntakeState.INTAKING;
    }
  }

  private void outtake(double power) {
    intakeMotor.setVoltage(-power*IntakeConstants.kNominalVoltage);
    if (intakeState != IntakeState.OUTTAKING) {
      intakeRunningTimer.reset();
      intakeRunningTimer.start();
      intakeState = IntakeState.OUTTAKING;
    }
  }

  private void stop() {
    intakeMotor.set(0);
  }

  public double getPivotAngleRadians() {
    return pivotEncoder.getPosition() / pivotGearRatio;
  }

  public void setPivotPower(double power) {
    pivotMotor.setVoltage(power * IntakeConstants.kNominalVoltage);
  }

  public boolean atSetpoint(double targetAngleDegrees) {
    return Math.abs(getPivotAngleRadians() - Units.degreesToRadians(targetAngleDegrees)) < Units.degreesToRadians(2);
  }

  public boolean getDeployed() {
    return atSetpoint(deployAngleDegrees);
  }

  public boolean isConeDetected() {
    return (intakeRunningTimer.get() > 0.1) && //excludes current spike when motor first starts
      (intakeMotor.getOutputCurrent() > 19) && //cone intake current threshold
      (intakeState == IntakeState.INTAKING);
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> intake());
  }

  public Command outtakeCommand() {
    return new InstantCommand(() -> outtake(IntakeConstants.kOuttakeMotorSpeed));
  }

  public Command shootCommand() {
    return new InstantCommand(() -> outtake(IntakeConstants.kShootMotorSpeed));
  }

  public Command stopCommand() {
    return new InstantCommand(() -> stop());
  }

  public Command pivotToDeploy() {
    return new PivotIntake(this, deployAngleDegrees);
  }

  public Command pivotToOuttake() {
    return new PivotIntake(this, outtakeAngleDegrees);
  }

  public Command pivotToRetract() {
    return new PivotIntake(this, retractAngleDegrees);
  }

  public Command pivotToHold() {
    return new PivotIntake(this, holdAngleDegrees);
  }

  public Command pivotToShoot() {
    return new PivotIntake(this, shootAngleDegrees);
  }

  public Command deployAndIntake() {
    return 
      pivotToDeploy()
      .alongWith(intakeCommand());
  }

  public Command pivotThenOuttake() {
    return new SequentialCommandGroup(
      pivotToOuttake(),
      //new WaitCommand(1),
      outtakeCommand()
    );
  }

  public Command pivotThenShoot() {
    return new SequentialCommandGroup(
      pivotToShoot(),
      shootCommand(),
      new WaitCommand(1)
    );
  }

  public Command holdAndStop() {
    return 
      pivotToHold()
      .alongWith(stopCommand());
  }

  public Command retractAndStop() {
    return 
      pivotToRetract()
      .alongWith(stopCommand());
  }

  private Command AutoConeIntake() {
    return new SequentialCommandGroup(
      new WaitCommand(0.25),
      pivotToRetract(),
      new WaitCommand(1),
      stopCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if(isConeDetected()) {AutoConeIntake().schedule();}

    SmartDashboard.putNumber("Intake Pivot", Units.radiansToDegrees(getPivotAngleRadians()));
  }
}
