// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private CANSparkMax pivotMotor;

  private AbsoluteEncoder pivotEncoder;

  private SparkMaxPIDController PivotController;

  private PneumaticHub pneumaticHub;

  public enum IntakeState {
    INTAKING, OUTTAKING, STOPPED
  }

  private IntakeState intakeState = IntakeState.STOPPED;

  private Timer intakeRunningTimer = new Timer();

  private double targetAngleRadians = 0;

  private double pivotMotorGearRatio = 50;

  private double retractPosition = 50;
  private double deployPosition = 130;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    pivotMotor = new CANSparkMax(IntakeConstants.kPivotMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    intakeMotor.setInverted(true);

    intakeMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
    pivotMotor.setSmartCurrentLimit(IntakeConstants.kPivotMotorCurrentLimit);

    intakeMotor.enableVoltageCompensation(IntakeConstants.kNominalVoltage);
    pivotMotor.enableVoltageCompensation(IntakeConstants.kNominalVoltage);

    intakeMotor.burnFlash();
    pivotMotor.burnFlash();

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor((2*Math.PI)*(pivotMotorGearRatio/2));
    pivotEncoder.setInverted(true);

    pneumaticHub = new PneumaticHub(IntakeConstants.kPneumaticHubCanId);
    pneumaticHub.enableCompressorAnalog(IntakeConstants.kCompressorMinPressure, IntakeConstants.kCompressorMaxPressure);
    
    PivotController = pivotMotor.getPIDController();
    PivotController.setFeedbackDevice(pivotEncoder);
    PivotController.setP(0.00010, 0);
    PivotController.setSmartMotionMaxVelocity(500, 0);
    PivotController.setSmartMotionMaxAccel(500, 0);
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
    intakeMotor.setVoltage(power*IntakeConstants.kNominalVoltage);
    if (intakeState != IntakeState.OUTTAKING) {
      intakeRunningTimer.reset();
      intakeRunningTimer.start();
      intakeState = IntakeState.OUTTAKING;
    }
  }

  private void stop() {
    intakeMotor.set(0);
  }

  private double getPivotAngleRadians() {
    return pivotEncoder.getPosition() / pivotMotorGearRatio;
  }

  public void setPivotTargetAngle(double targetAngleDegrees) {
    this.targetAngleRadians = Units.degreesToRadians(targetAngleDegrees);
    PivotController.setReference((targetAngleRadians * pivotMotorGearRatio*2), ControlType.kSmartMotion);
  }

  public boolean pivotAtSetpoint() {
    return Math.abs(getPivotAngleRadians() - targetAngleRadians) < Units.degreesToRadians(2);
  }

  public void deploy() {
    setPivotTargetAngle(deployPosition);
  }

  public void retract() {
    setPivotTargetAngle(retractPosition);
  }

  public boolean getDeployed() {
    return true;
  }

  public boolean getClosed() {
    return true;
  }

  public boolean isConeDetected() {
    return (intakeRunningTimer.get() > 0.1) && //excludes current spike when motor first starts
      (intakeMotor.getOutputCurrent() > 19) && //cone intake current threshold
      (intakeState == IntakeState.INTAKING) && 
      getClosed();
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> intake());
  }

  public Command outtakeCommand() {
    return new InstantCommand(() -> outtake(0.6));
  }

  public Command stopCommand() {
    return new InstantCommand(() -> stop());
  }

  public Command deployCommand() {
    return new InstantCommand(() -> deploy());
  }

  public Command retractCommand() {
    return new InstantCommand(() -> retract());
  }

  public Command intakeCone() {
    return 
      deployCommand()
      .andThen(intakeCommand());
  }

  public Command intakeCube() {
    return 
      deployCommand()
      .andThen(intakeCommand());
  }

  public Command retractAndStop() {
    return 
      retractCommand()
      .andThen(stopCommand());
  }

  private Command AutoConeIntake() {
    return new SequentialCommandGroup(
      new WaitCommand(0.25),
      retractCommand(),
      new WaitCommand(1),
      stopCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if(isConeDetected()) {AutoConeIntake().schedule();}
    SmartDashboard.putNumber("Intake Angle", Units.radiansToDegrees(getPivotAngleRadians()));
    SmartDashboard.putNumber("intake target angle degrees", Units.radiansToDegrees(targetAngleRadians));
    SmartDashboard.putNumber("intake target angle radians", targetAngleRadians);
  }
}
