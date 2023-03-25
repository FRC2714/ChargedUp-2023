// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.InterpolatingTreeMap;

public class Intake extends SubsystemBase {
  private Limelight m_limelight;

  private CANSparkMax intakeMotor;
  private CANSparkMax pivotMotor;

  private CANSparkMax topFlywheelMotor;
  private CANSparkMax bottomFlywheelMotor;

  private RelativeEncoder flywheelEncoder;

  private AbsoluteEncoder pivotEncoder;

  private InterpolatingTreeMap velocityMap = new InterpolatingTreeMap();
  private InterpolatingTreeMap pivotMap = new InterpolatingTreeMap();

  private double pivotGearRatio = 50;

  private double deployAngleDegrees = 200;
  private double holdAngleDegrees = 55;
  private double retractAngleDegrees = 20;
  private double shootAngleDegrees = 120;
  private double outtakeAngleDegrees = 180;

  private PIDController pivotController = new PIDController(0, 0, 0);
  private PIDController flywheelController = new PIDController(0, 0, 0);

  private static IntakeState intakeState = IntakeState.STOPPED;

  private Timer intakeRunningTimer = new Timer();

  /** Creates a new Intake. */
  public Intake(Limelight m_limelight) {
    this.m_limelight = m_limelight;
    
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
    pivotEncoder.setZeroOffset(200);

    topFlywheelMotor = new CANSparkMax(16, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkMax(17, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomFlywheelMotor.follow(topFlywheelMotor, true);
    topFlywheelMotor.setInverted(false);

    topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);

    flywheelEncoder = topFlywheelMotor.getEncoder();
    pivotEncoder.setVelocityConversionFactor(2*Math.PI*1.0);//gear ratio

    pivotController.disableContinuousInput();
    pivotController.setTolerance(Units.degreesToRadians(4));
  }

  private void populateVelocityMap() {
    velocityMap.put(0.0, 0.0);
    velocityMap.put(5.0, 500.0);
  }

  private double getDynamicFlywheelVelocity() {
    return m_limelight.isTargetVisible()
        ? velocityMap.getInterpolated(Units.metersToFeet(m_limelight.getDistanceToGoalMeters()) + 0)
        : 0;
  }

  private void populatePivotMap() {
    pivotMap.put(0.0, 150.0);
    pivotMap.put(5.0, 170.0);
  }

  private double getDynamicPivot() {
    return m_limelight.isTargetVisible()
        ? pivotMap.getInterpolated(Units.metersToFeet(m_limelight.getDistanceToGoalMeters()))
        : holdAngleDegrees;
  }

  public void setDynamicShooter() {
    setFlywheelTargetVelocity(getDynamicFlywheelVelocity());
    setPivotTarget(getDynamicPivot());
  }

  public enum IntakeState {
    INTAKING, OUTTAKING, STOPPED
  }

  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity() / 1.0;//gear ratio
  }

  public void setFlywheelTargetVelocity(double targetRPM) {
    flywheelController.setSetpoint(targetRPM);
  }

  private void setCalculatedFlywheelVoltage() {
    topFlywheelMotor.setVoltage(flywheelController.calculate(getFlywheelVelocity()));
  }

  private void intake() {
    //setFlywheelTargetVelocity(0);
    topFlywheelMotor.set(0.4);
    intakeMotor.setVoltage(IntakeConstants.kIntakeMotorSpeed*IntakeConstants.kNominalVoltage);
    if (intakeState != IntakeState.INTAKING) {
      intakeRunningTimer.reset();
      intakeRunningTimer.start();
      intakeState = IntakeState.INTAKING;
    }
  }

  private void outtake(double power) {
    //setFlywheelTargetVelocity(0);
    topFlywheelMotor.set(-0.4);
    intakeMotor.setVoltage(-power*IntakeConstants.kNominalVoltage);
    if (intakeState != IntakeState.OUTTAKING) {
      intakeRunningTimer.reset();
      intakeRunningTimer.start();
      intakeState = IntakeState.OUTTAKING;
    }
  }

  private void stop() {
    //setFlywheelTargetVelocity(0);
    topFlywheelMotor.set(0);
    intakeMotor.set(0);
  }

  public double getPivotAngleRadians() {
    return pivotEncoder.getPosition() / pivotGearRatio - Units.degreesToRadians(37);
  }

  public void setPivotPower(double power) {
    pivotMotor.setVoltage(power * IntakeConstants.kNominalVoltage);
  }

  public void setPivotTarget(double targetAngleDegrees) {
    if (pivotController.getP() == 0) { pivotController.setP(0.22);} //prevent jumping on enable
    pivotController.setSetpoint(Units.degreesToRadians(targetAngleDegrees));
  }

  public boolean atSetpoint() {
    return pivotController.atSetpoint();
  }

  public boolean isCubeDetected() {
    return (intakeRunningTimer.get() > 0.15) && //excludes current spike when motor first starts
      (intakeMotor.getOutputCurrent() > 25) && //cube intake current threshold
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
    return new InstantCommand(() -> setPivotTarget(deployAngleDegrees));
  }

  public Command pivotToOuttake() {
    return new InstantCommand(() -> setPivotTarget(outtakeAngleDegrees));
  }

  public Command pivotToRetract() {
    return new InstantCommand(() -> setPivotTarget(retractAngleDegrees));
  }

  public Command pivotToHold() {
    topFlywheelMotor.set(0);
    return new InstantCommand(() -> setPivotTarget(holdAngleDegrees));
  }

  public Command pivotToShoot() {
    return new InstantCommand(() -> setPivotTarget(shootAngleDegrees));
  }

  public Command deployAndIntake() {
    return new ParallelCommandGroup(
      pivotToDeploy(),
      intakeCommand());
  }

  public Command pivotThenOuttake() {
    return new SequentialCommandGroup(
      pivotToOuttake(),
      new WaitUntilCommand(() -> atSetpoint()),
      outtakeCommand()
    );
  }

  public Command pivotThenShoot() {
    return new SequentialCommandGroup(
      pivotToShoot(),
      new WaitUntilCommand(() -> atSetpoint()),
      shootCommand());
  }

  public Command holdAndStop() {
    return new ParallelCommandGroup(
      pivotToHold(),
      stopCommand());
  }

  public Command retractAndStop() {
    return new ParallelCommandGroup(
      pivotToRetract(), 
      stopCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isCubeDetected()) {pivotToHold().schedule();}
    
    setPivotPower(pivotController.calculate(getPivotAngleRadians()));
    setCalculatedFlywheelVoltage();
    
    SmartDashboard.putNumber("Intake Pivot", Units.radiansToDegrees(getPivotAngleRadians()));
  }
}
