// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

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
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.ShooterPreset;
import frc.robot.utils.TunableNumber;

public class Shooter extends SubsystemBase {
  private CANSparkMax kickerMotor;

  private CANSparkMax pivotMotor;
  private AbsoluteEncoder pivotEncoder;

  private CANSparkMax topFlywheelMotor;
  private CANSparkMax bottomFlywheelMotor;
  private RelativeEncoder flywheelEncoder;

  private PIDController pivotController = new PIDController(0, 0, 0);
  private PIDController flywheelController = new PIDController(0.5, 0, 0);

  public enum KickerState {
    INTAKING, OUTTAKING, STOPPED
  }

  private static KickerState kickerState = KickerState.STOPPED;
  private Timer kickerRunningTimer = new Timer();

  private boolean isShooterEnabled = false;

  public TunableNumber tunableVelocity = new TunableNumber("VELOCITY TUNEABLE");
  public TunableNumber tunablePivot = new TunableNumber("PIVOT TUNEABLE");
  
  /** Creates a new Shooter. */
  public Shooter() {
    kickerMotor = new CANSparkMax(ShooterConstants.kKickerMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ShooterConstants.kPivotMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    kickerMotor.setInverted(true);
    pivotMotor.setInverted(true);

    kickerMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    kickerMotor.setSmartCurrentLimit(ShooterConstants.kKickerMotorCurrentLimit);
    pivotMotor.setSmartCurrentLimit(ShooterConstants.kPivotMotorCurrentLimit);

    kickerMotor.enableVoltageCompensation(ShooterConstants.kKickerNominalVoltage);
    pivotMotor.enableVoltageCompensation(ShooterConstants.kKickerNominalVoltage);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(ShooterConstants.kPivotPositionConversionFactor);
    pivotEncoder.setInverted(false);
    pivotEncoder.setZeroOffset(ShooterConstants.kPivotEncoderZeroOffset);

    topFlywheelMotor = new CANSparkMax(ShooterConstants.kTopFlywheelMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkMax(ShooterConstants.kBottomFlywheelMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomFlywheelMotor.follow(topFlywheelMotor, true);
    topFlywheelMotor.setInverted(true);

    topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);

    flywheelEncoder = topFlywheelMotor.getEncoder();
    pivotEncoder.setVelocityConversionFactor(ShooterConstants.kFlywheelVelocityConversionFactor);//gear ratio

    pivotController.disableContinuousInput();
    pivotController.setTolerance(Units.degreesToRadians(7));

    tunablePivot.setDefault(ShooterConstants.kPivotHoldAngleDegrees);
    tunableVelocity.setDefault(0);
  }

  //enable funtions
  public void setShooterEnabled(boolean isShooterEnabled) {
    this.isShooterEnabled = isShooterEnabled;
  }

  //PIVOT
  public double getPivotAngleRadians() {
    return (pivotEncoder.getPosition() - (ShooterConstants.kPivotKinematicOffset)) / ShooterConstants.kPivotGearRatio;
  }

  public void setTargetPivot(double targetAngleDegrees) {
    pivotController.setP(ShooterConstants.kPivotP); //prevent jumping on enable p = 2.5
    pivotController.setSetpoint(Units.degreesToRadians(targetAngleDegrees));
  }

  public double getPivotTarget() {
    return pivotController.getSetpoint();
  }

  public boolean atPivotSetpoint() {
    return pivotController.atSetpoint();
  }

  public void setCalculatedPivotVoltage() {
    pivotMotor.setVoltage(isShooterEnabled ? 
      pivotController.calculate(getPivotAngleRadians())
      + ShooterConstants.kPivotFeedforward.calculate(pivotController.getSetpoint(), 0) : 0 );
  }

  //FLYWHEEL
  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity() / 1.0;//gear ratio
  }

  public void setTargetVelocity(double targetRPM) {
    flywheelController.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(targetRPM));
  }

  public double getVelocityTarget() {
    return flywheelController.getSetpoint();
  }

  public boolean atVelocitySetpoint() {
    return flywheelController.atSetpoint();
  }

  private void setCalculatedFlywheelVoltage() {
    topFlywheelMotor.setVoltage(isShooterEnabled ? flywheelController.calculate(getFlywheelVelocity()) : 0);
  }

  public void setTunable() {
    setTargetVelocity(tunableVelocity.get());
    setTargetPivot(tunablePivot.get());
  }

  private void kickerIntake() {
    kickerMotor.setVoltage(ShooterConstants.kKickerIntakeMotorSpeed*ShooterConstants.kKickerNominalVoltage);
    if (kickerState != KickerState.INTAKING) {
      kickerRunningTimer.reset();
      kickerRunningTimer.start();
    }
    kickerState = KickerState.INTAKING;
  }

  public KickerState getKickerState() {
    return kickerState;
  }

  public Command kickerOuttakeCommand(double power) {
    return new InstantCommand(() -> kickerOuttake(power));
  }

  private void kickerOuttake(double outtakeSpeed) {
    kickerMotor.set(outtakeSpeed);
    kickerState = KickerState.OUTTAKING;
  }

  public void stop() {
    setTargetVelocity(0);
    kickerMotor.set(0);
  }

  public Command stopCommand() {
    return new InstantCommand(() -> stop());
  }

  public Command intakeSequence() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> kickerIntake()),
      setPreset(ShooterConstants.kIntakePreset));
  }

  public Command outtakeSequence() {
    return new SequentialCommandGroup(
      setPreset(ShooterConstants.kOuttakePreset),
      new InstantCommand(() -> kickerOuttake(ShooterConstants.kKickerOuttakeMotorSpeed)));
  }

  public Command setPreset(ShooterPreset shooterPreset) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        setTargetVelocity(shooterPreset.FlywheelRPM);
        setTargetPivot(shooterPreset.PivotDegrees);
      }),
      new WaitUntilCommand(() -> atPivotSetpoint())
    );
  }

  public boolean isCurrentSpikeDetected() {
    return (kickerRunningTimer.get() > 0.15) && //excludes current spike when motor first starts
      (kickerMotor.getOutputCurrent() > 25) && //cube intake current threshold
      (kickerState == KickerState.INTAKING);
  }

  public boolean isCubeDetected() {
    return isCurrentSpikeDetected() 
      && (pivotController.getSetpoint() != Units.degreesToRadians(ShooterConstants.kPivotHoldAngleDegrees)) 
      && (flywheelController.getSetpoint() != 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setCalculatedPivotVoltage();
    setCalculatedFlywheelVoltage();
    
    SmartDashboard.putNumber("Shooter Pivot", Units.radiansToDegrees(getPivotAngleRadians()));
    SmartDashboard.putNumber("Shooter Target Pivot", Units.radiansToDegrees(getPivotTarget()));
    SmartDashboard.putNumber("Flywheel RPM", Units.radiansPerSecondToRotationsPerMinute(getFlywheelVelocity()));
    SmartDashboard.putNumber("Flywheel Target", flywheelController.getSetpoint());
  }
}
