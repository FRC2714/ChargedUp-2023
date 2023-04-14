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
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterScoreLevel;
import frc.robot.utils.ShooterPreset;
import frc.robot.utils.TunableNumber;

public class Shooter extends SubsystemBase {
  private Limelight m_frontLimelight;
  private LED m_armLED;

  private CANSparkMax kickerMotor;

  private CANSparkMax pivotMotor;
  private AbsoluteEncoder pivotEncoder;

  private CANSparkMax topFlywheelMotor;
  private CANSparkMax bottomFlywheelMotor;
  private RelativeEncoder flywheelEncoder;

  private InterpolatingTreeMap<Double, Double> pivotMap = new InterpolatingTreeMap<Double, Double>();
  private InterpolatingTreeMap<Double, Double> velocityMap = new InterpolatingTreeMap<Double, Double>();

  private PIDController pivotController = new PIDController(0, 0, 0);
  private PIDController flywheelController = new PIDController(0.5, 0, 0);

  public enum IntakeState {
    INTAKING, OUTTAKING, STOPPED
  }

  private static IntakeState shooterState = IntakeState.STOPPED;
  private Timer kickerRunningTimer = new Timer();

  private boolean isShooterEnabled = false;
  private boolean isDynamicEnabled = false;

  public TunableNumber tunableVelocity = new TunableNumber("VELOCITY TUNEABLE");
  public TunableNumber tunablePivot = new TunableNumber("PIVOT TUNEABLE");
  
  /** Creates a new Shooter. */
  public Shooter(Limelight m_frontLimelight, LED m_armLED) {
    this.m_frontLimelight = m_frontLimelight;
    this.m_armLED = m_armLED;
    
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
    pivotEncoder.setZeroOffset(200);

    topFlywheelMotor = new CANSparkMax(ShooterConstants.kTopFlywheelMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkMax(ShooterConstants.kBottomFlywheelMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomFlywheelMotor.follow(topFlywheelMotor, true);
    topFlywheelMotor.setInverted(true);

    topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);

    flywheelEncoder = topFlywheelMotor.getEncoder();
    pivotEncoder.setVelocityConversionFactor(2*Math.PI*1.0);//gear ratio

    pivotController.disableContinuousInput();
    pivotController.setTolerance(Units.degreesToRadians(4));

    tunablePivot.setDefault(ShooterConstants.kPivotHoldAngleDegrees);
    tunableVelocity.setDefault(0);
  }

  //enable funtions
  public void setShooterEnabled(boolean isShooterEnabled) {
    this.isShooterEnabled = isShooterEnabled;
  }

  public Command setDynamicEnabledCommand(boolean isDynamicEnabled, ShooterScoreLevel shooterScoreLevel) {
    return new InstantCommand(() -> {
      if (shooterScoreLevel == ShooterScoreLevel.HIGH) {
        pivotMap.clear();
        pivotMap.put(100.0, 15.0);
        pivotMap.put(180.0, 30.0);
        pivotMap.put(300.0, 30.0);
        pivotMap.put(450.0, 30.0);

        velocityMap.clear();
        velocityMap.put(100.0, 70.0);
        velocityMap.put(180.0, 90.0);
        velocityMap.put(300.0, 110.0);
        velocityMap.put(450.0, 145.0);
      } else if (shooterScoreLevel == ShooterScoreLevel.MIDDLE) {
        pivotMap.clear();
        pivotMap.put(50.0, 20.0);
        pivotMap.put(110.0, 30.0);
        pivotMap.put(180.0, 30.0);
        pivotMap.put(290.0, 30.0);
        
        velocityMap.clear();
        velocityMap.put(50.0, 40.0);
        velocityMap.put(110.0, 60.0);
        velocityMap.put(180.0, 80.0);
        velocityMap.put(290.0, 120.0);
      } else if (shooterScoreLevel == ShooterScoreLevel.LOW) {
        pivotMap.clear();
        pivotMap.put(30.0, 100.0);
        pivotMap.put(80.0, 100.0);
        pivotMap.put(180.0, 90.0);
        pivotMap.put(280.0, 80.0);

        velocityMap.clear();
        velocityMap.put(30.0, 20.0);
        velocityMap.put(80.0, 50.0);
        velocityMap.put(180.0, 90.0);
        velocityMap.put(280.0, 120.0);
      }
      this.isDynamicEnabled = isDynamicEnabled;
    });
  }

  //KICKER
  public void setKicker(double power) {
    kickerMotor.set(power);
  }

  public Command setKickerCommand(double power) {
    return new InstantCommand(() -> {
      if (power < 0) {
        m_armLED.setRed();
      }
      kickerMotor.set(power);
    });
  }

  //PIVOT
  public double getPivotAngleRadians() {
    return (pivotEncoder.getPosition() - (115)) / ShooterConstants.kPivotGearRatio;
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

  //Dynamic
  private double getDynamicPivot() {
    return m_frontLimelight.isTargetVisible()
      ? pivotMap.get(Units.metersToFeet(m_frontLimelight.getDistanceToGoalMeters()))
      : ShooterConstants.kPivotHoldAngleDegrees;
    //return tunablePivot.get();
  }

  private double getDynamicVelocity() {
    return m_frontLimelight.isTargetVisible()
      ? velocityMap.get(Units.metersToFeet(m_frontLimelight.getDistanceToGoalMeters()) + 0)
      : 0;
    //return tunableVelocity.get();
  }

  public void setDynamicShooter() {
    if(isDynamicEnabled) {
      setTargetVelocity(getDynamicVelocity());
      setTargetPivot(getDynamicPivot());
    }
  }

  private void kickerIntake() {
    kickerMotor.setVoltage(ShooterConstants.kKickerIntakeMotorSpeed*ShooterConstants.kKickerNominalVoltage);
    if (shooterState != IntakeState.INTAKING) {
      kickerRunningTimer.reset();
      kickerRunningTimer.start();
      shooterState = IntakeState.INTAKING;
    }
  }

  private void kickerOuttake() {
    kickerMotor.setVoltage(ShooterConstants.kKickerOuttakeMotorSpeed*ShooterConstants.kKickerNominalVoltage);
    if (shooterState != IntakeState.OUTTAKING) {
      kickerRunningTimer.reset();
      kickerRunningTimer.start();
      shooterState = IntakeState.OUTTAKING;
    }
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
      new InstantCommand(() -> kickerOuttake()));
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
      (shooterState == IntakeState.INTAKING);
  }

  public boolean isCubeDetected() {
    return isCurrentSpikeDetected() 
      && (pivotController.getSetpoint() != Units.degreesToRadians(ShooterConstants.kPivotHoldAngleDegrees)) 
      && (flywheelController.getSetpoint() != 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setDynamicShooter();
    setCalculatedPivotVoltage();
    setCalculatedFlywheelVoltage();

    // SmartDashboard.putNumber("front limelight distance to goal", m_frontLimelight.getDistanceToGoalInches());
    // SmartDashboard.putNumber("front limelight goal height", m_frontLimelight.getGoalHeight());
    
    SmartDashboard.putNumber("Shooter Pivot", Units.radiansToDegrees(getPivotAngleRadians()));
    SmartDashboard.putNumber("Shooter Target Pivot", Units.radiansToDegrees(getPivotTarget()));
    SmartDashboard.putNumber("Flywheel RPM", Units.radiansPerSecondToRotationsPerMinute(getFlywheelVelocity()));
    SmartDashboard.putNumber("Flywheel Target", flywheelController.getSetpoint());
    // SmartDashboard.putNumber("interpolated velocity", getDynamicFlywheelVelocity());
    // SmartDashboard.putNumber("interpolated pivot", getDynamicPivot());
    // SmartDashboard.putNumber("front distance from goal", Units.metersToFeet(m_frontLimelight.getDistanceToGoalMeters()));
    // SmartDashboard.putBoolean("target visible", m_frontLimelight.isTargetVisible());
  }
}
