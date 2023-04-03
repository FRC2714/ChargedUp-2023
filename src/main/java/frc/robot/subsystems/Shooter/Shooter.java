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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterScoreLevel;
import frc.robot.utils.InterpolatingTreeMap;
import frc.robot.utils.ShooterPreset;
import frc.robot.utils.TunableNumber;

public class Shooter extends SubsystemBase {
  private Limelight m_frontLimelight;

  private CANSparkMax kickerMotor;
  private CANSparkMax pivotMotor;

  private CANSparkMax topFlywheelMotor;
  private CANSparkMax bottomFlywheelMotor;

  private RelativeEncoder flywheelEncoder;

  private AbsoluteEncoder pivotEncoder;

  private InterpolatingTreeMap velocityMap = new InterpolatingTreeMap();

  private InterpolatingTreeMap highVelocityMap = new InterpolatingTreeMap();
  private InterpolatingTreeMap middleVelocityMap = new InterpolatingTreeMap();
  private InterpolatingTreeMap lowVelocityMap = new InterpolatingTreeMap();

  private InterpolatingTreeMap pivotMap = new InterpolatingTreeMap();

  private InterpolatingTreeMap highPivotMap = new InterpolatingTreeMap();
  private InterpolatingTreeMap middlePivotMap = new InterpolatingTreeMap();
  private InterpolatingTreeMap lowPivotMap = new InterpolatingTreeMap();

  private PIDController pivotController = new PIDController(0, 0, 0);
  private PIDController flywheelController = new PIDController(0.5, 0, 0);

  private ArmFeedforward pivotFeedforward = new ArmFeedforward(0, 0.49, 0.97, 0.01);

  public enum IntakeState {
    INTAKING, OUTTAKING, STOPPED
  }

  private static IntakeState shooterState = IntakeState.STOPPED;

  private Timer shooterRunningTimer = new Timer();

  private boolean isShooterEnabled = false;
  private boolean isDynamicEnabled = false;

  public TunableNumber tunableVelocity = new TunableNumber("VELOCITY TUNEABLE");
  public TunableNumber tunablePivot = new TunableNumber("PIVOT TUNEABLE");
  

  /** Creates a new Shooter. */
  public Shooter(Limelight m_frontLimelight) {
    this.m_frontLimelight = m_frontLimelight;
    
    kickerMotor = new CANSparkMax(ShooterConstants.kKickerMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ShooterConstants.kPivotMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    kickerMotor.setInverted(true);
    pivotMotor.setInverted(true);

    kickerMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    kickerMotor.setSmartCurrentLimit(ShooterConstants.kKickerMotorCurrentLimit);
    pivotMotor.setSmartCurrentLimit(ShooterConstants.kPivotMotorCurrentLimit);

    kickerMotor.enableVoltageCompensation(ShooterConstants.kNominalVoltage);
    pivotMotor.enableVoltageCompensation(ShooterConstants.kNominalVoltage);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(ShooterConstants.kPivotPositionConversionFactor);
    pivotEncoder.setInverted(false);
    pivotEncoder.setZeroOffset(100);

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

    populateVelocityMap();
    populatePivotMap();

    tunablePivot.setDefault(ShooterConstants.kPivotHoldAngleDegrees);
    tunableVelocity.setDefault(0);
  }

  //Populate Maps
  private void populatePivotMap() {
    highPivotMap.put(100.0, 15.0);
    highPivotMap.put(180.0, 30.0);
    highPivotMap.put(300.0, 30.0);
    highPivotMap.put(450.0, 30.0);

    middlePivotMap.put(50.0, 20.0);
    middlePivotMap.put(110.0, 30.0);
    middlePivotMap.put(180.0, 30.0);
    middlePivotMap.put(290.0, 30.0);

    lowPivotMap.put(30.0, 100.0);
    lowPivotMap.put(80.0, 100.0);
    lowPivotMap.put(180.0, 90.0);
    lowPivotMap.put(280.0, 80.0);
  }

  private void populateVelocityMap() {
    highVelocityMap.put(100.0, 70.0);
    highVelocityMap.put(180.0, 90.0);
    highVelocityMap.put(300.0, 110.0);
    highVelocityMap.put(450.0, 145.0);

    middlePivotMap.put(50.0, 40.0);
    middlePivotMap.put(110.0, 60.0);
    middlePivotMap.put(180.0, 80.0);
    middlePivotMap.put(290.0, 120.0);

    lowPivotMap.put(30.0, 20.0);
    lowPivotMap.put(80.0, 50.0);
    lowPivotMap.put(180.0, 90.0);
    lowPivotMap.put(280.0, 120.0);
  }

  //enable funtions
  public void setShooterEnabled(boolean isShooterEnabled) {
    this.isShooterEnabled = isShooterEnabled;
  }

  public Command setDynamicEnabledCommand(boolean isDynamicEnabled, ShooterScoreLevel shooterScoreLevel) {
    return new InstantCommand(() -> {
      if (shooterScoreLevel == ShooterScoreLevel.HIGH) {
        pivotMap = highPivotMap;
        velocityMap = highVelocityMap;
      } else if (shooterScoreLevel == ShooterScoreLevel.MIDDLE) {
        pivotMap = middlePivotMap;
        velocityMap = middleVelocityMap;
      } else if (shooterScoreLevel == ShooterScoreLevel.LOW) {
        pivotMap = lowPivotMap;
        velocityMap = lowVelocityMap;
      }
      this.isDynamicEnabled = isDynamicEnabled;
    });
  }

  //KICKER
  public Command kick() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> kickerMotor.set(-0.4))
    );
  }

  //PIVOT
  public double getPivotAngleRadians() {
    return pivotEncoder.getPosition() / ShooterConstants.kPivotGearRatio - Units.degreesToRadians(178);
  }

  public void setTargetPivot(double targetAngleDegrees) {
    if (pivotController.getP() == 0) { pivotController.setP(2.5);} //prevent jumping on enable 2.5
    pivotController.setSetpoint(Units.degreesToRadians(targetAngleDegrees));
  }

  public double getPivotTarget() {
    return pivotController.getSetpoint();
  }

  public boolean atPivotSetpoint() {
    return pivotController.atSetpoint();
  }

  public void setCalculatedPivotVoltage() {
    if (isShooterEnabled) {
      pivotMotor.setVoltage(
        pivotController.calculate(getPivotAngleRadians())
        + pivotFeedforward.calculate(pivotController.getSetpoint(), 0));
    } else {
      pivotMotor.setVoltage(0);
    }
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
    if (isShooterEnabled) {
      topFlywheelMotor.setVoltage(flywheelController.calculate(getFlywheelVelocity()));
    } else {
      topFlywheelMotor.setVoltage(0);
    }
  }

  public void setTunable() {
    setTargetVelocity(tunableVelocity.get());
    setTargetPivot(tunablePivot.get());
  }

  //Dynamic
  private double getDynamicPivot() {
    return m_frontLimelight.isTargetVisible()
      ? highVelocityMap.getInterpolated(Units.metersToFeet(m_frontLimelight.getDistanceToGoalMeters()))
      : ShooterConstants.kPivotHoldAngleDegrees;
    //return tunablePivot.get();
  }

  private double getDynamicVelocity() {
    return m_frontLimelight.isTargetVisible()
      ? highVelocityMap.getInterpolated(Units.metersToFeet(m_frontLimelight.getDistanceToGoalMeters()) + 0)
      : 0;
    //return tunableVelocity.get();
  }

  public void setDynamicShooter() {
    if(isDynamicEnabled) {
      setTargetVelocity(getDynamicVelocity());
      setTargetPivot(getDynamicPivot());
    }
  }

  private void intake() {
    setTargetVelocity(-100);
    //topFlywheelMotor.set(0.4);
    kickerMotor.setVoltage(ShooterConstants.kIntakeMotorSpeed*ShooterConstants.kNominalVoltage);
    if (shooterState != IntakeState.INTAKING) {
      shooterRunningTimer.reset();
      shooterRunningTimer.start();
      shooterState = IntakeState.INTAKING;
    }
  }

  private void outtake(double power) {
    setTargetVelocity(100);
    //topFlywheelMotor.set(-0.4);
    kickerMotor.setVoltage(-power*ShooterConstants.kNominalVoltage);
    if (shooterState != IntakeState.OUTTAKING) {
      shooterRunningTimer.reset();
      shooterRunningTimer.start();
      shooterState = IntakeState.OUTTAKING;
    }
  }

  public void stop() {
    //setFlywheelTargetVelocity(0);
    //topFlywheelMotor.set(0);
    setTargetVelocity(0);
    kickerMotor.set(0);
  }

  public Command intakeCommand() {
    return new InstantCommand(() -> intake());
  }

  public Command outtakeCommand() {
    return new InstantCommand(() -> outtake(ShooterConstants.kOuttakeMotorSpeed));
  }

  public Command stopCommand() {
    return new InstantCommand(() -> stop());
  }

  public Command pivotToIntake() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setTargetPivot(ShooterConstants.kPivotIntakeAngleDegrees)),
      new WaitUntilCommand(() -> atPivotSetpoint()));
  }

  public Command pivotToOuttake() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setTargetPivot(ShooterConstants.kPivotOuttakeAngleDegrees)),
      new WaitUntilCommand(() -> atPivotSetpoint()));
  }

  public Command pivotToRetract() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setTargetPivot(ShooterConstants.kPivotRetractAngleDegrees)),
      new WaitUntilCommand(() -> atPivotSetpoint()));
  }

  public Command pivotToHold() {
    flywheelController.setSetpoint(0);
    return new SequentialCommandGroup(
      new InstantCommand(() -> setTargetPivot(ShooterConstants.kPivotHoldAngleDegrees)),
      new WaitUntilCommand(() -> atPivotSetpoint()));
  }

  public Command pivotToShoot(double pivotAngleDegrees) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setTargetPivot(pivotAngleDegrees)),
      new WaitUntilCommand(() -> atPivotSetpoint()));
  }

  public Command intakeSequence() {
    return new ParallelCommandGroup(
      intakeCommand(),
      pivotToIntake());
  }

  public Command outtakeSequence() {
    return new SequentialCommandGroup(
      pivotToOuttake(),
      outtakeCommand());
  }

  public Command shootSequence(ShooterPreset shooterPreset) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setTargetVelocity(shooterPreset.getVelocityRPM())),
      pivotToShoot(shooterPreset.getPivotAngleDegrees()),
      kick());
  }

  public boolean isCurrentSpikeDetected() {
    return (shooterRunningTimer.get() > 0.15) && //excludes current spike when motor first starts
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
    setDynamicShooter();
    setCalculatedPivotVoltage();
    setCalculatedFlywheelVoltage();

    SmartDashboard.putNumber("front limelight distance to goal", m_frontLimelight.getDistanceToGoalInches());
    SmartDashboard.putNumber("front limelight goal height", m_frontLimelight.getGoalHeight());
    
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
