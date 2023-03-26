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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.InterpolatingTreeMap;

public class Shooter extends SubsystemBase {
  private Limelight m_frontLimelight;

  private CANSparkMax kickerMotor;
  private CANSparkMax pivotMotor;

  private CANSparkMax topFlywheelMotor;
  private CANSparkMax bottomFlywheelMotor;

  private RelativeEncoder flywheelEncoder;

  private AbsoluteEncoder pivotEncoder;

  private InterpolatingTreeMap velocityMap = new InterpolatingTreeMap();
  private InterpolatingTreeMap pivotMap = new InterpolatingTreeMap();

  private double pivotGearRatio = 50;

  private double deployAngleDegrees = 120;
  private double holdAngleDegrees = -30;
  private double retractAngleDegrees = -90;
  private double shootAngleDegrees = 45;
  private double outtakeAngleDegrees = 90;

  private PIDController pivotController = new PIDController(0, 0, 0);
  private PIDController flywheelController = new PIDController(0.5, 0, 0);

  //private ArmFeedforward pivotFeedforward = new ArmFeedforward(0, 0.49, 0.97, 0.01);

  public enum ShooterState {
    INTAKING, OUTTAKING, STOPPED
  }

  private static ShooterState shooterState = ShooterState.STOPPED;

  private Timer shooterRunningTimer = new Timer();

  /** Creates a new Shooter. */
  public Shooter(Limelight m_frontLimelight) {
    this.m_frontLimelight = m_frontLimelight;
    
    kickerMotor = new CANSparkMax(ShooterConstants.kKickerMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ShooterConstants.kPivotMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    kickerMotor.setInverted(true);
    pivotMotor.setInverted(false);

    kickerMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    kickerMotor.setSmartCurrentLimit(ShooterConstants.kKickerMotorCurrentLimit);
    pivotMotor.setSmartCurrentLimit(ShooterConstants.kPivotMotorCurrentLimit);

    kickerMotor.enableVoltageCompensation(ShooterConstants.kNominalVoltage);
    pivotMotor.enableVoltageCompensation(ShooterConstants.kNominalVoltage);

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

    m_frontLimelight.setAprilTagPipeline();

    populateVelocityMap();
    populatePivotMap();
  }

  private void populateVelocityMap() {
    velocityMap.put(0.0, 0.0);
    velocityMap.put(5.0, 1000.0);
  }

  private double getDynamicFlywheelVelocity() {
    return m_frontLimelight.isTargetVisible()
        ? velocityMap.getInterpolated(Units.metersToFeet(m_frontLimelight.getDistanceToGoalMeters()) + 0)
        : 0;
  }

  private void populatePivotMap() {
    pivotMap.put(0.0, 0.0);
    pivotMap.put(10.0, 120.0);
  }

  private double getDynamicPivot() {
    return m_frontLimelight.isTargetVisible()
        ? pivotMap.getInterpolated(Units.metersToFeet(m_frontLimelight.getDistanceToGoalMeters()))
        : holdAngleDegrees;
  }

  public void setDynamicShooter() {
    setFlywheelTargetVelocity(getDynamicFlywheelVelocity());
    setPivotTarget(getDynamicPivot());
  }

  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity() / 1.0;//gear ratio
  }

  public void setFlywheelTargetVelocity(double targetRPM) {
    flywheelController.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(targetRPM));
  }

  private void setCalculatedFlywheelVoltage() {
    topFlywheelMotor.setVoltage(flywheelController.calculate(getFlywheelVelocity()));
  }

  private void intake() {
    setFlywheelTargetVelocity(100);
    //topFlywheelMotor.set(0.4);
    kickerMotor.setVoltage(ShooterConstants.kIntakeMotorSpeed*ShooterConstants.kNominalVoltage);
    if (shooterState != ShooterState.INTAKING) {
      shooterRunningTimer.reset();
      shooterRunningTimer.start();
      shooterState = ShooterState.INTAKING;
    }
  }

  private void outtake(double power) {
    setFlywheelTargetVelocity(-100);
    //topFlywheelMotor.set(-0.4);
    kickerMotor.setVoltage(-power*ShooterConstants.kNominalVoltage);
    if (shooterState != ShooterState.OUTTAKING) {
      shooterRunningTimer.reset();
      shooterRunningTimer.start();
      shooterState = ShooterState.OUTTAKING;
    }
  }

  private void stop() {
    //setFlywheelTargetVelocity(0);
    //topFlywheelMotor.set(0);
    setFlywheelTargetVelocity(0);
    kickerMotor.set(0);
  }

  public double getPivotAngleRadians() {
    return pivotEncoder.getPosition() / pivotGearRatio - Units.degreesToRadians(37 + 86);
  }

  public void setPivotPower(double power) {
    pivotMotor.setVoltage(power * ShooterConstants.kNominalVoltage);
  }

  public void setCalculatedPivotVoltage() {
    // pivotMotor.setVoltage(
    //   pivotController.calculate(getPivotAngleRadians())
    //     + pivotFeedforward.calculate(pivotController.getSetpoint(), 0)
    //   );
    setPivotPower(pivotController.calculate(getPivotAngleRadians()));
  }

  public void setPivotTarget(double targetAngleDegrees) {
    if (pivotController.getP() == 0) { pivotController.setP(0.2);} //prevent jumping on enable
    pivotController.setSetpoint(Units.degreesToRadians(targetAngleDegrees));
  }

  public boolean atSetpoint() {
    return pivotController.atSetpoint();
  }

  public boolean isCubeDetected() {
    return (shooterRunningTimer.get() > 0.15) && //excludes current spike when motor first starts
      (kickerMotor.getOutputCurrent() > 25) && //cube shooter current threshold
      (shooterState == ShooterState.INTAKING);
  }

  public Command shooterCommand() {
    return new InstantCommand(() -> intake());
  }

  public Command outtakeCommand() {
    return new InstantCommand(() -> outtake(ShooterConstants.kOuttakeMotorSpeed));
  }

  public Command shootCommand() {
    return new InstantCommand(() -> outtake(ShooterConstants.kShootMotorSpeed));
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
    flywheelController.setSetpoint(0);
    return new InstantCommand(() -> setPivotTarget(holdAngleDegrees));
  }

  public Command pivotToShoot() {
    return new InstantCommand(() -> setPivotTarget(shootAngleDegrees));
  }

  public Command deployAndIntake() {
    return new ParallelCommandGroup(
      pivotToDeploy(),
      shooterCommand());
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
    if(isCubeDetected() 
    && (pivotController.getSetpoint() != Units.degreesToRadians(holdAngleDegrees)) 
    && (flywheelController.getSetpoint() != 0)) {pivotToHold().schedule();}
    
    setDynamicShooter();
    setCalculatedPivotVoltage();
    setCalculatedFlywheelVoltage();
    
    SmartDashboard.putNumber("Shooter Pivot", Units.radiansToDegrees(getPivotAngleRadians()));

    SmartDashboard.putNumber("Flywheel RPM", Units.radiansPerSecondToRotationsPerMinute(getFlywheelVelocity()));
    SmartDashboard.putNumber("interpolated velocity", getDynamicFlywheelVelocity());
    SmartDashboard.putNumber("interpolated pivot", getDynamicPivot());
    SmartDashboard.putNumber("front distance from goal", Units.metersToFeet(m_frontLimelight.getDistanceToGoalMeters()));
    SmartDashboard.putBoolean("target visible", m_frontLimelight.isTargetVisible());
  }
}