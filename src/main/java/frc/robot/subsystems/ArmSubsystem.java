// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SparkMaxCanId;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * THE CLAW's arm rotates exerting more force on it from gravity as it 
 * moves to horizontal (straight out) necessitating a sine(theta) term
 * adjustment to motor power. Controlling the arm smoothly takes
 * both the feedforward model using this term along with other terms
 * that add an arbitrary voltage to the feedback PID control loop that is 
 * adjusting the output onboard the Spark Max controller based on the 
 * NEO relative encoder.  
 * 
 * Additionally, trapezoidal motion manages smooth acceleration and 
 * deceleration while keeping the speed in check. We used the WPIlib
 * example ArmBotOnboard as template code and modified for the Spark Max. 
 * 
 * The arm feedforward terms (kS,kG,kA,kV) were found using SysID WPILib tool.
 * While the feedback terms (kP, kI, kD) are manually tuned.
 */
public class ArmSubsystem extends TrapezoidProfileSubsystem implements Loggable{
  private final CANSparkMax m_motor = new CANSparkMax(SparkMaxCanId.kArmMotorCanId,
                                                      MotorType.kBrushless);
  private final SparkMaxPIDController m_pid;
  private final RelativeEncoder m_encoder;

  private final ArmFeedforward m_feedforward =
    new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  private final SlewRateLimiter m_armSlew = new SlewRateLimiter(ArmConstants.kArmSlewRate);

  private double m_goal = ArmConstants.kArmOffsetRads;

/** Create a new ArmSubsystem. */
public ArmSubsystem() {
super(
    new TrapezoidProfile.Constraints(
        ArmConstants.kMaxVelocityRadPerSecond, 
        ArmConstants.kMaxAccelerationRadPerSecSquared),
    ArmConstants.kArmOffsetRads);

m_motor.restoreFactoryDefaults();

// Setup the encoder and pid controller
m_encoder = m_motor.getEncoder();
m_encoder.setPosition(ArmConstants.kArmOffsetRads);
m_pid = m_motor.getPIDController();
m_pid.setFeedbackDevice(m_encoder);

// Apply position and velocity conversion factors for the arm encoder. These
// are natively in rotations and RPM, however, we want these 
// in radians and radians per second to use with the Spark Max PID
// and WPILib ArmFeedforward APIs .
m_encoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
m_encoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);

// Set the PID values for the turning motor.
m_pid.setP(ArmConstants.kP);
m_pid.setI(ArmConstants.kI);
m_pid.setD(ArmConstants.kD);
m_pid.setFF(ArmConstants.kFF);
m_pid.setOutputRange(ArmConstants.kMinOutput,
        ArmConstants.kMaxOutput);

// Apply current limit and idle mode
m_motor.setIdleMode(IdleMode.kBrake);
m_motor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);

// Save the SPARK MAX configurations.
m_motor.burnFlash();
}

@Override
public void periodic(){
  super.setGoal(m_goal);
  super.periodic();
  if (isArmDown()) {
    resetDownPosition();
  }
  if (isArmUp()) {
    resetUpPosition();
  }
}

@Override
public void useState(TrapezoidProfile.State setpoint) {
  // Calculate the feedforward from the sepoint
  double feedforward = m_feedforward.calculate(setpoint.position,
                                               setpoint.velocity);
  
  // Add the feedforward to the PID output to get the motor output
  m_pid.setReference(setpoint.position, // - ArmConstants.kArmOffsetRads,
                     ControlType.kPosition, 0, feedforward);
}

public CommandBase setArmGoalCommand(double goal) {
return Commands.runOnce(() -> setArmGoal(goal), this);
}

public void set(double speed) {
  m_motor.set(m_armSlew.calculate(speed));
}

// public CommandBase manualArmOrHold(double speed) {
//   return Commands.either(setArmGoalCommand(getPositionRadians()).withName("auto arm"),
//                  Commands.run(()->set(speed)).withName("manual arm"),
//                  ()->(Math.abs(speed) < ArmConstants.kArmDeadband));
// }

@Log
public double getPositionRadians() {
  return m_encoder.getPosition(); // + ArmConstants.kArmOffsetRads;
}

public CommandBase setArmManual(DoubleSupplier speed) {
  return Commands.run(()->setArmGoal(getArmGoal()+speed.getAsDouble()/(2*Math.PI)),this);
}

@Log
public double getArmGoal() {
  return m_goal;
}

public void setArmGoal(double goal) {
  
  m_goal = MathUtil.clamp(goal,ArmConstants.kArmOffsetRads-0.1,ArmConstants.kArmMaxRads+0.1);
}

public void resetDownPosition() {
  m_encoder.setPosition(ArmConstants.kArmOffsetRads);
  m_goal = ArmConstants.kArmOffsetRads;
}

public void resetUpPosition() {
  m_encoder.setPosition(ArmConstants.kArmMaxRads);
  m_goal = ArmConstants.kArmMaxRads;
}

@Log
public boolean isArmDown() {
  return m_motor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
}

@Log
public boolean isArmUp() {
  return m_motor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
}

public void setEncoderPosition(double position) {
  m_encoder.setPosition(position);
}



}
