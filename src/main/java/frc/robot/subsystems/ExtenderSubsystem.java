// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SliderConstants;
import frc.robot.Constants.SparkMaxCanId;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ExtenderSubsystem extends TrapezoidProfileSubsystem implements Loggable {
  /** Creates a new ExtenderSubsystem. */ 
  private final CANSparkMax m_motor = new CANSparkMax(SparkMaxCanId.kSliderMotorCanId,
                                                      MotorType.kBrushless);
  private final SparkMaxPIDController m_pid;
  private final RelativeEncoder m_encoder;
  
  private final SimpleMotorFeedforward m_feedforward =
  new SimpleMotorFeedforward(
    SliderConstants.kSVolts,
    SliderConstants.kVVoltSecondPerMeters, SliderConstants.kAVoltSecondSquaredPerMeters);

  
  private final SlewRateLimiter m_sliderSlew = new SlewRateLimiter(SliderConstants.kSliderSlewRate);
  
  private double m_goal = 0;

  public ExtenderSubsystem() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(0, 0),
        // The initial position of the mechanism
        0);
        
        m_motor.restoreFactoryDefaults();

        // Setup the encoder and pid controller
        m_encoder = m_motor.getEncoder();
        m_pid = m_motor.getPIDController();
        m_pid.setFeedbackDevice(m_encoder);
        
        // Apply position and velocity conversion factors for the arm encoder. These
        // are natively in rotations and RPM, however, we want these 
        // in radians and radians per second to use with the Spark Max PID
        // and WPILib ArmFeedforward APIs .
        m_encoder.setPositionConversionFactor(SliderConstants.kArmEncoderPositionFactor);
        m_encoder.setVelocityConversionFactor(SliderConstants.kArmEncoderVelocityFactor);
        
        // Set the PID values for the turning motor.
        m_pid.setP(SliderConstants.kP);
        m_pid.setI(SliderConstants.kI);
        m_pid.setD(SliderConstants.kD);
        m_pid.setFF(SliderConstants.kFF);
        m_pid.setOutputRange(SliderConstants.kMinOutput,
                SliderConstants.kMaxOutput);
        
        // Apply current limit and idle mode
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setSmartCurrentLimit(SliderConstants.kArmMotorCurrentLimit);
        
        // Save the SPARK MAX configurations.
        m_motor.burnFlash();
        
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position,
                                                 setpoint.velocity);
    
    // Add the feedforward to the PID output to get the motor output
    m_pid.setReference((setpoint.position - SliderConstants.kSliderOffsetMeters),
                       ControlType.kPosition, 0, feedforward);
  }

  @Log
public double getPositionMeters() {
  return m_encoder.getPosition() + SliderConstants.kSliderOffsetMeters;
}

public CommandBase setArmManual(DoubleSupplier speed) {

  return Commands.run(()->setSliderGoal(getPositionMeters()+speed.getAsDouble()/2*Math.PI));
}

public double getSliderGoal() {
  return m_goal;
}

public void setSliderGoal(double goal) {
  m_goal = goal;
}

public void resetPosition() {
  m_encoder.setPosition(0);
}

@Log
public boolean isSliderOut() {
  return m_motor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
}
@Log
public boolean isSliderIn() {
  return m_motor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
}

}
