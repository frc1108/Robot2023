// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SparkMaxCanId;

public class ArmSubsystem extends TrapezoidProfileSubsystem {
  private final CANSparkMax m_motor = new CANSparkMax(SparkMaxCanId.kArmMotorCanId, MotorType.kBrushless);
  private final SparkMaxPIDController m_pid;
  private final RelativeEncoder m_encoder;

private final ArmFeedforward m_feedforward =
  new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

/** Create a new ArmSubsystem. */
public ArmSubsystem() {
super(
    new TrapezoidProfile.Constraints(
        ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared),
    ArmConstants.kArmOffsetRads);

m_pid = m_motor.getPIDController();
m_pid.setP(ArmConstants.kP);

m_motor.restoreFactoryDefaults();
m_motor.setSmartCurrentLimit(40);
m_motor.setIdleMode(IdleMode.kBrake);

m_encoder = m_motor.getEncoder();

m_motor.burnFlash();
}

@Override
public void useState(TrapezoidProfile.State setpoint) {
// Calculate the feedforward from the sepoint
double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
// Add the feedforward to the PID output to get the motor output
m_pid.setReference(setpoint.position, ControlType.kPosition, 0, feedforward);
// m_motor.set(setSetpoint(
//     ExampleSmartMotorController.PIDMode.kPosition, setpoint.position, feedforward / 12.0);
}

public CommandBase setArmGoalCommand(double kArmOffsetRads) {
return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
}  


  

//   
}
