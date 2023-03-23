// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.SliderConstants;
// import frc.robot.Constants.SparkMaxCanId;
// import io.github.oblarg.oblog.Loggable;

// public class SliderSubsystem extends SubsystemBase implements Loggable {
//   private final CANSparkMax m_motor = new CANSparkMax(SparkMaxCanId.kSliderMotorCanId,
//                                                       MotorType.kBrushless);
  
//   private final SlewRateLimiter m_sliderSlew = new SlewRateLimiter(SliderConstants.kSliderSlewRate);
  
//   /* TODO NEO 550 with 20:1  */
  
//   /** Creates a new SliderSubsystem. */
//   public SliderSubsystem() {
//     m_motor.restoreFactoryDefaults();
//     // Apply current limit and idle mode
//     m_motor.setIdleMode(IdleMode.kBrake);
//     m_motor.setSmartCurrentLimit(SliderConstants.kSliderMotorCurrentLimit);

//    // Save the SPARK MAX configurations.
// m_motor.burnFlash();
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public void set(double speed) {
//     speed = m_sliderSlew.calculate(speed);
//     m_motor.set(speed);
//   }
// }
