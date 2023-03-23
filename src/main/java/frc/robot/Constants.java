// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final boolean kMirrorPathByAlliance = true;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class SparkMaxCanId {
    public static final int kArmMotorCanId = 21;
    public static final int kSliderMotorCanId = 31;
    // public static final int kIntakeFrontMotorCanId = 51;
    // public static final int kIntakeBackMotorCanId = 52;
  }

  public static final class ArmConstants {
    public static final double kSportGearRatio = 20.0;
    public static final double kSportPinionPitchInches = 1.125;
    public static final double kChainCenterDistanceInches = 6;
    public static final double kArmGearRatio = 80.0;

    // SysID values (in radians and radians/sec)
    public static final double kSVolts = 0.11356;
    public static final double kGVolts = 0.29175;
    public static final double kVVoltSecondPerRad = 1.5928;
    public static final double kAVoltSecondSquaredPerRad = 0.030171;
    public static final double kMaxVelocityRadPerSecond = 5;
    public static final double kMaxAccelerationRadPerSecSquared = 8;
    public static final double kArmOffsetRads = -1.3645; //Starting angle
    public static final double kArmMaxRads = 4; //Ending angle

    public static final double kArmEncoderPositionFactor = ((2 * Math.PI) / kArmGearRatio); // radians
    public static final double kArmEncoderVelocityFactor = ((2 * Math.PI) / kArmGearRatio) / 60.0; // radians per second

    public static final double kArmEncoderPositionPIDMinInput = kArmOffsetRads; // radians
    public static final double kArmEncoderPositionPIDMaxInput = (1.5 * Math.PI); // Guess 

    public static final int kArmMotorCurrentLimit = 40; // amps
    public static final double kP = 0.78697; //10000x
    public static final double kPVel = 5.534E-11;
    public static final double kI = 0;
    public static final double kD = 0.00002194;
    public static final double kDVel = 0;
    public static final double kFF = 0;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    public static final double kMaxPositionErrorRad = 0.7854;
    public static final double kMaxVelocityErrorRadPerSec = 1.8656;
    public static final double kControlEffortVolts = 7;

    public static final double kMaxArmSpeedRpm = 
      NeoMotorConstants.kFreeSpeedRpm / kArmGearRatio ;
    public static final double kMaxArmRadiansPerSecond =
      Units.rotationsPerMinuteToRadiansPerSecond(kMaxArmSpeedRpm);

    public static final double kMaxArmSpeed = 0.5;
    public static final double kArmSlewRate = 2;
    public static final double kArmDeadband = 0.1;

    public static final double kArmHighCubeOffsetRads = Units.degreesToRadians(5);
  }

  public static final class SliderConstants {
    public static final int kSliderMotorCurrentLimit = 20;
    public static final double kSliderDeadband = 0.1;
    public static final double kSliderSlewRate = 2;
    public static final double kMaxSliderSpeed = 0.75;

    public static final double kSVolts = 0.26164;
    public static final double kVVoltSecondPerMeters = 1.3196; //13.196
    public static final double kAVoltSecondSquaredPerMeters = 0.19577;

    public static final double kMaxVelocityMetersPerSecond = 0.75;
    public static final double kMaxAccelerationMetersPerSecSquared = 1.5;
    public static final double kSliderOffsetMeters = 0; //Starting position
    public static final double kSliderMaxMeters = Units.inchesToMeters(-15); //Ending position
    public static final double kSliderGearRatio = 20; 
    public static final double kSliderPinionPitchInches = 1.214; 

    public static final double kArmEncoderPositionFactor = Units.inchesToMeters(Math.PI*kSliderPinionPitchInches / kSliderGearRatio); // meters
    public static final double kArmEncoderVelocityFactor = Units.inchesToMeters(Math.PI*kSliderPinionPitchInches / kSliderGearRatio) / 60.0; // meters per second

    // public static final double kArmEncoderPositionPIDMinInput = kArmOffsetRads; // radians
    // public static final double kArmEncoderPositionPIDMaxInput = (1.5 * Math.PI); // Guess 

    public static final int kArmMotorCurrentLimit = 40; // amps
    public static final double kP = 4; //10000x
    public static final double kPVel = 5.534E-11;
    public static final double kI = 0;
    public static final double kD = 0.2;
    public static final double kDVel = 0;
    public static final double kFF = 0;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
  
    // public static final double kMaxArmSpeedRpm = 
    //   NeoMotorConstants.kFreeSpeedRpm / kSliderGearRatio ;
    // public static final double kMaxArmRadiansPerSecond =
    //   Units.rotationsPerMinuteToRadiansPerSecond(kMaxArmSpeedRpm);

    public static final double kMaxArmSpeed = 0.4;
    public static final double kArmSlewRate = 2;
    public static final double kArmDeadband = 0.06;
    public static final double kSliderHighCubeMeters = Units.inchesToMeters(-12);
    public static final double kSliderStowMeters = Units.inchesToMeters(0);

  }

  public static final class REVPHConstants {
    public static final int kForwardElevator = 4;
    public static final int kReverseElevator = 5;
    public static final int kForwardClaw = 6;
    public static final int kReverseClaw = 7;
  }

  public static final class LedConstants {
    public static final int kLedPWMPort = 9;
    public static final int kLedCount = 60;
  }

}
