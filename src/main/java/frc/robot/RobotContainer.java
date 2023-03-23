// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SliderConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.LEDSubsystem;
// import frc.robot.subsystems.SliderSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.VisionSubsystem;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.Map;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  @Log private final DriveSubsystem m_swerve = new DriveSubsystem();
  @Log private final ArmSubsystem m_arm = new ArmSubsystem();
  @Log private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  @Log private final ClawSubsystem m_claw = new ClawSubsystem();
  // @Log private final SliderSubsystem m_slider = new SliderSubsystem();
   @Log private final ExtenderSubsystem m_slider = new ExtenderSubsystem();
  @Log private final VisionSubsystem m_vision = new VisionSubsystem();
   @Log private final Superstructure m_superStruct = new Superstructure(m_arm, m_slider, m_elevator, m_claw);

  private final Autos autos = new Autos(m_swerve);
  private final LEDSubsystem m_leds = new LEDSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(
                                             OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(
                                             OIConstants.kOperatorControllerPort);

  // Autonomous selector on dashboard
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private GenericEntry kAutoStartDelaySeconds;
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Autonomous selector options
    kAutoStartDelaySeconds = Shuffleboard.getTab("Live")
                                         .add("Auto Delay", 0)
                                         .withWidget(BuiltInWidgets.kNumberSlider)
                                         .withProperties((Map.of("Min", 0, "Max", 10, "Block increment", 1)))
                                         .getEntry();
    autoChooser.setDefaultOption("Nothing", Commands.waitSeconds(5));
    autoChooser.addOption("Example Path", autos.example());
    autoChooser.addOption("AutoBalance",m_swerve.autoBalance());
    SmartDashboard.putData("Auto Chooser",autoChooser);

    // Configure default commands
    m_swerve.setDefaultCommand(
      new RunCommand(
        () -> m_swerve.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
          true, true),m_swerve));
    
    // m_arm.setDefaultCommand(
    //   new RunCommand(
    //     () -> m_arm.set(-ArmConstants.kMaxArmSpeed*
    //       MathUtil.applyDeadband(m_operatorController.getRightY(),
    //       ArmConstants.kArmDeadband)),m_arm));

    // m_arm.setDefaultCommand(
    //   m_arm.manualArmOrHold(-ArmConstants.kMaxArmSpeed*
    //     MathUtil.applyDeadband(m_operatorController.getRightY(),
    //     ArmConstants.kArmDeadband))
    // );

    m_arm.setDefaultCommand(
      m_arm.setArmManual(()->-ArmConstants.kMaxArmSpeed*
      MathUtil.applyDeadband(m_operatorController.getRightY(),
      ArmConstants.kArmDeadband))
    );
          
  //   m_slider.setDefaultCommand(
  //     new RunCommand(
  //       () -> m_slider.set(SliderConstants.kMaxSliderSpeed*
  //         MathUtil.applyDeadband(m_operatorController.getLeftY(),
  //         SliderConstants.kSliderDeadband)),m_slider));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Set drive wheels in x pattern to keep in place
    m_driverController.rightBumper().whileTrue(Commands.run(m_swerve::setX));

    // Reset gyro when A button is pressed 
    m_driverController.a().onTrue(Commands.runOnce(m_swerve::zeroHeading));

    // Go through LED Patterns on driver button
    m_driverController.b().onTrue(Commands.runOnce(m_leds::nextPattern,m_leds));

    // Autobalance testing
    //m_driverController.y().whileTrue(m_swerve.autoBalance());

    // Auto score testing
    // m_driverController.x().onTrue(m_superStruct.scoreCubeAutoCommand());

    // Move the arm to 2 radians above horizontal when the 'A' button is pressed.
    m_operatorController.y().onTrue(m_arm.setArmGoalCommand(Units.degreesToRadians(30)));

    // Move the arm to neutral position when the 'B' button is pressed.
    m_operatorController
        .b()
        .onTrue(m_arm.setArmGoalCommand(Units.degreesToRadians(15) + Constants.ArmConstants.kArmOffsetRads));

    // Elevator control on POV
    m_operatorController.povUp().onTrue(m_elevator.upCommand());
    m_operatorController.povDown().onTrue(m_elevator.downCommand());

    // Claw control on button A (grip) & X (release)
    m_operatorController.a().onTrue(m_claw.gripCommand());
    m_operatorController.x().onTrue(m_claw.releaseCommand());

    // Request LED color change for human player stations
    m_operatorController.povLeft().onTrue(Commands.runOnce(
                                          ()->m_leds.setConePattern(),m_leds));
    m_operatorController.povRight().onTrue(Commands.runOnce(
                                          ()->m_leds.setCubePattern(),m_leds));
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
      Commands.waitSeconds(kAutoStartDelaySeconds.getDouble(0)),
      autoChooser.getSelected());
  }

  public void resetAutoHeading() {
    m_swerve.zeroHeading();
  }
}
