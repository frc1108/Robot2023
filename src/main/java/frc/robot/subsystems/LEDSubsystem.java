// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.ListIterator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LedConstants;
import frc.robot.utils.led.RainbowPattern;
import frc.robot.utils.led.SolidColorPattern;
import frc.robot.utils.led.TrobotAddressableLED;
import frc.robot.utils.led.TrobotAddressableLEDPattern;

public class LEDSubsystem extends Subsystem {
  private TrobotAddressableLED m_led = new TrobotAddressableLED(LedConstants.kLedPWMPort,
                                       LedConstants.kLedCount);

  private TrobotAddressableLEDPattern m_bluePattern = new SolidColorPattern(Color.kBlue);
	private TrobotAddressableLEDPattern m_redPattern = new SolidColorPattern(Color.kRed);
	private TrobotAddressableLEDPattern m_purplePattern = new SolidColorPattern(Color.kPurple);
	private TrobotAddressableLEDPattern m_yellowPattern = new SolidColorPattern(Color.kYellow);
	private TrobotAddressableLEDPattern m_disabledPattern = new RainbowPattern();
  
  private TrobotAddressableLEDPattern m_currentPattern;
  private List<TrobotAddressableLEDPattern> m_patternList;
  private ListIterator<TrobotAddressableLEDPattern> m_patternIterator;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_patternList = new ArrayList<TrobotAddressableLEDPattern>
                        (Arrays.asList(m_redPattern,m_bluePattern,
                                       m_yellowPattern, m_purplePattern,
                                       m_disabledPattern));

    m_patternIterator = m_patternList.listIterator();
    
    m_currentPattern =  //(DriverStation.getAlliance() == Alliance.Blue) ?
     m_bluePattern; // : m_redPattern;
  }


  public void nextPattern() {
       if (!m_patternIterator.hasNext()) {
        m_patternIterator = m_patternList.listIterator();
      }
      m_currentPattern = m_patternIterator.next();
  }

  public void setConePattern() {
    m_currentPattern = m_yellowPattern;
    m_patternIterator = m_patternList.listIterator(
                                      m_patternList.indexOf(m_yellowPattern));
  }

  public void setCubePattern() {
    m_currentPattern = m_purplePattern;
    m_patternIterator = m_patternList.listIterator(
                                      m_patternList.indexOf(m_purplePattern));
  }

  @Override
  public void periodic() {
    m_led.setPattern(m_currentPattern);
    // This method will be called once per scheduler run
  }
}
