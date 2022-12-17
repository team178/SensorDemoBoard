// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Map;

import org.letsbuildrockets.libs.TimeOfFlightSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class Robot extends TimedRobot {
  
  // DIO 0 = small switch
  // DIO 1 = medium switch
  // DIO 2 = bigger switch
  // DIO 3 = proximity sensor
  // I2C rev color sensor
  // CAN TOF sensor
  //    The ID parameter must match the hardware ID on the sticker on the front of the ToF sensor.
  // Talon motor controller
  // DIO 4-5 = Hall effect motor encoder
  // Built-in gyro?

  private final DigitalInput m_smallSwitch = new DigitalInput(0);
  private final DigitalInput m_mediumSwitch = new DigitalInput(1);
  private final DigitalInput m_largeSwitch = new DigitalInput(2);
  private final DigitalInput m_proxSensor = new DigitalInput(3);

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private ColorMatchResult m_colorMatch;

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  private final Encoder m_encoder = new Encoder(4, 5);

  private final TimeOfFlightSensor m_tof = new TimeOfFlightSensor(0x0623); // need to get actual id from sticker on sensor

  private ShuffleboardTab m_tab;

  @Override
  public void robotInit() {
    m_tab = Shuffleboard.getTab("SensorBoard");

    // Limit switches
    m_tab.addBoolean("Small Switch", () -> m_smallSwitch.get())
        .withPosition(0, 0);
    m_tab.addBoolean("Medium Switch", () -> m_mediumSwitch.get())
        .withPosition(1, 0);
    m_tab.addBoolean("Large Switch", () -> m_largeSwitch.get())
        .withPosition(2, 0);
    
    // Strange chinese proximity sensors
    m_tab.addBoolean("Proximity Sensor", () -> m_proxSensor.get())
        .withPosition(3, 0);

    // Encoder
    m_tab.add("Encoder", m_encoder)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(4, 0);

    // REV Color Sensor
    ShuffleboardLayout colorSensorLayout = m_tab.getLayout("Color Sensor", BuiltInLayouts.kGrid)
        .withPosition(0, 1)
        .withSize(2, 2);

    colorSensorLayout.addBoolean("Is Red?", () -> getColorSensorValue().equals("Red"))
        .withProperties(Map.of("Color when true", "Red", "Color when false", "Black"));
    colorSensorLayout.addBoolean("Is Blue?", () -> getColorSensorValue().equals("Blue"))
        .withProperties(Map.of("Color when true", "Blue", "Color when false", "Black"));
    colorSensorLayout.addBoolean("Is Green?", () -> getColorSensorValue().equals("Green"))
        .withProperties(Map.of("Color when true", "Green", "Color when false", "Black"));
    colorSensorLayout.addBoolean("Is Yellow?", () -> getColorSensorValue().equals("Yellow"))
        .withProperties(Map.of("Color when true", "Yellow", "Color when false", "Black"));

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    // Time of Flight sensor
    ShuffleboardLayout tofLayout = m_tab.getLayout("Time of Flight Sensor", BuiltInLayouts.kList)
        .withPosition(2, 1)
        .withSize(3, 4);

    tofLayout.addBoolean("In Range?", () -> m_tof.inRange());
    tofLayout.addNumber("Distance", () -> m_tof.inRange() ? m_tof.getDistance() : -1)
        .withWidget(BuiltInWidgets.kGraph)
        .withProperties(Map.of("Unit", "MM"));

  }
  
  public String getColorSensorValue() {
    if (m_colorMatch.color == kBlueTarget) {
      return "Blue";
    } else if (m_colorMatch.color == kRedTarget) {
      return "Red";
    } else if (m_colorMatch.color == kGreenTarget) {
      return "Green";
    } else if (m_colorMatch.color == kYellowTarget) {
      return "Yellow";
    } else {
      return "Unknown";
    }
  }

  @Override
  public void robotPeriodic() {
    Color m_detectedColor = m_colorSensor.getColor();
    m_colorMatch = m_colorMatcher.matchClosestColor(m_detectedColor);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
