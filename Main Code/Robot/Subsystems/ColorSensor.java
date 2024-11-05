package frc.robot.subsystems;
/*
 * Usman 3/3/24
 * Nothing else edited in the other files
 * 
 * NOTE FOR LATER:
 * SmartDashboard does NOT work in files other than Robot.java. I tested out this code, copy-pasted, in Robot.java, and it worked.
 * Doesn't seem to work by itself. Idk if it works in the back-end, or if it's able to work with motor control, but
 * it doesn't work with SmartDashboard at all.
 * So I guess...
 * To-do for 3/4/24, if working on Color Sensor:
 * 1. Transfer code to Robot.java, instead of in this file.
 * 2. Test to see if, even if it doesn't show on SmartDashboard, it's able to help control the intake motors.
 * GG
 */

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect pre-configured colors.
 */
public class ColorSensor extends TimedRobot {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private final Color kNoteTargetWithLight = new Color(0.569, 0.358, 0.073);
  private final Color kNoteTargetWithoutLight = new Color(0.553, 0.355, 0.092);

  private final SendableChooser<String> m_colorChooser = new SendableChooser<>();
  private String m_colorSelected;
  private final String
    red = "red",
    blue = "blue",
    green = "green";


  @Override
  public void robotInit() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);   
    m_colorMatcher.addColorMatch(kNoteTargetWithLight);
    m_colorMatcher.addColorMatch(kNoteTargetWithoutLight);

 
  }
  
  @Override
  public void robotPeriodic() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kNoteTargetWithLight) {
      colorString = "Note DETECTED";
      IntakeOutakeSubsystem.setFrontIntake(0);
    } else if (match.color == kNoteTargetWithoutLight) {
      colorString = "NOTE DETECTED";
      IntakeOutakeSubsystem.setFrontIntake(0);
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    switch (m_colorSelected) {
      case red:
        
        break;
      case blue:

        break;
      default:

        break;
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
}