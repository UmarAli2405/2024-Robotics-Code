package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LedSubsystem extends SubsystemBase {
//led maybe
    public static AddressableLED m_led = new AddressableLED(0);
    public static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);
    // public static AddressableLED m_led_2 = new AddressableLED(1);
    // public static AddressableLEDBuffer m_ledBuffer_2 = new AddressableLEDBuffer(150);

    public LedSubsystem() {
        
    }

    

    public static void startLed() { //purple terra cyborgs color
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) { 
             m_ledBuffer.setRGB(i, 54, 1, 63);
        }
        m_led.setData(m_ledBuffer);
    }

    //  public static void startSecondLed() { //purple terra cyborgs color
    //      m_led_2.setLength(m_ledBuffer_2.getLength());
    //      m_led_2.start();
    //      for (var i = 0; i < m_ledBuffer_2.getLength(); i++) { 
    //           m_ledBuffer_2.setRGB(i, 130, 2, 52);
    //      }
    //      m_led.setData(m_ledBuffer_2);
    //  }

    public static void redColor() { //The color red, what more do you want? - used for amp preset
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) { 
             m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    public static void blueColor() {  //The color blue - used for shoot preset
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) { 
             m_ledBuffer.setRGB(i, 0, 0, 255);
        }
        m_led.setData(m_ledBuffer);
    }

    public static void greenColor() { //The color green - used for intake preset
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) { 
             m_ledBuffer.setRGB(i, 0, 255, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    public static void yellowColor() { //The color yellow - used for store away preset
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) { 
             m_ledBuffer.setRGB(i, 255, 255, 0);
        }
        m_led.setData(m_ledBuffer);
    }

}