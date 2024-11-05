/*
 * Zaki Irfan 2/16/24 (cringe)
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ElevatorSubsystem extends SubsystemBase {
    public static CANSparkMax rightElevator;
    public static CANSparkMax leftElevator;
   // public DigitalInput limitSwitch = new DigitalInput(0);


    public ElevatorSubsystem() {
        rightElevator = new CANSparkMax(Constants.DriveConstants.rightElevatorSpark, MotorType.kBrushless);
        leftElevator = new CANSparkMax(Constants.DriveConstants.leftElevatorSpark, MotorType.kBrushless);
    }

    public void setElevator(double v) {
        rightElevator.setVoltage(v);
        leftElevator.setVoltage(-v);
    }
    
    /*
    public void setElevatorDown(double v) {
        if(limitSwitch.get() == false) {
            rightElevator.setVoltage(v);
            leftElevator.setVoltage(-v);
        }
        if(limitSwitch.get() == true) {
            rightElevator.setVoltage(0);
            leftElevator.setVoltage(0);            
        }
    }

    public void setElevatorUp(double v) {
        rightElevator.setVoltage(v);
        leftElevator.setVoltage(-v);
    }
    */

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        //System.out.println(limitSwitch.get());
    }
}
