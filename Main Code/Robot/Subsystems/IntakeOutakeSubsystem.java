/*
 * Anzar 2/16/24 stinky stinky code, just like all java code
 */

package frc.robot.subsystems;

import javax.management.RuntimeErrorException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class IntakeOutakeSubsystem extends SubsystemBase {

    public static CANSparkMax FrontIntakeMotor;
    public static CANSparkMax BackIntakeMotor;
    public static CANSparkMax RightWristMotor;
    public static CANSparkMax LeftWristMotor;

    
    


    Timer timer = new Timer();
    public static boolean toggleIntake = false;
    public static boolean togglePreset = false;
    public static boolean toggleInstantShoot = false; 
    
    public IntakeOutakeSubsystem() {
        BackIntakeMotor = new CANSparkMax(Constants.DriveConstants.intakeSpark, MotorType.kBrushless);
        FrontIntakeMotor = new CANSparkMax(Constants.DriveConstants.outakeSpark, MotorType.kBrushless);
        RightWristMotor = new CANSparkMax(Constants.DriveConstants.rightWristSpark, MotorType.kBrushless);
        LeftWristMotor = new CANSparkMax(Constants.DriveConstants.leftWristSpark, MotorType.kBrushless);
        
        
       

    }


   

    public static void setFrontIntake(double v) {
        // if (RobotContainer.m_commandController.getRightBumperPressed()) {
        //     toggleIntake = !toggleIntake;
        // }

        // if (toggleIntake) {
        FrontIntakeMotor.setVoltage(v);
        // }

        // else { 
        //     FrontIntakeMotor.setVoltage(0);
        // }
    }

    public static void setBackIntake(double v) {
        // if (RobotContainer.m_commandController.getLeftBumperPressed()) {
        //     toggleIntake = !toggleIntake;
        // }

        // if (toggleIntake) {
        BackIntakeMotor.setVoltage(v);
        // }

        // else { 
        //     FrontIntakeMotor.setVoltage(0);
        // }
    }
        
    public static void setWrist(double v) {
        RightWristMotor.setVoltage(v);
        LeftWristMotor.setVoltage(-v);
    }

    public void intakeOn(double v) {
        setFrontIntake(v);
        setBackIntake(v/12);
    }

    public void frontIntakeOn(double v) {
        setFrontIntake(v);
    }

    public void autoShootButActuallyAuto(double v, int ms) {
        setFrontIntake(-v);
        try {Thread.sleep(ms);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}
        setBackIntake(-v/12); 
    }

    public static void autoShoot(double v) {
        setFrontIntake(-v);
        try {Thread.sleep(700);}
        catch(InterruptedException e) {throw new RuntimeErrorException(null);}
        setBackIntake(-v/4); 
    }

    public void shoot(double v) { //main shoot for teleop
        setFrontIntake(-v);
        setBackIntake(-v/12);
        LedSubsystem.startLed();
    }

    public void shootInstant(double v) {
        setFrontIntake(v/12);
        setBackIntake(v);
        /*
        if (RobotContainer.m_commandController.getLeftStickButtonPressed()) {
            toggleInstantShoot = !toggleInstantShoot;
        }

        if (toggleInstantShoot) {
            setFrontIntake(-v);
            setBackIntake(-v/4);
        }

        else {
            setFrontIntake(0);
            setBackIntake(0);
        }
        */
     }

    
}
