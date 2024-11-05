package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
/*Table of Conents
 * Line 16 - Distances
 * Line 25 - Moving Wrists
 * Line 79 - Periodic
 * Line 83 - Shooter Presets 
 */

public class RevEncoder extends SubsystemBase {

    public static DutyCycleEncoder wristEncoder = new DutyCycleEncoder(0);
    public static DutyCycleEncoder armEncoder = new DutyCycleEncoder(1);


    public static void getWristDistance() {
        System.out.println("Wrist Distance:" + wristEncoder.get());
        //CHANGED FROM .get() to .getAbsolutePosition()
        //Wrist values between 0.01 and 0.48
        //TODO: Set the motor limit to not move when the encoder sees that its past 0.5, NOT A PRIORITY
    }

    public static void getArmDistance() {
        System.out.println("Arm Distance: " + armEncoder.get());
        //CHANGED FROM .get() to .getAbsolutePosition()
        //Arm values between 0.153 and 0.874
    }

    public void moveSpeakerWrist() {
        //reset-oriented code 
        //0.4 ideal for 12.7
        if (wristEncoder.get() > 0.4) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            IntakeOutakeSubsystem.setWrist(Constants.DriveConstants.wristVolts); //positive moves it up
        }
        else if (wristEncoder.get() < 0.43) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            IntakeOutakeSubsystem.setWrist(-Constants.DriveConstants.wristVolts);
        }
        else {
            IntakeOutakeSubsystem.setWrist(0);
        }
    }

    public void moveSpeakerArm() {
        //reset-oriented code
        if (armEncoder.get() > 0.05) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            ArmSubsystem.setArm(Constants.DriveConstants.armVolts); 
        }
        else if (armEncoder.get() < 0.06) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            ArmSubsystem.setArm(-Constants.DriveConstants.armVolts);
        }
        else {
            ArmSubsystem.setArm(0);
        }
    }

    public void moveAmpWrist() { 
        if (wristEncoder.get() < 0.235) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            IntakeOutakeSubsystem.setWrist(Constants.DriveConstants.wristVolts); 
        }
        else if (wristEncoder.get() > 0.285) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            IntakeOutakeSubsystem.setWrist(-Constants.DriveConstants.wristVolts);
        }
        else {
            IntakeOutakeSubsystem.setWrist(0);
        }
    }

    public void moveAmpArm() { 
        if (armEncoder.get() > -0.09) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            ArmSubsystem.setArm(Constants.DriveConstants.armVolts); 
        }
        else if (armEncoder.get() < -0.115) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            ArmSubsystem.setArm(-Constants.DriveConstants.armVolts);
        }
        else {
            ArmSubsystem.setArm(0);
        }
    }

    public double moveIntakeWrist() {
        if (wristEncoder.get() < 0.37) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            IntakeOutakeSubsystem.setWrist(Constants.DriveConstants.wristVolts); 
        }
        else if (wristEncoder.get() > 0.41) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            IntakeOutakeSubsystem.setWrist(-Constants.DriveConstants.wristVolts);
        }
        else {
            IntakeOutakeSubsystem.setWrist(0);
            return 0.0;
        }

        return 0.0;
    }

    public double moveIntakeArm() {
        if (armEncoder.get() > -0.29) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            ArmSubsystem.setArm(Constants.DriveConstants.armVolts); 
        }
        else if (armEncoder.get() < -0.32) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            ArmSubsystem.setArm(-Constants.DriveConstants.armVolts);
        }
        else {
            ArmSubsystem.setArm(0);
            return 0.0;
        }

        return 0.0;
    }

    public void moveHangingWrist() { //VALUES NEED TO BE TWEAKED
        if (wristEncoder.getAbsolutePosition() < 0.35) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            IntakeOutakeSubsystem.setWrist(Constants.DriveConstants.wristVolts); 
        }
        else if (wristEncoder.getAbsolutePosition() > 0.37) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            IntakeOutakeSubsystem.setWrist(-Constants.DriveConstants.wristVolts);
        }
        else {
            IntakeOutakeSubsystem.setWrist(0);
        }
    }

    public void moveHangingArm() { //VALUES NEED TO BE TWEAKED
        if (armEncoder.getAbsolutePosition() > -0.29) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            ArmSubsystem.setArm(Constants.DriveConstants.armVolts); 
        }
        else if (armEncoder.getAbsolutePosition() < -0.34) {
             //CHANGED FROM .get() to .getAbsolutePosition()
            ArmSubsystem.setArm(-Constants.DriveConstants.armVolts);
        }
        else {
            ArmSubsystem.setArm(0);
        }
    }   
    

    @Override
    public void periodic() {
        getArmDistance();
        getWristDistance();
    }

    public void shooterSpeakerPreset() {
        moveSpeakerWrist();
        moveSpeakerArm();
        LedSubsystem.blueColor();
    }

    public void shooterAmpPreset() {
        moveAmpWrist();
        moveAmpArm();
        LedSubsystem.redColor();
    }

    public void intakePreset() {
        moveIntakeWrist();
        moveIntakeArm();
        LedSubsystem.greenColor();
    }

    public void hangingShootPreset() {
        moveHangingWrist();
        moveHangingArm();
        LedSubsystem.yellowColor();
    }

}
