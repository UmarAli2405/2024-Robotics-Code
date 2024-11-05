/*
 * Zaki 2/12/24 your code is not working better look at it
 */

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    public static CANSparkMax leftArm;
    public static CANSparkMax rightArm;
   // public static DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    public ArmSubsystem() {
        leftArm = new CANSparkMax(Constants.DriveConstants.leftArmSpark, MotorType.kBrushless);
        rightArm = new CANSparkMax(Constants.DriveConstants.rightArmSpark, MotorType.kBrushless);        
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public static void setArm(double v) {
        leftArm.setVoltage(v);
        rightArm.setVoltage(-v);
    }

}
