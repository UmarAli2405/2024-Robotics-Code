package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final double MOUNT_ANGLE_DEGREES = 0.0;
    private final double LENS_HEIGHT_INCHES = 0.0;

    public double TargetPoll(double targetHeight) {
        boolean v = table.getEntry("tv").getInteger(0) == 1;

        if (v) {
            estimateDistance(MOUNT_ANGLE_DEGREES, LENS_HEIGHT_INCHES, targetHeight);
        }

        return -1;
    }

    public void estimateDistance(double limelightMountAngleDegrees, double limelightLensHeightInches, double goalHeightInches) {
        NetworkTableEntry _ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = _ty.getDouble(0.0);
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    
        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
        System.out.println(distanceFromLimelightToGoalInches);
    }
}
