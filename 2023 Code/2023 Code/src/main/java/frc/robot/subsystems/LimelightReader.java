package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightReader {

    public static LimelightReader Instance;

    public NetworkTableEntry tx;
    public NetworkTableEntry ty;
    public NetworkTableEntry ta;
    public NetworkTableEntry ts;

    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 90.0;
    
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 31.5;
    
    // distance from the target to the floor
    double goalHeightInches = 25.5;
    
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    public static void InitLimeLight()
    {
        if(null == Instance)
        {
            Instance = new LimelightReader();
        }
    }


    public void UpdateLimeCam()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
    }

    public void UpdateSmartDashboard()
    {
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double s = ts.getDouble(0.0);
        double area = ta.getDouble(0.0);
    
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightS", s);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("Estimated Cam Distance in Inches", distanceFromLimelightToGoalInches);
    }
    
    public double GetEstimatedDistance(){
    //calculate distance
        return distanceFromLimelightToGoalInches;
    }

 

}
