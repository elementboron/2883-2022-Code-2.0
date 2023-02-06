package frc.robot.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightReader {

    private static LimelightReader Instance;

    public static LimelightReader Instance(){
        if (Instance==null){
            Instance= new LimelightReader();
        }
        return Instance;
    }

    public NetworkTableEntry tx;
    public NetworkTableEntry ty;
    public NetworkTableEntry ta;
    public NetworkTableEntry ts;
    //this returns a double array of botpose, which is length 6, and holds <x,y,z,rotx,roty,rotz>, we only use x,y and rotz
    public NetworkTableEntry botPose;

 
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 90.0;
    
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 31.5;
    
    // distance from the target to the floor
    double goalHeightInches = 25.5;
    
    

    public static final double[] April3Pose = new double[] {6.2,0.6,0.0};
    public static final double[] DockWayPointRightRedInterior = new double[] {5.22,0.66,0.0};
    public static final double[] DockWayPointRightRedExterior = new double[] {3.1,0.3,0.0};
    public void UpdateLimeCam()
    {
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        ts = limelightTable.getEntry("ts");
        botPose = limelightTable.getEntry("botpose");
    }

    public void UpdateSmartDashboard()
    {
        //read values periodically

        //We assume that if we have some tx, then we also have other values related.

        double x = tx.getDouble(0.0);
        SmartDashboard.putNumber("LimelightX", x);


        double y = ty.getDouble(0.0);
        double s = ts.getDouble(0.0);
        double area = ta.getDouble(0.0);
    
        //post to smart dashboard periodically

        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightS", s);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("Estimated Cam Distance in Inches to ID 3", GetEstimatedDistance(April3Pose));

    }
    
    public double GetEstimatedDistance(double[] Target){
    //calculate distance
    if(botPose!=null){
        double[] poseInfo = botPose.getDoubleArray(new double[3]);
        if(poseInfo.length < 5)
        {
            return Double.MAX_VALUE;
        }
        double distance = Math.sqrt(Math.pow(poseInfo[0] - Target[0],2) + Math.pow(poseInfo[1] - Target[1],2) + Math.pow(poseInfo[2] - Target[2],2));
    //double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    //double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    //double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    //return distanceFromLimelightToGoalInches;
    return distance * 39.37;
    } else {
        return Double.MAX_VALUE;
    }
    }

    public static double[] GetDistanceVector(double x1, double y1, double rotz1, double x2, double y2, double rotz2){
        return new double[] {x2-x1, y2-y1, rotz2-rotz1};
    }

    public class BotPoseInfoHolder
    {
        public double X;
        public double Y;
        public double rotz;

        public BotPoseInfoHolder(double[] botposeArray)
        {
            X = botposeArray[0];
            Y = botposeArray[1];
            rotz = botposeArray[5];
        }
    }

    
}
 

