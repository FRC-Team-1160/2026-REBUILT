package frc.robot.Subsystems.Vision;

import frc.robot.Subsystems.Vision.VisionMeasurement;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;

import java.util.Optional;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotUtils;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

public class LimelightIO extends SubsystemBase{
    public boolean blueAlliance;
   private Pose2d llPose;
   private double timestamp;
   private final String limelightName = ShooterConstants.LIMELIGHT_NAME;
   // private final NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
   private final double[] defaultArray = {0};
   private final LimelightHelpers.PoseEstimate poseEstimator;
   // nt instance above if needed
  
   public LimelightIO(boolean blueAlliance){
    poseEstimator = getPoseEstimate();
    this.blueAlliance = blueAlliance;
   }

   public Optional<VisionMeasurement> returnLLVisionMeasurement(){
       if(poseEstimator == null || poseEstimator.tagCount == 0) return Optional.empty();
       llPose = poseEstimator.pose;
       timestamp = poseEstimator.timestampSeconds;
       return Optional.of(new VisionMeasurement(llPose, timestamp));
   }

   private LimelightHelpers.PoseEstimate getPoseEstimate() {
    LimelightHelpers.PoseEstimate est;
    if (blueAlliance) {est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);}
    else {est = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);}
    return est;
   }

   public int getLLTagCount() {
       LimelightHelpers.PoseEstimate est = getPoseEstimate();
       if (est == null) return 0;
       return est.tagCount;
   }

   // do these work? =====
   public double getLLRobotRelativeDistance(){
        LimelightHelpers.PoseEstimate est = getPoseEstimate();
        if (est == null || est.tagCount == 0) return 0;
        Pose2d pose = est.pose;
       // field relative
        return pose.getTranslation().getNorm();
   }

   public double getLLAbsoluteDistance(){
        // table.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(defaultArray); IF NOT MT2
        Pose2d llAbsoluteTag = poseEstimator.pose;
        return llAbsoluteTag.getTranslation().getNorm();
   }
   // ======

   public double getBotDistanceFromHubCenter(){
    double distance = RobotUtils.hypot((poseEstimator.pose.getX() - FieldConstants.HubMeasurements.ALLIANCEHUB_POSE.getX()), 
        poseEstimator.pose.getY() - FieldConstants.HubMeasurements.ALLIANCEHUB_POSE.getY());
    return distance;
   }

   public double getAngleDegreeOffsetFromHubCenter(double yawDeg) {
    double targetPoint = Math.atan2(FieldConstants.HubMeasurements.ALLIANCEHUB_POSE.getX() - poseEstimator.pose.getX(),
     FieldConstants.HubMeasurements.ALLIANCEHUB_POSE.getY() - poseEstimator.pose.getY()) * (180/Math.PI);
    double difference = yawDeg - targetPoint;
    return difference;
   }
}

