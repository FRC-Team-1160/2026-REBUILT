package frc.robot.Subsystems.Vision;

import frc.robot.Subsystems.Vision.VisionMeasurement;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;

import java.util.Optional;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class LimelightIO extends SubsystemBase{
   private Pose2d llPose;
   private double timestamp;
   private final String limelightName = "limelight";
   // private final NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
   private final double[] defaultArray = {0};
   private final LimelightHelpers.PoseEstimate poseEstimator =
               LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
   // nt instance above if needed
  
   public LimelightIO(){
        // this.limelightName = limelightName;
   }


   public Optional<VisionMeasurement> returnLLVisionMeasurement(){
          


       if(poseEstimator == null) return Optional.empty();
       if(poseEstimator.tagCount == 0) return Optional.empty();
       llPose = poseEstimator.pose;
       timestamp = poseEstimator.timestampSeconds;
       return Optional.of(new VisionMeasurement(llPose, timestamp));
   }


   public int getLLTagCount() {
       LimelightHelpers.PoseEstimate est =
       LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);


       if (est == null) return 0;
       return est.tagCount;
   }


   public double getLLRobotRelativeDistance(){
       LimelightHelpers.PoseEstimate est =
           LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);


       if (est == null || est.tagCount == 0) return 0.0;


       Pose2d pose = est.pose;
       // field relative
       return pose.getTranslation().getNorm();
   }


   public double getLLAbsoluteDistance(){
       // table.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(defaultArray); IF NOT MT2
       Pose2d llAbsoluteTag = poseEstimator.pose;
       return llAbsoluteTag.getTranslation().getNorm();
   }
}

