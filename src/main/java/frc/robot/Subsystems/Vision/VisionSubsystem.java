package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Vision.LimelightIO;
import frc.robot.Subsystems.Vision.VisionMeasurement;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

   private final LimelightIO limelight;

   public VisionSubsystem(LimelightIO limelightIO) {
       limelight = limelightIO;
       // pv later --- alex wtf does this mean
   }

   public Optional<VisionMeasurement> getVisionMeasurement() {
       // grab Limelight MegaTag2 pose
       Optional<VisionMeasurement> llMeasurement = limelight.returnLLVisionMeasurement();
       if (llMeasurement.isPresent()) {
           return llMeasurement;
       }
       return Optional.empty();
   }

   public double getVisionAbsoluteDistance(){
       // temp ll only
       return limelight.getLLAbsoluteDistance();
   }

// HELPERS

   public int getTagCount(){
       return limelight.getLLTagCount();
   }

   // ? ==
   public double getTagDistance(){
       return limelight.getLLAbsoluteDistance();
   }
   // ==

   public double getBotToHubDistance() {
    return limelight.getBotDistanceFromHubCenter();
   }

   public double getAngleDiffBotToHub(double yaw) {
    return limelight.getAngleDegreeOffsetFromHubCenter(yaw);
   }

   @Override
   public void periodic() {
    SmartDashboard.putNumber("LLTagCount", getTagCount());
    SmartDashboard.putNumber("botDistanceFromHub", getBotToHubDistance());
   }

   @Override
   public void simulationPeriodic() {

   }
}


