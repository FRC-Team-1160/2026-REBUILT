package frc.robot.Subsystems.Vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Vision.LimelightIO;
import frc.robot.Subsystems.Vision.VisionMeasurement;


import java.util.Optional;


public class VisionSubsystem extends SubsystemBase {


   private final LimelightIO limelight;


   public VisionSubsystem(LimelightIO limelightIO) {
       this.limelight = limelightIO;
       // pv later
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


   public double getTagDistance(){
       return limelight.getLLAbsoluteDistance();
   }


//    public Pose2d getPose2d(){
//        Pose2d curPose;
//        // temp
//        return curPose;   
//    }


//    public Pose3d return2dPoseTo3d(){
//        Pose3d curPose;
//        //temp
//        return curPose;
//    }


   @Override
   public void periodic() {


   }


   @Override
   public void simulationPeriodic() {


   }
}


