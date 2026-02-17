package frc.robot.Subsystems.Vision;

public record VisionMeasurement(
    edu.wpi.first.math.geometry.Pose2d pose,
    double timestampSeconds
){}