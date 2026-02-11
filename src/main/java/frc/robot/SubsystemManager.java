/*
package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.Swerve;
import frc.robot.RobotContainer.JoystickInputs;
import frc.robot.Subsystems.DriveTrain.DriveTrain;

public class SubsystemManager {

     public static final SubsystemManager instance = new SubsystemManager();

     public void update(JoystickInputs stick_inputs) {
         double stick_x = MathUtil.applyDeadband(-stick_inputs.drive_x(), 0.1, 1)
                     * Swerve.DRIVE_SPEED;
                double stick_y = MathUtil.applyDeadband(-stick_inputs.drive_y(), 0.1, 1)
                     * Swerve.DRIVE_SPEED;
                double stick_a = MathUtil.applyDeadband(-stick_inputs.drive_a(), 0.1, 1)
                     * Swerve.TURN_SPEED;

                double stick_speed = RobotUtils.hypot(stick_x, stick_y);
                // Renormalize movement if combined vector is overspeed
                stick_x *= Math.min(Swerve.DRIVE_SPEED / stick_speed, 1);
                stick_y *= Math.min(Swerve.DRIVE_SPEED / stick_speed, 1);

                DriveTrain.instance.setSwerveDrive(stick_x, stick_y, stick_a);
     };
}
*/