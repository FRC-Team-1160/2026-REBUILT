package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Shooter.Shooter.shooterModes;

public class ShooterController extends SubsystemBase{
    private Shooter shooter = new Shooter();

    public Command changeShooterMode(shooterModes mode) {
        return new InstantCommand(() -> {
            shooter.setShooterMode(mode);
        });
    };

    public Command enableShooter(boolean enable) {
        return new InstantCommand(() -> {
            shooter.enabled = enable;
        });
    };
}
