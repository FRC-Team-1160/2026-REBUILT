package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public abstract class Intake extends SubsystemBase {
    // funny version of the singleton pattern
    // where this class makes a single instance of its subclass
    public static Intake instance = Robot.isReal() ? new IntakeRealIO() : new IntakeSimIO();

    public abstract void setIntakeVoltage(double voltage);
    public abstract void runIntake(boolean on, boolean forward);
}