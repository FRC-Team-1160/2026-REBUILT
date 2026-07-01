package frc.robot.Subsystems.Agitator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorController extends SubsystemBase{
    private Agitator agitator = new Agitator();

    public Command runAgitatorAndGate(boolean forwards) {
        return new InstantCommand(() -> {
            agitator.runAgitation(forwards);
            agitator.runGate(forwards);
        });
    }

    public Command stopAgitatorAndGate() {
        return new InstantCommand(() -> {
            agitator.stopAgitation();
            agitator.stopGate();
        });
    }
}
