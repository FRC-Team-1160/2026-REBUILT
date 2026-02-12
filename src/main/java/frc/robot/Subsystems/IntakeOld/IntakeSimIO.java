package frc.robot.Subsystems.Intake;

import static frc.robot.Constants.IntakeConstants;

public class IntakeSimIO extends Intake {
    private double sim_intake_voltage;
    private double sim_deploy_voltage;

    protected IntakeSimIO() {
        sim_intake_voltage = 0;
        sim_deploy_voltage = 0;
    }

    @Override
    public void setDeployVoltage(double voltage) {
        sim_deploy_voltage = voltage;
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        sim_intake_voltage = voltage;
    }

    @Override
    public void runIntake(boolean on) {runIntake(on, true);}

    @Override
    public void runIntake(boolean on, boolean forward) {
        double voltage = on ? IntakeConstants.INTAKE_VOLTAGE : 0;
        double dir = forward ? 1 : -1;

        sim_intake_voltage = voltage * dir;
    }

    public double getVoltageUsed() {
        return sim_intake_voltage + sim_deploy_voltage;
    }
}
