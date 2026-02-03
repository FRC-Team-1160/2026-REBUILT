package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VoltageGetter;

import static frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements VoltageGetter {
    /* 
    public enum IntakeStates {
        OFF,
        FORWARD,
        BACKWARD
    }
        */

    private TalonFX m_intake = new TalonFX(IntakeConstants.INTAKE_PORT);

    // private double voltageUsed = 0;

    // idk how theyre going to deploy the intake

    public Intake() {}

    public void setIntakeVoltage(double voltage) {
        m_intake.setControl(new VoltageOut(voltage));
    }

    // would be better for commands to use this method instead
    // unless we want it to go backward
    public void runIntake(boolean on) {
        if (on) setIntakeVoltage(IntakeConstants.INTAKE_VOLTAGE);
        else setIntakeVoltage(0);
    }

    public void runIntake(boolean on, boolean forward) {
        double dir;

        forward ? dir = 1 : dir = -1;
    }

    /* 
    // probably dont use
    public void runIntake(IntakeStates currentState) {
        double voltage = 0;

        switch (currentState) {
            case FORWARD -> voltage = IntakeConstants.INTAKE_VOLTAGE;
            case BACKWARD -> voltage = -IntakeConstants.INTAKE_VOLTAGE;
            case OFF -> voltage = 0;
        }

        setIntakeVoltage(voltage);
    }
        */
    
    @Override
    public double getVoltageUsed() {
        return m_intake.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Voltage", getVoltageUsed());
    }
}
