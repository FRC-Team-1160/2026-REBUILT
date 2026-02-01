package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private TalonFX m_intake = new TalonFX(IntakeConstants.INTAKE_PORT);

    // private double voltageUsed = 0;

    // idk how theyre going to deploy the intake

    public Intake() {}

    public void setIntakeVoltage(double voltage) {
        m_intake.setControl(new VoltageOut(voltage));
    }

    // would be better for commands to use this method instead
    public void runIntake(boolean on) {
        if (on) setIntakeVoltage(IntakeConstants.INTAKE_VOLTAGE);
        else setIntakeVoltage(0);
    }

    
    public double getVoltageUsed() {
        return m_intake.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        double voltageUsed = getVoltageUsed();
        SmartDashboard.putNumber("Intake Voltage", voltageUsed);
    }
}
