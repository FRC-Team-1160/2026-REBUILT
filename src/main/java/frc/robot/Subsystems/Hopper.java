package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.HopperConstants;
import static frc.robot.Constants.Port;

public class Hopper extends SubsystemBase {

    private TalonFX m_agitator = new TalonFX(Port.AGITATOR_MOTOR);

    // public static final Hopper instance = new Hopper();

    public Hopper() {
        
    }

    public void setAgitatorVoltage(double voltage) {
        m_agitator.setControl(new VoltageOut(voltage));
    }

    public void runAgitator(boolean on, boolean forward) {
        double voltage = on ? HopperConstants.AGITATOR_VOLTAGE : 0;
        double direction = forward ? 1 : -1;

        setAgitatorVoltage(voltage * direction);
    }

    public double getVoltageUsed() {
        return m_agitator.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper Voltage", getVoltageUsed());
    }
}
