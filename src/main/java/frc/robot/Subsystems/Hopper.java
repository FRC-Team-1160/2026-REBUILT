package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VoltageGetter;

import static frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase implements VoltageGetter {

    private TalonFX m_agitator = new TalonFX(HopperConstants.AGITATOR_PORT);

    public Hopper() {

    }

    public void setAgitatorVoltage(double voltage) {
        m_agitator.setControl(new VoltageOut(voltage));
    }

    public void runAgitator(boolean onOff) {
        if (onOff) setAgitatorVoltage(HopperConstants.AGITATOR_VOLTAGE);
        else setAgitatorVoltage(0);
    }

    @Override
    public double getVoltageUsed() {
        return m_agitator.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper Voltage", getVoltageUsed());
    }
}
