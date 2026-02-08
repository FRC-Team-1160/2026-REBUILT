package frc.robot;


public class VoltageManager {
    private VoltageGetter subsystemsUsingVoltage[];

    public VoltageManager(VoltageGetter... subsystemsUsingVoltage) {
        this.subsystemsUsingVoltage = subsystemsUsingVoltage;
    }

    public double getTotalVoltageUsed() {
        double totalVoltageUsed = 0;

        for (VoltageGetter subsystem : subsystemsUsingVoltage) {
            totalVoltageUsed += subsystem.getVoltageUsed();
        }

        return totalVoltageUsed;
    }
}
