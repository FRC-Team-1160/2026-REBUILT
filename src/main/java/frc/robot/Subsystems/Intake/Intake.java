package frc.robot.Subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import edu.wpi.first.wpilibj.DigitalInput;
public class Intake extends SubsystemBase {

    private final SparkMax extenderMotor = new SparkMax(Port.INTAKE_EXTENDER_MOTOR, null);
    private final RelativeEncoder encoder = extenderMotor.getEncoder();
    private final SparkMaxConfig extenderMotorConfig = new SparkMaxConfig();
    private final AlternateEncoderConfig extenderEncoderConfig = new AlternateEncoderConfig();

    private TalonFX intakeMotor = new TalonFX(Port.INTAKE_MOTOR);

    private DigitalInput armLimitSwitch = new DigitalInput(1);
    private boolean limitReached = false;

    public Intake() {
        extenderMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(IntakeConstants.EXTENDER_CURRENT_LIMIT);
        extenderEncoderConfig.positionConversionFactor(IntakeConstants.EXTENDER_GEAR_RATIO);
        extenderMotorConfig.apply(extenderEncoderConfig);
        encoder.setPosition(0);
        extenderMotor.configure(extenderMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() { 
        limitReached = !armLimitSwitch.get(); // true/false of whether or not the limit switch is being hit
        super.periodic();
    }
}
