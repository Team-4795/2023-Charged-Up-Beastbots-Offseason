package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class IntakeIOReal implements IntakeIO {
    private final CANSparkMax intakeMotor = new CANSparkMax(0 /* Placeholder ID */, MotorType.kBrushless);

    public IntakeIOReal() {
        intakeMotor.restoreFactoryDefaults();
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.currentAmps = intakeMotor.getOutputCurrent();
        inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    }

    public void set(double input) {
      intakeMotor.setVoltage(MathUtil.clamp(input * 12.0, -12.0, 12.0));
    }
}
