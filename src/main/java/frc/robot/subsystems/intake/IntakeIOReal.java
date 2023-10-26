package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;

public class IntakeIOReal implements IntakeIO {
    private final CANSparkMax intakeMotor = new CANSparkMax(12, MotorType.kBrushed);
    //private final RelativeEncoder encoder = intakeMotor.getEncoder()

    public IntakeIOReal() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(25);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65000);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65000);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65000);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65000);
        intakeMotor.burnFlash();
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRPM = 0;
        inputs.currentAmps = intakeMotor.getOutputCurrent();
        inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    }

    public void set(double input) {
      intakeMotor.setVoltage(MathUtil.clamp(input * 12.0, -12.0, 12.0));
    }
}
