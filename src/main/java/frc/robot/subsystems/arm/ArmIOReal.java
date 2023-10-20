package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;

public class ArmIOReal implements ArmIO {
    private final CANSparkMax leftArmMotor = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(11, MotorType.kBrushless);

    private final AbsoluteEncoder encoder;

    public ArmIOReal() {
        rightArmMotor.restoreFactoryDefaults();
        rightArmMotor.setInverted(true);
        rightArmMotor.follow(leftArmMotor);
        leftArmMotor.setSmartCurrentLimit(30);
        rightArmMotor.setSmartCurrentLimit(30);
        encoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
        leftArmMotor.setIdleMode(IdleMode.kBrake);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.positionRev = encoder.getPosition();
        inputs.velocityRev = encoder.getVelocity();
        inputs.currentAmps = leftArmMotor.getOutputCurrent();
        inputs.appliedVolts = leftArmMotor.getAppliedOutput() * leftArmMotor.getBusVoltage();
    }

    public void set(double input) {
        leftArmMotor.setVoltage(MathUtil.clamp(input * 12.0, -12.0, 12.0));
    }
}
