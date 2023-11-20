package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmIOReal implements ArmIO {
    private final CANSparkMax leftArmMotor = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(11, MotorType.kBrushless);

    private final RelativeEncoder encoder;

    public ArmIOReal() {
        rightArmMotor.restoreFactoryDefaults();
        rightArmMotor.setInverted(true);
        leftArmMotor.follow(rightArmMotor, true);
        leftArmMotor.setSmartCurrentLimit(30);
        rightArmMotor.setSmartCurrentLimit(30);
        leftArmMotor.setOpenLoopRampRate(.5);
        leftArmMotor.setClosedLoopRampRate(.5);

        encoder = rightArmMotor.getEncoder();
        encoder.setPosition(0);
        encoder.setPositionConversionFactor(1.0/45.0);
        leftArmMotor.setIdleMode(IdleMode.kBrake);

        leftArmMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        leftArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        //leftArmMotor.setSoftLimit(SoftLimitDirection.kForward, (float)-0.2);
        //leftArmMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-0.9);

        
        rightArmMotor.burnFlash();
        leftArmMotor.burnFlash();
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.positionRev = encoder.getPosition();
        inputs.velocityRev = encoder.getVelocity();
        inputs.currentAmps = rightArmMotor.getOutputCurrent();
        inputs.appliedVolts = rightArmMotor.getAppliedOutput() * rightArmMotor.getBusVoltage();
        //SmartDashboard.putNumber("Position:", inputs.positionRev);
    }

    public void set(double input) {
        rightArmMotor.setVoltage(MathUtil.clamp(input * 12.0, -12.0, 12.0));
    }
}
