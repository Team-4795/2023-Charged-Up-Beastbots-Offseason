package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmVisualizer viz = new ArmVisualizer();

  private final CANSparkMax leftArmMotor = new CANSparkMax(0 /* Placeholder ID */, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor = new CANSparkMax(0 /* Placeholder ID */, MotorType.kBrushless);

  private final AbsoluteEncoder encoder;

  private final PIDController pidController;

  public double setpoint;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;
    encoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);

    rightArmMotor.follow(leftArmMotor);

    pidController = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);

  }

  public void moveUp() {
    leftArmMotor.set(1);
  }

  public void moveDown() {
    leftArmMotor.set(-1);
  }

  public void moveToSetpoint() {
    leftArmMotor.set(pidController.calculate(encoder.getPosition(), setpoint));
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);

    // Update visualizer position
    viz.update(inputs.positionRev);
  }
}
