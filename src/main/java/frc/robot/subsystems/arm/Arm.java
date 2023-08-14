package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmVisualizer viz = new ArmVisualizer();

  private final PIDController pidController;

  public double setpoint;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;

    pidController = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);

  }

  public void moveUp() {
    this.setpoint += 10;
  }

  public void moveDown() {
    this.setpoint -= 10;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.set(pidController.calculate(inputs.positionRev, setpoint));

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);

    // Update visualizer position
    viz.update(inputs.positionRev);
  }
}
