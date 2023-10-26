package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Setpoints;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Flywheel. */
  public Intake(IntakeIO io) {
    this.io = io;
    setDefaultCommand(new RunCommand(this::stop, this));
  }

  public void setIntake() {
    io.set(0.3);
  }

  public void setOutake() {
    io.set(-0.3);
  }

  public void stop() {
    io.set(0.0);
  }

  public void setIntakeSpeed(double speed){
    io.set(speed);
  }

  public static boolean isStoring(){
    return true;
  }

  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);

  }
}
