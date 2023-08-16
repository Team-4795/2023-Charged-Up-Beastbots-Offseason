package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Flywheel. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setIntake() {
    io.set(0.69);
  }

  public void setOutake() {
    io.set(-0.69);
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
