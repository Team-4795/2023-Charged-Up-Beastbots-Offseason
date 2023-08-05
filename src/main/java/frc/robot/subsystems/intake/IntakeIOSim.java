package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 5, 0.02);
  private double volts = 0.0;

  public void updateInputs(IntakeIOInputs inputs) {
    sim.update(Constants.DT);

    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.currentAmps = sim.getCurrentDrawAmps(); 
    inputs.appliedVolts = volts;
  }

  public void set(double input) {
    volts = MathUtil.clamp(input * 12.0, -12.0, 12.0);
    sim.setInputVoltage(volts);
  }
}
