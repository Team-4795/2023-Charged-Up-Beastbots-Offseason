package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmVisualizer viz = new ArmVisualizer();

  private final ProfiledPIDController pidController;

  public double setpoint;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;
    io.updateInputs(inputs);
    setpoint = inputs.positionRev; //stops the arm from movnig upon start up
    pidController = new ProfiledPIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD,
      new TrapezoidProfile.Constraints(0.25, 1));
  }

  public void moveUp() {
    if(setpoint < ArmConstants.maxPosRev)
      this.setpoint += 0.01;
  }

  public void setPoint(double Position) {
    setpoint = Position; 
  }
    
  public void moveDown() {
    if(setpoint > ArmConstants.minPosRev)
      this.setpoint -= 0.01;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.set(pidController.calculate(inputs.positionRev, setpoint));

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);
    SmartDashboard.putNumber("Arm setpoint", setpoint);

    // Update visualizer position
    viz.update(inputs.positionRev);

  }
}
