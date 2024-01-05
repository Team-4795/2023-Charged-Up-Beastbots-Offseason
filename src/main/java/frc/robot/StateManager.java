package frc.robot;

import frc.robot.Constants.CubeSetpointConstants;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.intake.*;
import frc.utils.Setpoints;
import org.littletonrobotics.junction.Logger;

public class StateManager extends VirtualSubsystem {
    private State state;
    private Gamepiece gamepiece;
    private Arm arm; 
    private Intake intake;


     public StateManager(Arm arm, Intake intake) {
        state = State.StowInFrame;
        gamepiece = Gamepiece.Cube;
        this.arm = arm; 
        this.intake = intake; 
    }


    public enum Gamepiece {
        Cube
    }


    public enum State {
        LowPickup(CubeSetpointConstants.kLowPickup),
        StowHigh(CubeSetpointConstants.kStowHigh),
        DoubleFeeder(CubeSetpointConstants.kDoubleFeeder),
        LowScore(CubeSetpointConstants.kLowScore),
        MidScore(CubeSetpointConstants.kMidScore),
        HighScore(CubeSetpointConstants.kHighScore),
        StowInFrame(CubeSetpointConstants.kStowInFrame),
        StowLow(CubeSetpointConstants.kStowLow);
        
        Setpoints cubeSetpoint;

        State(Setpoints cubeSetpoint) {
            this.cubeSetpoint = cubeSetpoint;
        }
    }


    public void pickCube() {
        gamepiece = Gamepiece.Cube;
    }

   

////////////////////////////////////////

// intake to be implemented errors are normal
// also states to be changed

    public void dpadUp() {
        if (Intake.isStoring()) {
            state = State.HighScore;
        } else {
            state = State.StowInFrame;
        }

        setSetpoints();
    }

    public void dpadLeft() {
        if (Intake.isStoring()) {
            state = State.StowInFrame;
        } else {
            state = State.StowInFrame;
        }

        setSetpoints();
    }

    public void dpadDown() {
        if (Intake.isStoring()) {
            state = State.LowScore;
        } else {
            state = State.LowPickup;
        }

        setSetpoints();
    }

    public void dpadRight() {
        if(Intake.isStoring()){ 
        state = State.MidScore; 
        } else { 
        state = State.StowInFrame;
        }
        setSetpoints();
    }

     public void setState(State state) {
        this.state = state;
        setSetpoints();
    }

    public Setpoints getSetpoints() {
        switch (getGamepiece()) {
            case Cube:
                return state.cubeSetpoint;
            default:
                throw new AssertionError("Invalid gamepiece");
        }
    }


    public Double getArmSetpoint() {
        return getSetpoints().arm;
    }

    public Double getOuttakeSetpoint() {
        return getSetpoints().outtake;
    }


    public State getState() {
        return state;
    }


    public Gamepiece getGamepiece() {
        return gamepiece;
    }

    // wont work as of now becaue no methods are implemented yet

    public void setSetpoints() {
       arm.setPoint(getArmSetpoint());
       intake.setIntakeSpeed(getOuttakeSetpoint());
    }

    public void periodic() {
        Logger.getInstance().recordOutput("StateManager/Gamepiece", gamepiece.toString());
        Logger.getInstance().recordOutput("StateManager/State", state.toString());
    }
}