package frc.robot;

import frc.robot.Constants.ConeSetpointConstants;
import frc.robot.Constants.CubeSetpointConstants;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.intake.*;
import frc.utils.Setpoints;
import org.littletonrobotics.junction.Logger;

public class StateManager extends VirtualSubsystem {
    private State state;
    private Gamepiece gamepiece;
    private static StateManager mInstance;

    public static StateManager getInstance() {
        if (mInstance == null) {
            mInstance = new StateManager();
        }

        return mInstance;
    }


     public StateManager() {
        state = State.StowInFrame;
        gamepiece = Gamepiece.Cube;
    }


    public enum Gamepiece {
        Cube,
        Cone,
    }


    public enum State {
        LowPickup(CubeSetpointConstants.kLowPickup, ConeSetpointConstants.kLowPickup),
        StowHigh(CubeSetpointConstants.kStowHigh, ConeSetpointConstants.kStowHigh),
        DoubleFeeder(CubeSetpointConstants.kDoubleFeeder, ConeSetpointConstants.kDoubleFeeder),
        LowScore(CubeSetpointConstants.kLowScore, ConeSetpointConstants.kLowScore),
        MidScore(CubeSetpointConstants.kMidScore, ConeSetpointConstants.kMidScore),
        HighScore(CubeSetpointConstants.kHighScore, ConeSetpointConstants.kHighScore),
        StowInFrame(CubeSetpointConstants.kStowInFrame, ConeSetpointConstants.kStowInFrame),
        StowLow(CubeSetpointConstants.kStowLow, ConeSetpointConstants.kStowLow);
        
        Setpoints cubeSetpoint;
        Setpoints coneSetpoint;

        State(Setpoints cubeSetpoint, Setpoints coneSetpoint) {
            this.cubeSetpoint = cubeSetpoint;
            this.coneSetpoint = coneSetpoint;
        }
    }


    public void pickCube() {
        gamepiece = Gamepiece.Cube;
    }

    public void pickCone() {
        gamepiece = Gamepiece.Cone;
    }

   

////////////////////////////////////////

// intake to be implemented errors are normal
// also states to be changed

    public void dpadUp() {
        if (Intake.isStoring()) {
            state = State.HighScore;
        } else {
            state = State.MidScore;
        }

        setSetpoints();
    }

    public void dpadLeft() {
        if (Intake.isStoring()) {
            state = State.LowScore;
        } else {
            state = State.LowPickup;
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
        state = State.LowPickup;
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
            case Cone:
                return state.coneSetpoint;
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
       // Arm.getInstance().setTargetPosition(getArmSetpoint());
       // Intake.getInstance().setOuttakeSpeed(getOuttakeSetpoint());
    }

    public void periodic() {
        Logger.getInstance().recordOutput("StateManager/Gamepiece", gamepiece.toString());
        Logger.getInstance().recordOutput("StateManager/State", state.toString());
    }
}