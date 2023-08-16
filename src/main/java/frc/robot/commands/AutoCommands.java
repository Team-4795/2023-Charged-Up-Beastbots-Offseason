package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.StateManager;
import frc.robot.StateManager.State;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

/* File where all commands used in auto routines can be stored */
public class AutoCommands {
    private Drive drive; 
    private Arm arm;
    private Intake intake; 
    private StateManager state;     
    public AutoCommands(Drive drive, Arm arm, Intake intake, StateManager state) {
        this.drive = drive; 
        this.arm = arm; 
        this.intake = intake; 
        this.state = state; 
    }

    public Command followtrajectoryCommand(PathPlannerTrajectory Trajectory) {
     return drive.followTrajectoryCommand(Trajectory);
    
    }
    public Command changeStateCommand(State state) {
        return Commands.runOnce(() -> {this.state.setState(state);});
    }
}