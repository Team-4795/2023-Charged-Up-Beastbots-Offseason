package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.StateManager;
import frc.robot.StateManager.State;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;

/* File where all commands used in auto routines can be stored */
public class AutoCommands {
    private DriveSubsystem drive;
    private Arm arm;
    private Intake intake;
    private StateManager state;

    public AutoCommands(DriveSubsystem drive, Arm arm, Intake intake, StateManager state) {
        this.drive = drive;
        this.arm = arm;
        this.intake = intake;
        this.state = state;
    }

    public Command followtrajectoryCommand(PathPlannerTrajectory Trajectory) {
        return drive.followTrajectoryCommand(Trajectory);

    }

    public Command changeStateCommand(State state) {
        return Commands.runOnce(() -> {
            this.state.setState(state);
        });
    }

    public Command Intake() {
        return Commands.runOnce(() -> {
            intake.setIntakeSpeed(0.2);
            ;
        });
    }

    public Command Outtake() {
        return Commands.runOnce(() -> {
            intake.setIntakeSpeed(-0.2);
            ;
        });
    }

    public Command Score(String Position) {
        StateManager.State desiredState;
        switch (Position) {
            case "high":
                desiredState = State.HighScore;
                break;
            case "mid":
                desiredState = State.MidScore;
                break;
            case "low":
                desiredState = State.LowScore;
                break;
            default:
                desiredState = State.StowInFrame;
        }
        return changeStateCommand(desiredState);
    }

    public Command autoStartUp(PathPlannerTrajectory trajectory){
        return Commands.runOnce(() -> drive.AutoStartUp(trajectory));
    }

    public Command ScoreTrajectory(String Position, PathPlannerTrajectory Trajectory) {
        return new ParallelDeadlineGroup(
                followtrajectoryCommand(Trajectory),
                Score(Position));
    }

    public Command IntakeTrajectory(PathPlannerTrajectory Trajectory) {
        return new ParallelDeadlineGroup(
                followtrajectoryCommand(Trajectory),
                new ParallelCommandGroup(
                    changeStateCommand(State.LowPickup),
                    Intake()
                ));
    }

}
