package frc.robot.autoRoutines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager.State;
import frc.robot.commands.AutoCommands;

public class twoCube extends AutoRoutine {

    PathPlannerTrajectory twoCube1 = PathPlanner.loadPath("Two Cube Part 1", new PathConstraints(4, 3));
    PathPlannerTrajectory twoCube2 = PathPlanner.loadPath("Two Cube Part 2", new PathConstraints(4, 3));

    @Override
    public Command load(AutoCommands autoCommands) {
        SequentialCommandGroup twoCube = new SequentialCommandGroup(
            autoCommands.autoStartUp(twoCube1),
            autoCommands.changeStateCommand(State.HighScore),
            autoCommands.Outtake(),
            autoCommands.IntakeTrajectory(twoCube1),
            autoCommands.ScoreTrajectory("mid", twoCube2),
            autoCommands.Outtake()
        );
        return twoCube;
    }       
    
}
