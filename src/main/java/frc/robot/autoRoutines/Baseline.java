package frc.robot.autoRoutines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager.State;
import frc.robot.commands.AutoCommands;

public class Baseline extends AutoRoutine {

    PathPlannerTrajectory baselinePath = PathPlanner.loadPath("Drive back", new PathConstraints(4, 3));

    @Override
    public Command load(AutoCommands autoCommands) {
        SequentialCommandGroup baseline = new SequentialCommandGroup(
            autoCommands.autoStartUp(baselinePath),
            autoCommands.changeStateCommand(State.LowScore).withTimeout(3),
            autoCommands.Outtake(),
            autoCommands.followtrajectoryCommand(baselinePath)
        );
        return baseline;
    }       
    
}
