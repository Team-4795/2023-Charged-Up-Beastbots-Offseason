package frc.robot.autoRoutines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StateManager.State;
import frc.robot.commands.AutoCommands;

public class OneCubeCableside extends AutoRoutine {
    PathPlannerTrajectory cubeCablePath = PathPlanner.loadPath("One Cube Cable", new PathConstraints(4, 3));

    @Override
    public Command load(AutoCommands autoCommands) {
        SequentialCommandGroup CubeCable = new SequentialCommandGroup(
            autoCommands.autoStartUp(cubeCablePath),
            autoCommands.changeStateCommand(State.LowScore).withTimeout(2),
            autoCommands.Outtake()
        );
        return CubeCable;
    }       
    
}
