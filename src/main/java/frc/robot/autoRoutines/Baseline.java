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
            autoCommands.changeStateCommand(State.LowScore),
            autoCommands.followtrajectoryCommand(baselinePath)
        );
        return baseline;
    }

    public Command AutoStartUp(PathPlannerTrajectory traj, boolean flip, EndEffectorIntake m_intake) {
        return 
            new SequentialCommandGroup( 
              new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if (flip) {
                zeroReverseHeading();
              } else {
                zeroHeading();
              }
              
              this.resetOdometry(PathPlannerTrajectory
                  .transformTrajectoryForAlliance(traj, DriverStation.getAlliance())
                  .getInitialHolonomicPose(), flip);
              
              this.setBreakMode();
            })
             // new InstantCommand(() -> m_intake.setOverrideStoring(true))
              );
    
      }

    private void setBreakMode() {
    }

    private void resetOdometry(Pose2d initialHolonomicPose, boolean flip) {
    }

    private void zeroHeading() {
    }

    private void zeroReverseHeading() {
    }
    
    
}
