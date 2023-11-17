package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoRoutines.AutoRoutine;
import frc.robot.autoRoutines.Nothing;
import frc.robot.autoRoutines.OneCubeCableside;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;


public class AutoSelector {
    private final LoggedDashboardChooser<AutoRoutine> chooser = new LoggedDashboardChooser<>("Auto Selector");

    AutoCommands autoCommands;

    public AutoSelector(DriveSubsystem drive, Arm arm, Intake intake, StateManager state) {
        autoCommands = new AutoCommands(drive, arm, intake, state);

        chooser.addDefaultOption("Nothing", new Nothing());
        chooser.addOption("OneCubeCable", new OneCubeCableside());
    }

    public Command getSelected() {
        return chooser.get().load(autoCommands);
    }
}
