package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoRoutines.AutoRoutine;
import frc.robot.autoRoutines.Nothing;
import frc.robot.commands.AutoCommands;


public class AutoSelector {
    private final LoggedDashboardChooser<AutoRoutine> chooser = new LoggedDashboardChooser<>("Auto Selector");

    AutoCommands autoCommands;

    public AutoSelector() {
        autoCommands = new AutoCommands();

        chooser.addDefaultOption("Nothing", new Nothing());
    }

    public Command getSelected() {
        return chooser.get().load(autoCommands);
    }
}
