package frc.robot.autoRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoCommands;

public abstract class AutoRoutine {
    public abstract Command load(AutoCommands autoCommands);
}
