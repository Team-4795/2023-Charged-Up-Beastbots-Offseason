package frc.robot.autoRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class Nothing extends AutoRoutine {
    public Command load(AutoCommands autoCommands) {
        return Commands.none();
    }
}
