package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class AutoBalance extends CommandBase{

    Drive driveBase; 
    double speed; 
    double error; 
    
    public AutoBalance(Drive driveBase) {
        this.driveBase = driveBase;
        addRequirements(driveBase);
    }


    @Override
    public void execute() {
        speed = driveBase.getElevationAngle();
        driveBase.runVelocity(new ChassisSpeeds(0, speed, 0));

    }

    @Override 
    public boolean isFinished() {
        return false; 
    }
     
    
}