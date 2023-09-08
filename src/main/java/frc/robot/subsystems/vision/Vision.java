package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
    public NetworkTable table; 
    private double tx;
    private double ty;
    private double ta;
    private double botPose;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("pipeline").setNumber(0);

    }

    @Override
    public void periodic() {
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("tx").getDouble(0.0);
        ta = table.getEntry("tx").getDouble(0.0);
        botPose = table.getEntry("botpose").getDouble(0.0);


        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
        SmartDashboard.putNumber("Bot Position", botPose);


    }
}
