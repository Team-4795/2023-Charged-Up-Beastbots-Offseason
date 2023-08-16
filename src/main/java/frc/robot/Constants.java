// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.utils.Setpoints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Seconds per cycle
  public static final double DT = 0.02;

  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /* Running a physics simulator. */
    SIM,

    /* Running on hardware. */
    REAL,

    /* Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidthX = Units.inchesToMeters(20.75);
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidthY = Units.inchesToMeters(20.75);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kTrackWidthX / 2, kTrackWidthY / 2),
      new Translation2d(kTrackWidthX / 2, -kTrackWidthY / 2),
      new Translation2d(-kTrackWidthX / 2, kTrackWidthY / 2),
      new Translation2d(-kTrackWidthX / 2, -kTrackWidthY / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // Chassis Angular Offset

    public static final double kChassisAngularOffset = -90;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 9;
  }

  public static final class ModuleConstants {
    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kWheelDiameterMeters = Units.inchesToMeters(2.9);
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorReduction = (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);

    public static final double kDrivingEncoderPositionFactor = 1.0 / kDrivingMotorReduction; // radians
    public static final double kDrivingEncoderVelocityFactor = (1.0 / kDrivingMotorReduction) / 60.0; // radians per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final int kDrivingMotorCurrentLimit = 60; // amps
    public static final int kTurningMotorCurrentLimit = 25; // amps
}

  public static final class ArmConstants{
    public static final double kP = 7;
    public static final double kI = 0.001;
    public static final double kD = 0;

    public static final double maxPosRev = 0.55;

    public static final double minPosRev = 0.0;
  }


  public static final class CubeSetpointConstants {
        public static final Setpoints kLowPickup = new Setpoints(0.0, 0);
        public static final Setpoints kStowHigh = new Setpoints(0.0, 0);
        public static final Setpoints kDoubleFeeder = new Setpoints(0.0, 0);
        public static final Setpoints kLowScore = new Setpoints(0.0, 0);
        public static final Setpoints kMidScore = new Setpoints(0.0, 0);
        public static final Setpoints kHighScore = new Setpoints(0.0, 0);
        public static final Setpoints kStowInFrame = new Setpoints(0.0, 0);
        public static final Setpoints kStowLow = new Setpoints(0.0, 0);
       
    }

    public static final class ConeSetpointConstants {
        public static final Setpoints kLowPickup = new Setpoints(0.0, 0);
        public static final Setpoints kStowHigh = new Setpoints(0.0, 0);
        public static final Setpoints kDoubleFeeder = new Setpoints(0.0, 0);
        public static final Setpoints kLowScore = new Setpoints(0.0, 0);
        public static final Setpoints kMidScore = new Setpoints(0.0, 0);
        public static final Setpoints kHighScore = new Setpoints(0.0, 0);
        public static final Setpoints kStowInFrame = new Setpoints(0.0, 0);
        public static final Setpoints kStowLow = new Setpoints(0.0, 0);
    }

    public static final class AutoConstants {
      public static PIDController AutoXcontroller = new PIDController(DT, DT, DT); 
      public static PIDController AutoYcontroller = new PIDController(DT, DT, DT); 
      public static PIDController AutoRotationcontroller = new PIDController(DT, DT, DT);
    }

}
