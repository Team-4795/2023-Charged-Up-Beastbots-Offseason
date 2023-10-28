// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.utils.Setpoints;
import java.util.Set;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Command;

import frc.utils.Setpoints;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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

  public static final Mode currentMode = Mode.REAL;

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
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 10.0; // radians per second
    public static final double kMagnitudeSlewRate = 10.0; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 10.0; // percent per second (1 = 100%)

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

    public static final double kChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;

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

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingMotorFreeSpeedRps = 5676 / 60;

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
    / kDrivingMotorReduction;

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
}

  public static final class ArmConstants{
    public static final double kP = 3;
    public static final double kI = 0;
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
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3; 
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      public static final PIDController AutoXcontroller = new PIDController(2, 0, 0);
      public static final PIDController AutoYcontroller = new PIDController(4, 0, 0);
      public static final PIDController AutoRotationcontroller = new PIDController(3.0, 0, 0);
  
      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;
      public static final double VisionXspeed = 0;
      public static final double VisionYspeed = 0;
      public static final double VisionMoveFastX = 0;
      public static final double VisionMoveFastY = .3;
      
      public static final double kIntakeDelay = 0.25;
      public static final double kIntakeWaitTime = 0.2;
      public static final double kOuttakeDelay = 0.25;
  
      public static final double toZeroBound = 0.000001;
  
      public static final double platformMaxAngle = 10;
      
      //constant speed during command
      public static final double balanceSpeed = 0.07;
  
      public static final double driveAngleThreshold = 12; //angle at which checking angle duration starts, in degrees
      //constant drive up speed
      public static final double driveBalanceSpeed = 0.4;
      //useless for Asheville
      public static final double angularVelocityErrorThreshold = 0.15;
      //coeffiecient of the polynomial function to calculate balancing speed
      public static final double polyCoeff = 1.3;
      //duration of checking for the angle to start autobalance 
      public static final double checkDuration = 0.075;
      //override duration for drive up to avoid foul
      public static final double overrideDuration = 4;
      public static final HashMap<String, Command> AutoEventMap = new HashMap<>();
  
      // Constraint for the motion profiled robot angle controller
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      public static final double zeroAngleThreshold = 0.15;
      public static final double deadbandValue = 11;
      public static final double oscillationTime = 0.06;
  
  
    }
    
    public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final int kOperatorControllerPort = 1;
      public static final double kDriveDeadband = 0.05;
      public static final double kArmDeadband = 0.05;
      public static final double kArmManualSpeed = 0.01;
    }
  
    public static final class ControlConstants {
      public static final GenericHID driverController = new GenericHID(OIConstants.kDriverControllerPort);
      public static final GenericHID operatorController = new GenericHID(OIConstants.kOperatorControllerPort);
  
      public static final int kArmUpAxis = 3;
      public static final int kArmDownAxis = 2;
      public static final int kDriveXSpeedAxis = 0;
      public static final int kDriveYSpeedAxis = 1;
      public static final int kDriveRotationAxis = 4;
      public static final int kAlignXSpeedAxis = 0;
      public static final int kAlignYSpeedAxis = 1;
  
      public static final JoystickButton driverA = new JoystickButton(driverController, 1);
      public static final JoystickButton driverB = new JoystickButton(driverController, 2);
      public static final JoystickButton driverX = new JoystickButton(driverController, 3);
      public static final JoystickButton driverY = new JoystickButton(driverController, 4);
      public static final JoystickButton driverBumperLeft = new JoystickButton(driverController, 5);
      public static final JoystickButton driverBumperRight = new JoystickButton(driverController, 6);
  
      public static final POVButton driverDpadUp = new POVButton(driverController, 0);
      public static final POVButton driverDpadLeft = new POVButton(driverController, 270);
      public static final POVButton driverDpadDown = new POVButton(driverController, 180);
      public static final POVButton driverDpadRight = new POVButton(driverController, 90);
  
      public static final JoystickButton operatorA = new JoystickButton(operatorController, 1);
      public static final JoystickButton operatorB = new JoystickButton(operatorController, 2);
      public static final JoystickButton operatorX = new JoystickButton(operatorController, 3);
      public static final JoystickButton operatorY = new JoystickButton(operatorController, 4);
      public static final JoystickButton operatorBumperLeft = new JoystickButton(operatorController, 5);
      public static final JoystickButton operatorBumperRight = new JoystickButton(operatorController, 6);
      public static final JoystickButton operatorJoystickRight = new JoystickButton(operatorController, 10);
  
      public static final POVButton operatorDpadUp = new POVButton(operatorController, 0);
      public static final POVButton operatorDpadLeft = new POVButton(operatorController, 270);
      public static final POVButton operatorDpadDown = new POVButton(operatorController, 180);
      public static final POVButton operatorDpadRight = new POVButton(operatorController, 90);
    }
    
    public static class VisionConstants {
      public static final String kSnakeEyesCamera = "OV5647";
      public static final double kCameraHeight = 21;
      public static final double kTargetHeight = 0;
      public static final double kCameraPitchRadians = 0;
      public static final double kTargetAngle = 2.6;
  
      /**
       * Physical location of the camera on the robot, relative to the center of the robot.
       */
      public static final Transform3d CAMERA_TO_ROBOT =
          new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
      public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
    }
}
