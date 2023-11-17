// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.arm.ArmIOReal;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Intake intake;
  private final Arm arm;
  private final DriveSubsystem drive;
  private final StateManager manager;
  // private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);


  // Dashboard inputs
  private final AutoSelector selector;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        intake = new Intake(new IntakeIOSim());
        arm = new Arm(new ArmIOSim());
        manager = new StateManager(arm, intake);
        drive = new DriveSubsystem();
        break;

      // Real robot, create hardware classes
      case REAL:
        intake = new Intake(new IntakeIOReal());
        arm = new Arm(new ArmIOReal());
        manager = new StateManager(arm, intake);
        drive = new DriveSubsystem();
        break;

      // Replayed robot, disable IO implementations
      default:
        intake = new Intake(new IntakeIO() {});
        arm = new Arm(new ArmIO() {});
        manager = new StateManager(arm, intake);
        drive = new DriveSubsystem();
        break;
    }
    
    selector = new AutoSelector(drive, arm, intake, manager);

    // Run arm using axis 1 (W,S) as input
    // Multiply input by 0.5 to reduce speed
    // arm.setDefaultCommand(
    //   Commands.run(
    //     () -> arm.run(controller.getRawAxis(1) * 0.5),
    //     arm
    //   )
    // );

    drive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> drive.drive(
              MathUtil.applyDeadband(ControlConstants.driverController.getRawAxis(ControlConstants.kDriveXSpeedAxis), OIConstants.kDriveDeadband),
              MathUtil.applyDeadband(-ControlConstants.driverController.getRawAxis(ControlConstants.kDriveYSpeedAxis), OIConstants.kDriveDeadband),
              MathUtil.applyDeadband(-ControlConstants.driverController.getRawAxis(ControlConstants.kDriveRotationAxis), OIConstants.kDriveDeadband),
              true,true),
          drive));

    // arm.setDefaultCommand(
    //     new RunCommand(
    //       () -> {
    //         double up = MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), OIConstants.kArmDeadband);
    //         double down = MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), OIConstants.kArmDeadband);

    //         double change = OIConstants.kArmManualSpeed * (Math.pow(up, 2) - Math.pow(down, 2));

    //         arm.move(change);
    //       }, arm)
    //   );


    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Key 'z' when using Keyboard 0 inside the Simulation GUI as port 0
    

    // controller.button(1)
    //     .whileTrue(new StartEndCommand(flywheel::run, flywheel::stop, flywheel));
    
    // operatorController.a().whileTrue(Commands.runOnce(arm::moveUp, arm));
    //operatorController.leftTrigger().whileTrue(Commands.run(arm::moveUp, arm));
    //operatorController.rightTrigger().whileTrue(Commands.run(arm::moveDown, arm));

    operatorController.povUp().onTrue(Commands.runOnce(manager::dpadUp));
    operatorController.povLeft().onTrue(Commands.runOnce(manager::dpadLeft));
    operatorController.povRight().onTrue(Commands.runOnce(manager::dpadRight));
    operatorController.povDown().onTrue(Commands.runOnce(manager::dpadDown));

    operatorController.rightTrigger().whileTrue(Commands.run(arm::moveDown, arm));
    operatorController.leftTrigger().whileTrue(Commands.run(arm::moveUp, arm));

    operatorController.leftBumper().whileTrue(Commands.run(intake::setIntake, intake));
    operatorController.rightBumper().whileTrue(Commands.run(intake::setOutake, intake));

    driverController.a().onTrue(Commands.runOnce(drive::zeroHeading, drive));

    }

    public void zeroHeading() {
      drive.zeroHeading();
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return selector.getSelected();
    //return Commands.none();
  }
}
