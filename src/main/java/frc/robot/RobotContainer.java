// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClawDepositCommand;
import frc.robot.commands.ClawIntakeCommand;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.kLauncherConstants.kLauncherDelay;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CANLauncher m_Launcher = new CANLauncher();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ClawSubsystem m_claw = new ClawSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_scoringController = new CommandXboxController(OIConstants.kScoringControllerPort);

  private SendableChooser<String> m_autoChooser = new SendableChooser<String>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

      SmartDashboard.putNumber(Constants.kDashboardKeys.AUTO_DELAY, 0);
      m_autoChooser.addOption("Left", "Left");
      m_autoChooser.addOption("Center", "Center");
      m_autoChooser.addOption("Right", "Right");


      // m_autoChooser.addOption("Shoot then Drive", getShootThenDriveAuto());
      SmartDashboard.putData(m_autoChooser);

      m_robotDrive.zeroHeading();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // boolean backActive = m_driverController.getBackButton();
    // boolean startActive = m_driverController.getStartButton();

    // if ((backActive == true) && (startActive == true)) {
    //   new InstantCommand(() -> m_robotDrive.zeroHeading());
    // }


    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    
    new JoystickButton(m_driverController, Button.kL1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY()*OIConstants.kDrivingSlowdown, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX()*OIConstants.kDrivingSlowdown, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX()*OIConstants.kDrivingSlowdown, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    new Trigger(() -> m_scoringController.getRightY() > .5).whileTrue(m_Launcher.getIntakeCommand());

    var launchSequence = new SequentialCommandGroup(new PrepareLaunch(m_Launcher).withTimeout(kLauncherDelay), new LaunchNote(m_Launcher));
    new Trigger(() -> m_scoringController.getRightY() < -.5).whileTrue(launchSequence);
    m_scoringController.povUp().whileTrue(new ClimbUpCommand(m_climber));
    m_scoringController.povDown().whileTrue(new ClimbDownCommand(m_climber));
  
    new Trigger (() -> m_scoringController.getLeftY() > .5).whileTrue(new ClawDepositCommand(m_claw));
    new Trigger (() -> m_scoringController.getLeftY() < -.5).whileTrue(new ClawIntakeCommand(m_claw));
 
    //m_scoringController.y().whileTrue(new ClawIntakeCommand(m_claw));
    //m_scoringController.a().whileTrue(new ClawDepositCommand(m_claw));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoDelay = SmartDashboard.getNumber(Constants.kDashboardKeys.AUTO_DELAY, 0);
    var wait = new WaitCommand(autoDelay);
    Command auto = new PrintCommand("No Auto Chosen");
    var autoName = m_autoChooser.getSelected();
    switch (autoName) {
      case "Left":
        auto = this.getShootThenDriveAuto(Math.toRadians(0.0));
        break;
      case "Right":
        auto = this.getShootThenDriveAuto(Math.toRadians(165));
        break;
      case "Center":
        auto = this.getShootThenDriveAuto(Math.toRadians(-150));
        break;
      default:
        break;
    }
    
    return new SequentialCommandGroup(wait, auto);
  }

  // Autonomous Command for shooting then driving 
  public Command getShootThenDriveAuto(double startingAngle){
    Command shootCommand = getShootAuto();
    Command driveCommand = getDriveAuto(startingAngle);
    SequentialCommandGroup SandD = new SequentialCommandGroup(shootCommand, driveCommand);
    return SandD;
  }

  public Command getShootAuto() {
    var launchSequence = new SequentialCommandGroup(new PrepareLaunch(m_Launcher).withTimeout(kLauncherDelay), new LaunchNote(m_Launcher).withTimeout(kLauncherDelay));

    return launchSequence;
  }

  public Command getDriveAuto(double startingAngleRadians) {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(startingAngleRadians)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(0.5, 0)),
    //     // End 2 meters straight ahead of where we started, facing forward
    //     new Pose2d(2, 0, new Rotation2d(0)),
    //     config);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(startingAngleRadians)),
        // Pass through these two interior waypoints, making an 's' curve path
        new Pose2d(.5, 0, new Rotation2d(startingAngleRadians)),
        // End 2 meters straight ahead of where we started, facing forward
        new Pose2d(4, 0, new Rotation2d(0))
      ),
      config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    //m_robotDrive.zeroHeading();
    var resetOdometryCommand = new InstantCommand(() -> {
      m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    });

    // Run path following command, then stop at the end.
    return resetOdometryCommand
      .andThen(swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)));
    //return new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true, true));
  }
}
