// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveSubsystem;

// public class TimedBasedDrive extends Command {
//   private DriveSubsystem s_DriveSubsystem;
//   private int seconds;
//   private double x;
//   private double y;
//   private double theta;

//   /** Creates a new TimedBasedDrive. */
//   public TimedBasedDrive(DriveSubsystem drive, x, y, theta, seconds) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     s_DriveSubsystem = drive;
//     this.seconds = 
//     timer = 0;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer = 0;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     timer += 20;
//     s_DriveSubsystem.drive(0, 0, 0, false, false);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
