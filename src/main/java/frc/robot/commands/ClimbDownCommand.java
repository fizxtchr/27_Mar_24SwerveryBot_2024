// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbDownCommand extends Command {
   ClimberSubsystem m_climber;
  /** Creates a new ClimbDownCommand. */
  public ClimbDownCommand(ClimberSubsystem climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setClimberSpeed(kClimberConstants.ClimbDownCommand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
