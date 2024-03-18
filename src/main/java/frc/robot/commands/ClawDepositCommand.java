// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class ClawDepositCommand extends Command {
  ClawSubsystem m_rollerclaw;
  /** Creates a new ClawDepositCommand. */
  public ClawDepositCommand(ClawSubsystem rollerclaw) {

    m_rollerclaw = rollerclaw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rollerclaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rollerclaw.setClawSpeed(kClawConstants.ClawDepositCommand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollerclaw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
