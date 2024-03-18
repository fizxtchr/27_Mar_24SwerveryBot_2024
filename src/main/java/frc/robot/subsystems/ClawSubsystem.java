// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClawConstants;

public class ClawSubsystem extends SubsystemBase {
  CANSparkBase m_rollerclaw =  new CANSparkMax(kClawConstants.kClawCanId, MotorType.kBrushed);
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    m_rollerclaw.setIdleMode(IdleMode.kBrake);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setClawSpeed(double speed) {
    m_rollerclaw.set(speed);
  }

  public void stop() {
    m_rollerclaw.set(0);
  }

  public void setClawWheel(double clawintakecommand) {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'setClawWheel'");
  }
}
