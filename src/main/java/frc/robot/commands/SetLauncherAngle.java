// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class SetLauncherAngle extends Command {
  private final TurretSubsystem m_turretSubsystem;
  private final Angle m_targetAngle;
  private final boolean m_waitUntilReach;

  public SetLauncherAngle(
      TurretSubsystem turretSubsystem,
      Angle targetAngle,
      boolean waitUntilReach) {
    m_turretSubsystem = turretSubsystem;
    m_targetAngle = targetAngle;
    m_waitUntilReach = waitUntilReach;

    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {
    m_turretSubsystem.setLauncherAngle(m_targetAngle);
  }

  @Override
  public boolean isFinished() {
    return !m_waitUntilReach || !m_turretSubsystem.isCalibrated() || m_turretSubsystem.atSetpoint();
  }
}
