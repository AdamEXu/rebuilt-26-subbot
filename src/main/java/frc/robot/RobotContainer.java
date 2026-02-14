// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetLauncherAngle;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.AdvantageLogger;
import frc.robot.util.TurretAimInputUtil;
import frc.robot.util.TurretAimInputUtil.TurretAimResult;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import pwrup.frc.core.controller.FlightStick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  private final FlightStick m_rightFlightStick =
      new FlightStick(OperatorConstants.kRightFlightStickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AdvantageLogger.recordOutput("Turret/AdvantageKitAvailable", AdvantageLogger.isAvailable());
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    configureTurretDefaultCommand();
    configureTurretButtonBindings();
  }

  private void configureTurretDefaultCommand() {
    m_turretSubsystem.setDefaultCommand(m_turretSubsystem.run(() -> {
      TurretAimResult aim = TurretAimInputUtil.calculateAim(
          m_rightFlightStick.getH2X(),
          m_rightFlightStick.getH2Y(),
          OperatorConstants.kTurretH2Deadband);

      AdvantageLogger.recordOutput("Turret/H2/rawX", aim.rawH2X());
      AdvantageLogger.recordOutput("Turret/H2/rawY", aim.rawH2Y());
      AdvantageLogger.recordOutput("Turret/H2/filteredX", aim.filteredH2X());
      AdvantageLogger.recordOutput("Turret/H2/filteredY", aim.filteredH2Y());
      AdvantageLogger.recordOutput("Turret/H2/magnitude", aim.magnitude());
      AdvantageLogger.recordOutput("Turret/H2/hasTarget", aim.targetAngle().isPresent());

      if (!m_turretSubsystem.isCalibrated()) {
        return;
      }

      aim.targetAngle().ifPresent(m_turretSubsystem::setLauncherAngle);
    }));
  }

  private void configureTurretButtonBindings() {
    m_rightFlightStick.trigger().onTrue(
        new SetLauncherAngle(m_turretSubsystem, Units.Degrees.of(0.0), false));
    m_rightFlightStick.B5().onTrue(
        new SetLauncherAngle(m_turretSubsystem, Units.Degrees.of(20.0), false));
    m_rightFlightStick.B6().onTrue(
        new SetLauncherAngle(m_turretSubsystem, Units.Degrees.of(-20.0), false));

    // Manual re-home to forward if the startup pin alignment was off.
    m_rightFlightStick.B7().onTrue(m_turretSubsystem.runOnce(m_turretSubsystem::calibrateForward));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
