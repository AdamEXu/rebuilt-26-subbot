// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.AdvantageLogger;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class TurretSubsystem extends SubsystemBase {
  private final SparkFlex m_turret =
      new SparkFlex(TurretConstants.kTurretMotorID, MotorType.kBrushless);
  private Angle m_setpoint = TurretConstants.kHomeAngle;
  private boolean m_isCalibrated = false;

  public TurretSubsystem() {
    SparkFlexConfig cfg = new SparkFlexConfig();
    cfg.idleMode(IdleMode.kBrake);
    cfg.inverted(TurretConstants.kMotorInverted);

    // Relative encoder reports turret degrees after conversion (includes gearing and sign).
    cfg.encoder
        .positionConversionFactor(TurretConstants.kTurretDegreesPerMotorRotation)
        .velocityConversionFactor(TurretConstants.kTurretDegreesPerMotorRotation / 60.0);

    cfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);

    m_turret.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    calibrateForward();
  }

  public void setLauncherAngle(Angle angle) {
    if (!m_isCalibrated) {
      return;
    }
    double requestedDeg = MathUtil.clamp(
        angle.in(Units.Degrees),
        TurretConstants.kMinAngle.in(Units.Degrees),
        TurretConstants.kMaxAngle.in(Units.Degrees));

    double currentSetpointDeg = m_setpoint.in(Units.Degrees);
    // Keep target continuous around wrap boundaries based on last commanded setpoint.
    double deltaDeg = Math.IEEEremainder(requestedDeg - currentSetpointDeg, 360.0);

    // Smooth nearby corrections, but still snap quickly for large moves.
    if (Math.abs(deltaDeg) < TurretConstants.kSmoothWindow.in(Units.Degrees)) {
      deltaDeg = MathUtil.clamp(
          deltaDeg,
          -TurretConstants.kSmoothMaxStepPerCycle.in(Units.Degrees),
          TurretConstants.kSmoothMaxStepPerCycle.in(Units.Degrees));
    }

    m_setpoint = Units.Degrees.of(currentSetpointDeg + deltaDeg);
  }

  public void setLauncherAngle(Rotation2d angle) {
    setLauncherAngle(Units.Degrees.of(angle.getDegrees()));
  }

  public Rotation2d getLauncherAngle() {
    return Rotation2d.fromDegrees(getTurretPosition().in(Units.Degrees));
  }

  public Angle getTurretPosition() {
    return Units.Degrees.of(TurretConstants.kTurretDirectionSign * m_turret.getEncoder().getPosition());
  }

  public void calibrateForward() {
    double rawHomeDeg = TurretConstants.kHomeAngle.in(Units.Degrees) * TurretConstants.kTurretDirectionSign;
    m_turret.getEncoder().setPosition(rawHomeDeg);
    m_setpoint = TurretConstants.kHomeAngle;
    m_isCalibrated = true;
  }

  public boolean isCalibrated() {
    return m_isCalibrated;
  }

  public boolean atSetpoint() {
    if (!m_isCalibrated) {
      return false;
    }
    return Math.abs(m_setpoint.in(Units.Degrees) - getTurretPosition().in(Units.Degrees))
        <= TurretConstants.kTolerance.in(Units.Degrees);
  }

  @Override
  public void periodic() {
    double positionDeg = getTurretPosition().in(Units.Degrees);
    double setpointDeg = m_setpoint.in(Units.Degrees);
    AdvantageLogger.recordOutput("Turret/Calibrated", m_isCalibrated);
    AdvantageLogger.recordOutput("Turret/PositionDeg", positionDeg);
    AdvantageLogger.recordOutput("Turret/SetpointDeg", setpointDeg);
    AdvantageLogger.recordOutput("Turret/ErrorDeg", setpointDeg - positionDeg);
    AdvantageLogger.recordOutput("Turret/AtSetpoint", atSetpoint());

    if (!m_isCalibrated) {
      m_turret.set(0.0);
      return;
    }
    double rawSetpointDeg = setpointDeg * TurretConstants.kTurretDirectionSign;
    m_turret.getClosedLoopController().setSetpoint(rawSetpointDeg, ControlType.kPosition);
  }
}
