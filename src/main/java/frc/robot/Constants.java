// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kLeftFlightStickPort = 2;
    public static final int kRightFlightStickPort = 3;
    public static final double kTurretH2Deadband = 0.12;
  }

  public static class TurretConstants {
    public static final int kTurretMotorID = 35;
    public static final boolean kMotorInverted = false;

    // Very conservative startup limits for safe bring-up and hand tuning.
    public static final int kCurrentLimitA = 10;
    public static final double kOpenLoopRampSeconds = 3.0;
    public static final double kClosedLoopRampSeconds = 3.0;
    // public static final double kMinOutput = -0.08;
    // public static final double kMaxOutput = 0.08;

    public static final double kP = 0.005;
    public static final double kI = 0.0;
    public static final double kD = 0.5;

    // Total gearing from motor to turret.
    // REV MAXPlanetary 3:1 + motor gear to turret gear stage.
    public static final double kGearboxReduction = 3.0;
    public static final int kPinionTeeth = 45; // outside gear on motor output
    public static final int kTurretRingTeeth = 55; // inside gear on turret
    public static final boolean kExternalMeshInvertsDirection = true;

    public static final double kTurretRotationsPerMotorRotation =
        (1.0 / kGearboxReduction)
            * ((double) kPinionTeeth / (double) kTurretRingTeeth)
            * (kExternalMeshInvertsDirection ? -1.0 : 1.0);
    // REV primary encoder conversion factors must be positive.
    public static final double kTurretDegreesPerMotorRotation = 360.0 * Math.abs(kTurretRotationsPerMotorRotation);
    public static final double kTurretDirectionSign = kExternalMeshInvertsDirection ? -1.0 : 1.0;

    public static final Angle kHomeAngle = Units.Degrees.of(0.0);
    // Allowed turret command window in field-relative degrees.
    // Set to +/-180 so H2 can command the full direction circle.
    public static final Angle kMinAngle = Units.Degrees.of(-180.0);
    public static final Angle kMaxAngle = Units.Degrees.of(180.0);
    public static final Angle kTolerance = Units.Degrees.of(3.0);
    public static final Angle kSmoothWindow = Units.Degrees.of(20.0);
    public static final Angle kSmoothMaxStepPerCycle = Units.Degrees.of(2.0);
  }
}
