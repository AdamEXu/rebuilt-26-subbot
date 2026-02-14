package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class TurretAimInputUtil {
  private TurretAimInputUtil() {
  }

  public static TurretAimResult calculateAim(double rawH2X, double rawH2Y, double deadband) {
    double rawMagnitude = Math.hypot(rawH2X, rawH2Y);
    if (rawMagnitude < deadband || rawMagnitude <= 0.0) {
      return new TurretAimResult(rawH2X, rawH2Y, 0.0, 0.0, 0.0, Optional.empty());
    }

    // Radial deadband keeps direction stable and feels smoother than per-axis deadbanding.
    double scaledMagnitude = Math.max(0.0, Math.min(1.0, (rawMagnitude - deadband) / (1.0 - deadband)));
    double scale = scaledMagnitude / rawMagnitude;
    double filteredX = rawH2X * scale;
    double filteredY = rawH2Y * scale * -1;
    double magnitude = Math.hypot(filteredX, filteredY);

    if (magnitude <= 0.0) {
      return new TurretAimResult(rawH2X, rawH2Y, 0.0, 0.0, 0.0, Optional.empty());
    }

    // H2 "up" is 0 deg, right is positive.
    Angle targetAngle = Units.Radians.of(Math.atan2(filteredX, -filteredY));
    return new TurretAimResult(rawH2X, rawH2Y, filteredX, filteredY, magnitude, Optional.of(targetAngle));
  }

  public record TurretAimResult(
      double rawH2X,
      double rawH2Y,
      double filteredH2X,
      double filteredH2Y,
      double magnitude,
      Optional<Angle> targetAngle) {
  }
}
