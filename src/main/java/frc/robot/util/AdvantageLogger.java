package frc.robot.util;

import java.lang.reflect.Method;

public final class AdvantageLogger {
  private static final String LOGGER_CLASS_NAME = "org.littletonrobotics.junction.Logger";
  private static final Method RECORD_DOUBLE = resolveMethod(String.class, double.class);
  private static final Method RECORD_BOOLEAN = resolveMethod(String.class, boolean.class);

  private AdvantageLogger() {
  }

  public static boolean isAvailable() {
    return RECORD_DOUBLE != null && RECORD_BOOLEAN != null;
  }

  public static void recordOutput(String key, double value) {
    invoke(RECORD_DOUBLE, key, value);
  }

  public static void recordOutput(String key, boolean value) {
    invoke(RECORD_BOOLEAN, key, value);
  }

  private static Method resolveMethod(Class<?>... signature) {
    try {
      Class<?> loggerClass = Class.forName(LOGGER_CLASS_NAME);
      return loggerClass.getMethod("recordOutput", signature);
    } catch (ReflectiveOperationException ignored) {
      return null;
    }
  }

  private static void invoke(Method method, Object... args) {
    if (method == null) {
      return;
    }
    try {
      method.invoke(null, args);
    } catch (ReflectiveOperationException ignored) {
      // Keep logging optional; runtime should not fail if AdvantageKit is unavailable.
    }
  }
}
