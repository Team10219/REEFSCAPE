// Copyright (c) 2025 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.MechanicalAdvantage;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class SparkUtil {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return;
      }
    }
    consumer.accept(values);
  }

  /** Return a value from a Spark (or the default if the value is invalid). */
  public static double ifOkOrDefault(
      SparkBase spark, DoubleSupplier supplier, double defaultValue) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      return value;
    } else {
      sparkStickyFault = true;
      return defaultValue;
    }
  }

  /**
   * Return a processed set of values from a Spark (or the default if one of the values is invalid).
   */
  public static double ifOkOrDefault(
      SparkBase spark,
      DoubleSupplier[] suppliers,
      Function<Double[], Double> transformer,
      double defaultValue) {
    Double[] values = new Double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return defaultValue;
      }
    }
    return transformer.apply(values);
  }

  /**
   * Return a mutable measure from a Spark (or a copy of the default mutable measure if invalid).
   * Converts the raw value from sourceUnit to targetUnit.
   *
   * @param sourceUnit the unit of the raw sensor value before conversion
   * @param targetUnit the desired unit to convert the final value into
   * @param defaultMutableMeasure the default mutable measure to return if supplier errors
   * @param <T> the type of MutableMeasure returned (e.g., MutVoltage, MutAngularVelocity)
   * @return a new mutable measure representing the converted value
   */
  @SuppressWarnings({"rawtypes", "unchecked"})
  public static <T extends MutableMeasure> T ifOkOrDefault(
      SparkBase spark,
      DoubleSupplier supplier,
      Unit sourceUnit,
      Unit targetUnit,
      T defaultMutableMeasure) {
    double value = supplier.getAsDouble();

    if (spark.getLastError() == REVLibError.kOk) {
      Measure sourceMeasure = sourceUnit.of(value);
      double convertedValue = sourceMeasure.in(targetUnit);
      T result = (T) targetUnit.mutable(convertedValue);
      return result;
    } else {
      sparkStickyFault = true;
      T result = (T) defaultMutableMeasure.copy();
      return result;
    }
  }

  /**
   * Return a mutable measure processed from multiple Spark suppliers (or a copy of the default if
   * any invalid). Applies the transformer function to raw values, then converts from sourceUnit to
   * targetUnit.
   */
  @SuppressWarnings({"rawtypes", "unchecked"})
  public static <T extends MutableMeasure> T ifOkOrDefault(
      SparkBase spark,
      DoubleSupplier[] suppliers,
      Function<Double[], Double> transformer,
      Unit sourceUnit,
      Unit targetUnit,
      T defaultMutableMeasure) {
    Double[] values = new Double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return (T) defaultMutableMeasure.copy();
      }
    }
    double transformedValue = transformer.apply(values);
    Measure sourceMeasure = sourceUnit.of(transformedValue);
    double convertedValue = sourceMeasure.in(targetUnit);
    T result = (T) targetUnit.mutable(convertedValue);
    return result;
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }
}
