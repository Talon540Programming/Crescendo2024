package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.PolynomialRegression;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FeedForwardCharacterization extends Command {
  private static final double START_DELAY_SECS = 2.0;

  private FeedForwardCharacterizationData data;
  private final String logKey;
  private final Consumer<Double> voltageConsumer;
  private final Supplier<Double> velocitySupplier;
  private final double rampRateVoltsPerSecond;
  private final double timeout;

  private final Timer timer = new Timer();

  public FeedForwardCharacterization(
      Subsystem subsystem,
      String mechanismName,
      Consumer<Double> voltageConsumer,
      Supplier<Double> velocitySupplier,
      double rampRateVoltsPerSecond,
      double timeout) {
    addRequirements(subsystem);
    this.logKey = subsystem.getName() + "/" + mechanismName;
    this.voltageConsumer = voltageConsumer;
    this.velocitySupplier = velocitySupplier;
    this.rampRateVoltsPerSecond = rampRateVoltsPerSecond;
    this.timeout = timeout;
  }

  @Override
  public void initialize() {
    data = new FeedForwardCharacterizationData(logKey);
    timer.restart();
  }

  @Override
  public void execute() {
    if (timer.get() < START_DELAY_SECS) {
      voltageConsumer.accept(0.0);
    } else {
      double voltage = (timer.get() - START_DELAY_SECS) * rampRateVoltsPerSecond;
      voltageConsumer.accept(voltage);
      data.add(velocitySupplier.get(), voltage);
    }
  }

  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0);
    timer.stop();
    data.calculateAndLogResults();
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= timeout;
  }

  public static class FeedForwardCharacterizationData {
    private static final String LOG_KEY_BASE = "FeedForwardCharacterizationResults";
    private final String logKey;
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public FeedForwardCharacterizationData(String logKey) {
      this.logKey = LOG_KEY_BASE + "/" + logKey;
    }

    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void calculateAndLogResults() {
      if (velocityData.isEmpty() || voltageData.isEmpty()) {
        return;
      }

      PolynomialRegression regression =
          new PolynomialRegression(
              velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
              voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
              1);

      Logger.recordOutput(logKey + "/SampleCount", velocityData.size());
      Logger.recordOutput(logKey + "/R^2", regression.R2());
      Logger.recordOutput(logKey + "/kS", regression.beta(0));
      Logger.recordOutput(logKey + "/kV", regression.beta(1));
    }
  }
}
