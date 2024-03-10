package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class CommandUtil {
  public static <T> Command conditional(
      Function<T, Command> conditionalSupplier,
      Supplier<Command> backupSupplier,
      Supplier<Optional<T>> supplier) {
    var valOpt = supplier.get();
    if (valOpt.isPresent()) {
      return conditionalSupplier.apply(valOpt.get());
    } else {
      return backupSupplier.get();
    }
  }
}
