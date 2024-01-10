package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.File;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class PathPlannerUtil {
  private static List<String> getPathNames() {
    File[] pathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();
    return pathFiles == null
        ? List.of()
        : Stream.of(pathFiles)
            .filter(file -> !file.isDirectory())
            .map(File::getName)
            .filter(name -> name.endsWith(".path"))
            .map(name -> name.substring(0, name.lastIndexOf(".")))
            .collect(Collectors.toList());
  }

  /**
   * Configure a SendableChooser with Commands to follow PathPlanner paths. AutoBuilder should be
   * configured prior to calling thus method.
   *
   * @param chooser SendableChooser to configure
   * @return the configured chooser
   * @throws IllegalStateException if AutoBuilder isn't configured
   */
  public static SendableChooser<Command> configureChooserWithPaths(
      SendableChooser<Command> chooser) {
    if (!AutoBuilder.isConfigured()) {
      throw new IllegalStateException(
          "AutoBuilder was not configured before attempting to create PathChooser");
    }

    for (var pathName : getPathNames()) {
      var path = PathPlannerPath.fromPathFile(pathName);
      chooser.addOption(
          pathName,
          AutoBuilder.followPath(path)
              .beforeStarting(
                  () ->
                      PoseEstimator.getInstance()
                          .resetPose(path.getPreviewStartingHolonomicPose())));
    }

    return chooser;
  }
}
