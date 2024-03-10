package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandSrimanController {
  /** Represents a digital button on an XboxController. */
  public enum Button {
    kLeftBumper(7),
    kRightBumper(8),
    kLeftStick(14),
    kRightStick(15),
    kA(1),
    kB(2),
    kX(4),
    kY(5),
    kBack(11),
    kStart(12);

    public final int value;

    Button(int value) {
      this.value = value;
    }
  }

  /** Represents an axis on an XboxController. */
  public enum Axis {
    kLeftX(0),
    kRightX(2),
    kLeftY(1),
    kRightY(3),
    kLeftTrigger(5),
    kRightTrigger(4);

    public final int value;

    Axis(int value) {
      this.value = value;
    }
  }

  private final GenericHID controller;

  public CommandSrimanController(int port) {
    controller = new GenericHID(port);
  }

  public double getLeftX() {
    return controller.getRawAxis(Axis.kLeftX.value);
  }

  public double getLeftY() {
    return controller.getRawAxis(Axis.kLeftY.value);
  }

  public double getRightX() {
    return controller.getRawAxis(Axis.kRightX.value);
  }

  public double getRightY() {
    return controller.getRawAxis(Axis.kRightY.value);
  }

  public double getLeftTrigger() {
    return controller.getRawAxis(Axis.kLeftTrigger.value);
  }

  public double getRightTrigger() {
    return controller.getRawAxis(Axis.kRightTrigger.value);
  }

  public Trigger leftTrigger() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawAxis(Axis.kLeftTrigger.value) >= 0.5)
        .castTo(Trigger::new);
  }

  public Trigger rightTrigger() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawAxis(Axis.kRightTrigger.value) >= 0.5)
        .castTo(Trigger::new);
  }

  public Trigger leftBumper() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kLeftBumper.value))
        .castTo(Trigger::new);
  }

  public Trigger rightBumper() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kRightBumper.value))
        .castTo(Trigger::new);
  }

  public Trigger leftStick() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kLeftStick.value))
        .castTo(Trigger::new);
  }

  public Trigger rightStick() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kRightStick.value))
        .castTo(Trigger::new);
  }

  public Trigger a() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kA.value))
        .castTo(Trigger::new);
  }

  public Trigger b() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kB.value))
        .castTo(Trigger::new);
  }

  public Trigger x() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kX.value))
        .castTo(Trigger::new);
  }

  public Trigger y() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kY.value))
        .castTo(Trigger::new);
  }

  public Trigger back() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kBack.value))
        .castTo(Trigger::new);
  }

  public Trigger start() {
    return new BooleanEvent(
            CommandScheduler.getInstance().getDefaultButtonLoop(),
            () -> controller.getRawButton(Button.kStart.value))
        .castTo(Trigger::new);
  }
}
