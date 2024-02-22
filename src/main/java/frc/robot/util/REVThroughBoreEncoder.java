package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Wrapper class around the wpilib {@link DutyCycleEncoder} and {@link Encoder} classes for the REV
 * ThroughBoreEncoder. Implements both the relative and absolute encoders of the device. <a
 * href="https://docs.revrobotics.com/through-bore-encoder/specifications">Datasheet</a>
 */
public class REVThroughBoreEncoder implements AutoCloseable {
  private final DutyCycleEncoder m_absoluteEncoder;
  private final Encoder m_relativeEncoder;
  private final Rotation2d m_absoluteEncoderOffset;

  /**
   * Create a ThroughBoreEncoder
   *
   * @param dutyCyclePort DIO port of the ABS (white) wire
   * @param relativeAPort DIO port of the ENC A (blue) white
   * @param relativeBPort DIO port of the ENC B (yellow) white
   * @param absoluteEncoderOffset Offset of the duty cycle encoder. This value is subtracted from
   *     the true absolute position to yield the result.
   */
  public REVThroughBoreEncoder(
      int dutyCyclePort, int relativeAPort, int relativeBPort, Rotation2d absoluteEncoderOffset) {
    m_absoluteEncoder = new DutyCycleEncoder(dutyCyclePort);
    m_absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    m_absoluteEncoderOffset = absoluteEncoderOffset;

    m_relativeEncoder = new Encoder(relativeAPort, relativeBPort, false);
    m_relativeEncoder.setDistancePerPulse((2 * Math.PI) / 2048);
  }

  /**
   * Get the absolute position of the DutyCycleEncoder accounting for the encoder offset.
   *
   * @return absolute position of the encoder.
   */
  public Rotation2d getAbsolutePosition() {
    return Rotation2d.fromRotations(m_absoluteEncoder.getAbsolutePosition())
        .minus(m_absoluteEncoderOffset);
  }

  /**
   * Get the distance traveled by the encoder in radians.
   *
   * @return the position of the encoder in radians.
   */
  public double getRelativePositionRad() {
    return m_relativeEncoder.getDistance();
  }

  /**
   * Get the rate of the encoder in radians per second.
   *
   * @return the rate or velocity of the encoder in radians per second.
   */
  public double getVelocityRadPerSecond() {
    return m_relativeEncoder.getRate();
  }

  @Override
  public void close() {
    m_absoluteEncoder.close();
    m_relativeEncoder.close();
  }
}
