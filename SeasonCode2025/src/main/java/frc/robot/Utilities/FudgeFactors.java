package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Builder;

/** Fudge factors for critical field locations which can be tuned per alliance at events. */
public final class FudgeFactors {
  @Builder
  public static class FudgedTransform {
    @Builder.Default private final Transform2d red = new Transform2d();

    @Builder.Default private final Transform2d blue = new Transform2d();

    public Transform2d getTransform() {
      return DriverStation.getAlliance()
          .map(alliance -> alliance == DriverStation.Alliance.Red ? red : blue)
          .orElseGet(Transform2d::new);
    }
  }

  private FudgeFactors() {}

  public static final FudgedTransform speaker = FudgedTransform.builder().build();

  public static final FudgedTransform amp = FudgedTransform.builder().build();

  public static final FudgedTransform centerPodiumAmpChain = FudgedTransform.builder().build();

  public static final FudgedTransform centerAmpSourceChain = FudgedTransform.builder().build();

  public static final FudgedTransform centerSourcePodiumChain = FudgedTransform.builder().build();
}
