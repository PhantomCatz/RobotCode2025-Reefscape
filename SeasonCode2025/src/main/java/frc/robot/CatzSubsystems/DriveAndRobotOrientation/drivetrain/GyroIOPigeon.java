
package frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.DriveConstants.GYRO_ID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon implements GyroIO {

  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  public GyroIOPigeon() {
    pigeon = new Pigeon2(GYRO_ID);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.gyroConnected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
    inputs.gyroAngle = -yaw.getValueAsDouble();
    inputs.gyroYawVel = -Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }
}