package frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class GyroIONavX implements GyroIO {
    private final AHRS navX;

    public GyroIONavX() {
        navX = new AHRS(Port.kMXP, (byte) 200);
        navX.enableLogging(true);

        new Thread(() -> {
          try {
            Thread.sleep(1000);
            navX.reset();
          } catch (Exception E) {
            
          }
        }).start();
        navX.reset(); // implicitly sets the gyro to 0 heading
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.gyroConnected    = navX.isConnected();

      inputs.gyroAngle        = navX.getAngle(); //Acumulated Yaw no rollover
      inputs.gyroYawDegrees   = navX.getYaw();
      inputs.gyroRollDegrees  = navX.getRoll(); 
           
      inputs.gyroYawVel       = navX.getRate();
      inputs.gyroAccelX       = navX.getWorldLinearAccelX();
      inputs.gyroAccelY       = navX.getWorldLinearAccelY();
    }

}

