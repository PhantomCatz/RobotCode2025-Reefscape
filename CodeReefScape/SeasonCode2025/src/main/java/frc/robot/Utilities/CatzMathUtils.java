package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;

public class CatzMathUtils {

    //TODO swerve specific should be a swerve util? Look for a library?
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }        
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
    
    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public static double Clamp(double min, double input, double max) {
        double clampedValue;
        if(input > max) {
            clampedValue = max;
        }
        else if(input < min) {
            clampedValue = min;
        }
        else {
            clampedValue = input;
        }
        return clampedValue;
    }

    public static double toUnitCircAngle(double angleRadians) {
        double rotations = angleRadians / (2 * Math.PI);
        return (angleRadians - Math.round(rotations - 0.500) * Math.PI * 2.0);
    }


    /************************************************************************************************
     * 
     * Conversions
     * 
     ***********************************************************************************************/
    public class Conversions {
        private static final double wheelCircumference = DriveConstants.driveConfig.wheelRadius() * 2.0 * Math.PI;
        private static final double gearRatio = DriveConstants.moduleGainsAndRatios.driveReduction();
    
        public static double RPSToMPS(double rps){
            return rps * wheelCircumference / gearRatio;
        }
    
        public static double MPSToRPS(double velocity){
            return velocity / wheelCircumference * gearRatio;
        }

        /**
         *  TODO update it works for all motors/controllers
         *  600 is a conversion factor from cnts/100 ms to cnts/minute
         * @param velocityCounts
         * @param gearRatio
         * @return
         */
        public static double velocityCntsToRPM(double velocityCounts, double gearRatio) {
            double motorRPM = velocityCounts * (600.0 / 2048.0);        
            double mechRPM = motorRPM / gearRatio;
            return mechRPM;
        }

        public static double velocityCntsToMPS(double velocitycounts, double circumference, double gearRatio){
            double wheelRPM = velocityCntsToRPM(velocitycounts, gearRatio);
            double wheelMPS = (wheelRPM * circumference) / 60;
            return wheelMPS;
        }
    }

}
