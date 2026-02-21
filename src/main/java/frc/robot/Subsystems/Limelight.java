package frc.robot.Subsystems;

import frc.robot.LimelightHelpers;

public class Limelight {
    private final String limelightName;

    public Limelight(String name) {
        limelightName = name;
    }
    
    public double getDeltaX(double targetHeight, double camHeight, double camAngle) {
        if (LimelightHelpers.getTV(limelightName)) 
        {
            double deltaY = targetHeight - camHeight;
            double angle = camAngle + LimelightHelpers.getTY(limelightName);
            return deltaY / Math.tan(Math.toRadians(angle));
        }
        // if the limelight doesn't have a valid target, return 0, causing the shooter to stop rather than throw an exception.
        return 0;
    } 
}

