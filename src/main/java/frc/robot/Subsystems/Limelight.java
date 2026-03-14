package frc.robot.Subsystems;

import frc.robot.LimelightHelpers;

public class Limelight {
    private final String limelightName;
    private final LimelightHelpers.RawFiducial[] fiducials;

    public Limelight(String name) {
        limelightName = name;
        fiducials = LimelightHelpers.getRawFiducials(limelightName);
    }
    
    public LimelightHelpers.RawFiducial identifyyTag(int tagID) {
        if (fiducials.length > 0) {
            for (LimelightHelpers.RawFiducial fiducial : fiducials) {
                if (fiducial.id == tagID) {
                    return fiducial;
                }
            }
        }
        return null;
    }
}

