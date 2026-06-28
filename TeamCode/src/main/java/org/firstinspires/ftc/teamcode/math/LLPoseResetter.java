package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import java.util.ArrayList;
import java.util.List;

public class LLPoseResetter {
    private final Limelight3A camera;
    private String status = "Waiting for reset";

    public LLPoseResetter(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, HardwareNames.limelight);
    }

    public void start() {
        camera.start();
    }

    public void stop() {
        camera.stop();
    }

    public boolean resetPose(Drivetrain drivetrain) {
        if (drivetrain.isMotionCommanded(Constants.limelightMotionCommandThreshold)) {
            status = "Stop robot before LL reset";
            return false;
        }

        Pose cameraPose = getRobotPoseFromCamera(drivetrain);
        if (cameraPose == null) return false;

        if (!isReasonableReset(drivetrain.getPose(), cameraPose)) {
            return false;
        }

        drivetrain.setPose(cameraPose);
        status = String.format(
                "Reset to x=%.1f y=%.1f h=%.1f",
                cameraPose.getX(),
                cameraPose.getY(),
                Math.toDegrees(cameraPose.getHeading())
        );
        return true;
    }

    public String getStatus() {
        return status;
    }

    private Pose getRobotPoseFromCamera(Drivetrain drivetrain) {
        List<Pose> samples = new ArrayList<>();
        long lastTimestamp = Long.MIN_VALUE;
        long startTime = System.currentTimeMillis();

        while (samples.size() < Constants.limelightPoseSampleCount
                && System.currentTimeMillis() - startTime
                < Constants.limelightPoseCollectionTimeoutMs) {
            if (drivetrain.isMotionCommanded(Constants.limelightMotionCommandThreshold)) {
                status = "LL reset aborted: robot moving";
                return null;
            }

            LLResult result = camera.getLatestResult();
            if (result == null) {
                sleepBriefly();
                continue;
            }

            long timestamp = result.getControlHubTimeStamp();
            if (timestamp == lastTimestamp) {
                sleepBriefly();
                continue;
            }
            lastTimestamp = timestamp;

            Pose sample = getRobotPoseFromResult(result);
            if (sample != null) {
                samples.add(sample);
            }
            sleepBriefly();
        }

        if (samples.size() < Constants.limelightMinimumPoseSamples) {
            status = String.format(
                    "Need %d LL samples, got %d",
                    Constants.limelightMinimumPoseSamples,
                    samples.size()
            );
            return null;
        }

        Pose preliminaryAverage = averagePoses(samples);
        List<Pose> filteredSamples = new ArrayList<>();
        for (Pose sample : samples) {
            double distance = Math.hypot(
                    sample.getX() - preliminaryAverage.getX(),
                    sample.getY() - preliminaryAverage.getY()
            );
            if (distance <= Constants.limelightOutlierDistanceInches) {
                filteredSamples.add(sample);
            }
        }

        if (filteredSamples.size() < Constants.limelightMinimumPoseSamples) {
            status = String.format(
                    "Rejected LL samples: %d/%d kept",
                    filteredSamples.size(),
                    samples.size()
            );
            return null;
        }

        status = String.format(
                "LL averaged %d/%d samples",
                filteredSamples.size(),
                samples.size()
        );
        return averagePoses(filteredSamples);
    }

    private Pose getRobotPoseFromResult(LLResult result) {
        if (!result.isValid()) {
            return null;
        }

        if (result.getBotposeTagCount() < Constants.limelightMinimumTagCount) {
            return null;
        }

        // getBotpose() is MegaTag1 and supplies an independent position and heading.
        return convertToPedroPose(result.getBotpose());
    }

    private Pose averagePoses(List<Pose> poses) {
        double x = 0;
        double y = 0;
        double headingSin = 0;
        double headingCos = 0;

        for (Pose pose : poses) {
            x += pose.getX();
            y += pose.getY();
            headingSin += Math.sin(pose.getHeading());
            headingCos += Math.cos(pose.getHeading());
        }

        int count = poses.size();
        return new Pose(
                x / count,
                y / count,
                Math.atan2(headingSin / count, headingCos / count)
        );
    }

    private void sleepBriefly() {
        try {
            Thread.sleep(10);
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
        }
    }

    private boolean isReasonableReset(Pose currentPose, Pose cameraPose) {
        if (Constants.limelightAllowLargePoseReset) {
            return true;
        }

        double distanceError = Math.hypot(
                cameraPose.getX() - currentPose.getX(),
                cameraPose.getY() - currentPose.getY()
        );
        double headingErrorDegrees = Math.abs(AngleUnit.normalizeDegrees(
                Math.toDegrees(cameraPose.getHeading() - currentPose.getHeading())
        ));

        if (distanceError > Constants.limelightMaxResetDistanceInches) {
            status = String.format(
                    "Rejected LL reset: %.1f in jump > %.1f",
                    distanceError,
                    Constants.limelightMaxResetDistanceInches
            );
            return false;
        }

        if (headingErrorDegrees > Constants.limelightMaxResetHeadingDegrees) {
            status = String.format(
                    "Rejected LL reset: %.1f deg jump > %.1f",
                    headingErrorDegrees,
                    Constants.limelightMaxResetHeadingDegrees
            );
            return false;
        }

        return true;
    }

    private Pose convertToPedroPose(Pose3D botpose) {
        Position position = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double headingRadians = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(position.x, position.y, headingRadians, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
