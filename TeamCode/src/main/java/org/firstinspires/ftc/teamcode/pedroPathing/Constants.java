package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

@Configurable
public class Constants {
    public static double maxPower = 1;
    public static double xVelocity = 86.8;
    public static double yVelocity = 54.5;

    public static boolean leftFrontReversed = true;
    public static boolean leftRearReversed = true;
    public static boolean rightFrontReversed = true;
    public static boolean rightRearReversed = false;

    public static double forwardPodY = 132.0 / 25.4;
    public static double strafePodX = -24 / 25.4;
    public static boolean forwardEncoderReversed = true;
    public static boolean strafeEncoderReversed = false;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.7)
            .centripetalScaling(0)
//            .forwardZeroPowerAcceleration(-25.93469313136796)
//            .lateralZeroPowerAcceleration(-67.34249184408006)
//            .translationalPIDFCoefficients(new PIDFCoefficients(
//                    0.16,
//                    0,
//                    0.02,
//                    0
//            ))
//            .translationalPIDFSwitch(4)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
//                    0.16,
//                    0,
//                    0.02,
//                    0
//            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.2,
                    0.0004,
                    0.06,
                    0.03
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    1.6,
                    0,
                    0.14,
                    0.026
            ))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
//                    0.01,
//                    0,
//                    0.000001,
//                    0.6,
//                    0.002
//            ))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
//                    0.01,
//                    0,
//                    0.000001,
//                    0.6,
//                    0.002
//            ))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                    0.13,
                    0.061,
                    0.00195
            ));
//            .drivePIDFSwitch(15)
//            .centripetalScaling(0.0005);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(maxPower)
            .leftFrontMotorName(HardwareNames.frontLeft)
            .leftRearMotorName(HardwareNames.backLeft)
            .rightFrontMotorName(HardwareNames.frontRight)
            .rightRearMotorName(HardwareNames.backRight)
            .leftFrontMotorDirection(motorDirection(leftFrontReversed))
            .leftRearMotorDirection(motorDirection(leftRearReversed))
            .rightFrontMotorDirection(motorDirection(rightFrontReversed))
            .rightRearMotorDirection(motorDirection(rightRearReversed))
            .xVelocity(xVelocity)
            .yVelocity(yVelocity)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(forwardPodY)
            .strafePodX(strafePodX)
            .forwardEncoderDirection(encoderDirection(forwardEncoderReversed))
            .strafeEncoderDirection(encoderDirection(strafeEncoderReversed))
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(HardwareNames.pinpoint)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.93,
            0.3,   // velocityConstraint: was 0.1 (nudged up slightly)
            0.3,   // translationalConstraint (in): was 0.1 (slightly looser -> less settle time)
            0.02,  // headingConstraint (rad ~1.1deg): was 0.009 (slightly looser)
            100,
            1.25,
            10,
            1
    );

    public static void applyTunableConstants() {
        driveConstants
                .maxPower(maxPower)
                .leftFrontMotorDirection(motorDirection(leftFrontReversed))
                .leftRearMotorDirection(motorDirection(leftRearReversed))
                .rightFrontMotorDirection(motorDirection(rightFrontReversed))
                .rightRearMotorDirection(motorDirection(rightRearReversed))
                .xVelocity(xVelocity)
                .yVelocity(yVelocity);

        double[] frontLeftVector = Pose.cartesianToPolar(xVelocity, -yVelocity);
        driveConstants.frontLeftVector = new Vector(frontLeftVector[0], frontLeftVector[1]).normalize();

        localizerConstants
                .forwardPodY(forwardPodY)
                .strafePodX(strafePodX)
                .forwardEncoderDirection(encoderDirection(forwardEncoderReversed))
                .strafeEncoderDirection(encoderDirection(strafeEncoderReversed));
    }

    private static DcMotorSimple.Direction motorDirection(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private static GoBildaPinpointDriver.EncoderDirection encoderDirection(boolean reversed) {
        return reversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        applyTunableConstants();
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
