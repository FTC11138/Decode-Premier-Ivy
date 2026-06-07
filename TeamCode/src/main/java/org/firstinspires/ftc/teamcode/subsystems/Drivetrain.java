package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

public class Drivetrain {
    private static Pose poseTransfer = new Pose();
    private static boolean poseTransferReady = false;
    public final DcMotorEx frontLeft;
    public final DcMotorEx frontRight;
    public final DcMotorEx backLeft;
    public final DcMotorEx backRight;
    public final Follower follower;
    private final Telemetry telemetry;
    private final PIDFController headingController = new PIDFController(error -> new PIDFCoefficients(
            Constants.driveHeadingKp,
            Constants.driveHeadingKi,
            Constants.driveHeadingKd,
            Constants.driveHeadingKf
    ));
    private boolean lockHeading = false;
    private boolean fieldCentricEnabled = true;
    private double headingTargetRadians = 0;
    private double fieldCentricHeadingOffsetRadians = 0;

    public Drivetrain(Robot robot) {
        follower = createFollower(robot.hardwareMap);
        frontLeft = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.frontLeft);
        frontRight = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.frontRight);
        backLeft = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.backLeft);
        backRight = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.backRight);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = robot.telemetry;
    }

    private static double signedSquare(double raw) {
        return Math.signum(raw) * Math.pow(raw, 2);
    }

    public static void localize(Pose pose) {
        poseTransfer = pose;
        poseTransferReady = true;
    }

    public void gateHeading(Alliance alliance) {
        lockHeading = true;
        if (alliance == Alliance.RED) {
            headingTargetRadians = Math.toRadians(Constants.gateOpenHeadingDegrees);
        } else {
            headingTargetRadians = Math.toRadians(180 - Constants.gateOpenHeadingDegrees);
        }
    }

    public void unlockHeading() {
        lockHeading = false;
    }

    public void robotCentricDrive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, 0, follower.getHeading());
    }

    public void fieldCentricDrive(double forward, double strafe, double turn, Alliance alliance) {
        if (!fieldCentricEnabled) {
            robotCentricDrive(forward, strafe, turn);
            return;
        }

        double headingRadians = follower.getHeading();
        if (alliance == Alliance.BLUE) headingRadians += Math.PI;

        drive(forward, strafe, turn, headingRadians - fieldCentricHeadingOffsetRadians, follower.getHeading());
    }

    public void arcadeDrive(double forward, double strafe, double turn, Alliance alliance) {
        fieldCentricDrive(forward, strafe, turn, alliance);
    }

    public void setFieldCentricEnabled(boolean fieldCentricEnabled) {
        this.fieldCentricEnabled = fieldCentricEnabled;
    }

    public void resetFieldCentricHeading(Alliance alliance) {
        fieldCentricHeadingOffsetRadians = follower.getHeading();
        if (alliance == Alliance.BLUE) fieldCentricHeadingOffsetRadians += Math.PI;
    }

    public void clearFieldCentricHeadingReset() {
        fieldCentricHeadingOffsetRadians = 0;
    }

    private void drive(double forward, double strafe, double turn, double driveHeadingRadians, double robotHeadingRadians) {
        forward = signedSquare(forward);
        strafe = signedSquare(strafe);

        if (lockHeading) {
            headingController.updateError(AngleUnit.normalizeRadians(headingTargetRadians - robotHeadingRadians));
            turn = -headingController.run();
        } else {
            turn = signedSquare(turn);
        }

        double x = strafe * Math.cos(-driveHeadingRadians) - forward * Math.sin(-driveHeadingRadians);
        double y = strafe * Math.sin(-driveHeadingRadians) + forward * Math.cos(-driveHeadingRadians);
        y *= Constants.driveFieldCentricYMultiplier;

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(turn), 1);

        frontLeft.setPower((y + x + turn) / denominator);
        frontRight.setPower((y - x - turn) / denominator);
        backLeft.setPower((y - x + turn) / denominator);
        backRight.setPower((y + x - turn) / denominator);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void usePreviousStartingPose() {
        if (poseTransferReady) {
            setStartingPose(poseTransfer);
        } else {
            setStartingPose(new Pose(
                    Constants.teleOpStartX,
                    Constants.teleOpStartY,
                    Math.toRadians(Constants.teleOpStartHeadingDegrees)
            ));
        }
    }

    public Command followPath(PathChain path) {
        return follow(follower, path);
    }

    public Command periodic() {
        return infinite(() -> {
            follower.update();
            poseTransfer = follower.getPose();
            poseTransferReady = true;

            telemetry.addData("Current X", follower.getPose().getX());
            telemetry.addData("Current Y", follower.getPose().getY());
            telemetry.addData("Current Heading", Math.toDegrees(follower.getHeading()));
            telemetry.addData("Heading Locked", lockHeading);
            telemetry.addData("Heading Target", headingTargetRadians);
            telemetry.addData("Field Centric Enabled", fieldCentricEnabled);
            telemetry.addData("Field Centric Offset", Math.toDegrees(fieldCentricHeadingOffsetRadians));
        });
    }
}
