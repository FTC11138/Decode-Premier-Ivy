package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robot.Alliance;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

import static org.firstinspires.ftc.teamcode.math.PoseMirror.mirror;

public class WaveLength {
    private static final double[] FAR_X = new double[]{43, 71, 100};
    private static final double[] FAR_Y = new double[]{9, 27};
    private static final double[] CLOSE_X = new double[]{50, 71, 85};
    private static final double[] CLOSE_Y = new double[]{63, 88, 111, 130};

    /*
     * FAR TABLE (X = distance/metric on horizontal axis, Y = vertical axis)
     *
     *                X →
     *          43         71         100
     * Y=27    (43,27)   (71,27)   (100,27)
     * Y=9     (43,9)    (71,9)    (100,9)
     *
     * Bilinear interpolation occurs inside the rectangle formed by:
     *   (x1,y1) -------- (x2,y1)
     *      |                |
     *      |     target     |
     *      |      (x,y)     |
     *      |                |
     *   (x1,y2) -------- (x2,y2)
     *
     *
     * CLOSE TABLE
     *
     *                X →
     *          50         71         85
     * Y=130   (50,130)   (71,130)   (85,130)
     * Y=111   (50,111)   (71,111)   (85,111)
     * Y=88    (50,88)    (71,88)    (85,88)
     * Y=63    (50,63)    (71,63)    (85,63)
     *
     * Example:
     * If X=50 and Y=100, interpolation uses the surrounding cell:
     *
     *          50         71
     * Y=111   ●-----------●
     *          |           |
     *          |   X,Y     |
     *          |  (50,100) |
     * Y=88    ●-----------●
     *
     * First interpolate along X at Y=88 and Y=111,
     * then interpolate those two results along Y.
     */

    private static final Interpolation2D farVelocityInterpolation = new BilinearInterpolation(
            FAR_X,
            FAR_Y,
            new double[][]{
                    // rows = X (43, 71, 100); cols = Y (9, 27)
                    {-1940, -1800},
                    {-2010, -1950},
                    {-2080, -2040}
            });

    private static final Interpolation2D farHoodInterpolation = new BilinearInterpolation(
            FAR_X,
            FAR_Y,
            new double[][]{
                    // rows = X (43, 71, 100); cols = Y (9, 27)
                    {0.09, 0.18},
                    {0.09, 0.12},
                    {0.11, 0.11}
            });

    private static final Interpolation2D closeVelocityInterpolation = new BilinearInterpolation(
            CLOSE_X,
            CLOSE_Y,
            new double[][]{
                    // rows = X (50, 71, 85); cols = Y (63, 88, 111, 130)
                    {-1530, -1420, -1350, -1305},
                    {-1650, -1560, -1520, -1470},
                    {-1760, -1630, -1580, -1560},
            }
    );

    private static final Interpolation2D closeHoodInterpolation = new BilinearInterpolation(
            CLOSE_X,
            CLOSE_Y,
            new double[][]{
                    // rows = X (50, 71, 85); cols = Y (63, 88, 111, 130)
                    {0.40, 0.52, 0.55, 0.59},
                    {0.30, 0.34, 0.38, 0.43},
                    {0.19, 0.24, 0.25, 0.26},
            }
    );

    public static double getVelocityWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.BLUE ? currentPosition : mirror(currentPosition);
        if (pose.getY() > 48) {
            return getCloseVelocityWithInterpolation(currentPosition, alliance);
        } else {
            return getFarVelocityWithInterpolation(currentPosition, alliance);
        }
    }

    public static double getCloseVelocityWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.BLUE ? currentPosition : mirror(currentPosition);
        return Math.abs(closeVelocityInterpolation.interpolate(
                clampToTable(pose.getX(), CLOSE_X),
                clampToTable(pose.getY(), CLOSE_Y)
        ));
    }

    public static double getFarVelocityWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.BLUE ? currentPosition : mirror(currentPosition);
        return Math.abs(farVelocityInterpolation.interpolate(
                clampToTable(pose.getX(), FAR_X),
                clampToTable(pose.getY(), FAR_Y)
        ));
    }

    public static double getHoodWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.BLUE ? currentPosition : mirror(currentPosition);
        if (pose.getY() > 48) {
            return getCloseHoodWithInterpolation(currentPosition, alliance);
        } else {
            return getFarHoodWithInterpolation(currentPosition, alliance);
        }
    }

    public static double getCloseHoodWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.BLUE ? currentPosition : mirror(currentPosition);
        return closeHoodInterpolation.interpolate(
                clampToTable(pose.getX(), CLOSE_X),
                clampToTable(pose.getY(), CLOSE_Y)
        );
    }

    public static double getFarHoodWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.BLUE ? currentPosition : mirror(currentPosition);
        return farHoodInterpolation.interpolate(
                clampToTable(pose.getX(), FAR_X),
                clampToTable(pose.getY(), FAR_Y)
        );
    }

    public static double getDistanceToGoal(Pose currentPosition, Alliance alliance) {
        Pose goal = alliance.getGoal();
        return Math.hypot(goal.getX() - currentPosition.getX(), goal.getY() - currentPosition.getY());
    }

    private static double clampToTable(double value, double[] table) {
        return Math.max(table[0], Math.min(table[table.length - 1], value));
    }
}
