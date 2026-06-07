package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robot.Alliance;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

import static org.firstinspires.ftc.teamcode.math.PoseMirror.mirror;

public class WaveLength {
    private static final double[] FAR_X = new double[]{43, 71, 100};
    private static final double[] FAR_Y = new double[]{6, 27};
    private static final double[] CLOSE_X = new double[]{38, 61, 85};
    private static final double[] CLOSE_Y = new double[]{63, 88, 111, 135.5};

    private static final Interpolation2D farVelocityInterpolation = new BilinearInterpolation(
            FAR_X,
            FAR_Y,
            new double[][]{
                    {1675, 1640},
                    {1675, 1520},
                    {1620, 1520}
            });

    private static final Interpolation2D farHoodInterpolation = new BilinearInterpolation(
            FAR_X,
            FAR_Y,
            new double[][]{
                    {0.47, 0.43},
                    {0.37, 0.33},
                    {0.31, 0.31}
            });

    private static final Interpolation2D closeVelocityInterpolation = new BilinearInterpolation(
            CLOSE_X,
            CLOSE_Y,
            new double[][]{
                    {1520, 1470, 1430, 1400},
                    {1415, 1390, 1380, 1375},
                    {1350, 1330, 1325, 1338},
            }
    );

    private static final Interpolation2D closeHoodInterpolation = new BilinearInterpolation(
            CLOSE_X,
            CLOSE_Y,
            new double[][]{
                    {0.37, 0.38, 0.43, 0.47},
                    {0.36, 0.37, 0.38, 0.43},
                    {0.31, 0.33, 0.36, 0.38},
            }
    );

    public static double getVelocityWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.RED ? currentPosition : mirror(currentPosition);
        if (pose.getY() > 48) {
            return closeVelocityInterpolation.interpolate(clampToTable(pose.getX(), CLOSE_X), clampToTable(pose.getY(), CLOSE_Y));
        } else {
            return farVelocityInterpolation.interpolate(clampToTable(pose.getX(), FAR_X), clampToTable(pose.getY(), FAR_Y));
        }
    }

    public static double getHoodWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.RED ? currentPosition : mirror(currentPosition);
        if (pose.getY() > 48) {
            return closeHoodInterpolation.interpolate(clampToTable(pose.getX(), CLOSE_X), clampToTable(pose.getY(), CLOSE_Y));
        } else {
            return farHoodInterpolation.interpolate(clampToTable(pose.getX(), FAR_X), clampToTable(pose.getY(), FAR_Y));
        }
    }

    public static double getDistanceToGoal(Pose currentPosition, Alliance alliance) {
        Pose goal = alliance.goal;
        return Math.hypot(goal.getX() - currentPosition.getX(), goal.getY() - currentPosition.getY());
    }

    private static double clampToTable(double value, double[] table) {
        return Math.max(table[0], Math.min(table[table.length - 1], value));
    }
}
