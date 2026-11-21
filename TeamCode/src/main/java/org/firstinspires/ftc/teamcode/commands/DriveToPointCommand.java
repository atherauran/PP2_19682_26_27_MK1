package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * 沿一条路径开到终点，朝向从起点朝向线性过渡到终点朝向。
 * 路径点里最后一个是终点，前面的是曲线控制点（没有就是直线）。
 * 子类决定起点：固定的位姿，或指令开始时机器人的当前位姿。
 * 路径跑完或者机器人卡住时指令结束。
 */
public abstract class DriveToPointCommand extends CommandBase {
    private final FollowerSubsystem follower;
    private final Pose[] points;
    private final double endHeading;
    private boolean holdEnd = true;
    private double brakingStrength = 1.0;
    private double maxPower = 1.0;

    protected DriveToPointCommand(FollowerSubsystem follower, Pose... points) {
        this.follower = follower;
        this.points = points;
        this.endHeading = points[points.length - 1].getHeading();
    }

    protected abstract Pose startPose();

    public DriveToPointCommand setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    public DriveToPointCommand setBrakingStrength(double brakingStrength) {
        this.brakingStrength = brakingStrength;
        return this;
    }

    public DriveToPointCommand setMaxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    @Override
    public void initialize() {
        follower.breakFollowing();
        follower.follower.setMaxPower(maxPower);

        Pose start = startPose();
        List<Pose> route = new ArrayList<>();
        route.add(new Pose(start.getX(), start.getY()));
        for (Pose point : points) route.add(new Pose(point.getX(), point.getY()));

        Curve curve = route.size() == 2
                ? new BezierLine(route.get(0), route.get(1))
                : new BezierCurve(route);
        PathBuilder builder = new PathBuilder(follower.follower);
        builder.addPath(curve)
                .setLinearHeadingInterpolation(start.getHeading(), endHeading)
                .setBrakingStrength(brakingStrength);
        follower.followPath(builder.build(), holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy() || follower.isRobotStuck();
    }
}
