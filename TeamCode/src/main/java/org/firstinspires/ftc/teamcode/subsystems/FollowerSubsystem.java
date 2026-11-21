package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.FieldPose.HALF_FIELD_IN;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.FieldPose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class FollowerSubsystem extends SubsystemBase {
    private final TelemetryManager telemetry;
    public final Follower follower;

    public FollowerSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        follower = Constants.createFollower(hardwareMap);
        this.telemetry = telemetry;
        register();
    }

    @Override
    public void periodic() {
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
    }

    public void saveAutoPose() {
        Pose current = follower.getPose();
        double x = current.getX();
        double y = current.getY();
        double heading = current.getHeading();
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)) return;

        FieldPose.saveAutoPose(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading));

        // 交接的是 Pedro 坐标。下面换算成 FTC 坐标显示，方便和 FTC 场地图对照。
        telemetry.addData("FTC X (in)", HALF_FIELD_IN - y);
        telemetry.addData("FTC Y (in)", x - HALF_FIELD_IN);
        telemetry.addData("FTC heading (deg)", AngleUnit.DEGREES.normalize(Math.toDegrees(heading) + 90));
    }

    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose.copy());
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public boolean isRobotStuck() {
        return follower.isRobotStuck();
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public void breakFollowing() {
        follower.breakFollowing();
    }
}
