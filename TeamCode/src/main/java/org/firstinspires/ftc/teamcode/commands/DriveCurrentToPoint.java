package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

public class DriveCurrentToPoint extends DriveToPointCommand {
    private final FollowerSubsystem follower;

    public DriveCurrentToPoint(FollowerSubsystem follower, Pose... points) {
        super(follower, points);
        this.follower = follower;
    }

    @Override
    protected Pose startPose() {
        return follower.getPose();
    }
}
