package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

public class DrivePointToPoint extends DriveToPointCommand {
    private final Pose start;

    public DrivePointToPoint(FollowerSubsystem follower, Pose start, Pose... points) {
        super(follower, points);
        this.start = start;
    }

    @Override
    protected Pose startPose() {
        return start;
    }
}
