package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_FIRE_TURRET;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_INTAKE_FIRST_CONTROL;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_INTAKE_FIRST_END;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_INTAKE_GATE;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_INTAKE_GATE_CONTROL;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_INTAKE_GATE_END;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_INTAKE_SECOND_CONTROL;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_INTAKE_SECOND_END;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_PARK;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_SHOOT;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.BLUE_CLOSE_START;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.CLOSE_FIRE_DISTANCE;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.CLOSE_FIRE_INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.CLOSE_HOLD_DISTANCE;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.CLOSE_SHOOT_WAIT_MS;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.SHOOT_TIME_MS;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.DriveCurrentToPoint;
import org.firstinspires.ftc.teamcode.commands.DrivePointToPoint;
import org.firstinspires.ftc.teamcode.constants.FieldPose;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;

/**
 * 蓝方近端 18 球：预载 3 球，再收第二排、三次闸门、第一排，每次回到同一个发射点发射，最后停靠。
 * 发射和移动的同时，loop() 每帧按 {@link #distance} 设定飞轮转速。
 */
@Autonomous(name = "BLUE_CLOSE_18_MK2")
public class BlueClose18 extends OpMode {
    /** 回发射点途中，吸球电机再多转这么久才停，让最后一个球吸进来。 */
    private static final long INTAKE_STOP_DELAY_MS = 400;

    private final Robot robot = new Robot();
    private FollowerSubsystem follower;
    private TelemetryManager panelsTelemetry;

    /** 飞轮按这个距离（英寸）对应的转速运转，为 0 时不运转。 */
    private double distance = 50;

    private final List<Command> steps = new ArrayList<>();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = new FollowerSubsystem(hardwareMap, panelsTelemetry);
        follower.setStartingPose(BLUE_CLOSE_START.copy());
        robot.autoInit(hardwareMap);
        panelsTelemetry.update();
        Drawing.init();
    }

    @Override
    public void init_loop() {
        follower.follower.update();
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
        showPose();
    }

    @Override
    public void start() {
        FieldPose.saveAutoPose(null);
        follower.saveAutoPose();
        follower.follower.activateAllPIDFs();

        // 预载三球：开闸门、开到发射点、发射。
        run(() -> distance = CLOSE_FIRE_DISTANCE);
        run(() -> robot.shooter.turretToDegree(BLUE_CLOSE_FIRE_TURRET));
        run(() -> robot.intake.gateOpen());
        steps.add(new DrivePointToPoint(follower, BLUE_CLOSE_START, BLUE_CLOSE_SHOOT));
        pause(CLOSE_SHOOT_WAIT_MS);
        run(() -> robot.intake.intakeFire(CLOSE_FIRE_INTAKE_POWER));
        pause(SHOOT_TIME_MS);
        run(() -> robot.intake.gateClose());
        run(() -> distance = CLOSE_HOLD_DISTANCE);

        // 第二排球。
        run(() -> robot.intake.intakeIn());
        intakeWhileDriving(1500, new DriveCurrentToPoint(follower,
                BLUE_CLOSE_INTAKE_SECOND_CONTROL, BLUE_CLOSE_INTAKE_SECOND_END));
        driveToShoot();
        fire();

        // 第一次闸门：先在闸门口吸球，再往里开一点。
        add(gateIntake(2000));
        intakeWhileDriving(1500, new DriveCurrentToPoint(follower, BLUE_CLOSE_INTAKE_GATE_END));
        driveToShoot();
        fire();

        // 第二、三次闸门。
        for (int cycle = 0; cycle < 2; cycle++) {
            add(gateIntake(3500));
            driveToShoot();
            fire();
        }

        // 第一排球。最后一次发射后不关闸门，直接停靠。
        run(() -> robot.intake.intakeIn());
        intakeWhileDriving(1300, new DriveCurrentToPoint(follower,
                BLUE_CLOSE_INTAKE_FIRST_CONTROL, BLUE_CLOSE_INTAKE_FIRST_END));
        pause(300);
        driveToShoot();
        feed();

        steps.add(new DriveCurrentToPoint(follower, BLUE_CLOSE_PARK));
        pause(100);
        run(() -> distance = 0);
        run(this::stop);

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(steps.toArray(new Command[0])));
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        follower.follower.update();
        follower.saveAutoPose();
        Drawing.drawDebug(follower.follower);
        robot.shooter.setShooterByDis(distance);
        showPose();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
        follower.breakFollowing();
    }

    /** 开到发射点：同时转炮台、飞轮对准发射距离，吸球电机延迟一会儿再停。 */
    private void driveToShoot() {
        add(new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new WaitCommand(INTAKE_STOP_DELAY_MS),
                        new InstantCommand(() -> robot.intake.intakeStop())
                ),
                new InstantCommand(() -> distance = CLOSE_FIRE_DISTANCE),
                new InstantCommand(() -> robot.shooter.turretToDegree(BLUE_CLOSE_FIRE_TURRET)),
                new DriveCurrentToPoint(follower, BLUE_CLOSE_SHOOT)
        ));
    }

    private void feed() {
        run(() -> robot.intake.gateOpen());
        run(() -> robot.intake.intakeFire(CLOSE_FIRE_INTAKE_POWER));
        pause(SHOOT_TIME_MS);
    }

    private void fire() {
        feed();
        run(() -> robot.intake.gateClose());
        run(() -> distance = CLOSE_HOLD_DISTANCE);
    }

    private Command gateIntake(long timeoutMs) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> robot.intake.intakeIn()),
                deadline(timeoutMs, new DriveCurrentToPoint(follower,
                        BLUE_CLOSE_INTAKE_GATE_CONTROL, BLUE_CLOSE_INTAKE_GATE))
        );
    }

    private void intakeWhileDriving(long timeoutMs, Command drive) {
        add(deadline(timeoutMs, drive));
    }

    private static Command deadline(long timeoutMs, Command drive) {
        return new ParallelDeadlineGroup(new WaitCommand(timeoutMs), drive);
    }

    private void add(Command command) {
        steps.add(command);
    }

    private void run(Runnable action) {
        steps.add(new InstantCommand(action));
    }

    private void pause(long ms) {
        steps.add(new WaitCommand(ms));
    }

    private void showPose() {
        Pose pose = follower.getPose();
        panelsTelemetry.addData("posX:", pose.getX());
        panelsTelemetry.addData("posY:", pose.getY());
        panelsTelemetry.addData("heading:", pose.getHeading());
        panelsTelemetry.update();
    }
}
