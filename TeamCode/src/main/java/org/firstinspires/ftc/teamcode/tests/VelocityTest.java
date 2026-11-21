package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.constants.PanelConstants.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.FieldPose;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.TargetSelector;

@TeleOp
public class VelocityTest extends LinearOpMode {
    Robot robot = new Robot();
    JoinedTelemetry joinedTele;
    double distance;
    boolean shootOn = false;

    double previousTarget = Double.NaN;
    double inBandSince = Double.NaN, dropTime = Double.NaN, lastRecovery = Double.NaN;
    boolean wasStable;
    final ElapsedTime speedClock = new ElapsedTime();

    // 回速指“第一次低于目标范围”到“重新进入范围并站稳”的时间，起转阶段不算。
    private void measureRecovery(double target, double actual) {
        double now = speedClock.seconds();
        double tolerance = Math.max(1, VELOCITY_TOR);
        if (!shootOn || target == 0 || actual <= 0 || robot.shooter.batteryVolts == 0
                || robot.shooter.hasFlywheelFeedbackFault() || target != previousTarget) {
            wasStable = false;
            inBandSince = dropTime = lastRecovery = Double.NaN;
        } else if (Math.abs(actual - Math.abs(target)) <= tolerance) {
            if (Double.isNaN(inBandSince)) inBandSince = now;
            // 连续 0.15 秒都在范围内才算站稳，偶尔一次读到目标不算。
            if (now - inBandSince >= 0.15) {
                if (!Double.isNaN(dropTime)) lastRecovery = inBandSince - dropTime;
                dropTime = Double.NaN;
                wasStable = true;
            }
        } else {
            inBandSince = Double.NaN;
            if (wasStable && actual < Math.abs(target) - tolerance) {
                dropTime = now;
                wasStable = false;
            }
        }
        previousTarget = target;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        waitForStart();
        if (isStopRequested()) return;
        robot.drivetrain.pinPoint.setPosition(FieldPose.takeTeleOpStart());

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1);
            Pose2D current = robot.drivetrain.getPosition();
            Pose2D target = TargetSelector.getTarget(current.getY(DistanceUnit.INCH), true);
            distance = Math.hypot(target.getY(DistanceUnit.INCH) - current.getY(DistanceUnit.INCH),
                    target.getX(DistanceUnit.INCH) - current.getX(DistanceUnit.INCH));

            if (gamepad1.dpadUpWasPressed()) VELOCITY += 20;
            if (gamepad1.dpadDownWasPressed()) VELOCITY -= 20;
            if (gamepad1.dpadLeftWasPressed()) FIRE_POWER += 0.01;
            if (gamepad1.dpadRightWasPressed()) FIRE_POWER -= 0.01;
            FIRE_POWER = Math.max(0, Math.min(FIRE_POWER, 1));

            if (gamepad1.yWasPressed()) POSITION += 0.01;
            if (gamepad1.aWasPressed()) POSITION -= 0.01;
            POSITION = Math.max(0, Math.min(POSITION, 1));

            if (gamepad1.right_trigger > 0) robot.intake.intakeFire(FIRE_POWER);
            else robot.intake.intakeStop();

            if (gamepad1.rightBumperWasPressed()) {
                shootOn = true;
                robot.intake.gateOpen();
            }
            if (gamepad1.leftBumperWasPressed()) {
                robot.intake.gateClose();
                shootOn = false;
                robot.shooter.turretToDegree(0);
            }

            if (shootOn)robot.shooter.setShooterVelocity(VELOCITY);
            else robot.shooter.shooterStop();
            robot.shooter.panelTo(POSITION);

            measureRecovery(VELOCITY, Math.abs(robot.shooter.flywheelSpeed));
            joinedTele.addData("distance", distance);
            joinedTele.addData("intakePower", FIRE_POWER);
            joinedTele.addData("target velocity", VELOCITY);
            joinedTele.addData("current velocity", shootOn && VELOCITY != 0
                    ? robot.shooter.flywheelSpeed : robot.shooter.getShooterVelocity());
            joinedTele.addData("flywheel output (V)", robot.shooter.appliedFlywheelVolts);
            if (Double.isFinite(lastRecovery)) joinedTele.addData("last recovery (s)", lastRecovery);
            joinedTele.addData("left / right (ticks/s)", "%.0f / %.0f",
                    robot.shooter.leftShooter.getVelocity(), robot.shooter.rightShooter.getVelocity());
            joinedTele.addData("Flywheel feedback fault", robot.shooter.hasFlywheelFeedbackFault());
            joinedTele.addData("POSITION", POSITION);
            joinedTele.update();
        }
    }
}
