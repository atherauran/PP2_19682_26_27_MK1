package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.constants.PanelConstants.FIRE_POWER;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.PANEL_MAX;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.PANEL_MIN;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.TURRET_ABS_RANGE_DEGREE;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Manual TeleOp 2027", group = "Competition")
public class ManualTeleOp extends LinearOpMode {
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double FLYWHEEL_STEP = 20.0;
    private static final double TURRET_STEP_DEG = 1.0;
    private static final double PANEL_STEP = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain();
        Intake intake = new Intake();
        Shooter shooter = new Shooter();
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        JoinedTelemetry display = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        boolean flywheelOn = false;
        double flywheelVelocity = 1800.0;
        double panelPosition = 0;
        double turretAngle = 0.0;

        try {
            intake.gateClose();
            shooter.panelTo(panelPosition);
            waitForStart();

            while (opModeIsActive()) {
                drivetrain.drive(gamepad1, 1.0);

                if (gamepad1.leftBumperWasPressed()) flywheelOn = !flywheelOn;
                // 用 | 而不是 ||：两个手柄的按键事件都要读掉，同时按下只算一次。
                if (gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed()) flywheelVelocity += FLYWHEEL_STEP;
                if (gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed()) flywheelVelocity -= FLYWHEEL_STEP;
                if (gamepad1.dpadLeftWasPressed() | gamepad2.dpadLeftWasPressed()) turretAngle += TURRET_STEP_DEG;
                if (gamepad1.dpadRightWasPressed() | gamepad2.dpadRightWasPressed()) turretAngle -= TURRET_STEP_DEG;
                if (gamepad2.yWasPressed()) panelPosition += PANEL_STEP;
                if (gamepad2.aWasPressed()) panelPosition -= PANEL_STEP;

                flywheelVelocity = Math.max(0.0, flywheelVelocity);
                panelPosition = Range.clip(panelPosition, PANEL_MIN, PANEL_MAX);
                turretAngle = Range.clip(turretAngle, -TURRET_ABS_RANGE_DEGREE, TURRET_ABS_RANGE_DEGREE);
                shooter.panelTo(panelPosition);
                shooter.turretToDegree(turretAngle);
                if (flywheelOn && flywheelVelocity > 0.0) {
                    shooter.setShooterVelocity(flywheelVelocity);
                } else {
                    shooter.shooterStop();
                }

                if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                    intake.gateClose();
                    intake.intakeIn();
                } else if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                    intake.gateOpen();
                    intake.intakeOut(gamepad1.left_trigger);
                } else if (gamepad1.right_bumper) {
                    intake.gateOpen();
                    intake.intakeFire(FIRE_POWER);
                } else {
                    intake.intakeStop();
                    intake.gateClose();
                }

                display.addData("Flywheel", flywheelOn ? "ON" : "OFF");
                display.addData("Flywheel target / actual (ticks/s)", "%.0f / %.0f",
                        flywheelVelocity, shooter.getShooterVelocity());
                display.addData("Panel position", "%.2f", panelPosition);
                display.addData("Turret target / actual (deg)", "%.1f / %.1f",
                        turretAngle, shooter.getTurretDegree());
                display.update();
            }
        } finally {
            drivetrain.drive(gamepad1, 0.0);
            shooter.shooterStop();
            shooter.turretRotateStop();
            intake.intakeStop();
            intake.gateClose();
        }
    }
}
