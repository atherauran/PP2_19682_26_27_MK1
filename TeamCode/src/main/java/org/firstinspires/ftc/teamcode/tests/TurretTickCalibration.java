package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.RobotConstants.TURRET_FULL_RANGE_DEGREE;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.TURRET_FULL_RANGE_ENCODER;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * 炮台“编码器格数 ↔ 角度”标定。
 *
 * <ol>
 *   <li>在炮台上贴一个指针，底盘上固定一个量角器。
 *   <li>右保险杠正向点动，左保险杠反向点动（低功率，松开就停）。点动到第一个参考角度，
 *       按 X 把编码器清零，这里就是 0 度参考点。
 *   <li>继续点动到第二个参考角度，用量角器量出转过的真实角度。两点相距越远，误差越小，
 *       建议转到另一侧的极限。
 *   <li>屏幕上的 rawTicks 就是转过这个角度对应的格数。把 {@code RobotConstants} 里的
 *       TURRET_FULL_RANGE_DEGREE 填成量出的角度，TURRET_FULL_RANGE_ENCODER 填成 rawTicks。
 *       两个数只是一个比值，但必须来自同一次测量。
 * </ol>
 */
@TeleOp(name = "Turret Tick Calibration", group = "Tuning")
public class TurretTickCalibration extends LinearOpMode {
    private static final double JOG_POWER = 0.15; // 点动功率，故意给低，方便对准角度

    private final Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        JoinedTelemetry joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        joinedTele.addData("说明", "右保险杠 = 正向点动，左保险杠 = 反向点动，X = 把当前位置清零为 0 度");
        joinedTele.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                robot.shooter.turret.setPower(JOG_POWER);
            } else if (gamepad1.left_bumper) {
                robot.shooter.turret.setPower(-JOG_POWER);
            } else {
                robot.shooter.turret.setPower(0.0);
            }

            if (gamepad1.xWasPressed()) {
                robot.shooter.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.shooter.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            int rawTicks = robot.shooter.turret.getCurrentPosition();
            double estimatedDeg = rawTicks / TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE;

            joinedTele.addData("rawTicks（记这个）", rawTicks);
            joinedTele.addData("按当前比例估算的角度（仅供对照）", estimatedDeg);
            joinedTele.addData("当前比例", TURRET_FULL_RANGE_ENCODER + " 格 / " + TURRET_FULL_RANGE_DEGREE + " 度");
            joinedTele.update();
        }
    }
}
