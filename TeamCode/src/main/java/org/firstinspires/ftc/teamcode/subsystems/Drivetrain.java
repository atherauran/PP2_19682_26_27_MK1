package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.LEFT_BACK;
import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.PIN_POINT;
import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.RIGHT_BACK;
import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.RIGHT_FRONT;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** 麦轮底盘和 Pinpoint 里程计。自动里底盘由 Pedro 的 Follower 控制，不用这个类。 */
public class Drivetrain {
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    public GoBildaPinpointDriver pinPoint;

    public void init(HardwareMap hardwareMap) {
        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, PIN_POINT);
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);
        rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // 数值要和 pedroPathing/Constants.java 里的 localizerConstants 一致。
        pinPoint.setOffsets(-107, -15, DistanceUnit.MM);
        pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinPoint.resetPosAndIMU();
    }

    public void drive(Gamepad gamepad, double powerScale) {
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;
        leftFront.setPower((forward + strafe + turn) * powerScale);
        leftBack.setPower((forward - strafe + turn) * powerScale);
        rightFront.setPower((forward - strafe - turn) * powerScale);
        rightBack.setPower((forward + strafe - turn) * powerScale);
    }

    /** 以场地为参照的驾驶：摇杆往哪推，车就往场地的哪个方向走，和车头朝向无关。 */
    public void driveFieldOriented(Gamepad gamepad, double powerScale) {
        double y = -gamepad.left_stick_y, x = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;
        pinPoint.update();

        double stickAngle = Math.atan2(y, x) * 180 / Math.PI;
        double power = Math.hypot(x, y);
        double angle = (360 - pinPoint.getPosition().getHeading(AngleUnit.DEGREES)) + stickAngle;

        double sin = Math.sin(Math.toRadians(angle) - Math.PI / 4);
        double cos = Math.cos(Math.toRadians(angle) - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        leftFront.setPower((power * cos / max + turn) * powerScale);
        rightFront.setPower((power * sin / max - turn) * powerScale);
        leftBack.setPower((power * sin / max + turn) * powerScale);
        rightBack.setPower((power * cos / max - turn) * powerScale);
    }

    public double getHeading() {
        return pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public Pose2D getPosition() {
        pinPoint.update();
        return pinPoint.getPosition();
    }
}
