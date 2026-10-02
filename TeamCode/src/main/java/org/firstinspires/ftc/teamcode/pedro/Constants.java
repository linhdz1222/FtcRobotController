package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Matrix;
import com.pedropathing.controllers.Controller;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class Constants {

    // 1. Cấu hình Hệ thống truyền động Mecanum (Drivetrain Config)
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.backLeftName.set("backLeft");
                c.frontRightName.set("frontRight");
                c.backRightName.set("backRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    // 2. Cấu hình Pinpoint Odometry (Localizer Config)
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(-182.0 / 25.4); // Chuyển mm sang inch (182 mm / 25.4)
                c.yPodOffset.set(-196.0 / 25.4); // Chuyển mm sang inch (196 mm / 25.4)
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
    );

    // 3. Cấu hình Thuật toán điều khiển Foresight & PID (Foresight Config)
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.3);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

// Cài đặt Hằng số Feedforward khi thả trôi (Coast) và phanh (Brake)
                c.coast.set(Controller.proportionalFeedforward(0.010978350889324107)); // Hệ số trôi tự nhiên của động cơ
                c.brake.set(Controller.proportionalFeedforward(0.008731598255925491)); // Hệ số phanh chủ động của động cơ

// Cài đặt Điều khiển phản hồi góc xoay (Heading Feedback)
                c.headingFeedback.set(Controller.proportional(5.258721785960744));     // Độ nhạy PID điều chỉnh góc hướng (Heading)

// Cài đặt Hệ số phanh xoay (Heading Brake)
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063)); // Lực phanh khi xoay góc
            }
    );

    // 4. Phương thức tạo và khởi tạo Follower cho v3
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
