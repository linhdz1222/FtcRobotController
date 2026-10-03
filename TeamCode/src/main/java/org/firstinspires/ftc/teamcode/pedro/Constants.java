package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // 1. Cấu hình hệ thống truyền động Mecanum (Drivetrain Config)
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                // Đặt tên thiết bị khớp với cấu hình phần cứng trên Driver Station
                c.frontLeftName.set("lf");  // Motor trước - trái
                c.backLeftName.set("lr");   // Motor sau - trái
                c.frontRightName.set("rf"); // Motor trước - phải
                c.backRightName.set("rr");  // Motor sau - phải

                // Đảo chiều motor bên trái để đúng chiều quay tiến/lùi
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                // Bật chế độ phanh động cơ chủ động khi không cấp lực lái
                c.manualBrakeMode.set(true);
            }
    );

    // 2. Cấu hình cảm biến định vị GoBILDA Pinpoint (Localizer Config)
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint"); // Tên cảm biến Pinpoint trên Hardware Map

                // Tọa độ khoảng cách từ tâm cảm biến Pinpoint tới Tâm xoay của robot (đơn vị: Inch)
                c.xPodOffset.set(2.187);  // Vị trí Pod X (Tiến/Lùi)
                c.yPodOffset.set(-4.572); // Vị trí Pod Y (Đi ngang)

                // Hướng đọc đếm xung của 2 trục Encoder Pinpoint
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }
    );

    // 3. Cấu hình thuật toán Foresight & Động lực học (Foresight Config)
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // --- BỘ ĐIỀU KHIỂN CHUYỂN ĐỘNG TỊNH TIẾN ---
                // Khai báo bộ điều khiển P khi ở xa (primary = 0.3) và khi ở gần điểm dừng (secondary = 0.1)
                Controller primaryTranslationalForward = Controller.proportional(0.3);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.3);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);

                // Ghép mảnh (Piecewise): Chuyển mượt sang P phụ khi khoảng cách tới đích nhỏ hơn 2.5 inch
                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                // --- HẰNG SỐ FEEDFORWARD (Hệ số lực kéo/duy trì tốc độ) ---
                c.coast.set(Controller.proportionalFeedforward(0.010978350889324107)); // Hệ số Feedforward khi thả trôi
                c.brake.set(Controller.proportionalFeedforward(0.008731598255925491)); // Hệ số Feedforward khi phanh

                // --- PHẢN HỒI VÒNG KÍN CHỈNH GÓC XOAY (Heading Feedback) ---
                c.headingFeedback.set(Controller.proportional(5.258721785960744)); // Hệ số P điều chỉnh góc hướng (Heading)

                // --- MA TRẬN VÀ VECTƠ PHANH ĐỘNG LỰC HỌC ---
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695)); // Hệ số phanh xoay góc
                c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));          // Hệ số phanh tuyến tính
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));    // Hệ số phanh phi tuyến

                // --- GIỚI HẠN VẬN TỐC VÀ GIA TỐC THỰC TẾ (Được đo từ SysId / AutoTune) ---
                c.maxAchievableForwardVelocity.set(72.72923108818539); // Vận tốc tiến tối đa (inch/s)
                c.maxAchievableStrafeVelocity.set(52.34323936525474);  // Vận tốc đi ngang tối đa (inch/s)
                c.naturalForwardDeceleration.set(85.01144677379789);   // Gia tốc giảm tốc tự nhiên khi tiến (inch/s²)
                c.naturalStrafeDeceleration.set(104.49787535782846);   // Gia tốc giảm tốc tự nhiên khi đi ngang (inch/s²)
            }
    );

    // 4. Hàm khởi tạo và kết nối các thành phần vào Follower
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig), // Bộ định vị GoBILDA Pinpoint
                new Mecanum(h, drivetrainConfig),           // Hệ thống truyền động Mecanum
                new Foresight(foresightConfig)             // Thuật toán điều khiển quỹ đạo Foresight
        );
    }
}