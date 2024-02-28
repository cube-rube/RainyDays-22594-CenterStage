package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Scorer;

@TeleOp(group = "test")
public class FlapServosTesting extends LinearOpMode {
    public Intake intake;
    private final ElapsedTime runtime = new ElapsedTime();
    private Servo leftServo;
    private Servo rightServo;
    public FtcDashboard dashboard;

    public static double OPEN_LEFT_POS = 0;
    public static double OPEN_RIGHT_POS = 0;
    public static double CLOSE_LEFT_POS = 0;
    public static double CLOSE_RIGHT_POS = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();
        preStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                leftServo.setPosition(OPEN_LEFT_POS);
            }
            if (gamepad1.b) {
                leftServo.setPosition(CLOSE_LEFT_POS);
            }
            if (gamepad1.x) {
                rightServo.setPosition(OPEN_RIGHT_POS);
            }
            if (gamepad1.a) {
                rightServo.setPosition(CLOSE_RIGHT_POS);
            }

            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("left_port", leftServo.getPortNumber());
            telemetry.addData("left_man", leftServo.getManufacturer());
            telemetry.addData("right_port", rightServo.getPortNumber());
            telemetry.addData("right_man", rightServo.getManufacturer());
            telemetry.addData("x", gamepad1.x);
            telemetry.addData("y", gamepad1.y);
            telemetry.addData("a", gamepad1.a);
            telemetry.addData("b", gamepad1.b);

            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        leftServo = hardwareMap.get(Servo.class, "left_flap");
        rightServo = hardwareMap.get(Servo.class, "right_flap");

        telemetry.update();
    }

    private void preStart() {
        runtime.reset();
    }
}
