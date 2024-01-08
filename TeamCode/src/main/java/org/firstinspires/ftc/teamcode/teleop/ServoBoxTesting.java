package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "ServoBeamRightTesting")
public class ServoBoxTesting extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Servo servoRotationBox;
    public FtcDashboard dashboard;
    public static double SERVO_POS = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            servoRotationBox.setPosition(SERVO_POS);


            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        servoRotationBox = hardwareMap.get(Servo.class, "servo_rotation_box");

        telemetry.update();
    }
}
