package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.Scorer;

@Config
@TeleOp(name = "HolderTest", group = "test")
public class ScorerTesting extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;
    private Servo holder;
    public static double SERVO_POS = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            holder.setPosition(SERVO_POS);

            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        holder = hardwareMap.get(Servo.class, "servo_hold_upper");

        telemetry.update();
    }
}
