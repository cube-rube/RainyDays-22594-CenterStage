package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Intake;

@TeleOp(group = "test")
public class FlapTesting extends LinearOpMode {
    public Intake intake;
    private final ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();
        preStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                intake.closeLeftFlap();
            } else {
                intake.openLeftFlap();
            }
            if (gamepad1.x) {
                intake.closeRightFlap();
            } else {
                intake.openRightFlap();
            }

            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("x", gamepad1.x);
            telemetry.addData("y", gamepad1.y);
            telemetry.addData("a", gamepad1.a);
            telemetry.addData("b", gamepad1.b);

            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        intake = new Intake(this);

        telemetry.update();
    }

    private void preStart() {
        runtime.reset();
    }
}
