package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Intake;

@TeleOp(group = "test")
public class IntakeTesting extends LinearOpMode {
    public Intake intake;
    private final ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        waitForStart();
        preStart();

        while (opModeIsActive()) {
            intake.testingSensor();
            intake.opControlSensor();

            telemetry.addData("Runtime", runtime.toString());
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
