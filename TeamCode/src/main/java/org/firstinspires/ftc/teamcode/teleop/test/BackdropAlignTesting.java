package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OperatorDrive;

@TeleOp(group = "test")
public class BackdropAlignTesting extends LinearOpMode {
    public OperatorDrive operatorDrive;
    private final ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        runtime.reset();
        operatorDrive.runtimeReset();

        while (opModeIsActive()) {
            operatorDrive.sendAngle();
            operatorDrive.drive();
            operatorDrive.telemetry();

            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        operatorDrive = new OperatorDrive(this, dashboard);

        telemetry.update();
    }
}
