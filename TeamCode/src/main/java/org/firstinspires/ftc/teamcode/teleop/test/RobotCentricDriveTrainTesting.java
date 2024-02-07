package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OperatorDrive;

@TeleOp(name = "RobotCentricDriveNoPIDTesting", group = "test")
public class RobotCentricDriveTrainTesting extends LinearOpMode {
    public OperatorDrive operatorDrive;
    private final ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();
        operatorDrive.runtimeReset();

        while (opModeIsActive()) {
            operatorDrive.driveFieldCentric();
            operatorDrive.sendAngle();
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
