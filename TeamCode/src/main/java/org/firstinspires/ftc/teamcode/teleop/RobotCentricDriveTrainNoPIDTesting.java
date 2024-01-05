package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.BasicDrive;

@TeleOp(name = "RobotCentricDriveNoPIDTesting")
public class RobotCentricDriveTrainNoPIDTesting extends LinearOpMode {
    public BasicDrive basicDrive;
    private final ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();
        basicDrive.runtimeReset();

        while (opModeIsActive()) {
            basicDrive.driveRobotCentric();
            basicDrive.testing();

            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        basicDrive = new BasicDrive(this, dashboard);

        telemetry.update();
    }
}
