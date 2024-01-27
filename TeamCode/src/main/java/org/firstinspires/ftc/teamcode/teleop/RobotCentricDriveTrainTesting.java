package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OperatorDrive;

@TeleOp(name = "RobotCentricDriveNoPIDTesting")
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
            operatorDrive.driveRobotCentric();
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
