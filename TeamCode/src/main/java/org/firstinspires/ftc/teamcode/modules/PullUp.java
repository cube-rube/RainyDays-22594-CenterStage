package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PullUp {
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final DcMotor motor;
    private final ElapsedTime timer;
    private double tempPower = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0;
    public static double reference = 0;
    private double lastError = 0;
    private double integralSum = 0;

    public PullUp(LinearOpMode linearOpMode) {
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad1;
        timer = new ElapsedTime();

        motor = hardwareMap.get(DcMotor.class, "motor_up");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("PullUp: ", "Initialized");
    }

    public void tele() {
        if (tempPower != 0) {
            motor.setPower(tempPower);
        } else {
            motor.setPower(-gamepad.right_stick_y);
        }
        if (gamepad.a) {
            tempPower = -gamepad.right_stick_y;
        }
        telemetry.addData("Pull_Up power: ", motor.getPower());
    }

    public void movePID() {
        double encoderPos = motor.getCurrentPosition();
        double error = reference - encoderPos;
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double out = (kP * error) + (kI * integralSum) + (kD * derivative) + kG;

        motor.setPower(out);

        lastError = error;

        timer.reset();
    }
}
