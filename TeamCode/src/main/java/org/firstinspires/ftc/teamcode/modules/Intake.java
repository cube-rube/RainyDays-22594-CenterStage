package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake {
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final DcMotor motor;
    private int direction = 0;
    public static final int TPR = 336;

    public Intake(LinearOpMode linearOpMode) {
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad1;

        motor = hardwareMap.get(DcMotor.class, "motor_intake");
        motor.setDirection(DcMotor.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Intake: Initialized");
    }

    public void opControlOld() {
        if (gamepad.dpad_down) {
            motor.setPower(1);
        } else if (gamepad.dpad_up) {
            motor.setPower(-1);
        } else {
            motor.setPower(0);
        }
    }

    public void opControl() {
        if (gamepad.dpad_down) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(1);
            direction = 1;
        } else if (gamepad.dpad_up) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-1);
            direction = -1;
        } else {
            int temp = motor.getCurrentPosition() % TPR;
            if (direction == 1) {
                motor.setTargetPosition(motor.getCurrentPosition() + TPR - temp);
            } else {
                motor.setTargetPosition(motor.getCurrentPosition() - temp);
            }
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("intake_motor_pos", motor.getCurrentPosition());
    }

    public void testing() {
        if (gamepad.dpad_down) {
            motor.setPower(0.1);
        } else if (gamepad.dpad_up) {
            motor.setPower(-0.1);
        } else {
            motor.setPower(0);
        }

        telemetry.addData("intake_motor_pos", motor.getCurrentPosition());
        telemetry.addData("intake_motor_power", motor.getPowerFloat());
    }
}
