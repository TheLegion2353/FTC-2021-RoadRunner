package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel extends RobotPart {
	private double kP = 0.01;
	private double kI = 0.0;
	private double kD = 0.0;
	private HardwareControllerEx motorController;

	public Carousel(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp);
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, null, motor);
	}

	public void setSpeed(double speed) {
		motorController.setSpeed(speed);
	}

	@Override
	public void driverUpdate() {
		if (gamepad != null) {
			if (gamepad.dpad_up) {
				motorController.setSpeed(1.0);
			} else if (gamepad.dpad_down) {
				motorController.setSpeed(-1.0);
			} else {
				motorController.setSpeed(0.0);
			}
		}
	}
}