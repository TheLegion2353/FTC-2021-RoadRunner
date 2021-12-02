package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide extends RobotPart {
	private double kP = 0.01;
	private double kI = 0.0;
	private double kD = 0.0;
	private double position = 0.0;
	private Telemetry telemetry;

	private HardwareControllerEx motorController;

	public LinearSlide(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp);
		telemetry = tel;
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, new PID(kP, kI, kD, position), motor);
	}

	@Override
	public void driverUpdate() {
		if (gamepad.right_bumper) {
			position += 500.0 * motorController.getPID(0).getElapsedTime();
		} else if (gamepad.left_bumper) {
			position -= 500.0 * motorController.getPID(0).getElapsedTime();
		}
		motorController.setSpeed(position);
		telemetry.addData("Set Point of Linear Slide: ", position);
		telemetry.addData("Position of Linear Slide: ", motorController.getPos());
	}
}