package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;


public class Robot {
	private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	private MecanumDrivetrain drivetrain;
	private Carousel carousel;
	private Arm arm;
	private LinearSlide slide;
	private Intake intake;
	private Gamepad gamepad = null;
	private Telemetry telemetry = null;
	private TrajectorySequence BlueCloseTrajecotry = null;

	public Robot(Gamepad gp, Telemetry t, HardwareMap hwMap) {
		telemetry = t;
		gamepad = gp;
		drivetrain = new MecanumDrivetrain(gamepad, telemetry, hwMap);
		BlueCloseTrajecotry = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
			.lineToConstantHeading(new Vector2d(-62.5, 55.0))
			.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
				carousel.setSpeed(1.0);
			})
			.waitSeconds(3.0)
			.addTemporalMarker(() -> {
				carousel.setSpeed(0.0);
			})
			.lineToLinearHeading(new Pose2d(0.0, 60.0, 0.0))
			.lineToConstantHeading(new Vector2d(60, 50))
			.build();
	}

	public void runAuto() throws InterruptedException {
		drivetrain.getDrivetrain().followTrajectorySequence(BlueCloseTrajecotry);
	}

	public void update() {
		if (drivetrain != null) {
			drivetrain.update();
		} else {
			telemetry.addLine("Drivetrain null!");
		}

		if (carousel != null) {
			carousel.update();
		} else {
			telemetry.addLine("Carousel null!");
		}

		if (arm != null) {
			arm.update();
		} else {
			telemetry.addLine("Arm null!");
		}

		if (slide != null) {
			slide.update();
		} else {
			telemetry.addLine("Linear slide null!");
		}

		telemetry.update();
		clock.reset();
	}

	public void setCarouselMotor(DcMotorEx motor) {
		carousel = new Carousel(gamepad, motor, telemetry);
	}

	public void setArm(DcMotorEx motor, AnalogInput pot) {
		arm = new Arm(gamepad, motor, telemetry);
		arm.setPotentiometer(pot);
	}

	public void setLinearSlide(DcMotorEx motor) {
		slide = new LinearSlide(gamepad, motor, telemetry);
	}

	public void setIntake(DcMotorEx motor) {
		intake = new Intake(gamepad, motor, telemetry);
	}
}