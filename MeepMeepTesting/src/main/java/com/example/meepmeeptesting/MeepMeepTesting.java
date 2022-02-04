package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				.setStartPose(new Pose2d(-10.0, -62.0, Math.PI / 2.0))
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(drive.getPoseEstimate())
								.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
								.addTemporalMarker(() -> {
									//arm.setPosition(1.715);
								})
								.waitSeconds(1.5)
								.addTemporalMarker(() -> {
									//intake.setSpeed(-0.4);
								})
								.waitSeconds(1.0)
								.addTemporalMarker(() -> {
									//intake.setSpeed(0.0);
									//arm.setPosition(1.35);
								})
								.lineToLinearHeading(new Pose2d(-64.0, -52.5, Math.PI / 4.0))  // go to the carousel thing
								.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
									//carousel.setSpeed(1.0);
								})
								.waitSeconds(3.5)
								.addTemporalMarker(() -> {
									//carousel.setSpeed(0.0);
								})
								.lineToLinearHeading(new Pose2d(-60, -34, Math.PI / 2.0))  // park
								.resetVelConstraint()
								.build()
				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}
}