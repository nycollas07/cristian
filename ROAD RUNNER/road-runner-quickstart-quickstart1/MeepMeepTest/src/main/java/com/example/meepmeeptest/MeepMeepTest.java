package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600)
        .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
        .setTheme(new ColorSchemeRedDark())
        .setBackgroundAlpha(1f);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints( 51.77443277603339, 40, Math.toRadians(180), Math.toRadians(180), 11.50)

                 /* AUDIENCE SIDE ESQUERDO*/ //  .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-35, 62.6, Math.toRadians(-90)))
                 /* BACKDROP SIDE ESQUERDO*/ //    .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(11.5, 62.6, Math.toRadians(-90)))
                 /* BACKDROP SIDE DIREITO*/  //   .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(11.5, -62.6, Math.toRadians(90)))
                 /* AUDIENCE SIDE DIREITO*/     .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-35, -62.6, Math.toRadians(90)))

                        .waitSeconds(30)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}