package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * Static Class to define Gamefield Vector positions.
 * These are used in start Position estimates and in automatic targetic.
 *
 * The static class also has PosStorage defined, to pass the last position in autonomous mode
 * to following TeleOp mode
 */
public class GameField {
    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    public static final Vector2d ORIGIN = new Vector2d(0,0);
    public static final Pose2d ORIGINPOSE = new Pose2d(0,0,Math.toRadians(0));

    // Declare and assign starting pose of robot
    //TODO: Update start position correctly.
    public static final Pose2d BLUE_WAREHOUSE_STARTPOS =  new Pose2d(-61,7,Math.toRadians(180));
    public static final Pose2d BLUE_STORAGE_STARTPOS =  new Pose2d(-61,-40,Math.toRadians(180));
    public static final Pose2d RED_WAREHOUSE_STARTPOS =  new Pose2d(61,7,Math.toRadians(0));
    public static final Pose2d RED_STORAGE_STARTPOS =  new Pose2d(61,-40,Math.toRadians(0));
    public static final Pose2d BLUE_MIDDLE_STARTPOS =  new Pose2d(-61,-30.5,Math.toRadians(180));
    public static final Pose2d RED_MIDDLE_STARTPOS =  new Pose2d(61,-29,Math.toRadians(0));


    public enum DEBUG_LEVEL{
        NONE,
        MINIMUM,
        MAXIMUM
    }
    public static DEBUG_LEVEL debugLevel = DEBUG_LEVEL.MINIMUM;

    //Define and declare Playing Alliance
    public enum PLAYING_ALLIANCE{
        RED_ALLIANCE,
        BLUE_ALLIANCE,
    }
    public static PLAYING_ALLIANCE playingAlliance = PLAYING_ALLIANCE.BLUE_ALLIANCE;
    public static double ALLIANCE_FACTOR = 1;


    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        WAREHOUSE,
        STORAGE,
        MIDDLE,
        ORIGIN
    }
    public static START_POSITION startPosition = START_POSITION.WAREHOUSE;

    public enum PARKING_LOCATION{
        WAREHOUSE,
        STORAGE,
        NO_PARKING
    }

    public enum AUTONOMOUS_ROUTE {
        THROUGH_BARRIER,
        ALONG_WALL,
        NOT_APPLICABLE
    }

    public static boolean END_PARKING_FACING_SHARED_SHIPPING_HUB = false;

    //Define targets for Vision to determine Autonomous mode action based on camera detection
    public enum VISION_IDENTIFIED_TARGET {
        LEVEL1,
        LEVEL2,
        LEVEL3,
        UNKNOWN;
    };

    public enum VISION_IDENTIFIER{
        DUCK,
        MARKER
    };
    public static VISION_IDENTIFIER visionIdentifier = VISION_IDENTIFIER.DUCK;

    //Static fields to pass Pos from Autonomous to TeleOp
    public static boolean poseSetInAutonomous = false;
    public static Pose2d currentPose = new Pose2d();

}
