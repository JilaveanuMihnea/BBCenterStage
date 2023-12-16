package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Spec {
    // motors
    // movement
    public static float MOVEMENT_SPEED = 1f;

    //systems speed
    public static float TAGA_SPEED = 0.4f;
    public static float SLIDER_SPEED_UP = 0.6f;
    public static float SLIDER_SPEED_DOWN = 0.2f;


    //systems movement
    public static int TAGA_TICK_60DEG;
    public static int SLIDER_TICK_MAXH;
    public static int SLIDER_TICK_GRAB;
    public static int SLIDER_THRESHOLD_RAISE;
    public static int TAGA_THRESHOLD_LOWER;


    // servos
    public static float CLOSED_POS_LEFT = 1f;
    public static float CLOSED_POS_RIGHT = 0.8f;
    public static float OPENED_POS_RIGHT = 1f;
    public static float OPENED_POS_LEFT = 0.8f;
    public static float HOLD_ALIGN = 0.8f;
    public static float DRONE_ADJUST = 0.8f;
    public static float DRONE_LAUNCH = 1f;


    public static int BUTTON_DELAY = 200;
}
