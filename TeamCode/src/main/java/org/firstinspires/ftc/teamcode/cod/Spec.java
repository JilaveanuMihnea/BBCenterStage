package org.firstinspires.ftc.teamcode.cod;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Spec {
    // motors
    // movement
    public static float MOVEMENT_SPEED = 1f;

    //systems speed
    public static float TAGA_SPEED = 0.3f;
    public static float TAGA_SPEED_AUTO = 0.6f;
    public static float SLIDER_SPEED_UP = 1f;
    public static float SLIDER_SPEED_DOWN = 0.9f;


    //systems movement
    public static int TAGA_TICK_60DEG = 1050;
    public static int TAGA_PE_SPATE = 1400;
    public static int TAGA_MAX = 1600;
    public static int SLIDER_TICK_MAXH= 1550; //1584
    public static int SLIDER_TICK_GRAB = 800;
    public static int SLIDER_THRESHOLD_RAISE;
    public static int TAGA_THRESHOLD_LOWER=100;


    // servos
    public static float CLOSED_POS_LEFT = 0.99f;
    public static float OPENED_POS_LEFT = 0.6f; // 0.4
    public static float CLOSED_POS_RIGHT = 0.15f;
    public static float OPENED_POS_RIGHT = 0.55f; // 5

    public static float HOLD_ALIGN = 0.48f; // creste = mai jos +0.07 pt hold place
    public static float HOLD_PLACE = 0.55f;
    public static float HOLD_SAFE = 0.2f; //0
    public static float DRONE_ADJUST = 0.15f; //creste = ridica || 355grade modifica putin !!
    public static float DRONE_HOLD = 1f;
    public static float DRONE_LAUNCH = 0.35f;


    public static int BUTTON_DELAY = 200;
}
