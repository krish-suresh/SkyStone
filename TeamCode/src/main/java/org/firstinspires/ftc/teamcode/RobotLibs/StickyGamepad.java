package org.firstinspires.ftc.teamcode.RobotLibs;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StickyGamepad {

    private Gamepad gamepad;

    public boolean dpad_up, dpad_down, dpad_left, dpad_right;
    public boolean a, b, x, y;
    public boolean left_bumper, right_bumper;
    public boolean left_stick_button, right_stick_button;

    private boolean dpad_up_up, dpad_down_up, dpad_left_up, dpad_right_up;
    private boolean a_up, b_up, x_up, y_up;
    private boolean left_bumper_up, right_bumper_up;
    private boolean left_stick_button_up, right_stick_button_up;

    public StickyGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        if (gamepad.dpad_down) {
            if (dpad_down_up) {
                dpad_down_up = false;
                if (!dpad_down) {
                    dpad_down = true;
                } else if (dpad_down) {
                    dpad_down = false;
                }
            }
        } else {
            dpad_down_up = true;
        }

        if (gamepad.dpad_up) {
            if (dpad_up_up) {
                dpad_up_up = false;
                if (!dpad_up) {
                    dpad_up = true;
                } else if (dpad_up) {
                    dpad_up = false;
                }
            }
        } else {
            dpad_up_up = true;
        }

        if (gamepad.dpad_left) {
            if (dpad_left_up) {
                dpad_left_up = false;
                if (!dpad_left) {
                    dpad_left = true;
                } else if (dpad_left) {
                    dpad_left = false;
                }
            }
        } else {
            dpad_left_up = true;
        }

        if (gamepad.dpad_right) {
            if (dpad_right_up) {
                dpad_right_up = false;
                if (!dpad_right) {
                    dpad_right = true;
                } else if (dpad_right) {
                    dpad_right = false;
                }
            }
        } else {
            dpad_right_up = true;
        }

        if (gamepad.a) {
            if (a_up) {
                a_up = false;
                if (!a) {
                    a = true;
                } else if (a) {
                    a = false;
                }
            }
        } else {
            a_up = true;
        }

        if (gamepad.b) {
            if (b_up) {
                b_up = false;
                if (!b) {
                    b = true;
                } else if (b) {
                    b = false;
                }
            }
        } else {
            b_up = true;
        }

        if (gamepad.x) {
            if (x_up) {
                x_up = false;
                if (!x) {
                    x = true;
                } else if (x) {
                    x = false;
                }
            }
        } else {
            x_up = true;
        }

        if (gamepad.y) {
            if (y_up) {
                y_up = false;
                if (!y) {
                    y = true;
                } else if (y) {
                    y = false;
                }
            }
        } else {
            y_up = true;
        }
        if (gamepad.left_bumper) {
            if (left_bumper_up) {
                left_bumper_up = false;
                if (!left_bumper) {
                    left_bumper = true;
                } else if (left_bumper) {
                    left_bumper = false;
                }
            }
        } else {
            left_bumper_up = true;
        }


        if (gamepad.right_bumper) {
            if (right_bumper_up) {
                right_bumper_up = false;
                if (!right_bumper) {
                    right_bumper = true;
                } else if (right_bumper) {
                    right_bumper = false;
                }
            }
        } else {
            right_bumper_up = true;
        }
    }
}
