package org.firstinspires.ftc.teamcode;

public class Button {

    public enum State // button states
    {
        DOWN,   // moment press down
        HELD,   // continued press down
        UP,     // moment of release
        OFF,    // continued release
        NOT_INITIALIZED
    }

    State state = State.NOT_INITIALIZED;
    String name = "<NO_NAME>";
    String action = "<NO_ACTION>";

    public Button(String _name, String _action) {
        name = _name;
        action = _action;
    }

    public State updateState(boolean buttonPressed) {
        if (buttonPressed) {
            if (state == State.OFF || state == State.UP || state == State.NOT_INITIALIZED)
                state = state.DOWN;
            else
                state = state.HELD;
        }
        else {
            if (state == State.HELD || state == State.DOWN)
                state = State.UP;
            else
                state = State.OFF;
        }
        return state;
    }

    public boolean isDown() { return state == State.DOWN; }
    
    public boolean isUp() {
        return state == State.UP;
    }

}