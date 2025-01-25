package frc.robot.data;

import frc.robot.data.State;


public class StateManager {
    private State currentState;

    public StateManager(State initialState) {
        currentState = initialState;
    }

    public void setState(State newState) {
        switch (newState) {
            case State.Idling state -> {
                    currentState = newState;
                    break;
                }
            case State.Driving state -> {
                    currentState = newState;
                    break;
                }
            case State.Climbing state -> {
                    currentState = newState;
                    break;
                }
        }
    }

    public State getState() {
        return currentState;
    }
}
