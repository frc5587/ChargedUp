package frc.robot.subsystems;

import frc.robot.subsystems.ArmStateMachine.Input;

public enum ArmState {
    STOWED {
        public ArmState next(Input input) {
            switch(input) {
                case LOWER:
                    return LOWERING;
                case RAISE:
                    return RAISING;
                default:
                    return this;
            }
        }
    },

    STOWING {
        public ArmState next(Input input) {
            switch(input) {
                case DONE:
                    return STOWED;
                default:
                    return this;
            }
        }
    },

    RAISED {
        public ArmState next(Input input) {
            switch(input) {
                case LOWER:
                    return LOWERING;
                case INTERRUPT:
                    return LOWERING;
                default:
                    return this;
            }
        }
    },

    RAISING {
        public ArmState next(Input input) {
            switch(input) {
                case DONE:
                    return RAISED;
                case INTERRUPT:
                    return LOWERING;
                default:
                    return this;
            }
        }
    },

    LOWERED {
        public ArmState next(Input input) {
            switch(input) {
                case STOW:
                    return STOWING;
                case RAISE:
                    return RAISING;
                default:
                    return this;
            }
        }
    },

    LOWERING {
        public ArmState next(Input input) {
            switch(input) {
                case DONE:
                    return LOWERED;
                case INTERRUPT:
                    return this;
                default:
                    return this;
            }
        }
    };

    public ArmState next(Input input) {
        return this;
    }
}
