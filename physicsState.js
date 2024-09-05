import {Vector2} from linear_algebra.js;

class physicsState {
    static g_GRAVITY = 10;
    static g_EPSILON = 1e-5;

    #m_objects = [];
    #m_constraints = [];
    #m_forceGenerators = [];

    constructor() {

    }

    simulationStep(dt) {

    }

}