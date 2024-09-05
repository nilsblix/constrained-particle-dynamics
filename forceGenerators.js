import {Vector2} from "./linear_algebra.js";

/* 
    When adding a new force generator, these functions must be in them and spelled correctly
        --> apply (applies a force to all objects)
        --> getEnergyApplied (calculates the work done against the force applied)
*/

export class Gravity {
    static g_GRAVITY = 6;

    constructor() {}

    apply(m_objects) {
        for (let i = 0; i < m_objects.length; i++) {
            m_objects[i].force = Vector2.addVectors(m_objects[i].force, new Vector2(0, - this.g_GRAVITY));
        }
    }

    getEnergyApplied(m_objects) {
        let energy = 0;
        for (let i = 0; i < m_objects.length; i++) {
            energy += m_objects[i].mass * m_objects[i].pos.y * g_GRAVITY;
        }
        return energy;
    }

}

export class LinearDamping {
    static g_MU = 0.9;

    constructor() {}

    apply(m_objects) {
        for (let i = 0; i < m_objects.length; i++) {
            m_objects[i].force = Vector2.subtractVectors(Vector2.scaleVector(m_objects[i].vel, g_MU));
        }
    }

    getEnergyApplied(m_objects) {
        return 0;
    }

}