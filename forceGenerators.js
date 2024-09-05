import {Vector2} from "./linear_algebra.js";

/* 
    When adding a new force generator, these functions must be in them and spelled correctly
        --> apply (applies a force to all objects)
        --> getEnergyApplied (calculates the work done against the force applied)
*/

export class Gravity {
    constructor() {
        this.gravity = 2;
    }

    apply(m_objects) {
        for (let i = 0; i < m_objects.length; i++) {
            // m_objects[i].force = Vector2.addVectors(m_objects[i].force, new Vector2(0, - this.g_GRAVITY));
            m_objects[i].force = new Vector2(0, -this.gravity);
        }
    }

    getEnergyApplied(m_objects) {
        let energy = 0;

        for (let i = 0; i < m_objects.length; i++) {
            energy += m_objects[i].mass * m_objects[i].pos.y * this.gravity;
        }
        return energy;
    }

}

export class LinearDamping {
    constructor() {
        this.MU = 0.0001;
    }

    apply(m_objects) {
        for (let i = 0; i < m_objects.length; i++) {
            m_objects[i].force = Vector2.subtractVectors(m_objects[i].force, Vector2.scaleVector(m_objects[i].vel, this.MU));
        }
    }

    getEnergyApplied(m_objects) {
        return 0;
    }

}