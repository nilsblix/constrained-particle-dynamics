import {Vector2} from "./linear_algebra.js";

export class DynamicObject {
    constructor(pos, mass, drawing_radius = 0.5) {
        this.pos = pos;
        this.vel = new Vector2(0,0);
        this.force = new Vector2(0,0);
        this.mass = mass;
        this.wass = 1 / mass;
        this.drawing_radius = drawing_radius;
    }

    test_integrate(dt) {
        const acc = Vector2.scaleVector(this.force, this.wass);
        console.log("force: " + this.force.toString());

        this.pos.x = acc.x;
        this.pos.y = acc.y;

        this.force = new Vector2(0,0);
    }

    symplecticEuler(dt) {
        const acc = Vector2.scaleVector(this.force, this.wass);

        this.vel = Vector2.addVectors(this.vel, Vector2.scaleVector(acc, dt));
        this.pos = Vector2.addVectors(this.pos, Vector2.scaleVector(this.vel, dt));
        this.force = new Vector2(0,0);
    }

    RK4(dt) {
        const acc = Vector2.scaleVector(this.force, this.wass);

        // RK4 method
        const k1 = this.vel;
        const k1v = acc;

        const k2 = Vector2.addVectors(this.vel, Vector2.scaleVector(k1v, 0.5 * dt));
        const k2v = acc;

        const k3 = Vector2.addVectors(this.vel, Vector2.scaleVector(k2v, 0.5 * dt));
        const k3v = acc;

        const k4 = Vector2.addVectors(this.vel, Vector2.scaleVector(k3v, 0.5 * dt));
        const k4v = acc;

        const reciprocal_6 = 1 / 6;

        const sum_1 = Vector2.addVectors(k1, Vector2.scaleVector(k2, 2));
        const sum_2 = Vector2.addVectors(k4, Vector2.scaleVector(k3, 2));
        const sum_pos = Vector2.addVectors(sum_1, sum_2);
        this.pos = Vector2.addVectors(this.pos, Vector2.scaleVector(sum_pos, reciprocal_6 * dt));

        const sum_1_v = Vector2.addVectors(k1v, Vector2.scaleVector(k2v, 2));
        const sum_2_v = Vector2.addVectors(k4v, Vector2.scaleVector(k3v, 2));
        const sum_vel = Vector2.addVectors(sum_1_v, sum_2_v);
        this.vel = Vector2.addVectors(this.vel, Vector2.scaleVector(sum_vel, reciprocal_6 * dt));

        this.force = new Vector2(0,0);
    }

    calculateKineticEnergy() {
        return 0.5 * this.mass * this.vel.sqr_magnitude();
    }

}