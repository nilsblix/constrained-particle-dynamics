import {Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";
import {Colours, LineWidths, Extras} from "./render_settings.js";

export class DynamicObject {
    constructor(pos, mass, drawing_radius = 0.05) {
        this.pos = pos;
        this.vel = new Vector2(0,0);
        this.force = new Vector2(0,0);
        this.prev_force = new Vector2(0,0);
        this.mass = mass;
        this.wass = 1 / mass;
        this.drawing_radius = drawing_radius;
    }

    backwardEuler(dt) {
        const acc = Vector2.scaleVector(this.force, this.wass);

        const delta_x = Vector2.scaleVector(this.vel, dt);
        this.pos = Vector2.addVectors(this.pos, delta_x);

        const delta_v = Vector2.scaleVector(acc, dt);
        this.vel = Vector2.addVectors(this.vel, delta_v);

        this.force = new Vector2(0,0);
    }

    velocityVerlet(dt) {
        const curr_acc = Vector2.scaleVector(this.force, this.wass);
        const prev_acc = Vector2.scaleVector(this.prev_force, this.wass);

        const delta_x_1 = Vector2.scaleVector(this.vel, dt);
        const delta_x_2 = Vector2.scaleVector(curr_acc, 0.5 * dt * dt);
        const delta_x = Vector2.addVectors(delta_x_1, delta_x_2);

        this.pos = Vector2.addVectors(this.pos, delta_x);

        const delta_v_1 = Vector2.scaleVector(curr_acc, 0.5 * dt);
        const delta_v_2 = Vector2.scaleVector(prev_acc, 0.5 * dt);
        const delta_v = Vector2.addVectors(delta_v_1, delta_v_2);

        this.vel = Vector2.addVectors(this.vel, delta_v);

        this.prev_force = this.force;
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

    render(c) {
        const canv_pos = Units.sim_canv(this.pos);
        const radius = this.drawing_radius * Units.scale_s_c;

        c.fillStyle = Colours.INNER_DYNAMIC_OBJECT;
        // c.fillStyle = "#FFFFFF";
        // c.fillStyle = "#e3a88a";
        c.strokeStyle = Colours.OUTER_DYNAMIC_OBJECT; // do all right of this file
        c.lineWidth = LineWidths.DYNAMIC_OBJECT;
        
        c.beginPath();
        c.arc(canv_pos.x, canv_pos.y, radius, 0, 2 * Math.PI);
        c.fill();
        c.stroke();
        c.closePath();
    }

}