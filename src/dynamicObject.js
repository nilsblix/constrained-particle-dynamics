import {Vector2} from "./linear_algebra.js";
import {Units} from "./units.js";
import {Colours, LineWidths, Extras} from "./render_settings.js";

export class DynamicObject {
    constructor(pos, mass, drawing_radius = 0.05) {
        // translational dynamics
        this.pos = pos;
        this.vel = new Vector2(0,0);
        this.force = new Vector2(0,0);
        this.m = mass;
        this.w = 1 / mass;

        // auxiliary stuff
        this.radius = drawing_radius;

        // rotational dynamics
        this.theta = 0; // [rad] angular position
        // TEMPORARY: 10 should be 0
        this.omega = 0; // [rad / s] angular velocity
        this.tau = 0;   // [rad / s^2] force accumulator, acc = alpha
        this.I = 1 / 2 * this.m * this.radius ** 2; // [kg * m^2] moment of ineartia

    }

    symplecticEulerTranslation(dt) { // translational 
        const acc = Vector2.scaleVector(this.force, this.w);

        this.vel = Vector2.addVectors(this.vel, Vector2.scaleVector(acc, dt));
        this.pos = Vector2.addVectors(this.pos, Vector2.scaleVector(this.vel, dt));

        this.force = new Vector2(0,0);
    }
    
    symplecticEulerRotation(dt) {
        const alpha = this.tau / this.I;

        this.omega += alpha * dt;
        this.theta += this.omega * dt;

        this.tau = 0;
    }

    calculateKineticEnergy() {
        // both translational and rotational kinetic energy
        return 0.5 * this.m * this.vel.sqr_magnitude() + 0.5 * this.I * this.omega * this.omega;
    }

    render(c) {
        const canv_pos = Units.sim_canv(this.pos);
        const radius = this.radius * Units.scale_s_c;

        c.fillStyle = Colours.INNER_DYNAMIC_OBJECT;
        c.strokeStyle = Colours.OUTER_DYNAMIC_OBJECT; 
        c.lineWidth = LineWidths.DYNAMIC_OBJECT;
        
        c.beginPath();
        c.arc(canv_pos.x, canv_pos.y, radius, 0, 2 * Math.PI);
        c.fill();
        c.stroke();
        c.closePath();

        // draw the small dot visualising rotation
        const rotation_radius = Extras.DYNAMIC_OBJECT_ROTATIONAL_RADIUS_MULT * this.radius;
        const m = Extras.DYNAMIC_OBJECT_ROTATIONAL_POSITION_MULT;
        const offset = Vector2.scaleVector(new Vector2(Math.cos(this.theta), Math.sin(this.theta)), m * this.radius);
        const rotation_pos = Vector2.addVectors(this.pos, offset);
        c.fillStyle = Colours.OUTER_DYNAMIC_OBJECT;
        c.beginPath();
        c.arc(Units.sim_canv_x(rotation_pos), Units.sim_canv_y(rotation_pos), Units.scale_s_c * rotation_radius, 0, 2 * Math.PI);
        c.fill();
        c.closePath();
    }


}