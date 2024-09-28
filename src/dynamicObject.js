import {Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";
import {Colours, LineWidths, Extras} from "./render_settings.js";

export class DynamicObject {
    constructor(pos, mass, drawing_radius = 0.05) {
        // translational dynamics
        this.pos = pos;
        this.vel = new Vector2(0,0);
        this.force = new Vector2(0,0);
        this.m = mass;
        this.w = 1 / mass;

        // rotational dynamics

        // auxiliary stuff
        this.radius = drawing_radius;
    }

    symplecticEuler(dt) {
        const acc = Vector2.scaleVector(this.force, this.w);

        this.vel = Vector2.addVectors(this.vel, Vector2.scaleVector(acc, dt));
        this.pos = Vector2.addVectors(this.pos, Vector2.scaleVector(this.vel, dt));
        this.force = new Vector2(0,0);
    }

    calculateKineticEnergy() {
        return 0.5 * this.m * this.vel.sqr_magnitude();
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
    }

}