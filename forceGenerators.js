import {Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";

/* 
    When adding a new force generator, these functions must be in them and spelled correctly
        --> apply (applies a force to all objects)
        --> getEnergyApplied (calculates the work done against the force applied)
        --> render(c) (unusual)
*/

export class Gravity {
    constructor() {
        this.gravity = 1.4;
    }

    apply(m_objects) {
        for (let i = 0; i < m_objects.length; i++) {
            // m_objects[i].force = Vector2.addVectors(m_objects[i].force, new Vector2(0, - this.g_GRAVITY));
            m_objects[i].force = Vector2.addVectors(m_objects[i].force, Vector2.scaleVector(new Vector2(0, -this.gravity), m_objects[i].mass));
        }
    }

    getEnergyApplied(m_objects) {
        let energy = 0;

        for (let i = 0; i < m_objects.length; i++) {
            energy += m_objects[i].mass * m_objects[i].pos.y * this.gravity;
        }
        return energy;
    }

    render(c) {

    }

}

export class LinearDamping {
    constructor() {
        this.MU = 9e-2;
    }

    apply(m_objects) {
        for (let i = 0; i < m_objects.length; i++) {
            m_objects[i].force = Vector2.subtractVectors(m_objects[i].force, Vector2.scaleVector(m_objects[i].vel, this.MU));
        }
    }

    getEnergyApplied(m_objects) {
        return 0;
    }

    render(c) {

    }

}

export class MouseSpring {
    #rest_length = 0;
    #stiffness_const = 6;

    constructor(id) {
        this.particle_id = id;
        this.mouse_pos = new Vector2(0,0);
        this.active = false;
    }

    apply(m_objects) {
        const obj = m_objects[this.particle_id];

        const dist = Vector2.distance(this.mouse_pos, obj.pos);
        const displacement = dist - this.#rest_length;
        const force = 2 * this.#stiffness_const * displacement;
        let dir = Vector2.subtractVectors(this.mouse_pos, obj.pos);

        if (dist == 0) 
            dir = Vector2.scaleVector(dir, 1e-4);
        else
            dir = Vector2.scaleVector(dir, 1 / dist);

        const total_force = Vector2.scaleVector(dir, force);
        m_objects[this.particle_id].force = Vector2.addVectors(m_objects[this.particle_id].force, total_force);
    }

    getClosestDynamicObject(m_objects) {
        for (let i = 0; i < m_objects.length; i++) {
            const v = Vector2.subtractVectors(m_objects[i].pos, this.mouse_pos);
            const dist2 = v.x * v.x + v.y * v.y;
            const rad = m_objects[i].drawing_radius;
            if (dist2 < rad * rad) {
                this.particle_id = i;
                return true;
            }
        }
        return false;
    }

    render(c, m_objects) {
        const num_segments = 25;
        const width = 0.2;

        const draw_thickness = 10;
        const border_width = 2;

        const obj = m_objects[this.particle_id];

        let segments = [];

        const dist = Vector2.distance(this.mouse_pos, obj.pos);

        const dir = Vector2.scaleVector(Vector2.subtractVectors(obj.pos, this.mouse_pos), 1 / dist);
        const dirT = new Vector2(-dir.y, dir.x);

        const seg_length = dist / num_segments;
        for (let i = 0; i < num_segments; i++) {
            const length_along_dir = seg_length * i;
            const length_along_dirT = width * ((i % 2)  - 0.5);
            const A = Vector2.addVectors(this.mouse_pos, Vector2.scaleVector(dir, length_along_dir));
            const B = Vector2.scaleVector(dirT, length_along_dirT);
            segments.push(Vector2.addVectors(A, B));
        }

        for (let i = 0; i < num_segments; i++) {
            const start = Units.sim_canv(segments[i]);
            const end = i < num_segments - 1 ? Units.sim_canv(segments[i + 1]) : Units.sim_canv(obj.pos);

            c.beginPath();

            // border
            c.lineWidth = draw_thickness;
            c.strokeStyle = "#000000";
            c.moveTo(start.x, start.y);
            c.lineTo(end.x, end.y);
            c.stroke();

            // interiour
            c.lineWidth = draw_thickness - border_width;
            c.strokeStyle = "#FFFFFF";
            c.moveTo(start.x, start.y);
            c.lineTo(end.x, end.y);
            c.stroke();

            c.closePath();
        }

        c.fillStyle = "rgba(156, 58, 58, 1)";
        c.strokeStyle = "#000000";
        c.lineWidth = 2;
        
        const mouse_canvas_pos = Units.sim_canv(this.mouse_pos);
        const radius = 20;

        c.beginPath();
        c.arc(mouse_canvas_pos.x, mouse_canvas_pos.y, radius, 0, 2 * Math.PI);
        c.fill();
        c.stroke();
        c.closePath();

    }
}