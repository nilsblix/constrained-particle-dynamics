import {Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";
import {Colours, LineWidths, Extras} from "./render_settings.js";

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

export class Wind {
    constructor() {
        this.wind = 1;
    }

    apply(m_objects) {
        for (let i = 0; i < m_objects.length; i++) {
            m_objects[i].force = Vector2.addVectors(m_objects[i].force, Vector2.scaleVector(new Vector2(this.wind, 0), m_objects[i].mass));

        }
    }

    getEnergyApplied(m_objects) {
        let energy = 0;

        for (let i = 0; i < m_objects.length; i++) {
            if (this.wind > 0)
                energy += m_objects[i].mass * (Units.WIDTH - m_objects[i].pos.x) * this.wind;
            else 
                energy += m_objects[i].mass * m_objects[i].pos.x * this.wind;   
        }
        return energy;
    }

    render(c) {

    }

}

export class LinearDamping {
    constructor() {
        this.MU = 3e-1;
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

export class SpringJoint {
    #stiffness_const = 5;

    constructor(id1, id2, m_objects) {
        this.id1 = id1;
        this.id2 = id2;
        // this.rest_length = Vector2.distance(m_objects[id1].pos, m_objects[id2].pos);
        this.rest_length = 0.5;
    }

    setStiffness(value) {
        this.#stiffness_const = value;
    }

    apply(m_objects) {
        const obj1 = m_objects[this.id1];
        const obj2 = m_objects[this.id2];

        const dist = Vector2.distance(obj1.pos, obj2.pos);
        const displacement = dist - this.rest_length;
        const force = this.#stiffness_const * displacement;
        let dir = Vector2.subtractVectors(obj1.pos, obj2.pos);
        if (dir == 0) 
            dir = Vector2.scaleVector(dir, 1e-4);
        else {
            dir = Vector2.scaleVector(dir, 1 / dist);
        }

        const total_force = Vector2.scaleVector(dir, force);
        obj1.force = Vector2.subtractVectors(obj1.force, total_force);
        obj2.force = Vector2.addVectors(obj2.force, total_force);
    }

    getEnergyApplied(m_objects) {
        const obj1 = m_objects[this.id1];
        const obj2 = m_objects[this.id2];

        const dist = Vector2.distance(obj1.pos, obj2.pos);
        const displacement = dist - this.rest_length;

        return 1 / 2 * this.#stiffness_const * displacement * displacement;
    }

    render(c, m_objects) {
        const obj1 = m_objects[this.id1];
        const obj2 = m_objects[this.id2];

        const num_segments = Extras.SPRINGJOINT_NUM_SEGMENTS;
        const width = 0.2;

        const outer_holding_circle_width_1 = obj1.drawing_radius * 1;
        const outer_holding_circle_width_2 = obj2.drawing_radius * 1;

        let segments = [];

        const dist = Vector2.distance(obj1.pos, obj2.pos);

        const dir = Vector2.scaleVector(Vector2.subtractVectors(obj1.pos, obj2.pos), 1 / dist);
        const dirT = new Vector2(-dir.y, dir.x);

        const spring_end_delta = Vector2.scaleVector(dir.negated(), outer_holding_circle_width_1);
        const spring_end = Vector2.addVectors(obj1.pos, spring_end_delta);
        const spring_start_delta = Vector2.scaleVector(dir, outer_holding_circle_width_2);
        const spring_start = Vector2.addVectors(obj2.pos, spring_start_delta);

        const spring_length = Vector2.distance(spring_start, spring_end);

        const seg_length = spring_length / num_segments;
        for (let i = 0; i < num_segments; i++) {
            const length_along_dir = seg_length * i;
            const length_along_dirT = width * ((i % 2)  - 0.5);
            const A = Vector2.addVectors(spring_start, Vector2.scaleVector(dir, length_along_dir));
            const B = Vector2.scaleVector(dirT, length_along_dirT);
            segments.push(Vector2.addVectors(A, B));
        }

        // connection to object:
        // constants:
        const connection_obj1_radius = Math.sqrt(2 * obj1.drawing_radius * Units.scale_s_c);
        const connection_obj1_delta = Vector2.scaleVector(dirT, Units.scale_c_s * connection_obj1_radius);
        const connection_obj1_pos_1 = Units.sim_canv(Vector2.addVectors(spring_end, connection_obj1_delta));
        const connection_obj1_pos_2 = Units.sim_canv(Vector2.addVectors(spring_end, connection_obj1_delta.negated()));
        const connection_obj1_pos_3 = Units.sim_canv(Vector2.addVectors(obj1.pos, connection_obj1_delta));
        const connection_obj1_pos_4 = Units.sim_canv(Vector2.addVectors(obj1.pos, connection_obj1_delta.negated()));
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        // draw rectangle:
        c.beginPath();

        c.arc(Units.sim_canv_x(obj1.pos), Units.sim_canv_y(obj1.pos), connection_obj1_radius, 0, 2 * Math.PI);

        c.moveTo(connection_obj1_pos_1.x, connection_obj1_pos_1.y);
        c.lineTo(connection_obj1_pos_3.x, connection_obj1_pos_3.y);
        c.lineTo(connection_obj1_pos_4.x, connection_obj1_pos_4.y);
        c.lineTo(connection_obj1_pos_2.x, connection_obj1_pos_2.y);
        
        c.stroke();
        c.fill();
        c.closePath();
        // draw circle at connection to object
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        
        c.beginPath();
        c.fillStyle = c.strokeStyle;
        c.arc(Units.sim_canv_x(obj1.pos), Units.sim_canv_y(obj1.pos), 0.3 * connection_obj1_radius, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        // connection to mouse:
        // constants:
        const connection_obj2_radius = Math.sqrt(2 * obj2.drawing_radius * Units.scale_s_c);
        const connection_obj2_delta = Vector2.scaleVector(dirT, Units.scale_c_s * connection_obj2_radius);
        const connection_obj2_pos_1 = Units.sim_canv(Vector2.addVectors(spring_start, connection_obj2_delta));
        const connection_obj2_pos_2 = Units.sim_canv(Vector2.addVectors(spring_start, connection_obj2_delta.negated()));
        const connection_obj2_pos_3 = Units.sim_canv(Vector2.addVectors(obj2.pos, connection_obj2_delta));
        const connection_obj2_pos_4 = Units.sim_canv(Vector2.addVectors(obj2.pos, connection_obj2_delta.negated()));
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        // draw rectangle:
        c.beginPath();

        c.arc(Units.sim_canv_x(obj2.pos), Units.sim_canv_y(obj2.pos), connection_obj2_radius, 0, 2 * Math.PI);

        c.moveTo(connection_obj2_pos_2.x, connection_obj2_pos_2.y);
        c.lineTo(connection_obj2_pos_4.x, connection_obj2_pos_4.y);
        c.lineTo(connection_obj2_pos_3.x, connection_obj2_pos_3.y);
        c.lineTo(connection_obj2_pos_1.x, connection_obj2_pos_1.y);

        c.stroke();
        c.fill();
        c.closePath();
        // draw circle at connection to object
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        
        c.beginPath();
        c.fillStyle = c.strokeStyle;
        c.arc(Units.sim_canv_x(obj2.pos), Units.sim_canv_y(obj2.pos), 0.3 * connection_obj2_radius, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        for (let i = 0; i < num_segments; i++) {
            const start = Units.sim_canv(segments[i]);
            const end = i < num_segments - 1 ? Units.sim_canv(segments[i + 1]) : Units.sim_canv(Vector2.addVectors(spring_end, Vector2.scaleVector(dirT, - 0.5* width)));

            c.beginPath();

            // border
            c.lineWidth = LineWidths.SPRING_SEGMENTS + LineWidths.SPRING_SEGMENTS_BORDER;
            c.strokeStyle = Colours.OUTER_SPRING_SEGMENTS;
            c.lineCap = Extras.SPRING_SEGMENTS_ENDCAPS;
            c.moveTo(start.x, start.y);
            c.lineTo(end.x, end.y);
            c.stroke();

            // interiour
            c.lineWidth = LineWidths.SPRING_SEGMENTS;
            c.strokeStyle = Colours.INNER_SPRING_SEGMENTS;
            c.moveTo(start.x, start.y);
            c.lineTo(end.x, end.y);
            c.stroke();

            c.closePath();
        }

        // endline:
        c.lineCap = Extras.SPRING_EXTREME_ENDCAPS;
        // border 1:
        const border1_start = Units.sim_canv(Vector2.addVectors(spring_start, Vector2.scaleVector(dirT, 0.5* width)));
        const border1_end = Units.sim_canv(Vector2.addVectors(spring_start, Vector2.scaleVector(dirT, - 0.5* width)));

        c.beginPath();

        c.lineWidth = LineWidths.SPRING_EXTREME_SEGMENTS + LineWidths.SPRING_EXTREME_SEGMENTS_BORDER;
        c.strokeStyle = Colours.OUTER_SPRING_SEGMENTS;
        c.moveTo(border1_start.x, border1_start.y);
        c.lineTo(border1_end.x, border1_end.y);
        c.stroke();

        // interiour
        c.lineWidth = LineWidths.SPRING_EXTREME_SEGMENTS;
        c.strokeStyle = Colours.INNER_SPRING_SEGMENTS;
        c.moveTo(border1_start.x, border1_start.y);
        c.lineTo(border1_end.x, border1_end.y);
        c.stroke();

        c.closePath();

        c.fillStyle = "rgba(156, 58, 58, 1)";
        c.strokeStyle = "#000000";
        c.lineWidth = 2;

        // border 2
        const border2_start = Units.sim_canv(Vector2.addVectors(spring_end, Vector2.scaleVector(dirT, 0.5* width)));
        const border2_end = Units.sim_canv(Vector2.addVectors(spring_end, Vector2.scaleVector(dirT, - 0.5* width)));

        c.beginPath();

        c.lineWidth = LineWidths.SPRING_EXTREME_SEGMENTS + LineWidths.SPRING_EXTREME_SEGMENTS_BORDER;
        c.strokeStyle = Colours.OUTER_SPRING_SEGMENTS;
        c.moveTo(border2_start.x, border2_start.y);
        c.lineTo(border2_end.x, border2_end.y);
        c.stroke();

        // interiour
        c.lineWidth = LineWidths.SPRING_EXTREME_SEGMENTS;
        c.strokeStyle = Colours.INNER_SPRING_SEGMENTS;
        c.moveTo(border2_start.x, border2_start.y);
        c.lineTo(border2_end.x, border2_end.y);
        c.stroke();

        c.closePath();
    }
}

export class MouseSpring {
    #rest_length = 0;
    #stiffness_const = 10;

    constructor(id) {
        this.particle_id = id;
        this.mouse_pos = new Vector2(0,0);
        this.active = false;
    }

    setStiffness(value) {
        this.#stiffness_const = value;
    }

    apply(m_objects) {
        const obj = m_objects[this.particle_id];
        this.#rest_length = obj.drawing_radius;

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
            const rad = m_objects[i].drawing_radius + 0.08;
            if (dist2 < rad * rad) {
                this.particle_id = i;
                return true;
            }
        }
        return false;
    }

    getEnergyApplied(m_objects) {
        if (!this.active)
            return 0;

        let energy = 0;

        const obj = m_objects[this.particle_id];

        const dist = Vector2.distance(this.mouse_pos, obj.pos);
        const displacement = dist - this.#rest_length;

        energy += 0.5 * obj.mass * displacement * displacement;
        return energy;
    }

    render(c, m_objects) {
        const obj = m_objects[this.particle_id];

        const num_segments = Extras.MOUSESPRING_NUM_SEGMENTS;
        const width = 0.2;

        const outer_holding_circle_width = obj.drawing_radius * 1;
        const mouse_radius = 0.1;

        let segments = [];

        const dist = Vector2.distance(this.mouse_pos, obj.pos);

        const dir = Vector2.scaleVector(Vector2.subtractVectors(obj.pos, this.mouse_pos), 1 / dist);
        const dirT = new Vector2(-dir.y, dir.x);

        const spring_end_delta = Vector2.scaleVector(dir.negated(), outer_holding_circle_width);
        const spring_end = Vector2.addVectors(obj.pos, spring_end_delta);
        const spring_start_delta = Vector2.scaleVector(dir, mouse_radius);
        const spring_start = Vector2.addVectors(this.mouse_pos, spring_start_delta);

        const spring_length = Vector2.distance(spring_start, spring_end);

        const seg_length = spring_length / num_segments;
        for (let i = 0; i < num_segments; i++) {
            const length_along_dir = seg_length * i;
            const length_along_dirT = width * ((i % 2)  - 0.5);
            const A = Vector2.addVectors(spring_start, Vector2.scaleVector(dir, length_along_dir));
            const B = Vector2.scaleVector(dirT, length_along_dirT);
            segments.push(Vector2.addVectors(A, B));
        }

        // connection to object:
        // constants:
        const connection_obj_radius = Math.sqrt(2 * obj.drawing_radius * Units.scale_s_c);
        const connection_obj_delta = Vector2.scaleVector(dirT, Units.scale_c_s * connection_obj_radius);
        const connection_obj_pos_1 = Units.sim_canv(Vector2.addVectors(spring_end, connection_obj_delta));
        const connection_obj_pos_2 = Units.sim_canv(Vector2.addVectors(spring_end, connection_obj_delta.negated()));
        const connection_obj_pos_3 = Units.sim_canv(Vector2.addVectors(obj.pos, connection_obj_delta));
        const connection_obj_pos_4 = Units.sim_canv(Vector2.addVectors(obj.pos, connection_obj_delta.negated()));
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        // draw rectangle:
        c.beginPath();

        c.arc(Units.sim_canv_x(obj.pos), Units.sim_canv_y(obj.pos), connection_obj_radius, 0, 2 * Math.PI);

        c.moveTo(connection_obj_pos_1.x, connection_obj_pos_1.y);
        c.lineTo(connection_obj_pos_3.x, connection_obj_pos_3.y);
        c.lineTo(connection_obj_pos_4.x, connection_obj_pos_4.y);
        c.lineTo(connection_obj_pos_2.x, connection_obj_pos_2.y);
        
        c.stroke();
        c.fill();
        c.closePath();
        // draw circle at connection to object
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        
        c.beginPath();
        c.fillStyle = c.strokeStyle;
        c.arc(Units.sim_canv_x(obj.pos), Units.sim_canv_y(obj.pos), 0.3 * connection_obj_radius, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        // mouse circle:
        // constants
        const mouse_canvas_pos = Units.sim_canv(this.mouse_pos);
        // init settings
        c.fillStyle = Colours.INNER_MOUSE_SPRING_MOUSE_CIRCLE;
        c.strokeStyle = Colours.OUTER_MOUSE_SPRING_MOUSE_CIRCLE;
        c.lineWidth = LineWidths.MOUSE_SPRING_MOUSE_CIRCLE;
        // draw circle at mouse
        c.beginPath();
        c.arc(mouse_canvas_pos.x, mouse_canvas_pos.y, Units.scale_s_c * mouse_radius, 0, 2 * Math.PI);
        c.fill();
        c.stroke();
        c.closePath();

        // connection to mouse:
        // constants:
        const connection_mouse_radius = Math.sqrt(2 * obj.drawing_radius * Units.scale_s_c);
        const connection_mouse_delta = Vector2.scaleVector(dirT, Units.scale_c_s * connection_mouse_radius);
        const connection_mouse_pos_1 = Units.sim_canv(Vector2.addVectors(spring_start, connection_mouse_delta));
        const connection_mouse_pos_2 = Units.sim_canv(Vector2.addVectors(spring_start, connection_mouse_delta.negated()));
        const connection_mouse_pos_3 = Units.sim_canv(Vector2.addVectors(this.mouse_pos, connection_mouse_delta));
        const connection_mouse_pos_4 = Units.sim_canv(Vector2.addVectors(this.mouse_pos, connection_mouse_delta.negated()));
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        // draw rectangle:
        c.beginPath();

        c.arc(mouse_canvas_pos.x, mouse_canvas_pos.y, connection_mouse_radius, 0, 2 * Math.PI);

        c.moveTo(connection_mouse_pos_2.x, connection_mouse_pos_2.y);
        c.lineTo(connection_mouse_pos_4.x, connection_mouse_pos_4.y);
        c.lineTo(connection_mouse_pos_3.x, connection_mouse_pos_3.y);
        c.lineTo(connection_mouse_pos_1.x, connection_mouse_pos_1.y);

        c.stroke();
        c.fill();
        c.closePath();
        // draw circle at connection to object
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        
        c.beginPath();
        c.fillStyle = c.strokeStyle;
        c.arc(mouse_canvas_pos.x, mouse_canvas_pos.y, 0.3 * connection_mouse_radius, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        for (let i = 0; i < num_segments; i++) {
            const start = Units.sim_canv(segments[i]);
            const end = i < num_segments - 1 ? Units.sim_canv(segments[i + 1]) : Units.sim_canv(Vector2.addVectors(spring_end, Vector2.scaleVector(dirT, - 0.5* width)));

            c.beginPath();

            // border
            c.lineWidth = LineWidths.SPRING_SEGMENTS + LineWidths.SPRING_SEGMENTS_BORDER;
            c.strokeStyle = Colours.OUTER_SPRING_SEGMENTS;
            c.lineCap = Extras.SPRING_SEGMENTS_ENDCAPS;
            c.moveTo(start.x, start.y);
            c.lineTo(end.x, end.y);
            c.stroke();

            // interiour
            c.lineWidth = LineWidths.SPRING_SEGMENTS;
            c.strokeStyle = Colours.INNER_SPRING_SEGMENTS;
            c.moveTo(start.x, start.y);
            c.lineTo(end.x, end.y);
            c.stroke();

            c.closePath();
        }

        // endline:
        c.lineCap = Extras.SPRING_EXTREME_ENDCAPS;
        // border 1:
        const border1_start = Units.sim_canv(Vector2.addVectors(spring_start, Vector2.scaleVector(dirT, 0.5* width)));
        const border1_end = Units.sim_canv(Vector2.addVectors(spring_start, Vector2.scaleVector(dirT, - 0.5* width)));

        c.beginPath();

        c.lineWidth = LineWidths.SPRING_EXTREME_SEGMENTS + LineWidths.SPRING_EXTREME_SEGMENTS_BORDER;
        c.strokeStyle = Colours.OUTER_SPRING_SEGMENTS;
        c.moveTo(border1_start.x, border1_start.y);
        c.lineTo(border1_end.x, border1_end.y);
        c.stroke();

        // interiour
        c.lineWidth = LineWidths.SPRING_EXTREME_SEGMENTS;
        c.strokeStyle = Colours.INNER_SPRING_SEGMENTS;
        c.moveTo(border1_start.x, border1_start.y);
        c.lineTo(border1_end.x, border1_end.y);
        c.stroke();

        c.closePath();

        c.fillStyle = "rgba(156, 58, 58, 1)";
        c.strokeStyle = "#000000";
        c.lineWidth = 2;

        // border 2
        const border2_start = Units.sim_canv(Vector2.addVectors(spring_end, Vector2.scaleVector(dirT, 0.5* width)));
        const border2_end = Units.sim_canv(Vector2.addVectors(spring_end, Vector2.scaleVector(dirT, - 0.5* width)));

        c.beginPath();

        c.lineWidth = LineWidths.SPRING_EXTREME_SEGMENTS + LineWidths.SPRING_EXTREME_SEGMENTS_BORDER;
        c.strokeStyle = Colours.OUTER_SPRING_SEGMENTS;
        c.moveTo(border2_start.x, border2_start.y);
        c.lineTo(border2_end.x, border2_end.y);
        c.stroke();

        // interiour
        c.lineWidth = LineWidths.SPRING_EXTREME_SEGMENTS;
        c.strokeStyle = Colours.INNER_SPRING_SEGMENTS;
        c.moveTo(border2_start.x, border2_start.y);
        c.lineTo(border2_end.x, border2_end.y);
        c.stroke();

        c.closePath();

    }
}