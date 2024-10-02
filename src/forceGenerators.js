import { LinkConstraint } from "./core_constraints.js";
import {Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";
import {Colours, LineWidths, Extras} from "./render_settings.js";
import { DynamicObject } from "./dynamicObject.js";

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
            m_objects[i].force = Vector2.addVectors(m_objects[i].force, Vector2.scaleVector(new Vector2(0, -this.gravity), m_objects[i].m));
        }
    }

    getEnergyApplied(m_objects) {
        let energy = 0;

        for (let i = 0; i < m_objects.length; i++) {
            energy += m_objects[i].m * m_objects[i].pos.y * this.gravity;
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
            m_objects[i].force = Vector2.addVectors(m_objects[i].force, Vector2.scaleVector(new Vector2(this.wind, 0), m_objects[i].m));

        }
    }

    getEnergyApplied(m_objects) {
        let energy = 0;

        for (let i = 0; i < m_objects.length; i++) {
            if (this.wind > 0)
                energy += m_objects[i].m * (Units.WIDTH - m_objects[i].pos.x) * this.wind;
            else 
                energy += m_objects[i].m * m_objects[i].pos.x * this.wind;   
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
            m_objects[i].tau -= m_objects[i].omega * 0.1 * this.MU;
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
    
    // t has to be from index 2 to index 1
    constructor(state_1, state_2, m_objects) { // state = {Entity entity, Vector2 offset, float prev_theta, float t_param, Vector2 applied_pos}
        this.state_1 = {    entity: state_1.entity, 
                            offset: state_1.offset, 
                            prev_theta: state_1.prev_theta, 
                            t_param: state_1.t_param, 
                            applied_pos: state_1.applied_pos};

        this.state_2 = {    entity: state_2.entity, 
                            offset: state_2.offset, 
                            prev_theta: state_1.prev_theta, 
                            t_param: state_2.t_param, 
                            applied_pos: state_2.applied_pos};

        // this.rest_length = Vector2.distance(state_1.applied_pos, state_2.applied_pos);
        this.rest_length = 0;
    }

    setStiffness(value) {
        this.#stiffness_const = value;
    }

    #getLinkConstraintAppliedPos(state, m_objects) { // assumes that entity is of a LinkConstraint
        const A = m_objects[state.entity.id1];
        const B = m_objects[state.entity.id2];
        const A_B = Vector2.subtractVectors(A.pos, B.pos);
        const proj = Vector2.addVectors(B.pos, Vector2.scaleVector(A_B, state.t_param));
        return proj;
    }

    #updateDynamicObjectRotationOffset(state) {
        const offset_magnitude = state.offset.magnitude();
        const offset_1_angle = Math.atan2(state.offset.y, state.offset.x);
        const delta_1 = state.entity.theta - state.prev_theta;
        state.offset.x = Math.cos(offset_1_angle + delta_1) * offset_magnitude;
        state.offset.y = Math.sin(offset_1_angle + delta_1) * offset_magnitude;

        state.prev_theta = state.entity.theta;
    }

    #apply_to_dynamicObject(state, other_applied_pos, force) { // this assumes that entity is of a DynamicObject
        const obj = state.entity;
    
        this.#updateDynamicObjectRotationOffset(state);
        state.applied_pos = Vector2.addVectors(obj.pos, state.offset);

        const translational_dir = Vector2.subtractVectors(other_applied_pos, obj.pos).normalized();
        const applied_dir = Vector2.subtractVectors(other_applied_pos, obj.pos);

        const linear_force_vec = Vector2.scaleVector(translational_dir, force);
        obj.force = Vector2.addVectors(obj.force, linear_force_vec);

        const applied_force_vec = Vector2.scaleVector(applied_dir, force);
        obj.tau += Vector2.cross(state.offset, applied_force_vec);
    }   

    #apply_to_linkConstraint(state, m_objects, other_applied_pos, force) {
        const obj1 = m_objects[state.entity.id1];
        const obj2 = m_objects[state.entity.id2];

        state.applied_pos = this.#getLinkConstraintAppliedPos(state, m_objects);

        const v = Vector2.subtractVectors(other_applied_pos, state.applied_pos).normalized();
        const linear_force_vec = Vector2.scaleVector(v, force);

        // translational forces
        obj1.force = Vector2.addVectors(obj1.force, linear_force_vec);
        obj2.force = Vector2.addVectors(obj2.force, linear_force_vec);

        // rotational forces (torque)
        const A_B = Vector2.subtractVectors(obj1.pos, obj2.pos);
        const COM = Vector2.addVectors(obj2.pos, Vector2.scaleVector(A_B, obj1.m / (obj1.m + obj2.m)));
        const torque = Vector2.cross(Vector2.subtractVectors(state.applied_pos, COM), linear_force_vec);

        const v1 = Vector2.subtractVectors(obj1.pos, COM).normalized();
        const torque_dir_1 = new Vector2(- v1.y, v1.x);
        obj1.force = Vector2.addVectors(obj1.force, Vector2.scaleVector(torque_dir_1, torque));

        const v2 = Vector2.subtractVectors(obj2.pos, COM).normalized();
        const torque_dir_2 = new Vector2(- v2.y, v2.x);
        obj2.force = Vector2.addVectors(obj2.force, Vector2.scaleVector(torque_dir_2, torque));

    }

    apply(m_objects) {
        const dist = Vector2.distance(this.state_1.applied_pos, this.state_2.applied_pos);
        const displacement = dist - this.rest_length;
        const force = this.#stiffness_const * displacement;

        if (this.state_1.entity instanceof DynamicObject) {
            this.#apply_to_dynamicObject(this.state_1, this.state_2.applied_pos, force);
        } else if (this.state_1.entity instanceof LinkConstraint) {
            this.#apply_to_linkConstraint(this.state_1, m_objects, this.state_2.applied_pos, force);
        }

        if (this.state_2.entity instanceof DynamicObject) {
            this.#apply_to_dynamicObject(this.state_2, this.state_1.applied_pos, force);
        } else if (this.state_2.entity instanceof LinkConstraint) {
            this.#apply_to_linkConstraint(this.state_2, m_objects, this.state_1.applied_pos, force);
        }

    }

    getEnergyApplied(m_objects) {
        const obj1 = m_objects[this.id1];
        const obj2 = m_objects[this.id2];

        let energy = 0;

        const applied_pos_1 = this.state_1.applied_pos;
        const applied_pos_2 = this.state_2.applied_pos;

        // simple spring hookes law
        const dist = Vector2.distance(applied_pos_1, applied_pos_2);
        const displacement = dist - this.rest_length;
        const force = this.#stiffness_const * displacement;
        energy += 0.5 * force * displacement;

        return energy;
        
    }

    render(c, m_objects) {
        const num_segments = Extras.SPRINGJOINT_NUM_SEGMENTS;
        const width = 0.2;

        const outer_holding_circle_width_1 = 2 * Extras.SPRING_JOINT_ENDS_RADIUS;
        const outer_holding_circle_width_2 = 2 * Extras.SPRING_JOINT_ENDS_RADIUS;

        const pos_1 = this.state_1.applied_pos;
        const pos_2 = this.state_2.applied_pos;

        let segments = [];

        const dist = Vector2.distance(pos_1, pos_2);

        const dir = Vector2.scaleVector(Vector2.subtractVectors(pos_1, pos_2), 1 / dist);
        const dirT = new Vector2(-dir.y, dir.x);

        const spring_end_delta = Vector2.scaleVector(dir.negated(), outer_holding_circle_width_1);
        const spring_end = Vector2.addVectors(pos_1, spring_end_delta);
        const spring_start_delta = Vector2.scaleVector(dir, outer_holding_circle_width_2);
        const spring_start = Vector2.addVectors(pos_2, spring_start_delta);

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
        const connection_obj1_radius = Extras.SPRING_JOINT_ENDS_RADIUS;
        const connection_obj1_delta = Vector2.scaleVector(dirT, connection_obj1_radius);
        const connection_obj1_pos_1 = Units.sim_canv(Vector2.addVectors(spring_end, connection_obj1_delta));
        const connection_obj1_pos_2 = Units.sim_canv(Vector2.addVectors(spring_end, connection_obj1_delta.negated()));
        const connection_obj1_pos_3 = Units.sim_canv(Vector2.addVectors(pos_1, connection_obj1_delta));
        const connection_obj1_pos_4 = Units.sim_canv(Vector2.addVectors(pos_1, connection_obj1_delta.negated()));
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        // draw rectangle:
        c.beginPath();

        c.arc(Units.sim_canv_x(pos_1), Units.sim_canv_y(pos_1), Units.scale_s_c * connection_obj1_radius, 0, 2 * Math.PI);

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
        c.arc(Units.sim_canv_x(pos_1), Units.sim_canv_y(pos_1), 0.3 * connection_obj1_radius, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        // connection to mouse:
        // constants:
        const connection_obj2_radius = Extras.SPRING_JOINT_ENDS_RADIUS
        const connection_obj2_delta = Vector2.scaleVector(dirT, connection_obj2_radius);
        const connection_obj2_pos_1 = Units.sim_canv(Vector2.addVectors(spring_start, connection_obj2_delta));
        const connection_obj2_pos_2 = Units.sim_canv(Vector2.addVectors(spring_start, connection_obj2_delta.negated()));
        const connection_obj2_pos_3 = Units.sim_canv(Vector2.addVectors(pos_2, connection_obj2_delta));
        const connection_obj2_pos_4 = Units.sim_canv(Vector2.addVectors(pos_2, connection_obj2_delta.negated()));
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        // draw rectangle:
        c.beginPath();

        c.arc(Units.sim_canv_x(pos_2), Units.sim_canv_y(pos_2), Units.scale_s_c * connection_obj2_radius, 0, 2 * Math.PI);

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
        c.arc(Units.sim_canv_x(pos_2), Units.sim_canv_y(pos_2), 0.3 * connection_obj2_radius, 0, 2 * Math.PI);
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
    #prev_theta = 0;

    constructor() {
        this.object = null;
        this.t_linkConstraint = null;
        this.mouse_pos = new Vector2(0,0);
        this.active = false;
        this.offset = new Vector2(0,0);
    }

    setStiffness(value) {
        this.#stiffness_const = value;
    }

    #apply_to_dynamicObject(m_objects) {
        this.#rest_length = this.object.radius;

        const offset_magnitude = this.offset.magnitude();
        const offset_angle = Math.atan2(this.offset.y, this.offset.x);
        const delta = this.object.theta - this.#prev_theta;
        this.offset.x = Math.cos(offset_angle + delta) * offset_magnitude;
        this.offset.y = Math.sin(offset_angle + delta) * offset_magnitude;

        this.#prev_theta = this.object.theta;

        const applied_pos = Vector2.addVectors(this.object.pos, this.offset);
        const dist = Vector2.distance(this.mouse_pos, applied_pos);
        const displacement = dist - this.#rest_length;

        if (dist == 0) 
            return;
        
        const linear_dir =  (Vector2.subtractVectors(this.mouse_pos, this.object.pos)).normalized();
        const applied_dir = (Vector2.subtractVectors(this.mouse_pos, applied_pos)).normalized();

        const force = this.#stiffness_const * displacement;

        // translational force
        const linear_force_vec = Vector2.scaleVector(applied_dir, force);
        this.object.force = Vector2.addVectors(this.object.force, linear_force_vec);

        // rotational force (torque)
        const applied_force_vec = Vector2.scaleVector(applied_dir, force);
        this.object.tau += Vector2.cross(this.offset, applied_force_vec); // tau is torque accumulator
    }

    #apply_to_linkConstraint(m_objects) {
        const obj1 = m_objects[this.object.id1];
        const obj2 = m_objects[this.object.id2];

        this.#rest_length = 0.2;

        const A_B = Vector2.subtractVectors(obj1.pos, obj2.pos);
        const applied_pos = Vector2.addVectors(obj2.pos, Vector2.scaleVector(A_B, this.t_linkConstraint));
        const dist = Vector2.distance(this.mouse_pos, applied_pos);
        const displacement = dist - this.#rest_length;

        if (dist == 0)
            return;

        const force = this.#stiffness_const * displacement;
        const v = Vector2.scaleVector(Vector2.subtractVectors(this.mouse_pos, applied_pos), 1 / dist);
        const force_vec = Vector2.scaleVector(v, force);

        const COM = Vector2.addVectors(obj2.pos, Vector2.scaleVector(A_B, obj1.m / (obj1.m + obj2.m)));


        // translational forces
        obj1.force = Vector2.addVectors(obj1.force, force_vec);
        obj2.force = Vector2.addVectors(obj2.force, force_vec);
        
        // rotational forces (adds to objects translational force)
        const torque = Vector2.cross(Vector2.subtractVectors(applied_pos, COM), force_vec);

        const v1 = Vector2.subtractVectors(obj1.pos, COM).normalized();
        const torque_dir_1 = new Vector2(- v1.y, v1.x);
        obj1.force = Vector2.addVectors(obj1.force, Vector2.scaleVector(torque_dir_1, torque));

        const v2 = Vector2.subtractVectors(obj2.pos, COM).normalized();
        const torque_dir_2 = new Vector2(- v2.y, v2.x);
        obj2.force = Vector2.addVectors(obj2.force, Vector2.scaleVector(torque_dir_2, torque));
        
    }

    apply(m_objects) {
        if (this.object instanceof DynamicObject) 
            this.#apply_to_dynamicObject(m_objects);
        else if (this.object instanceof LinkConstraint)
            this.#apply_to_linkConstraint(m_objects);
    }

    getClosestObject(m_objects, m_constraints) {
        // for objects
        for (let i = 0; i < m_objects.length; i++) {
            const v = Vector2.subtractVectors(m_objects[i].pos, this.mouse_pos);
            const dist2 = v.x * v.x + v.y * v.y;
            const rad = m_objects[i].radius;
            if (dist2 < rad * rad) {
                this.object = m_objects[i];
                this.offset = Vector2.subtractVectors(this.mouse_pos, m_objects[i].pos);
                this.#prev_theta = m_objects[i].theta;
                return true;
            }
        }
        // for linkConstraints
        for (let i = 0; i < m_constraints.length; i++) {
            if (!(m_constraints[i] instanceof LinkConstraint))
                continue;
            const A = m_objects[m_constraints[i].id1];
            const B = m_objects[m_constraints[i].id2];
            // finish this, make sure its working
            const C_B = Vector2.subtractVectors(this.mouse_pos, B.pos);
            const A_B = Vector2.subtractVectors(A.pos, B.pos);
            const t = Math.max(Math.min(C_B.dot(A_B) / A_B.sqr_magnitude(), 1), 0);
            const proj = Vector2.addVectors(B.pos, Vector2.scaleVector(A_B, t));
            const dist2 = Vector2.subtractVectors(proj, this.mouse_pos).sqr_magnitude();
            const sim_line_width = Units.scale_c_s * LineWidths.OUTER_LINKCONSTRAINT;
            if (dist2 < sim_line_width * sim_line_width) {
                this.object = m_constraints[i];
                this.t_linkConstraint = t;

                return true;
            }
        }

        return false;
    }

    getEnergyApplied(m_objects) {
        // TODO
        // change this so that it works with linkconst also

        if (!this.active)
            return 0;

        if (this.object instanceof DynamicObject) {
            let energy = 0;

            const applied_pos = Vector2.addVectors(this.object.pos, this.offset);

            // simple spring hookes law
            const dist = Vector2.distance(this.mouse_pos, applied_pos);
            const displacement = dist - this.#rest_length;
            const force = this.#stiffness_const * displacement;
            energy += 0.5 * force * displacement;

            return energy;
        } else if (this.object instanceof LinkConstraint) {
            let energy = 0;

            const obj1 = m_objects[this.object.id1];
            const obj2 = m_objects[this.object.id2];

            const A_B = Vector2.subtractVectors(obj1.pos, obj2.pos);
            const applied_pos = Vector2.addVectors(obj2.pos, Vector2.scaleVector(A_B, this.t_linkConstraint));
            const dist = Vector2.distance(this.mouse_pos, applied_pos);
            const displacement = dist - this.#rest_length;

            const force = this.#stiffness_const * displacement;

            energy += 0.5 * force * displacement;

            return energy;
        }
        
    }

    render(c, m_objects) {

        const start_pos = this.mouse_pos;
        let end_pos;
        if (this.object instanceof DynamicObject) {
            end_pos = Vector2.addVectors(this.object.pos, this.offset);
        } else if (this.object instanceof LinkConstraint) {
            const A = m_objects[this.object.id1];
            const B = m_objects[this.object.id2];
            const A_B = Vector2.subtractVectors(A.pos, B.pos);
            end_pos = Vector2.addVectors(B.pos, Vector2.scaleVector(A_B, this.t_linkConstraint));
        }

        const num_segments = Extras.MOUSESPRING_NUM_SEGMENTS;
        const width = 0.2;

        const mouse_radius = 2 * Extras.SPRING_JOINT_ENDS_RADIUS;

        let segments = [];

        const dist = Vector2.distance(start_pos, end_pos);

        const dir = Vector2.scaleVector(Vector2.subtractVectors(end_pos, start_pos), 1 / dist);
        const dirT = new Vector2(-dir.y, dir.x);

        const spring_end_delta = Vector2.scaleVector(dir.negated(), mouse_radius);
        const spring_end = Vector2.addVectors(end_pos, spring_end_delta);
        const spring_start_delta = Vector2.scaleVector(dir, mouse_radius);
        const spring_start = Vector2.addVectors(start_pos, spring_start_delta);

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
        const connection_end_radius = 0.5 * mouse_radius;
        const connection_end_delta = Vector2.scaleVector(dirT, connection_end_radius);
        const connection_end_pos_1 = Units.sim_canv(Vector2.addVectors(spring_end, connection_end_delta));
        const connection_end_pos_2 = Units.sim_canv(Vector2.addVectors(spring_end, connection_end_delta.negated()));
        const connection_end_pos_3 = Units.sim_canv(Vector2.addVectors(end_pos, connection_end_delta));
        const connection_end_pos_4 = Units.sim_canv(Vector2.addVectors(end_pos, connection_end_delta.negated()));
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        // draw rectangle:
        c.beginPath();

        c.arc(Units.sim_canv_x(end_pos), Units.sim_canv_y(end_pos), Units.scale_s_c * connection_end_radius, 0, 2 * Math.PI);

        c.moveTo(connection_end_pos_1.x, connection_end_pos_1.y);
        c.lineTo(connection_end_pos_3.x, connection_end_pos_3.y);
        c.lineTo(connection_end_pos_4.x, connection_end_pos_4.y);
        c.lineTo(connection_end_pos_2.x, connection_end_pos_2.y);
        
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
        c.arc(Units.sim_canv_x(end_pos), Units.sim_canv_y(end_pos), 0.3 * connection_end_radius, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        // mouse circle:
        // constants
        const mouse_canvas_pos = Units.sim_canv(start_pos);
        // init settings
        c.fillStyle = Colours.INNER_MOUSE_SPRING_MOUSE_CIRCLE;
        c.strokeStyle = Colours.OUTER_MOUSE_SPRING_MOUSE_CIRCLE;
        c.lineWidth = LineWidths.MOUSE_SPRING_MOUSE_CIRCLE;
        // draw circle at mouse, OPTIONAL
        // c.beginPath();
        // c.arc(mouse_canvas_pos.x, mouse_canvas_pos.y, Units.scale_s_c * mouse_radius, 0, 2 * Math.PI);
        // c.fill();
        // c.stroke();
        // c.closePath();

        // connection to mouse:
        // constants:
        const connection_start_radius = 0.5 * mouse_radius;
        const connection_mouse_delta = Vector2.scaleVector(dirT, connection_start_radius);
        const connection_start_pos_1 = Units.sim_canv(Vector2.addVectors(spring_start, connection_mouse_delta));
        const connection_start_pos_2 = Units.sim_canv(Vector2.addVectors(spring_start, connection_mouse_delta.negated()));
        const connection_start_pos_3 = Units.sim_canv(Vector2.addVectors(start_pos, connection_mouse_delta));
        const connection_start_pos_4 = Units.sim_canv(Vector2.addVectors(start_pos, connection_mouse_delta.negated()));
        // init settings
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_ENDS;
        // draw rectangle:
        c.beginPath();

        c.arc(mouse_canvas_pos.x, mouse_canvas_pos.y, Units.scale_s_c * connection_start_radius, 0, 2 * Math.PI);

        c.moveTo(connection_start_pos_2.x, connection_start_pos_2.y);
        c.lineTo(connection_start_pos_4.x, connection_start_pos_4.y);
        c.lineTo(connection_start_pos_3.x, connection_start_pos_3.y);
        c.lineTo(connection_start_pos_1.x, connection_start_pos_1.y);

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
        c.arc(mouse_canvas_pos.x, mouse_canvas_pos.y, 0.3 * connection_start_radius, 0, 2 * Math.PI);
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