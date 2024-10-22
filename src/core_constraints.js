import {SparseMatrixBlock, SparseMatrix, Vector, Vector2} from "./linear_algebra.js";
import {Units} from "./units.js";
import {Colours, LineWidths, Extras} from "./render_settings.js";

/*
    When adding a new constraint, 
    these functions must be in them spelled correctly and be returning these specific things:
        --> C_i (the constraint value), return: scalar
        --> C_dot (the time D of C), return: scalar
        --> J_i (the jacobian of C wrt position vectors (q)), return: list of SparseMatrixBlocks
        --> J_i_dot (time D of J_i), return: list of SparseMatrixBlocks
        --> render(c, m_objects) (self-explanatory)
*/

export class FixedYConstraint {
    constructor(id, height) {
        this.p_id = id;
        this.height = height;
    }

    C_i(q) {
        return q.elements[3 * this.p_id + 1] - this.height;
    }

    C_i_dot(q, q_dot) {
        return q_dot.elements[3 * this.p_id + 1];
    }

    J_i(q, ith_row) {
        // last 1 is the data, actual jacobian
        return [new SparseMatrixBlock(ith_row, 3 * this.p_id + 1, 1)];
    }

    J_i_dot(q, q_dot, ith_row) {
        return [];
    }

    render(c, m_objects, lagrange_mult) {

        const obj = m_objects[this.p_id];

        const obj_rad = obj.radius;

        const connection_circle_rad = Units.scale_s_c * (obj_rad) + Extras.FIXEDPOSCONSTRAINT_OUTSIDE_OBJECT_BORDER_WIDTH;

        // const correction_edges = Units.scale_c_s * (0.5*Colours.INNER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT + Colours.INNER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT - Colours.OUTER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT);
        const correction_edges = 0;

        const horizontal_pos2 = Vector2.addVectors(obj.pos, new Vector2(0, - correction_edges - obj_rad - 0.05));
        const horizontal_pos1 = Vector2.addVectors(horizontal_pos2, new Vector2(-1.5 * obj_rad - 0.2, 0));
        const horizontal_pos3 = Vector2.addVectors(horizontal_pos2, new Vector2( 1.5 * obj_rad + 0.2, 0));

        const connection_obj_pos1 = Vector2.addVectors(horizontal_pos2, new Vector2(- Units.scale_c_s * connection_circle_rad, 0));
        const connection_obj_pos2 = Vector2.addVectors(obj.pos, new Vector2(- Units.scale_c_s * connection_circle_rad, 0));
        const connection_obj_pos3 = Vector2.addVectors(obj.pos, new Vector2(Units.scale_c_s * connection_circle_rad, 0));
        const connection_obj_pos4 = Vector2.addVectors(horizontal_pos2, new Vector2(Units.scale_c_s * connection_circle_rad, 0));

        // obj.render(c);

        // draw the connecting things
        // settings
        c.fillStyle = Colours.INNER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT;
        c.strokeStyle = Colours.OUTER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT;
        // non adjustable settings:
        c.lineWidth = LineWidths.OUTER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT;
        c.lineCap = "butt";
        // drawing
        c.beginPath();
        c.arc(Units.sim_canv_x(obj.pos), Units.sim_canv_y(obj.pos), connection_circle_rad, 0, 2 * Math.PI);
        c.moveTo(Units.sim_canv_x(connection_obj_pos1), Units.sim_canv_y(connection_obj_pos1));
        c.lineTo(Units.sim_canv_x(connection_obj_pos2), Units.sim_canv_y(connection_obj_pos2));
        c.lineTo(Units.sim_canv_x(connection_obj_pos3), Units.sim_canv_y(connection_obj_pos3));
        c.lineTo(Units.sim_canv_x(connection_obj_pos4), Units.sim_canv_y(connection_obj_pos4));
        c.stroke();
        c.fill();
        c.closePath();
        // draw the small inner circle
        c.lineWidth = 0;
        c.beginPath();
        c.fillStyle = c.strokeStyle;
        c.arc(Units.sim_canv_x(obj.pos), Units.sim_canv_y(obj.pos), 0.3 * connection_circle_rad, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        // draw the horizontal line:
        // settings:
        c.strokeStyle = Colours.OUTER_FIXEDPOSCONSTRAINT_HORIZONTAL_LINE;
        c.lineCap = Extras.FIXEDPOSCONSTRAINT_HORIZONTAL_LINE_ENDCAPS;
        // non adjustable settings
        c.lineWidth = LineWidths.OUTER_FIXEDPOSCONSTRAINT_HORIZONTAL_LINE;
        // draw commands
        // border
        c.beginPath();
        c.moveTo(Units.sim_canv_x(horizontal_pos1), Units.sim_canv_y(horizontal_pos1));
        c.lineTo(Units.sim_canv_x(horizontal_pos3), Units.sim_canv_y(horizontal_pos3));
        c.stroke();
        // c.fill();
        c.closePath();
        // fill (inner)
        c.lineWidth = LineWidths.INNER_FIXEDPOSCONSTRAINT_HORIZONTAL_LINE;
        c.strokeStyle = Colours.INNER_FIXEDPOSCONSTRAINT_HORIZONTAL_LINE;
        c.beginPath();
        c.moveTo(Units.sim_canv_x(horizontal_pos1), Units.sim_canv_y(horizontal_pos1));
        c.lineTo(Units.sim_canv_x(horizontal_pos3), Units.sim_canv_y(horizontal_pos3));
        c.stroke();
        // c.fill();
        c.closePath();

        // draw the small 5 circles
        const dist = Vector2.distance(horizontal_pos1, horizontal_pos3);
        const small_circle_rad = (dist / 2) / 10;
        for (let i = 0; i <= 5; i++) {
            const t = i / 5;

            let pos = Vector2.addVectors(horizontal_pos1, Vector2.scaleVector(Vector2.subtractVectors(horizontal_pos3, horizontal_pos1), t));
            // project:
            const r1 = horizontal_pos1.x + small_circle_rad;
            const r2 = horizontal_pos3.x - small_circle_rad;
            pos.x = r1 + (pos.x - horizontal_pos1.x) * (r2 - r1) / (horizontal_pos3.x - horizontal_pos1.x);
            pos.y -= small_circle_rad + 1/2 * Units.scale_c_s * (LineWidths.OUTER_FIXEDYCONSTRAINT_HORIZONTAL_LINE);

            c.fillStyle = Colours.INNER_FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT;
            c.strokeStyle = Colours.OUTER_FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT;
            c.lineWidth = 1;
            c.beginPath();
            c.arc(Units.sim_canv_x(pos), Units.sim_canv_y(pos), Units.scale_s_c * small_circle_rad, 0, 2 * Math.PI);
            c.stroke();
            c.fill();
            c.closePath();
        } 

    }
}

export class FixedXConstraint {
    constructor(id, x0) {
        this.p_id = id;
        this.x0 = x0;
    }

    C_i(q) {
        return q.elements[3 * this.p_id] - this.x0;
    }

    C_i_dot(q, q_dot) {
        return q_dot.elements[3 * this.p_id];
    }

    J_i(q, ith_row) {
        return [new SparseMatrixBlock(ith_row, 3 * this.p_id, 1)];
    }

    J_i_dot(q, q_dot, ith_row) {
        return [];
    }

    render(c, m_objects, lagrange_mult) {
        const obj = m_objects[this.p_id];
        const canv_pos = Units.sim_canv(obj.pos);
        const rad = obj.radius;
        const canvas_rad = rad * Units.scale_s_c;

        c.lineCap = Extras.FIXEDXCONSTRAINT_VERTICAL_LINE_ENDCAPS;  
        
        // vertical
        const ver_pos1 = Vector2.addVectors(canv_pos, new Vector2(0, - 2.6 * canvas_rad));
        const ver_pos2 = Vector2.addVectors(canv_pos, new Vector2(0,   2.6 * canvas_rad));

        c.beginPath();

        // border
        c.lineWidth = LineWidths.OUTER_FIXEDXCONSTRAINT_VERTICAL_LINE;
        c.strokeStyle = Colours.OUTER_FIXEDXCONSTRAINT_VERTICAL_LINE;
        c.moveTo(ver_pos1.x, ver_pos1.y);
        c.lineTo(ver_pos2.x, ver_pos2.y);
        c.stroke();

        // int
        c.lineWidth = LineWidths.INNER_FIXEDXCONSTRAINT_VERTICAL_LINE;
        c.strokeStyle = Colours.INNER_FIXEDXCONSTRAINT_VERTICAL_LINE;
        c.moveTo(ver_pos1.x, ver_pos1.y);
        c.lineTo(ver_pos2.x, ver_pos2.y);
        c.stroke();

        c.closePath();

        // m_objects[this.p_id].render(c)


        // horizontal
        c.lineCap = Extras.FIXEDXCONSTRAINT_HORIZONTAL_LINE_ENDCAPS

        const hor_pos1 = Vector2.addVectors(canv_pos, new Vector2(- 0.97 * canvas_rad, canvas_rad));
        const hor_pos2 = Vector2.addVectors(canv_pos, new Vector2(  0.97 * canvas_rad, canvas_rad));

        c.beginPath();

        // border
        c.lineWidth = LineWidths.OUTER_FIXEDXCONSTRAINT_HORIZONTAL_LINE;
        c.strokeStyle = Colours.OUTER_FIXEDXCONSTRAINT_HORIZONTAL_LINE;
        c.moveTo(hor_pos1.x, hor_pos1.y);
        c.lineTo(hor_pos2.x, hor_pos2.y);
        c.stroke();

        // int
        c.lineWidth = LineWidths.INNER_FIXEDXCONSTRAINT_HORIZONTAL_LINE;
        c.strokeStyle = Colours.INNER_FIXEDXCONSTRAINT_HORIZONTAL_LINE;
        c.moveTo(hor_pos1.x, hor_pos1.y);
        c.lineTo(hor_pos2.x, hor_pos2.y);
        c.stroke();

        c.closePath();

        // circles
        // c.fillStyle = "#FFFFFF";
        // c.strokeStyle = "#000000";
        // c.lineWidth = 1;
        // const c_radius = 0.5 * radius;
        // const c1 = Vector2.addVectors(pos1, new Vector2(  0.8 * c_radius, 1.4 * c_radius));
        // const c2 = Vector2.addVectors(pos2, new Vector2(- 0.8 * c_radius, 1.4 * c_radius));

        // c.beginPath();
        // c.arc(c1.x, c1.y, c_radius, 0, 2 * Math.PI);
        // c.fill();
        // c.stroke();
        // c.closePath();

        // c.beginPath();
        // c.arc(c2.x, c2.y, c_radius, 0, 2 * Math.PI);
        // c.fill();
        // c.stroke();
        // c.closePath();
    }
}

export class LinkConstraint { // constrains two particles to be a fixed distance apart
    constructor(id1, id2, l0) {
        this.id1 = id1;
        this.id2 = id2;
        this.l0 = l0;
    }

    C_i(q) {
        const dx = q.elements[3 * this.id2] - q.elements[3 * this.id1];
        const dy = q.elements[3 * this.id2 + 1] - q.elements[3 * this.id1 + 1];
        return 0.5 * (dx * dx + dy * dy - this.l0 * this.l0);
    }

    C_i_dot(q, q_dot) {
        const dx = q.elements[3 * this.id2] - q.elements[3 * this.id1];
        const dy = q.elements[3 * this.id2 + 1] - q.elements[3 * this.id1 + 1];
        const dvx = q_dot.elements[3 * this.id2] - q_dot.elements[3 * this.id1];
        const dvy = q_dot.elements[3 * this.id2 + 1] - q_dot.elements[3 * this.id1 + 1];
        return dx * dvx + dy * dvy;
    }

    J_i(q, ith_row) {
        const wrt_x1 = new SparseMatrixBlock(ith_row, 3 * this.id1,     -1 * (q.elements[3 * this.id2] - q.elements[3 * this.id1]));
        const wrt_y1 = new SparseMatrixBlock(ith_row, 3 * this.id1 + 1, -1 * (q.elements[3 * this.id2 + 1] - q.elements[3 * this.id1 + 1]));
        const wrt_x2 = new SparseMatrixBlock(ith_row, 3 * this.id2,     1 * (q.elements[3 * this.id2] - q.elements[3 * this.id1]));
        const wrt_y2 = new SparseMatrixBlock(ith_row, 3 * this.id2 + 1, 1 * (q.elements[3 * this.id2 + 1] - q.elements[3 * this.id1 + 1]));
        return [wrt_x1, wrt_y1, wrt_x2, wrt_y2];
    }

    J_i_dot(q, q_dot, ith_row) {
        const wrt_x1 = new SparseMatrixBlock(ith_row, 3 * this.id1,     -1 * (q_dot.elements[3 * this.id2] - q_dot.elements[3 * this.id1]));
        const wrt_y1 = new SparseMatrixBlock(ith_row, 3 * this.id1 + 1, -1 * (q_dot.elements[3 * this.id2 + 1] - q_dot.elements[3 * this.id1 + 1]));
        const wrt_x2 = new SparseMatrixBlock(ith_row, 3 * this.id2,     1 * (q_dot.elements[3 * this.id2] - q_dot.elements[3 * this.id1]));
        const wrt_y2 = new SparseMatrixBlock(ith_row, 3 * this.id2 + 1, 1 * (q_dot.elements[3 * this.id2 + 1] - q_dot.elements[3 * this.id1 + 1]));
        return [wrt_x1, wrt_y1, wrt_x2, wrt_y2];
    }

    render(c, m_objects, lagrange_mult, lagrange_mult_limit) {  
        c.lineCap = Extras.LINKCONSTRAINT_ENDCAPS;       

        const pos1 = Units.sim_canv(m_objects[this.id1].pos);
        const pos2 = Units.sim_canv(m_objects[this.id2].pos);
        // const dir = (Vector2.subtractVectors(pos1, pos2)).normalized();

        c.beginPath();

        // border
        c.lineWidth = LineWidths.OUTER_LINKCONSTRAINT;
        c.strokeStyle = Colours.OUTER_LINKCONSTRAINT;
        c.moveTo(pos1.x, pos1.y);
        c.lineTo(pos2.x, pos2.y);
        c.stroke();

        // interiour

        let c_value = 0;
        if (Extras.LINKCONSTRAINT_STRESS_BOOL && lagrange_mult) 
            c_value = 255 * (Math.abs(lagrange_mult) / lagrange_mult_limit);

        // const c_value =  Extras.LINKCONSTRAINT_STRESS_BOOL ? lagrange_mult ? Extras.LINKCONSTRAINT_STRESS_MULTIPLIER * Math.abs(lagrange_mult) : 0
                                                        //    :  0
        // const c_value = 0;
        const color = lagrange_mult < 0
            ? "rgba("     + (255 - c_value) + ", " + 255             + ", " + (255 - c_value) + ", 1)" :
              "rgba(255," + (255 - c_value) + ", " + (255 - c_value) + ", " + 255             + ")";


        c.lineWidth = LineWidths.INNER_LINKCONSTRAINT;
        c.strokeStyle = color;
        c.moveTo(pos1.x, pos1.y);
        c.lineTo(pos2.x, pos2.y);
        c.stroke();

        c.closePath();

        // m_objects[this.id1].render(c);
        // m_objects[this.id2].render(c);
    
    }
}

export class OffsetLinkConstraint {
    constructor(state_1, state_2, m_objects) {
        this.state_1 = {    id: m_objects.indexOf(state_1.entity), 
                            offset: state_1.offset, 
                            prev_theta: state_1.prev_theta, 
                            t_param: state_1.t_param, 
                            applied_pos: state_1.applied_pos};

        this.state_2 = {    id: m_objects.indexOf(state_2.entity), 
                            offset: state_2.offset, 
                            prev_theta: state_2.prev_theta, 
                            t_param: state_2.t_param, 
                            applied_pos: state_2.applied_pos};


        const p1 = Vector2.addVectors(m_objects[this.state_1.id].pos, this.state_1.offset);                    
        const p2 = Vector2.addVectors(m_objects[this.state_2.id].pos, this.state_2.offset);                    
        this.l0 = Vector2.distance(p1, p2);

    }

    #updateDynamicObjectRotationOffset(state, q) {
        const offset_magnitude = state.offset.magnitude();
        const angle = Math.atan2(state.offset.y, state.offset.x);
        const delta = q.elements[3 * state.id + 2] - state.prev_theta;
        state.offset.x = Math.cos(angle + delta) * offset_magnitude;
        state.offset.y = Math.sin(angle + delta) * offset_magnitude;

        state.prev_theta = q.elements[3 * state.id + 2];
    }

    C_i(q) {

        this.#updateDynamicObjectRotationOffset(this.state_1, q);
        this.#updateDynamicObjectRotationOffset(this.state_2, q);

        const x1 = q.elements[3 * this.state_1.id];
        const y1 = q.elements[3 * this.state_1.id + 1];
        const cos_1 = this.state_1.offset.x;
        const sin_1 = this.state_1.offset.y;

        const x2 = q.elements[3 * this.state_2.id];
        const y2 = q.elements[3 * this.state_2.id + 1];
        const cos_2 = this.state_2.offset.x;
        const sin_2 = this.state_2.offset.y;

        return 1/2 * ((x1 + cos_1 - x2 - cos_2) ** 2 + (y1 + sin_1 - y2 - sin_2) ** 2 - this.l0 ** 2);

    }

    C_i_dot(q, q_dot) {
        const x1 = q.elements[3 * this.state_1.id];
        const y1 = q.elements[3 * this.state_1.id + 1];
        const cos_1 = this.state_1.offset.x;
        const sin_1 = this.state_1.offset.y;
        const x1_dot = q_dot.elements[3 * this.state_1.id];
        const y1_dot = q_dot.elements[3 * this.state_1.id + 1];
        const omega1 = q_dot.elements[3 * this.state_1.id + 2];

        const x2 = q.elements[3 * this.state_2.id];
        const y2 = q.elements[3 * this.state_2.id + 1];
        const cos_2 = this.state_2.offset.x;
        const sin_2 = this.state_2.offset.y;
        const x2_dot = q_dot.elements[3 * this.state_2.id];
        const y2_dot = q_dot.elements[3 * this.state_2.id + 1];
        const omega2 = q_dot.elements[3 * this.state_2.id + 2];

        return (x1_dot - omega1 * sin_1 - x2_dot + omega2 * sin_2) * (x1 + cos_1 - x2 - cos_2)
             + (y1_dot + omega1 * cos_1 - y2_dot - omega2 * cos_2) * (y1 + sin_1 - y2 - sin_2);

    }

    J_i(q, ith_row) {
        const x1 = q.elements[3 * this.state_1.id];
        const y1 = q.elements[3 * this.state_1.id + 1];
        const cos_1 = this.state_1.offset.x;
        const sin_1 = this.state_1.offset.y;

        const x2 = q.elements[3 * this.state_2.id];
        const y2 = q.elements[3 * this.state_2.id + 1];
        const cos_2 = this.state_2.offset.x;
        const sin_2 = this.state_2.offset.y;

        const wrt_x1 = new SparseMatrixBlock(ith_row, 3 * this.state_1.id,       x1 + cos_1 - x2 - cos_2);
        const wrt_y1 = new SparseMatrixBlock(ith_row, 3 * this.state_1.id + 1,   y1 + sin_1 - y2 - sin_2);
        const wrt_t1 = new SparseMatrixBlock(ith_row, 3 * this.state_1.id + 2,   - sin_1 * (x1 + cos_1 - x2 - cos_2) + cos_1 * (y1 + sin_1 - y2 - sin_2));
        const wrt_x2 = new SparseMatrixBlock(ith_row, 3 * this.state_2.id,       - (x1 + cos_1 - x2 - cos_2));
        const wrt_y2 = new SparseMatrixBlock(ith_row, 3 * this.state_2.id + 1,   - (y1 + sin_1 - y2 - sin_2));
        const wrt_t2 = new SparseMatrixBlock(ith_row, 3 * this.state_2.id + 2,   sin_2 * (x1 + cos_1 - x2 - cos_2) - cos_2 * (y1 + sin_1 - y2 - sin_2));
        return [wrt_x1, wrt_y1, wrt_t1, wrt_x2, wrt_y2, wrt_t2];
    }

    J_i_dot(q, q_dot, ith_row) {
        const x1 = q.elements[3 * this.state_1.id];
        const y1 = q.elements[3 * this.state_1.id + 1];
        const cos_1 = this.state_1.offset.x;
        const sin_1 = this.state_1.offset.y;
        const x1_dot = q_dot.elements[3 * this.state_1.id];
        const y1_dot = q_dot.elements[3 * this.state_1.id + 1];
        const omega1 = q_dot.elements[3 * this.state_1.id + 2];

        const x2 = q.elements[3 * this.state_2.id];
        const y2 = q.elements[3 * this.state_2.id + 1];
        const cos_2 = this.state_2.offset.x;
        const sin_2 = this.state_2.offset.y;
        const x2_dot = q_dot.elements[3 * this.state_2.id];
        const y2_dot = q_dot.elements[3 * this.state_2.id + 1];
        const omega2 = q_dot.elements[3 * this.state_2.id + 2];

        const wrt_x1 = new SparseMatrixBlock(ith_row, 3 * this.state_1.id,      x1_dot - omega1 * sin_1 - x2_dot + omega2 * sin_2);
        const wrt_y1 = new SparseMatrixBlock(ith_row, 3 * this.state_1.id + 1,  y1_dot + omega1 * cos_1 - y2_dot - omega2 * cos_2);
        const wrt_t1 = new SparseMatrixBlock(ith_row, 3 * this.state_1.id + 2,  - omega1 * cos_1 * (x1 + cos_1 - x2 - cos_2)
                                                                                - sin_1 * (x1_dot - omega1 * sin_1 - x2_dot + omega2 * sin_2)
                                                                                - omega1 * sin_1 * (y1 + sin_1 - y2 - sin_2)
                                                                                + cos_1 * (y1_dot + omega1 * cos_1 - y2_dot - omega2 * cos_2));
        

        const wrt_x2 = new SparseMatrixBlock(ith_row, 3 * this.state_2.id,      - (x1_dot - omega1 * sin_1 - x2_dot + omega2 * sin_2));
        const wrt_y2 = new SparseMatrixBlock(ith_row, 3 * this.state_2.id + 1,  - (y1_dot + omega1 * cos_1 - y2_dot - omega2 * cos_2));
        const wrt_t2 = new SparseMatrixBlock(ith_row, 3 * this.state_2.id + 2,  + omega2 * cos_2 * (x1 + cos_1 - x2 - cos_2)
                                                                                + sin_2 * (x1_dot - omega1 * sin_1 - x2_dot + omega2 * sin_2)
                                                                                + omega2 * sin_2 * (y1 + sin_1 - y2 - sin_2)
                                                                                - cos_2 * (y1_dot + omega1 * cos_1 - y2_dot - omega2 * cos_2));

        return [wrt_x1, wrt_y1, wrt_t1, wrt_x2, wrt_y2, wrt_t2];

    }

    render(c, m_objects, lagrange_mult, lagrange_mult_limit) {
        c.lineCap = Extras.LINKCONSTRAINT_ENDCAPS;       

        const width = 0.2;

        const link_pos1 = Units.sim_canv(Vector2.addVectors(m_objects[this.state_1.id].pos, this.state_1.offset));
        const link_pos2 = Units.sim_canv(Vector2.addVectors(m_objects[this.state_2.id].pos, this.state_2.offset));
        // const dir = (Vector2.subtractVectors(pos1, pos2)).normalized();

        c.beginPath();

        // border
        c.fillStyle = Colours.INNER_SPRING_ENDS;
        c.strokeStyle = Colours.OUTER_SPRING_ENDS;
        c.lineWidth = LineWidths.SPRING_EXTREME_SEGMENTS_BORDER + LineWidths.INNER_LINKCONSTRAINT;
        c.moveTo(link_pos1.x, link_pos1.y);
        c.lineTo(link_pos2.x, link_pos2.y);
        c.stroke();

        // interiour

        let c_value = 0;
        if (Extras.LINKCONSTRAINT_STRESS_BOOL && lagrange_mult) 
            c_value = 255 * (Math.abs(lagrange_mult) / lagrange_mult_limit);
        
        // const c_value = 0;
        const color = lagrange_mult < 0
            ? "rgba("     + (255 - c_value) + ", " + 255             + ", " + (255 - c_value) + ", 1)" :
              "rgba(255," + (255 - c_value) + ", " + (255 - c_value) + ", " + 255             + ")";


        c.lineWidth = LineWidths.INNER_LINKCONSTRAINT;
        c.strokeStyle = color;
        c.moveTo(link_pos1.x, link_pos1.y);
        c.lineTo(link_pos2.x, link_pos2.y);
        c.stroke();

        c.closePath();

        // m_objects[this.state_1.id].render(c);
        // m_objects[this.state_2.id].render(c);

        const outer_holding_circle_width_1 = 2 * Extras.SPRING_JOINT_ENDS_RADIUS;
        const outer_holding_circle_width_2 = 2 * Extras.SPRING_JOINT_ENDS_RADIUS;

        const pos_1 = Vector2.addVectors(m_objects[this.state_1.id].pos, this.state_1.offset);
        const pos_2 = Vector2.addVectors(m_objects[this.state_2.id].pos, this.state_2.offset);

        const dist = Vector2.distance(pos_1, pos_2);

        const dir = Vector2.scaleVector(Vector2.subtractVectors(pos_1, pos_2), 1 / dist);
        const dirT = new Vector2(-dir.y, dir.x);

        const spring_end_delta = Vector2.scaleVector(dir.negated(), outer_holding_circle_width_1);
        const spring_end = Vector2.addVectors(pos_1, spring_end_delta);
        const spring_start_delta = Vector2.scaleVector(dir, outer_holding_circle_width_2);
        const spring_start = Vector2.addVectors(pos_2, spring_start_delta);

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

export class FixedRotationConstraint {
    constructor(id, theta) {
        this.id = id;
        this.theta = theta;
    }

    #normalize_angle(angle) {
        return angle % (2 * Math.PI);
    }

    C_i(q) {
        return this.#normalize_angle(q.elements[3 * this.id + 2] - this.theta);
    }

    C_i_dot(q, q_dot) {
        return this.#normalize_angle(q_dot.elements[3 * this.id + 2]);
    }

    J_i(q, ith_row) {
        return [new SparseMatrixBlock(ith_row, 3 * this.id + 2, 1)];
    }

    J_i_dot(q, q_dot, ith_row) {
        return [];
    }

    render(c, m_objects, lagrange_mult) {

        const obj = m_objects[this.id];

        // draw the small dot visualising rotation
        c.fillStyle = "#ffffff";
        c.strokeStyle = Colours.OUTER_DYNAMIC_OBJECT; 
        c.lineWidth = LineWidths.DYNAMIC_OBJECT;

        let rotation_radius = Extras.DYNAMIC_OBJECT_ROTATIONAL_RADIUS_MULT * obj.radius;
        let m = Extras.DYNAMIC_OBJECT_ROTATIONAL_POSITION_MULT;
        let offset = Vector2.scaleVector(new Vector2(Math.cos(obj.theta + Math.PI), Math.sin(obj.theta + Math.PI)), m * obj.radius);
        let rotation_pos = Vector2.addVectors(obj.pos, offset);
        c.beginPath();
        c.arc(Units.sim_canv_x(rotation_pos), Units.sim_canv_y(rotation_pos), Units.scale_s_c * rotation_radius, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        offset = Vector2.scaleVector(new Vector2(Math.cos(obj.theta), Math.sin(obj.theta)), m * obj.radius);
        rotation_pos = Vector2.addVectors(obj.pos, offset);
        c.beginPath();
        c.arc(Units.sim_canv_x(rotation_pos), Units.sim_canv_y(rotation_pos), Units.scale_s_c * rotation_radius, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

    }

}

export class FixedOmegaConstraint {
    constructor(id, theta, velocity) {
        this.id = id;
        this.prev_theta = theta;
        this.velocity = velocity;
    }

    #normalize_angle(angle) {
        return angle % (2 * Math.PI);
    }

    C_i(q) {
        const delta = q.elements[3 * this.id + 2] - this.prev_theta;
        return this.#normalize_angle(delta - this.velocity);
    }

    C_i_dot(q, q_dot) {
        return this.#normalize_angle(q_dot.elements[3 * this.id + 2]);
    }

    J_i(q, ith_row) {
        return [new SparseMatrixBlock(ith_row, 3 * this.id + 2, 1)];
    }

    J_i_dot(q, q_dot, ith_row) {
        this.prev_theta = q.elements[3 * this.id + 2];
        return [];
    }

    render(c, m_objects, lagrange_mult) {
        // draw this one as the first forcegenerator

        const obj = m_objects[this.id];
        const angle = Extras.FIXED_OMEGA_CONSTRAINT_ANGLE;
        const inner_rad = Units.scale_c_s * LineWidths.INNER_FIXED_OMEGA_CONSTRAINT + obj.radius;

        c.fillStyle = Colours.INNER_FIXED_OMEGA_CONSTRAINT;
        c.strokeStyle = Colours.OUTER_FIXED_OMEGA_CONSTRAINT;
        c.lineWidth = LineWidths.OUTER_FIXED_OMEGA_CONSTRAINT;

        // arrow 1
        const top_axle = - obj.theta;
        c.beginPath();
        c.arc(Units.sim_canv_x(obj.pos), Units.sim_canv_y(obj.pos), Units.scale_s_c * inner_rad, top_axle - angle, top_axle + angle);
        // c.arc(Units.sim_canv_x(obj.pos), Units.sim_canv_y(obj.pos), Units.scale_s_c * inner_rad, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        // arrow 2
        const bottom_axle = - obj.theta + Math.PI;
        c.beginPath();
        c.arc(Units.sim_canv_x(obj.pos), Units.sim_canv_y(obj.pos), Units.scale_s_c * inner_rad, bottom_axle - angle, bottom_axle + angle);
        // c.arc(Units.sim_canv_x(obj.pos), Units.sim_canv_y(obj.pos), Units.scale_s_c * inner_rad, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();

        // triangles:
        const o = 1 / 2 * Units.scale_c_s * LineWidths.INNER_FIXED_OMEGA_CONSTRAINT
        // triangle 1
        let offset = Vector2.scaleVector(new Vector2(Math.cos(-top_axle + angle), Math.sin(-top_axle + angle)), inner_rad - o);
        let tip_1 = Vector2.addVectors(obj.pos, offset);
        let dir = Vector2.subtractVectors(tip_1, obj.pos).normalized();
        let dir_t = new Vector2(-dir.y, dir.x);
        let tri_1 = Vector2.addVectors(tip_1, Vector2.scaleVector(dir_t, Extras.FIXED_OMEGA_CONSTRAINT_TRIANGLE_SIZE));
        let tri_2 = Vector2.subtractVectors(tip_1, Vector2.scaleVector(dir_t, Extras.FIXED_OMEGA_CONSTRAINT_TRIANGLE_SIZE));
        let tri_3 = tri_2;
        tri_2 = Vector2.subtractVectors(tri_2, Vector2.scaleVector(dir, Extras.FIXED_OMEGA_CONSTRAINT_TRIANGLE_SIZE));
        tri_3 = Vector2.addVectors(tri_3, Vector2.scaleVector(dir, Extras.FIXED_OMEGA_CONSTRAINT_TRIANGLE_SIZE));

        c.fillStyle = Colours.INNER_FIXED_OMEGA_CONSTRAINT;
        c.strokeStyle = Colours.OUTER_FIXED_OMEGA_CONSTRAINT;
        c.lineWidth = LineWidths.OUTER_FIXED_OMEGA_CONSTRAINT;
        c.beginPath();
        c.moveTo(Units.sim_canv_x(tri_2), Units.sim_canv_y(tri_2));
        c.lineTo(Units.sim_canv_x(tri_1), Units.sim_canv_y(tri_1));
        c.lineTo(Units.sim_canv_x(tri_3), Units.sim_canv_y(tri_3));
        c.stroke();
        c.fill();
        c.closePath();

        // triangle 2
        offset = Vector2.scaleVector(new Vector2(Math.cos(-bottom_axle + angle), Math.sin(-bottom_axle + angle)), inner_rad - o);
        tip_1 = Vector2.addVectors(obj.pos, offset);
        dir = Vector2.subtractVectors(tip_1, obj.pos).normalized();
        dir_t = new Vector2(-dir.y, dir.x);
        tri_1 = Vector2.addVectors(tip_1, Vector2.scaleVector(dir_t, Extras.FIXED_OMEGA_CONSTRAINT_TRIANGLE_SIZE));
        tri_2 = Vector2.subtractVectors(tip_1, Vector2.scaleVector(dir_t, Extras.FIXED_OMEGA_CONSTRAINT_TRIANGLE_SIZE));
        tri_3 = tri_2;
        tri_2 = Vector2.subtractVectors(tri_2, Vector2.scaleVector(dir, Extras.FIXED_OMEGA_CONSTRAINT_TRIANGLE_SIZE));
        tri_3 = Vector2.addVectors(tri_3, Vector2.scaleVector(dir, Extras.FIXED_OMEGA_CONSTRAINT_TRIANGLE_SIZE));

        c.fillStyle = Colours.INNER_FIXED_OMEGA_CONSTRAINT;
        c.strokeStyle = Colours.OUTER_FIXED_OMEGA_CONSTRAINT;
        c.lineWidth = LineWidths.OUTER_FIXED_OMEGA_CONSTRAINT;
        c.beginPath();
        c.moveTo(Units.sim_canv_x(tri_2), Units.sim_canv_y(tri_2));
        c.lineTo(Units.sim_canv_x(tri_1), Units.sim_canv_y(tri_1));
        c.lineTo(Units.sim_canv_x(tri_3), Units.sim_canv_y(tri_3));
        c.stroke();
        c.fill();
        c.closePath();

    }

}