import {SparseMatrixBlock, SparseMatrix, Vector, Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";
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
        return q.elements[2 * this.p_id + 1] - this.height;
    }

    C_i_dot(q, q_dot) {
        return q_dot.elements[2 * this.p_id + 1];
    }

    J_i(q, ith_row) {
        // last 1 is the data, actual jacobian
        return [new SparseMatrixBlock(ith_row, 2 * this.p_id + 1, 1)];
    }

    J_i_dot(q, q_dot, ith_row) {
        return [new SparseMatrixBlock(ith_row, 2 * this.p_id + 1, 0)];
    }

    render(c, m_objects, lagrange_mult) {

        const obj = m_objects[this.p_id];

        const obj_rad = obj.radius;

        const small_circle_rad = obj_rad / 4;
        const connection_circle_rad = obj_rad * Units.scale_s_c / 3; // Math.sqrt(2 * obj_rad * Units.scale_s_c);

        const correction_edges = Units.scale_c_s * (0.5*LineWidths.OUTER_FIXEDYCONSTRAINT_HORIZONTAL_LINE);
        // const correction_edges = 0;

        const horizontal_pos2 = Vector2.addVectors(obj.pos, new Vector2(0, - correction_edges - obj_rad));
        const horizontal_pos1 = Vector2.addVectors(horizontal_pos2, new Vector2(-1.3 * obj_rad, 0));
        const horizontal_pos3 = Vector2.addVectors(horizontal_pos2, new Vector2( 1.3 * obj_rad, 0));

        const c1 = Vector2.addVectors(horizontal_pos1, new Vector2(0.2 * obj_rad, - correction_edges - small_circle_rad));
        const c2 = Vector2.addVectors(horizontal_pos2, new Vector2(0, - correction_edges - small_circle_rad));
        const c3 = Vector2.addVectors(horizontal_pos3, new Vector2(- 0.2 * obj_rad, - correction_edges - small_circle_rad));

        const connection_obj_pos1 = Vector2.addVectors(horizontal_pos2, new Vector2(- Units.scale_c_s * connection_circle_rad, 0));
        const connection_obj_pos2 = Vector2.addVectors(obj.pos, new Vector2(- Units.scale_c_s * connection_circle_rad, 0));
        const connection_obj_pos3 = Vector2.addVectors(obj.pos, new Vector2(Units.scale_c_s * connection_circle_rad, 0));
        const connection_obj_pos4 = Vector2.addVectors(horizontal_pos2, new Vector2(Units.scale_c_s * connection_circle_rad, 0));
        
        obj.render(c);

        // draw the connecting things
        // settings
        c.fillStyle = Colours.INNER_FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT;
        c.strokeStyle = Colours.OUTER_FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT;
        // non adjustable settings:
        c.lineWidth = LineWidths.OUTER_FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT;
        c.lineCap = Extras.FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT_ENDCAPS;
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
        c.strokeStyle = Colours.OUTER_FIXEDXCONSTRAINT_HORIZONTAL_LINE;
        c.lineCap = "round";
        // non adjustable settings
        c.lineWidth = LineWidths.OUTER_FIXEDYCONSTRAINT_HORIZONTAL_LINE;
        // draw commands
        // border
        c.beginPath();
        c.moveTo(Units.sim_canv_x(horizontal_pos1), Units.sim_canv_y(horizontal_pos1));
        c.lineTo(Units.sim_canv_x(horizontal_pos3), Units.sim_canv_y(horizontal_pos3));
        c.stroke();
        // c.fill();
        c.closePath();
        // fill (inner)
        c.lineWidth = LineWidths.INNER_FIXEDYCONSTRAINT_HORIZONTAL_LINE;
        c.strokeStyle = Colours.INNER_FIXEDYCONSTRAINT_HORIZONTAL_LINE;
        c.beginPath();
        c.moveTo(Units.sim_canv_x(horizontal_pos1), Units.sim_canv_y(horizontal_pos1));
        c.lineTo(Units.sim_canv_x(horizontal_pos3), Units.sim_canv_y(horizontal_pos3));
        c.stroke();
        // c.fill();
        c.closePath();

        // draw the three small circles
        // settings:
        c.fillStyle = Colours.INNER_FIXEDYCONSTRAINT_HORIZONTAL_LINE;
        c.strokeStyle = Colours.OUTER_FIXEDXCONSTRAINT_HORIZONTAL_LINE;
        // non adjustable settings:
        c.lineWidth = LineWidths.INNER_FIXEDYCONSTRAINT_HORIZONTAL_LINE;
        // draw commands:
        c.beginPath();
        c.arc(Units.sim_canv_x(c1), Units.sim_canv_y(c1), Units.scale_s_c * small_circle_rad, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();
        c.beginPath();
        c.arc(Units.sim_canv_x(c2), Units.sim_canv_y(c2), Units.scale_s_c * small_circle_rad, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();
        c.beginPath();
        c.arc(Units.sim_canv_x(c3), Units.sim_canv_y(c3), Units.scale_s_c * small_circle_rad, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();        

    }
}

export class FixedXConstraint {
    constructor(id, x0) {
        this.p_id = id;
        this.x0 = x0;
    }

    C_i(q) {
        return q.elements[2 * this.p_id] - this.x0;
    }

    C_i_dot(q, q_dot) {
        return q_dot.elements[2 * this.p_id];
    }

    J_i(q, ith_row) {
        return [new SparseMatrixBlock(ith_row, 2 * this.p_id, 1)];
    }

    J_i_dot(q, q_dot, ith_row) {
        return [new SparseMatrixBlock(ith_row, 2 * this.p_id, 0)];
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

        m_objects[this.p_id].render(c)


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
        const dx = q.elements[2 * this.id2] - q.elements[2 * this.id1];
        const dy = q.elements[2 * this.id2 + 1] - q.elements[2 * this.id1 + 1];
        return 0.5 * (dx * dx + dy * dy - this.l0 * this.l0);
    }

    C_i_dot(q, q_dot) {
        const dx = q.elements[2 * this.id2] - q.elements[2 * this.id1];
        const dy = q.elements[2 * this.id2 + 1] - q.elements[2 * this.id1 + 1];
        const dvx = q_dot.elements[2 * this.id2] - q_dot.elements[2 * this.id1];
        const dvy = q_dot.elements[2 * this.id2 + 1] - q_dot.elements[2 * this.id1 + 1];
        return dx * dvx + dy * dvy;
    }

    J_i(q, ith_row) {
        const wrt_x1 = new SparseMatrixBlock(ith_row, 2 * this.id1,     -1 * (q.elements[2 * this.id2] - q.elements[2 * this.id1]));
        const wrt_y1 = new SparseMatrixBlock(ith_row, 2 * this.id1 + 1, -1 * (q.elements[2 * this.id2 + 1] - q.elements[2 * this.id1 + 1]));
        const wrt_x2 = new SparseMatrixBlock(ith_row, 2 * this.id2,     1 * (q.elements[2 * this.id2] - q.elements[2 * this.id1]));
        const wrt_y2 = new SparseMatrixBlock(ith_row, 2 * this.id2 + 1, 1 * (q.elements[2 * this.id2 + 1] - q.elements[2 * this.id1 + 1]));
        return [wrt_x1, wrt_y1, wrt_x2, wrt_y2];
    }

    J_i_dot(q, q_dot, ith_row) {
        const wrt_x1 = new SparseMatrixBlock(ith_row, 2 * this.id1,     -1 * (q_dot.elements[2 * this.id2] - q_dot.elements[2 * this.id1]));
        const wrt_y1 = new SparseMatrixBlock(ith_row, 2 * this.id1 + 1, -1 * (q_dot.elements[2 * this.id2 + 1] - q_dot.elements[2 * this.id1 + 1]));
        const wrt_x2 = new SparseMatrixBlock(ith_row, 2 * this.id2,     1 * (q_dot.elements[2 * this.id2] - q_dot.elements[2 * this.id1]));
        const wrt_y2 = new SparseMatrixBlock(ith_row, 2 * this.id2 + 1, 1 * (q_dot.elements[2 * this.id2 + 1] - q_dot.elements[2 * this.id1 + 1]));
        return [wrt_x1, wrt_y1, wrt_x2, wrt_y2];
    }

    render(c, m_objects, lagrange_mult) {  
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

        const c_value =  Extras.LINKCONSTRAINT_STRESS_BOOL ? lagrange_mult ? Extras.LINKCONSTRAINT_STRESS_MULTIPLIER * Math.abs(lagrange_mult) : 0
                                                           :  0
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

        m_objects[this.id1].render(c);
        m_objects[this.id2].render(c);
    
    }
}