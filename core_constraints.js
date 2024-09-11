import {SparseMatrixBlock, SparseMatrix, Vector, Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";

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
        const canv_pos = Units.sim_canv(m_objects[this.p_id].pos);
        const radius = m_objects[this.p_id].drawing_radius * Units.scale_s_c;

        const lineWidth = 5;
        const borderWidth = 2;     
        c.lineCap = "square";       

        const pos1 = Vector2.addVectors(canv_pos, new Vector2(- 2.0 * radius, 1.2 * radius));
        const pos2 = Vector2.addVectors(canv_pos, new Vector2(  2.0 * radius, 1.2 * radius));

        c.beginPath();

        // border
        c.lineWidth = lineWidth;
        c.strokeStyle = "#000000";
        c.moveTo(pos1.x, pos1.y);
        c.lineTo(pos2.x, pos2.y);
        c.stroke();

        // int
        c.lineWidth = lineWidth - borderWidth;
        c.strokeStyle = "#FFFFFF";
        c.moveTo(pos1.x, pos1.y);
        c.lineTo(pos2.x, pos2.y);
        c.stroke();

        c.closePath();

        // circles
        c.fillStyle = "#FFFFFF";
        c.strokeStyle = "#000000";
        c.lineWidth = 1;
        const c_radius = Math.sqrt(2 * radius);
        const c1 = Vector2.addVectors(pos1, new Vector2(  0.8 * c_radius, 1.4 * c_radius));
        const c2 = Vector2.addVectors(pos2, new Vector2(- 0.8 * c_radius, 1.4 * c_radius));
        const middle_line_pos = Vector2.scaleVector(Vector2.addVectors(pos1, pos2), 1/2);
        const c3 = Vector2.addVectors(middle_line_pos, new Vector2(0, 1.4 * c_radius));

        c.beginPath();
        c.arc(c1.x, c1.y, c_radius, 0, 2 * Math.PI);
        c.fill();
        c.stroke();
        c.closePath();

        c.beginPath();
        c.arc(c2.x, c2.y, c_radius, 0, 2 * Math.PI);
        c.fill();
        c.stroke();
        c.closePath();

        c.beginPath();
        c.arc(c3.x, c3.y, c_radius, 0, 2 * Math.PI);
        c.fill();
        c.stroke();
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
        const canv_pos = Units.sim_canv(m_objects[this.p_id].pos);
        const radius = m_objects[this.p_id].drawing_radius * Units.scale_s_c;

        const lineWidth = 5;
        const borderWidth = 2;     
        c.lineCap = "round";  
        
        // vertical
        const ver_pos1 = Vector2.addVectors(canv_pos, new Vector2(0, - 2.6 * radius));
        const ver_pos2 = Vector2.addVectors(canv_pos, new Vector2(0,   2.6 * radius));

        c.beginPath();

        // border
        c.lineWidth = lineWidth;
        c.strokeStyle = "#000000";
        c.moveTo(ver_pos1.x, ver_pos1.y);
        c.lineTo(ver_pos2.x, ver_pos2.y);
        c.stroke();

        // int
        c.lineWidth = lineWidth - borderWidth;
        c.strokeStyle = "#FFFFFF";
        c.moveTo(ver_pos1.x, ver_pos1.y);
        c.lineTo(ver_pos2.x, ver_pos2.y);
        c.stroke();

        c.closePath();

        m_objects[this.p_id].render(c)


        // horizontal
        const hor_pos1 = Vector2.addVectors(canv_pos, new Vector2(- 0.97 * radius, 0));
        const hor_pos2 = Vector2.addVectors(canv_pos, new Vector2(  0.97 * radius, 0));

        c.beginPath();

        // border
        c.lineWidth = lineWidth;
        c.strokeStyle = "#000000";
        c.moveTo(hor_pos1.x, hor_pos1.y);
        c.lineTo(hor_pos2.x, hor_pos2.y);
        c.stroke();

        // int
        c.lineWidth = lineWidth - borderWidth;
        c.strokeStyle = "#FFFFFF";
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

export class LineConstraint {
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
        const lineWidth = 10;
        const borderWidth = 4;     
        c.lineCap = 'round';       

        const pos1 = Units.sim_canv(m_objects[this.id1].pos);
        const pos2 = Units.sim_canv(m_objects[this.id2].pos);
        const dir = (Vector2.subtractVectors(pos1, pos2)).normalized();

        c.beginPath();

        // border
        c.lineWidth = lineWidth;
        c.strokeStyle = "#000000";
        c.moveTo(pos1.x, pos1.y);
        c.lineTo(pos2.x, pos2.y);
        c.stroke();

        // interiour

        const c_value = 4 * Math.abs(lagrange_mult); // Math.abs(dot);
        const color = lagrange_mult < 0
            ? "rgba(" + (255 - c_value) + ", " + (255 - 0.02 * c_value) + ", 255, 1)" :
              "rgba(255, " + (255 - c_value) + ", " + (255 - c_value) + ", " + (255 - 0.02 * c_value) + ")";


        c.lineWidth = lineWidth - borderWidth;
        c.strokeStyle = color;
        c.moveTo(pos1.x, pos1.y);
        c.lineTo(pos2.x, pos2.y);
        c.stroke();

        c.closePath();

        m_objects[this.id1].render(c);
        m_objects[this.id2].render(c);
    
    }
}