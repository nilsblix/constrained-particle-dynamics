import {SparseMatrixBlock, SparseMatrix, Vector, Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";

export class FixedPosConstraint {
    constructor(id, pos) {
        this.id = id;
        this.x0 = pos.x;
        this.y0 = pos.y;
    }

    render(c, m_objects, lagrange_mult) {
        const canv_pos = Units.sim_canv(m_objects[this.id].pos);
        const radius = m_objects[this.id].drawing_radius * Units.scale_s_c;

        const lineWidth = 4;
        const borderWidth = 1;     
        c.lineCap = "square";       

        const pos1 = Vector2.addVectors(canv_pos, new Vector2(- 2.6 * radius, 1.2 * radius));
        const pos2 = Vector2.addVectors(canv_pos, new Vector2(  2.6 * radius, 1.2 * radius));

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

        m_objects[this.id].render(c);

    }
}