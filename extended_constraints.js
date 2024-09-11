import {SparseMatrixBlock, SparseMatrix, Vector, Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";

export class FixedPosConstraint {
    constructor(id, pos) {
        this.id = id;
        this.x0 = pos.x;
        this.y0 = pos.y;
    }

    render(c, m_objects, lagrange_mult) {
        const obj = m_objects[this.id];

        const line_thickness = 3;
        const border_thickness = 2;
        const obj_rad = obj.drawing_radius;

        const connection_circle_rad = Units.scale_s_c * obj_rad / 3;

        const correction_edges = Units.scale_c_s * (0.5*line_thickness + border_thickness);

        const horizontal_pos2 = Vector2.addVectors(obj.pos, new Vector2(0, - correction_edges - obj_rad));
        const horizontal_pos1 = Vector2.addVectors(horizontal_pos2, new Vector2(-1.8 * obj_rad, 0));
        const horizontal_pos3 = Vector2.addVectors(horizontal_pos2, new Vector2( 1.8 * obj_rad, 0));

        const connection_obj_pos1 = Vector2.addVectors(horizontal_pos2, new Vector2(- Units.scale_c_s * connection_circle_rad, 0));
        const connection_obj_pos2 = Vector2.addVectors(obj.pos, new Vector2(- Units.scale_c_s * connection_circle_rad, 0));
        const connection_obj_pos3 = Vector2.addVectors(obj.pos, new Vector2(Units.scale_c_s * connection_circle_rad, 0));
        const connection_obj_pos4 = Vector2.addVectors(horizontal_pos2, new Vector2(Units.scale_c_s * connection_circle_rad, 0));
        
        obj.render(c);

        // draw the connecting things
        // settings
        c.fillStyle = "#FFFFFF";
        c.strokeStyle = "#000000";
        // non adjustable settings:
        c.lineWidth = border_thickness;
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
        c.strokeStyle = "#000000";
        c.lineCap = "round";
        // non adjustable settings
        c.lineWidth = line_thickness + border_thickness;
        // draw commands
        // border
        c.beginPath();
        c.moveTo(Units.sim_canv_x(horizontal_pos1), Units.sim_canv_y(horizontal_pos1));
        c.lineTo(Units.sim_canv_x(horizontal_pos3), Units.sim_canv_y(horizontal_pos3));
        c.stroke();
        // c.fill();
        c.closePath();
        // fill (inner)
        c.lineWidth = line_thickness;
        c.strokeStyle = "#FFFFFF";
        c.beginPath();
        c.moveTo(Units.sim_canv_x(horizontal_pos1), Units.sim_canv_y(horizontal_pos1));
        c.lineTo(Units.sim_canv_x(horizontal_pos3), Units.sim_canv_y(horizontal_pos3));
        c.stroke();
        // c.fill();
        c.closePath();

    }
}