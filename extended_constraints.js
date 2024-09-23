import {SparseMatrixBlock, SparseMatrix, Vector, Vector2} from "./linear_algebra.js";
import {Units} from "./main.js";
import {Colours, LineWidths, Extras} from "./render_settings.js";

export class FixedPosConstraint {
    constructor(id, pos) {
        this.id = id;
        this.x0 = pos.x;
        this.y0 = pos.y;
    }

    render(c, m_objects, lagrange_mult) {
        const obj = m_objects[this.id];

        const obj_rad = obj.drawing_radius;

        const connection_circle_rad = Units.scale_s_c * obj_rad / 3;

        // const correction_edges = Units.scale_c_s * (0.5*Colours.INNER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT + Colours.INNER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT - Colours.OUTER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT);
        const correction_edges = 0;

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

    }
}