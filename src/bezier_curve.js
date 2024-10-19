import {Vector2} from "./linear_algebra.js";
import {Units} from "./units.js";
import {Colours, LineWidths, Extras} from "./render_settings.js";

export class CubicBezier {
    constructor(left_pos, right_pos, interactable_radius) {
        this.i0 = new InteractableObject(left_pos, interactable_radius);
        this.i1 = new InteractableObject(Vector2.addVectors(left_pos, new Vector2(0.5, 1)), interactable_radius);
        this.i2 = new InteractableObject(right_pos, interactable_radius);
        this.i3 = new InteractableObject(Vector2.addVectors(right_pos, new Vector2(-0.5, 1)), interactable_radius);
    }

    getPointOnCurve(t) { // t is a scalar between 0 and 1
        const term_1 = Vector2.scaleVector(this.i0.pos, (1 - t) ** 3);
        const term_2 = Vector2.scaleVector(this.i1.pos, 3 * t * (1 - t) ** 2);
        const term_3 = Vector2.scaleVector(this.i3.pos, 3 * t * t * (1 - t));
        const term_4 = Vector2.scaleVector(this.i2.pos, t ** 3);

        const a = Vector2.addVectors(term_1, term_2);
        const b = Vector2.addVectors(term_3, term_4);

        return Vector2.addVectors(a, b);
    }

    update(snap_to_grid, mouse) {
        this.i0.update(snap_to_grid, mouse);
        this.i1.update(snap_to_grid, mouse);
        this.i2.update(snap_to_grid, mouse);
        this.i3.update(snap_to_grid, mouse);
    }

    render(c, num_objects) {
        // border
        c.lineWidth = LineWidths.OUTER_CUBIC_BEZIER_LINE;
        c.strokeStyle = Colours.OUTER_CUBIC_BEZIER;
        for (let i = 0; i < Extras.CUBIC_BEZIER_NUM_SEGMENTS; i++) {
            const increment = 1 / Extras.CUBIC_BEZIER_NUM_SEGMENTS;
            const t = i / Extras.CUBIC_BEZIER_NUM_SEGMENTS;
            const pos1 = Units.sim_canv(this.getPointOnCurve(t));
            const pos2 = Units.sim_canv(this.getPointOnCurve(t + increment));

            c.beginPath();
            c.moveTo(pos1.x, pos1.y);
            c.lineTo(pos2.x, pos2.y);
            c.stroke();
            c.closePath();
        }
        
        // inner
        c.lineWidth = LineWidths.INNER_CUBIC_BEZIER_LINE;
        c.strokeStyle = Colours.INNER_CUBIC_BEZIER;
        for (let i = 0; i < Extras.CUBIC_BEZIER_NUM_SEGMENTS; i++) {
            const increment = 1 / Extras.CUBIC_BEZIER_NUM_SEGMENTS;
            const t = i / Extras.CUBIC_BEZIER_NUM_SEGMENTS;
            const pos1 = Units.sim_canv(this.getPointOnCurve(t));
            const pos2 = Units.sim_canv(this.getPointOnCurve(t + increment));

            c.beginPath();
            c.moveTo(pos1.x, pos1.y);
            c.lineTo(pos2.x, pos2.y);
            c.stroke();
            c.closePath();
        }

        this.i0.render(c);
        this.i2.render(c);

        // preview objects
        const n = num_objects;
        c.fillStyle = Colours.INNER_PREVIEW_BEZIER_OBJECTS;
        c.strokeStyle = Colours.OUTER_PREVIEW_BEZIER_OBJECTS; 
        c.lineWidth = LineWidths.DYNAMIC_OBJECT;
        // c.lineWidth = 1;
        for (let i = 0; i <= n; i++) {
            const t = i / n;
            const prev = Units.sim_canv(this.getPointOnCurve(t));
            // const prev = Units.sim_canv(new Vector2(5, 5));
            c.beginPath();
            c.arc(prev.x, prev.y, Units.scale_s_c * Extras.PREVIEW_BEZIER_OBJECTS_RADIUS, 0, 2 * Math.PI);
            c.fill();
            c.stroke();
            c.closePath();
        }

        // lines
        // line 1
        c.lineCap = "round";
        c.strokeStyle = Colours.INNER_CUBIC_BEZIER_SUPPORTING_LINES;
        c.lineWidth = LineWidths.INNER_CUBIC_BEZIER_LINE;
        c.beginPath();
        c.moveTo(Units.sim_canv_x(this.i0.pos), Units.sim_canv_y(this.i0.pos));
        c.lineTo(Units.sim_canv_x(this.i1.pos), Units.sim_canv_y(this.i1.pos));
        c.stroke();
        c.closePath();
        // line 2
        c.strokeStyle = Colours.INNER_CUBIC_BEZIER_SUPPORTING_LINES;
        c.lineWidth = LineWidths.INNER_CUBIC_BEZIER_LINE;
        c.beginPath();
        c.moveTo(Units.sim_canv_x(this.i2.pos), Units.sim_canv_y(this.i2.pos));
        c.lineTo(Units.sim_canv_x(this.i3.pos), Units.sim_canv_y(this.i3.pos));
        c.stroke();
        c.closePath();

        this.i1.render(c);
        this.i3.render(c);

    }

}

export class InteractableObject { // object lives in sim-space
    constructor(pos, radius = 0.1) {
        this.pos = pos;
        this.radius = radius;
        this.held_down = false;
        this.offset = new Vector2(0,0);
    }

    update(snap_to_grid, mouse) {
        // update if mouse has pressed on this object
        
        const transformed_pos = snap_to_grid ? Units.snap_to_grid(mouse.sim_pos) : mouse.sim_pos;
        const v = Vector2.subtractVectors(this.pos, mouse.sim_pos);
        const dist2 = v.sqr_magnitude();
        if (dist2 < this.radius * this.radius && mouse.left_down && !this.held_down) {
            if (snap_to_grid)
                this.offset = new Vector2(0,0);
            else 
                this.offset = v;
            this.held_down = true;
        }

        // update this position if it is still held down
        if (this.held_down) {
            this.pos = Vector2.addVectors(this.offset, transformed_pos);
        }

        // if not mouse left is pressed --> become free from mouse again
        if (!mouse.left_down && this.held_down) {
            this.offset = new Vector2(0,0);
            this.held_down = false;
        }
    }

    render(c) {
        const canv_pos = Units.sim_canv(this.pos);
        const radius = this.radius * Units.scale_s_c;

        const color = this.held_down ? Colours.INNER_INTERACTABLEOBJECT_HELD_DOWN : Colours.INNER_INTERACTABLEOBJECT;
    
        c.fillStyle = color;
        c.strokeStyle = Colours.OUTER_INTERACTABLEOBJECT;
        c.lineWidth = LineWidths.INTERACTABLEOBJECT;
            
        c.beginPath();
        c.arc(canv_pos.x, canv_pos.y, radius, 0, 2 * Math.PI);
        c.fill();
        c.stroke();
        c.closePath();       
    }
}