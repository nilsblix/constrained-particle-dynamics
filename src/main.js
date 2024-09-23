import {Vector2} from "./linear_algebra.js";
import {PhysicsState} from "./physicsState.js";
import {DynamicObject} from "./dynamicObject.js"

const canvas = document.getElementById("myCanvas");
const c = canvas.getContext("2d");

const x_offset = 190;
// const y_offset = 80;
canvas.width = window.innerWidth - x_offset;
// canvas.height = window.innerHeight - y_offset;

export class Units {
    static WIDTH = 10;
    static RATIO = 16 / 9;
    static HEIGHT = this.WIDTH / this.RATIO;

    static scale_c_s = this.WIDTH / canvas.width;
    static scale_s_c = canvas.width / this.WIDTH;

    static render_num_lines_x = 20;
    static render_num_lines_y = this.render_num_lines_x / Units.RATIO;

    static canv_sim_x(pos) {
        return this.WIDTH * pos.x / canvas.width;
    }

    static canv_sim_y(pos) {
        return this.HEIGHT * (canvas.height - pos.y) / canvas.height;
    }

    static canv_sim(pos) {
        return new Vector2(this.canv_sim_x(pos), this.canv_sim_y(pos));
    }

    static sim_canv_x(pos) {
        return canvas.width * pos.x / this.WIDTH;
    }

    static sim_canv_y(pos) {
        return canvas.height * (this.HEIGHT - pos.y) / this.HEIGHT;
    }

    static sim_canv(pos) {
        return new Vector2(this.sim_canv_x(pos), this.sim_canv_y(pos));
    }

    static snap_to_grid_x(pos) {
        const l_x = 2 * this.render_num_lines_x;
        const x = pos.x + 0.5 * this.WIDTH / l_x;
        return this.WIDTH / l_x * Math.floor(l_x / this.WIDTH * x);
    }

    static snap_to_grid_y(pos) {
        const l_y = 2 * this.render_num_lines_y;
        const y = pos.y + 0.5 * this.HEIGHT / l_y;
        return this.HEIGHT / l_y * Math.floor(l_y / this.HEIGHT * y);
    }

    static snap_to_grid(pos) {
        return new Vector2(this.snap_to_grid_x(pos), this.snap_to_grid_y(pos));
    }
}

// canvas.height = Units.scale_s_c * (Units.HEIGHT) - y_offset;
canvas.height = Units.scale_s_c * (Units.HEIGHT);


/* 
int main() {
    return 0;
}
*/

var physicsState = new PhysicsState();
let solver = {
    dt: 1 / 60,
    sim_steps: 8,
    simulating: false,
    physics_frame_time: -1,
    averaged_physics_frame_time: -1,
    render_frame_time: -1,
    standard_radius: 0.05,
};

let physic_entites_manager = {
    active: false,
    snap_to_grid: false,
    line_start_id: -1,
    drawing_line: false,
    drawing_spring_joint: false,
    recent_entities: [],

    dynamicObject() {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const pos = this.snap_to_grid ? Units.snap_to_grid(mouse_sim_pos) : mouse_sim_pos;
        physicsState.addObject(new DynamicObject(pos, 1, solver.standard_radius));
        const length = physicsState.getDynamicObjectsLength();
        this.recent_entities.push({type: "DynamicObject", id: length - 1});
    },

    fixedXConstraint() {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        physicsState.addFixedXConstraint(id);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});
    },

    fixedYConstraint() {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        physicsState.addFixedYConstraint(id);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});
    },

    fixedPosConstraint() {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        physicsState.addFixedPosConstraint(id);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});
    },

    removeMostRecentEntity() {
        // if no recent entities exist, dont do anything
        if (this.recent_entities.length == 0)
            return;

        const last_entity = this.recent_entities.pop();
        if (last_entity.type == "DynamicObject")
            physicsState.removeByIdDynamicObject(last_entity.id);
        if (last_entity.type == "ForceGenerator")
            physicsState.removeByIdForceGenerator(last_entity.id);
        if (last_entity.type == "Constraint")
            physicsState.removeLastConstraint();

    }
}

let mouse = {
    canv_pos: new Vector2(0,0),
    on_canvas: false,
    left_down:  false,

    toString() {
        return "canv_pos: " + this.canv_pos + " on_canvas: " + this.on_canvas + " left_down: " + this.left_down;
    }
}

let keyboard = {
    arrow_up:  false,
}

function updateDisplayedDebugs() {
    if (!solver.simulating)
        solver.physics_frame_time = -1;

    document.getElementById("physics_frame_time").innerHTML = solver.averaged_physics_frame_time.toFixed(1);
    document.getElementById("render_frame_time").innerHTML = solver.render_frame_time.toFixed(1);
    document.getElementById("CFS_ms").innerHTML = physicsState.CFS_ms.toFixed(1);
    document.getElementById("CFS_accumulated_error").innerHTML = physicsState.CFS_accumulated_error.toFixed(2);
    document.getElementById("system_energy").innerHTML = physicsState.system_energy.toFixed(3);
    document.getElementById("C_value").innerHTML = physicsState.C_value.toFixed(5);
    document.getElementById("C_dot_value").innerHTML = physicsState.C_dot_value.toFixed(5);
}

function updateSliderValues() {
    const options = document.getElementById("option");
    const slider = document.getElementById("rangeSlider");

    const selected = options.value;
    if (selected == "gravity") {
        physicsState.setGravity(slider.value);
    } else if (selected == "linear damping") {
        physicsState.setLinearDampingMU(slider.value);
    } else if (selected == "mouse spring") {
        physicsState.setMouseSpringStiffness(slider.value);
    } else if (selected == "spring joint") {
        physicsState.setSpringJointStiffness(slider.value);
    }
}

function setupScene(version) {
    switch (version) {
        case "null":
            break;

        case "pratt truss":

            const pratt_truss_radius = 0.05;
            const height_pratt_truss = Units.HEIGHT / 2;

            const pratt_p_delta = 0.6;

            const t0 = new DynamicObject(new Vector2(2, height_pratt_truss), 1, pratt_truss_radius);
            const t1 = new DynamicObject(new Vector2(3, height_pratt_truss), 1, pratt_truss_radius);
            const t2 = new DynamicObject(new Vector2(3, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t3 = new DynamicObject(new Vector2(4, height_pratt_truss), 1, pratt_truss_radius);
            const t4 = new DynamicObject(new Vector2(4, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t5 = new DynamicObject(new Vector2(5, height_pratt_truss), 1, pratt_truss_radius);
            const t6 = new DynamicObject(new Vector2(5, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t7 = new DynamicObject(new Vector2(6, height_pratt_truss), 1, pratt_truss_radius);
            const t8 = new DynamicObject(new Vector2(6, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t9 = new DynamicObject(new Vector2(7, height_pratt_truss), 1, pratt_truss_radius);
            const t10 = new DynamicObject(new Vector2(7, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t11 = new DynamicObject(new Vector2(8, height_pratt_truss), 1, pratt_truss_radius);
            
            const t12 = new DynamicObject(new Vector2(2, 2+height_pratt_truss), 1, 2*pratt_truss_radius);
            const t13 = new DynamicObject(new Vector2(8, 2+height_pratt_truss), 5, 3*pratt_truss_radius);

            const t14 = new DynamicObject(new Vector2(2, 1+height_pratt_truss), 1, pratt_truss_radius);
            const t15 = new DynamicObject(new Vector2(8, 1+height_pratt_truss), 1, pratt_truss_radius);

            const t16 = new DynamicObject(new Vector2(5, -pratt_p_delta+height_pratt_truss), 1, 1*pratt_truss_radius);
            const t17 = new DynamicObject(new Vector2(5, -2*pratt_p_delta+height_pratt_truss), 5, 2.5*pratt_truss_radius);




            physicsState.addObject(t0);
            physicsState.addObject(t1);
            physicsState.addObject(t2);
            physicsState.addObject(t3);
            physicsState.addObject(t4);
            physicsState.addObject(t5);
            physicsState.addObject(t6);
            physicsState.addObject(t7);
            physicsState.addObject(t8);
            physicsState.addObject(t9);
            physicsState.addObject(t10);
            physicsState.addObject(t11);

            physicsState.addObject(t12);
            physicsState.addObject(t13);

            physicsState.addObject(t14);
            physicsState.addObject(t15);

            physicsState.addObject(t16);
            physicsState.addObject(t17);

            physicsState.addFixedPosConstraint(12);
            physicsState.addFixedYConstraint(13);

            physicsState.addLineConstraint(0, 2);
            physicsState.addLineConstraint(2, 4);
            physicsState.addLineConstraint(4, 6);
            physicsState.addLineConstraint(6, 8);
            physicsState.addLineConstraint(8, 10);
            physicsState.addLineConstraint(10, 11);
            physicsState.addLineConstraint(2, 3);
            physicsState.addLineConstraint(4, 5);
            physicsState.addLineConstraint(5, 8);
            physicsState.addLineConstraint(7, 10);
            
            physicsState.addLineConstraint(1, 2);
            physicsState.addLineConstraint(3, 4);
            physicsState.addLineConstraint(5, 6);
            physicsState.addLineConstraint(7, 8);
            physicsState.addLineConstraint(9, 10);
            physicsState.addLineConstraint(0, 1);
            physicsState.addLineConstraint(1, 3);
            physicsState.addLineConstraint(3, 5);
            physicsState.addLineConstraint(5, 7);
            physicsState.addLineConstraint(7, 9);
            physicsState.addLineConstraint(9, 11);

            physicsState.addLineConstraint(0, 14);
            physicsState.addLineConstraint(12, 14);
            physicsState.addLineConstraint(11, 15);
            physicsState.addLineConstraint(15, 13);

            physicsState.addLineConstraint(5, 16);
            physicsState.addLineConstraint(16, 17);


            break;

        case "king post truss":

            const kp_truss_height = Units.HEIGHT / 2;
            const kp_truss_radius = 0.07;
            let delta_h = 0.8;

            const kp0 = new DynamicObject(new Vector2(3.5, kp_truss_height), 1);
            const kp1 = new DynamicObject(new Vector2(4.25, kp_truss_height), 1);
            const kp2 = new DynamicObject(new Vector2(5, kp_truss_height), 1);
            const kp3 = new DynamicObject(new Vector2(5.75, kp_truss_height), 1);
            const kp4 = new DynamicObject(new Vector2(6.5, kp_truss_height), 1);
            const kp5 = new DynamicObject(new Vector2(4.25, delta_h+kp_truss_height), 1);
            const kp6 = new DynamicObject(new Vector2(5.75, delta_h+kp_truss_height), 1);
            const kp7 = new DynamicObject(new Vector2(5, 2*delta_h+kp_truss_height), 1);

            const kp8 = new DynamicObject(new Vector2(5, -0.5 + kp_truss_height), 1);
            const kp9 = new DynamicObject(new Vector2(5, -1 + kp_truss_height), 1);
            const kp10 = new DynamicObject(new Vector2(5, -1.5 + kp_truss_height), 10, 0.15 + kp_truss_radius);


            physicsState.addObject(kp0);
            physicsState.addObject(kp1);
            physicsState.addObject(kp2);
            physicsState.addObject(kp3);
            physicsState.addObject(kp4);
            physicsState.addObject(kp5);
            physicsState.addObject(kp6);
            physicsState.addObject(kp7);

            physicsState.addObject(kp8);
            physicsState.addObject(kp9);
            physicsState.addObject(kp10);


            // physicsState.addConstraint(new FixedXConstraint(0, kp0.pos.x));
            // physicsState.addConstraint(new FixedYConstraint(0, kp0.pos.y));
            physicsState.addFixedPosConstraint(0);
            // physicsState.addFixedPosConstraint(0);
            physicsState.addFixedYConstraint(4);

            physicsState.addLineConstraint(0, 1);
            physicsState.addLineConstraint(1, 2);
            physicsState.addLineConstraint(2, 3);
            physicsState.addLineConstraint(3, 4);
            physicsState.addLineConstraint(0, 5);
            physicsState.addLineConstraint(1, 5);
            physicsState.addLineConstraint(2, 5);
            physicsState.addLineConstraint(2, 6);
            physicsState.addLineConstraint(3, 6);
            physicsState.addLineConstraint(4, 6);
            physicsState.addLineConstraint(5, 7);
            physicsState.addLineConstraint(2, 7);
            physicsState.addLineConstraint(6, 7);

            physicsState.addLineConstraint(2, 8);
            physicsState.addLineConstraint(8, 9);
            physicsState.addLineConstraint(9, 10);
            






            break;
        
        case "large bridge structure":

            const lbs_radius = 0.05;

            const p = [
            new DynamicObject(new Vector2(3.4, 1),   1, lbs_radius),
            new DynamicObject(new Vector2(3, 1.6),   1, lbs_radius),
            new DynamicObject(new Vector2(3.8, 1.6), 1, lbs_radius),
            new DynamicObject(new Vector2(3, 2.4),   1, lbs_radius),
            new DynamicObject(new Vector2(3.8, 2.4),   1, lbs_radius),
            new DynamicObject(new Vector2(3, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(3.8, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(3, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(3.8, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(4.6, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(4.6, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(5.4, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(5.4, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(6.2, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(6.2, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(7, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(7, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(7, 2.4),   1, lbs_radius),
            new DynamicObject(new Vector2(6.2, 2.4),   1, lbs_radius),
            new DynamicObject(new Vector2(7, 1.6),   1, lbs_radius),
            new DynamicObject(new Vector2(6.2, 1.6),   1, lbs_radius),
            new DynamicObject(new Vector2(6.6, 1),   1, lbs_radius),
            new DynamicObject(new Vector2(3.4, 4.8),   1, lbs_radius),
            new DynamicObject(new Vector2(6.6, 4.8),   1, lbs_radius),
            new DynamicObject(new Vector2(1.6, 3.6),   1, lbs_radius),
            new DynamicObject(new Vector2(1.6, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(1.6, 2.8),   1, lbs_radius),
            new DynamicObject(new Vector2(1.6, 2.4),   10, 4*lbs_radius)
            ]    

            for (let i = 0; i < p.length; i++) {
                physicsState.addObject(p[i]);
            }

            physicsState.addFixedPosConstraint(0);
            physicsState.addFixedYConstraint(21);

            physicsState.addLineConstraint(0, 1);
            physicsState.addLineConstraint(0, 2);
            physicsState.addLineConstraint(1, 2);
            //
            physicsState.addLineConstraint(1, 3);
            physicsState.addLineConstraint(1, 4);
            physicsState.addLineConstraint(2, 3);
            physicsState.addLineConstraint(2, 4);
            physicsState.addLineConstraint(3, 4);
            //
            physicsState.addLineConstraint(3, 5);
            physicsState.addLineConstraint(3, 6);
            physicsState.addLineConstraint(4, 5);
            physicsState.addLineConstraint(4, 6);
            physicsState.addLineConstraint(5, 6);
            //
            physicsState.addLineConstraint(5, 7);
            physicsState.addLineConstraint(5, 8);
            physicsState.addLineConstraint(6, 7);
            physicsState.addLineConstraint(6, 8);
            physicsState.addLineConstraint(7, 8);
            //
            physicsState.addLineConstraint(6, 10);
            physicsState.addLineConstraint(6, 9);
            physicsState.addLineConstraint(8, 9);
            physicsState.addLineConstraint(8, 10);
            physicsState.addLineConstraint(9, 10);
            //
            physicsState.addLineConstraint(9, 11);
            physicsState.addLineConstraint(9, 12);
            physicsState.addLineConstraint(10, 11);
            physicsState.addLineConstraint(10, 12);
            physicsState.addLineConstraint(11, 12);
            //
            physicsState.addLineConstraint(11, 13);
            physicsState.addLineConstraint(11, 14);
            physicsState.addLineConstraint(12, 13);
            physicsState.addLineConstraint(12, 14);
            physicsState.addLineConstraint(13, 14);
            //
            physicsState.addLineConstraint(13, 15);
            physicsState.addLineConstraint(13, 16);
            physicsState.addLineConstraint(14, 15);
            physicsState.addLineConstraint(14, 16);
            physicsState.addLineConstraint(15, 16);
            //
            physicsState.addLineConstraint(17, 13);
            physicsState.addLineConstraint(17, 15);
            physicsState.addLineConstraint(18, 13);
            physicsState.addLineConstraint(18, 15);
            physicsState.addLineConstraint(17, 18);
            //
            physicsState.addLineConstraint(19, 17);
            physicsState.addLineConstraint(19, 18);
            physicsState.addLineConstraint(20, 17);
            physicsState.addLineConstraint(20, 18);
            physicsState.addLineConstraint(19, 20);
            //
            physicsState.addLineConstraint(19, 21);
            physicsState.addLineConstraint(20, 21);
            //
            physicsState.addLineConstraint(7, 22);
            physicsState.addLineConstraint(8, 22);
            //
            physicsState.addLineConstraint(14, 23);
            physicsState.addLineConstraint(16, 23);
            //
            physicsState.addLineConstraint(5, 24);
            physicsState.addLineConstraint(7, 24);
            //
            physicsState.addLineConstraint(24, 25);
            physicsState.addLineConstraint(25, 26);
            physicsState.addLineConstraint(26, 27);



            break;
    
        case "crane structure" :
            const c0 = new DynamicObject(new Vector2(3, 0.5), 1, solver.standard_radius);
            const c1 = new DynamicObject(new Vector2(4, 0.5), 1, solver.standard_radius);
            const c2 = new DynamicObject(new Vector2(3, 1.5), 1, solver.standard_radius);
            const c3 = new DynamicObject(new Vector2(4, 1.5), 1, solver.standard_radius);
            const c4 = new DynamicObject(new Vector2(3, 2.5), 1, solver.standard_radius);
            const c5 = new DynamicObject(new Vector2(4, 2.5), 1, solver.standard_radius);
            const c6 = new DynamicObject(new Vector2(3, 3.5), 1, solver.standard_radius);
            const c7 = new DynamicObject(new Vector2(4, 3.5), 1, solver.standard_radius);
            const c8 = new DynamicObject(new Vector2(3, 4.5), 1, solver.standard_radius);
            const c9 = new DynamicObject(new Vector2(4, 4.5), 1, solver.standard_radius);
            const c10 = new DynamicObject(new Vector2(5, 3.5), 1, solver.standard_radius);
            const c11 = new DynamicObject(new Vector2(6, 3.5), 1, solver.standard_radius);
            const c12 = new DynamicObject(new Vector2(7, 3.5), 1, solver.standard_radius);
            const c13 = new DynamicObject(new Vector2(7, 4), 1, solver.standard_radius);
            const c14 = new DynamicObject(new Vector2(6, 4 + 1/6), 1, solver.standard_radius);
            const c15 = new DynamicObject(new Vector2(5, 4 + 1/3), 1, solver.standard_radius);
            const c16 = new DynamicObject(new Vector2(2, 3.5), 1, solver.standard_radius);
            const c17 = new DynamicObject(new Vector2(7, 3.2), 1, solver.standard_radius);
            const c18 = new DynamicObject(new Vector2(7, 2.9), 1, solver.standard_radius);
            const c19 = new DynamicObject(new Vector2(7, 2.6), 5, 3*solver.standard_radius);

            physicsState.addObject(c0);
            physicsState.addObject(c1);
            physicsState.addObject(c2);
            physicsState.addObject(c3);
            physicsState.addObject(c4);
            physicsState.addObject(c5);
            physicsState.addObject(c6);
            physicsState.addObject(c7);
            physicsState.addObject(c8);
            physicsState.addObject(c9);
            physicsState.addObject(c10);
            physicsState.addObject(c11);
            physicsState.addObject(c12);
            physicsState.addObject(c13);
            physicsState.addObject(c14);
            physicsState.addObject(c15);
            physicsState.addObject(c16);
            physicsState.addObject(c17);
            physicsState.addObject(c18);
            physicsState.addObject(c19);

            physicsState.addFixedPosConstraint(0);
            physicsState.addFixedYConstraint(1);
            // physicsState.addFixedPosConstraint(16);

            //
            physicsState.addLineConstraint(0, 1);
            //
            physicsState.addLineConstraint(0, 2);
            physicsState.addLineConstraint(1, 3);
            physicsState.addLineConstraint(0, 3);
            physicsState.addLineConstraint(2, 3);
            //
            physicsState.addLineConstraint(2, 4);
            physicsState.addLineConstraint(3, 5);
            physicsState.addLineConstraint(3, 4);
            physicsState.addLineConstraint(4, 5);
            //
            physicsState.addLineConstraint(4, 6);
            physicsState.addLineConstraint(5, 7);
            physicsState.addLineConstraint(4, 7);
            physicsState.addLineConstraint(6, 7);
            //
            physicsState.addLineConstraint(6, 8);
            physicsState.addLineConstraint(7, 9);
            physicsState.addLineConstraint(7, 8);
            physicsState.addLineConstraint(8, 9);
            // outer line on attachment
            physicsState.addLineConstraint(7, 10);
            physicsState.addLineConstraint(10, 11);
            physicsState.addLineConstraint(11, 12);
            physicsState.addLineConstraint(12, 13); // is vertical, also outer line
            physicsState.addLineConstraint(13, 14);
            physicsState.addLineConstraint(14, 15);
            physicsState.addLineConstraint(9, 15);
            // inner diagonals
            physicsState.addLineConstraint(7, 15);
            physicsState.addLineConstraint(11, 15);
            physicsState.addLineConstraint(11, 13);
            // inner verticals
            physicsState.addLineConstraint(10, 15);
            physicsState.addLineConstraint(11, 14);
            // left triangle
            physicsState.addLineConstraint(6, 16);
            physicsState.addLineConstraint(8, 16);
            // right, weight
            physicsState.addLineConstraint(12, 17);
            physicsState.addLineConstraint(17, 18);
            physicsState.addLineConstraint(18, 19);
            // left, weight












            break;
    }
}

function renderBackground() {
    // colors
    const backgroundColor = "#1a1e21";
    const big_line_color = "rgba(90, 90, 90, 1)"
    const small_line_color = "rgba(40, 40, 40, 1)"
    // vars
    const lines_x = Units.render_num_lines_x;
    const lines_y = Units.render_num_lines_y;

    c.beginPath();
    c.fillStyle = backgroundColor;
    c.rect(0, 0, canvas.width, canvas.height);
    c.fill();
    c.closePath();

    // small lines
    c.strokeStyle = small_line_color;
    c.lineWidth = 0.8;
    for (let i = 0; i < lines_x; i++) {
        const clip_space = i / lines_x;
        const delta = 0.5 * 1 / lines_x;
        c.beginPath();
        c.moveTo((clip_space + delta) * canvas.width, 0);
        c.lineTo((clip_space + delta) * canvas.width, canvas.height);
        c.stroke();
        c.closePath();
    }

    for (let i = lines_y; i > 0; i--) {
        const clip_space = i / lines_y;
        const delta = 0.5 * 1 / lines_y;
        c.beginPath();
        c.moveTo(0, (clip_space + delta) * canvas.height);
        c.lineTo(canvas.width, (clip_space + delta) * canvas.height);
        c.stroke();
        c.closePath();
    }

    // big lines
    c.strokeStyle = big_line_color;
    c.lineWidth = 0.8;
    for (let i = 0; i < lines_x; i++) {
        const clip_space = i / lines_x;
        c.beginPath();
        c.moveTo(clip_space * canvas.width, 0);
        c.lineTo(clip_space * canvas.width, canvas.height);
        c.stroke();
        c.closePath();
    }

    for (let i = lines_y; i > 0; i--) {
        const clip_space = i / lines_y;
        c.beginPath();
        c.moveTo(0, clip_space * canvas.height);
        c.lineTo(canvas.width, clip_space * canvas.height);
        c.stroke();
        c.closePath();
    }

    // draw when adding objects etc to the physics engine
    if (!physic_entites_manager.active)
        return;
    // settings:
    c.fillStyle = "rgba(156, 58, 58, 1)";
    c.strokeStyle = "#000000";
    c.lineWidth = 3;
    // constants
    const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
    if (physic_entites_manager.snap_to_grid) {
        const c_pos = Units.sim_canv(Units.snap_to_grid(mouse_sim_pos));
        c.beginPath();
        c.arc(c_pos.x, c_pos.y, 10, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();
    } else {
        c.beginPath();
        c.arc(mouse.canv_pos.x, mouse.canv_pos.y, 10, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();
    }

    // c.beginPath();
    // c.fillStyle = "#ff0000";
    // const c_pos_2 = Units.sim_canv(new Vector2(1,2));
    // c.arc(c_pos_2.x, c_pos_2.y, 10, 0, 2 * Math.PI);
    // c.fill();
    // c.closePath();

}

function start() {
    // setupScene("pratt truss");
    setupScene("king post truss");
    // setupScene("large bridge structure");
    // setupScene("crane structure");
    solver.simulating = false;
    physicsState.initConstraintManager();

    physicsState.setGravity(1.6);
    physicsState.setLinearDampingMU(0.1);
    physicsState.setMouseSpringStiffness(10);

    // stp is snap to grid
    console.log("Keybinds: Simulate: s, Step sim: LArr, SloMo: UpArr, Full Reset: R, Reset: r, Mousespring: LM, Interact Mode: i, STP: I, (while i) DO: 1, ConstX: x, ConstY: y, ConstP, p, ConstLine: l + LM")
}

let time_sum = 0;
let time_frames = 0;

function update() {

    updateSliderValues();

    // console.log(mouse.toString());
    if (mouse.on_canvas && !physic_entites_manager.active)
        physicsState.updateMouse(Units.canv_sim(mouse.canv_pos), mouse.left_down)

    // physics
    if (solver.simulating) {
        let p_st = performance.now();
            physicsState.step_simulation(solver.dt, solver.sim_steps);
        let p_et = performance.now();
        // solver.physics_frame_time = p_et - p_st;
        time_sum += p_et - p_st;
        time_frames++;

        if (time_frames > 10) {
            time_sum /= time_frames;
            solver.averaged_physics_frame_time = time_sum;
            time_sum = 0;
            time_frames = 0;
        }
    } else if (keyboard.arrow_up) {
        physicsState.step_simulation(solver.dt / solver.sim_steps, 1);
    }

    // render
    let r_st = performance.now();
        c.clearRect(0, 0, canvas.width, canvas.height);
        renderBackground();
        physicsState.render(c);
    let r_et = performance.now();
    solver.render_frame_time = r_et - r_st;

    //debugs
    updateDisplayedDebugs();

    requestAnimationFrame(update);
}

document.addEventListener("keydown", function(event) {
    if (event.key == "s") {
        solver.simulating = !solver.simulating;
    }
    if (event.key == "r") {
        physicsState = new PhysicsState();
        start();
    }
    if (event.key == "R") {
        physicsState = new PhysicsState();
        solver.simulating = false;
        physicsState.initConstraintManager();
        setupScene("null");
    }
    if (event.key == "ArrowRight" && !solver.simulating) {
        let p_st = performance.now();
            physicsState.step_simulation(solver.dt / solver.sim_steps, 1);
        let p_et = performance.now();
        solver.physics_frame_time = p_et - p_st;
    }
    if (event.key == "ArrowUp" && !solver.simulating) {
        keyboard.arrow_up = true;
    }
    // add to physics manager
    if (event.key == "i") {
        physic_entites_manager.active = !physic_entites_manager.active;
    }
    if (event.key == "I") {
        physic_entites_manager.snap_to_grid = !physic_entites_manager.snap_to_grid;
    }
    if (event.key == "1" && physic_entites_manager.active) {
        physic_entites_manager.dynamicObject();
    }
    if (event.key == "x" && physic_entites_manager.active) {
        physic_entites_manager.fixedXConstraint();
    }
    if (event.key == "y" && physic_entites_manager.active) {
        physic_entites_manager.fixedYConstraint();
    }
    if (event.key == "p" && physic_entites_manager.active) {
        physic_entites_manager.fixedPosConstraint();
    }
    if (event.key == "l" && physic_entites_manager.active) {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        const mouse_over_dynamicObject = (id != -1);
        if (mouse_over_dynamicObject && !physic_entites_manager.drawing_line) {
            physic_entites_manager.line_start_id = id;
            physic_entites_manager.drawing_line = true;

        }
    }
    if (event.key == "k" && physic_entites_manager.active) {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        const mouse_over_dynamicObject = (id != -1);
        if (mouse_over_dynamicObject && !physic_entites_manager.drawing_spring_joint) {
            physic_entites_manager.line_start_id = id;
            physic_entites_manager.drawing_spring_joint = true;
        }
    }
    if (event.key == "d" && physic_entites_manager.active) {
        physic_entites_manager.removeMostRecentEntity();
    }

});

document.addEventListener("keyup", function(event) {
    if (event.key == "ArrowUp" && !solver.simulating) {
        keyboard.arrow_up = false;
    }
    if (event.key == "l" && physic_entites_manager.active) {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        const mouse_over_dynamicObject = (id != -1);
        if (mouse_over_dynamicObject && physic_entites_manager.drawing_line) {
            if (id != physic_entites_manager.line_start_id) {
                physicsState.addLineConstraint(physic_entites_manager.line_start_id, id);
                const c_length = physicsState.getConstraintLength();
                physic_entites_manager.recent_entities.push({type: "Constraint", id: c_length - 1});
            }
            physic_entites_manager.drawing_line = false;
        }
    }
    if (event.key == "k" && physic_entites_manager.active) {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        const mouse_over_dynamicObject = (id != -1);
        if (mouse_over_dynamicObject && physic_entites_manager.drawing_spring_joint) {
            physicsState.addSpringJoint(physic_entites_manager.line_start_id, id);
            const gen_length = physicsState.getForceGeneratorsLength();
            physic_entites_manager.recent_entities.push({type: "ForceGenerator", id: gen_length - 1});
        }
        physic_entites_manager.drawing_spring_joint = false;
    }
});

canvas.addEventListener("mouseenter", function(event) {
    mouse.on_canvas = true;
});

canvas.addEventListener("mouseleave", function(event) {
    mouse.on_canvas = false;
    mouse.left_down = false;
});

canvas.addEventListener("mousedown", function(event) {
    mouse.left_down = true;
});

canvas.addEventListener("mouseup", function(event) {
    mouse.left_down = false;
});

canvas.addEventListener("mousemove", function(event) {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.x;
    const y = event.clientY - rect.y;
    mouse.canv_pos = new Vector2(x, y);
})



start();
update();