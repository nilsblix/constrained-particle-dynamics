import {Vector2} from "./linear_algebra.js";
import {PhysicsState} from "./physicsState.js";
import {DynamicObject} from "./dynamicObject.js"
import {FixedXConstraint, FixedYConstraint, LineConstraint} from "./numeric_constraints.js";

const canvas = document.getElementById("myCanvas");
const c = canvas.getContext("2d");

const x_offset = 30;
const y_offset = 20;
canvas.width = window.innerWidth - x_offset;
// canvas.height = window.innerHeight - y_offset;

export class Units {
    static WIDTH = 10;
    static RATIO = 16 / 9;
    static HEIGHT = this.WIDTH / this.RATIO;

    static scale_c_s = this.WIDTH / canvas.width;
    static scale_s_c = canvas.width / this.WIDTH;

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
}

canvas.height = Units.scale_s_c * (Units.HEIGHT) - y_offset;


/* void main() {
    return 0;
}
*/

var physicsState = new PhysicsState();
let solver = {
    dt: 1 / 60,
    sim_steps: 8,
    simulating: false,
    physics_frame_time: -1,
    render_frame_time: -1,
};

let mouse = {
    canv_pos: new Vector2(0,0),
    on_canvas: false,
    left_down:  false,

    toString() {
        return "canv_pos: " + this.canv_pos + " on_canvas: " + this.on_canvas + " left_down: " + this.left_down;
    }
}

function updateDisplayedDebugs() {
    if (!solver.simulating)
        solver.physics_frame_time = -1;

    document.getElementById("physics_frame_time").innerHTML = solver.physics_frame_time.toFixed(1);
    document.getElementById("render_frame_time").innerHTML = solver.render_frame_time.toFixed(3);
    document.getElementById("CFS_ms").innerHTML = physicsState.CFS_ms.toFixed(3);
    document.getElementById("CFS_accumulated_error").innerHTML = physicsState.CFS_accumulated_error.toFixed(3);
    document.getElementById("system_energy").innerHTML = physicsState.system_energy.toFixed(3);
    document.getElementById("C_value").innerHTML = physicsState.C_value.toFixed(3);
    document.getElementById("C_dot_value").innerHTML = physicsState.C_dot_value.toFixed(3);
}

function setupScene(version) {
    switch (version) {
        case "test 1":

            const d_radius = 0.1;

            const h = Units.HEIGHT / 2 + 0.02;

            const l0 = 0.4;
                
            const id_0 = new DynamicObject(new Vector2(5, h), 1, d_radius);
            const id_1 = new DynamicObject(new Vector2(5 - l0, h), 1, d_radius);
            const id_2 = new DynamicObject(new Vector2(5 - l0, h + l0), 1, d_radius);
            const id_3 = new DynamicObject(new Vector2(5 - 2 * l0, h + l0), 1, d_radius);

            physicsState.addObject(id_0);
            physicsState.addObject(id_1);
            physicsState.addObject(id_2);
            physicsState.addObject(id_3);

            physicsState.addConstraint(new FixedYConstraint(0, id_0.pos.y));
            physicsState.addConstraint(new FixedXConstraint(0, id_0.pos.x));

            // physicsState.addConstraint(new FixedXConstraint(3, id_3.pos.x));

            // physicsState.addConstraint(new FixedXConstraint(1, id_1.pos.x));

            // physicsState.addConstraint(new FixedYConstraint(2, id_2.pos.y));
            // physicsState.addConstraint(new FixedXConstraint(2, id_2.pos.x));

            physicsState.addConstraint(new LineConstraint(0, 1, Vector2.distance(id_0.pos, id_1.pos)));
            physicsState.addConstraint(new LineConstraint(1, 2, Vector2.distance(id_1.pos, id_2.pos)));
            physicsState.addConstraint(new LineConstraint(2, 3, Vector2.distance(id_2.pos, id_3.pos)));

            break;
    
        case "test 2":

            const d_radius_2 = 0.2;
            const height = Units.HEIGHT / 2;
            const l0_2 = 0.8;

            const d0 = new DynamicObject(new Vector2(4, height), 1, d_radius_2);
            const d1 = new DynamicObject(new Vector2(6, height), 1, d_radius_2);
            const d2 = new DynamicObject(new Vector2(5, height + 1), 1, d_radius_2);
            const d3 = new DynamicObject(new Vector2(5 + 0.5, height + 1 + l0_2), 1, d_radius_2);
            const d4 = new DynamicObject(new Vector2(5 + 2 * 0.03, height + 1 + 2 * l0_2), 1, d_radius_2);

            physicsState.addObject(d0);
            physicsState.addObject(d1);
            physicsState.addObject(d2);
            physicsState.addObject(d3);
            physicsState.addObject(d4);

            physicsState.addFixedPosConstraint(0);
            // physicsState.addConstraint(new FixedYConstraint(1, d1.pos.y));

            physicsState.addConstraint(new LineConstraint(0, 1, Vector2.distance(d0.pos, d1.pos)));
            physicsState.addConstraint(new LineConstraint(0, 2, Vector2.distance(d0.pos, d2.pos)));
            physicsState.addConstraint(new LineConstraint(1, 2, Vector2.distance(d1.pos, d2.pos)));
            physicsState.addConstraint(new LineConstraint(2, 3, Vector2.distance(d2.pos, d3.pos)));
            physicsState.addConstraint(new LineConstraint(3, 4, Vector2.distance(d3.pos, d4.pos)));

            // physicsState.addLineConstraint(0, 1);
            // physicsState.addLineConstraint(0, 2);
            // physicsState.addLineConstraint(1, 2);
            // physicsState.addLineConstraint(2, 3);
            // physicsState.addLineConstraint(3, 4);


            break;
        
        case "truss":

            const case_truss_radius = 0.07;
            const height_truss = Units.HEIGHT / 2;

            const t0 = new DynamicObject(new Vector2(2, height_truss), 1, case_truss_radius);
            const t1 = new DynamicObject(new Vector2(3, height_truss), 1, case_truss_radius);
            const t2 = new DynamicObject(new Vector2(3, 1 + height_truss), 1, case_truss_radius);
            const t3 = new DynamicObject(new Vector2(4, height_truss), 1, case_truss_radius);
            const t4 = new DynamicObject(new Vector2(4, 1 + height_truss), 1, case_truss_radius);
            const t5 = new DynamicObject(new Vector2(5, height_truss), 1, case_truss_radius);
            const t6 = new DynamicObject(new Vector2(5, 1 + height_truss), 1, case_truss_radius);
            const t7 = new DynamicObject(new Vector2(6, height_truss), 1, case_truss_radius);
            const t8 = new DynamicObject(new Vector2(6, 1 + height_truss), 1, case_truss_radius);
            const t9 = new DynamicObject(new Vector2(7, height_truss), 1, case_truss_radius);
            const t10 = new DynamicObject(new Vector2(7, 1 + height_truss), 1, case_truss_radius);
            const t11 = new DynamicObject(new Vector2(8, height_truss), 1, case_truss_radius);

            // const t12 = new DynamicObject(new Vector2(8.3, height_truss), 1, case_truss_radius);
            // const t13 = new DynamicObject(new Vector2(8.6, height_truss), 1, case_truss_radius);
            // const t14 = new DynamicObject(new Vector2(8.9, height_truss), 1, case_truss_radius);
            // const t15 = new DynamicObject(new Vector2(9.2, height_truss), 1, case_truss_radius);
            // const t16 = new DynamicObject(new Vector2(9.8, height_truss), 1, 3 * case_truss_radius);


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
            // physicsState.addObject(t12);
            // physicsState.addObject(t13);
            // physicsState.addObject(t14);
            // physicsState.addObject(t15);
            // physicsState.addObject(t16);

            // physicsState.addConstraint(new FixedXConstraint(0, t0.pos.x));
            physicsState.addConstraint(new FixedYConstraint(0, t0.pos.y));

            physicsState.addConstraint(new LineConstraint(1, 2, Vector2.distance(t1.pos, t2.pos)));
            physicsState.addConstraint(new LineConstraint(3, 4, Vector2.distance(t3.pos, t4.pos)));
            physicsState.addConstraint(new LineConstraint(5, 6, Vector2.distance(t5.pos, t6.pos)));
            physicsState.addConstraint(new LineConstraint(7, 8, Vector2.distance(t7.pos, t8.pos)));
            physicsState.addConstraint(new LineConstraint(9, 10, Vector2.distance(t9.pos, t10.pos)));
            physicsState.addConstraint(new LineConstraint(0, 1, Vector2.distance(t0.pos, t1.pos)));
            physicsState.addConstraint(new LineConstraint(1, 3, Vector2.distance(t1.pos, t3.pos)));
            physicsState.addConstraint(new LineConstraint(3, 5, Vector2.distance(t3.pos, t5.pos)));
            physicsState.addConstraint(new LineConstraint(5, 7, Vector2.distance(t5.pos, t7.pos)));
            physicsState.addConstraint(new LineConstraint(7, 9, Vector2.distance(t7.pos, t9.pos)));
            physicsState.addConstraint(new LineConstraint(9, 11, Vector2.distance(t9.pos, t11.pos)));
            physicsState.addConstraint(new LineConstraint(0, 2, Vector2.distance(t0.pos, t2.pos)));
            physicsState.addConstraint(new LineConstraint(2, 4, Vector2.distance(t2.pos, t4.pos)));
            physicsState.addConstraint(new LineConstraint(4, 6, Vector2.distance(t4.pos, t6.pos)));
            physicsState.addConstraint(new LineConstraint(6, 8, Vector2.distance(t6.pos, t8.pos)));
            physicsState.addConstraint(new LineConstraint(8, 10, Vector2.distance(t8.pos, t10.pos)));
            physicsState.addConstraint(new LineConstraint(10, 11, Vector2.distance(t10.pos, t11.pos)));
            physicsState.addConstraint(new LineConstraint(2, 3, Vector2.distance(t2.pos, t3.pos)));
            physicsState.addConstraint(new LineConstraint(4, 5, Vector2.distance(t4.pos, t5.pos)));
            physicsState.addConstraint(new LineConstraint(5, 8, Vector2.distance(t5.pos, t8.pos)));
            physicsState.addConstraint(new LineConstraint(7, 10, Vector2.distance(t7.pos, t10.pos)));
            
            // physicsState.addConstraint(new LineConstraint(11, 12, Vector2.distance(t11.pos, t12.pos)));
            // physicsState.addConstraint(new LineConstraint(12, 13, Vector2.distance(t12.pos, t13.pos)));
            // physicsState.addConstraint(new LineConstraint(13, 14, Vector2.distance(t13.pos, t14.pos)));
            // physicsState.addConstraint(new LineConstraint(14, 15, Vector2.distance(t14.pos, t15.pos)));
            // physicsState.addConstraint(new LineConstraint(15, 16, Vector2.distance(t15.pos, t16.pos)));

            break;
        }
}

function renderBackground() {
    // colors
    const backgroundColor = "rgba(64, 64, 64, 1)";
    const big_line_color = "rgba(120, 120, 120, 1)"
    const small_line_color = "rgba(98, 98, 98, 1)"
    // vars
    const lines_x = 20;
    const lines_y = Math.floor(lines_x / Units.RATIO);

    c.beginPath();
    c.fillStyle = backgroundColor;
    c.rect(0, 0, canvas.width, canvas.height);
    c.fill();
    c.closePath();

    // small lines
    c.strokeStyle = small_line_color;
    c.lineWidth = 1;
    for (let i = 0; i < lines_x; i++) {
        const clip_space = i / lines_x;
        const delta = 0.5 * 1 / lines_x;
        c.beginPath();
        c.moveTo((clip_space + delta) * canvas.width, 0);
        c.lineTo((clip_space + delta) * canvas.width, canvas.height);
        c.stroke();
        c.closePath();
    }

    for (let i = 0; i < lines_y; i++) {
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
    c.lineWidth = 1.25;
    for (let i = 0; i < lines_x; i++) {
        const clip_space = i / lines_x;
        c.beginPath();
        c.moveTo(clip_space * canvas.width, 0);
        c.lineTo(clip_space * canvas.width, canvas.height);
        c.stroke();
        c.closePath();
    }

    for (let i = 0; i < lines_y; i++) {
        const clip_space = i / lines_y;
        c.beginPath();
        c.moveTo(0, clip_space * canvas.height);
        c.lineTo(canvas.width, clip_space * canvas.height);
        c.stroke();
        c.closePath();
    }

}

function start() {
    setupScene("truss");
    solver.simulating = false;
    physicsState.initConstraintManager();
}

function update() {

    // console.log(mouse.toString());
    if (mouse.on_canvas)
        physicsState.updateMouse(Units.canv_sim(mouse.canv_pos), mouse.left_down)

    // physics
    if (solver.simulating) {
        let p_st = performance.now();
            physicsState.step_simulation(solver.dt, solver.sim_steps);
        let p_et = performance.now();
        solver.physics_frame_time = p_et - p_st;
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
    if (event.key == "ArrowRight" && !solver.simulating) {
        let p_st = performance.now();
            physicsState.step_simulation(solver.dt / solver.sim_steps, 1);
        let p_et = performance.now();
        solver.physics_frame_time = p_et - p_st;
    }
});

canvas.addEventListener("mouseenter", function(event) {
    mouse.on_canvas = true;
});

canvas.addEventListener("mouseleave", function(event) {
    mouse.on_canvas = false;
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