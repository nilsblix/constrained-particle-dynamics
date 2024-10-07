import { Vector2 } from "./linear_algebra.js";

export function initUnits(canvas) {
    const x_offset = 20; // px
    canvas.width = window.innerWidth - x_offset;
    Units.init(canvas);
    canvas.height = Units.scale_s_c * Units.HEIGHT;
    Units.canvas.height = canvas.height;
}

export class Units {
    static WIDTH = 10;
    static RATIO = 16 / 9;
    static HEIGHT = this.WIDTH / this.RATIO;

    static scale_c_s = 0;
    static scale_s_c = 0;

    static canvas = { width: 0, height: 0 };

    static render_num_lines_x = 20;
    static render_num_lines_y = this.render_num_lines_x / Units.RATIO;

    static init(canvas) {
        this.scale_c_s = this.WIDTH / canvas.width;
        this.scale_s_c = canvas.width / this.WIDTH;

        this.canvas.width = canvas.width;
    }

    static canv_sim_x(pos) {
        return this.WIDTH * pos.x / this.canvas.width;
    }

    static canv_sim_y(pos) {
        return this.HEIGHT * (this.canvas.height - pos.y) / this.canvas.height;
    }

    static canv_sim(pos) {
        return new Vector2(this.canv_sim_x(pos), this.canv_sim_y(pos));
    }

    static sim_canv_x(pos) {
        return this.canvas.width * pos.x / this.WIDTH;
    }

    static sim_canv_y(pos) {
        return this.canvas.height * (this.HEIGHT - pos.y) / this.HEIGHT;
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