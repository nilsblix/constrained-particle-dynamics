import {Vector2} from linear_algebra.js;

let canvas = document.getElementById("myCanvas");
let c = canvas.getContext("2d");

const x_offset = 30;
const y_offset = 50;
canvas.width = window.innerWidth - x_offset;
canvas.height = window.innerHeight - y_offset;

class units {
    static WIDTH = 10;
    static HEIGHT = canvas.height * WIDTH / canvas.width;

    static canv_sim_x(pos) {
        return this.WIDTH * pos.x / canvas.width;
    }

    static canv_sim_y(pos) {
        return this.HEIGHT * (canvas.height - pos.y) / canvas.height;
    }

    static canv_sim(pos) {
        return new Vector2(canv_sim_x(pos), canv_sim_y(pos));
    }

    static sim_canv_x(pos) {
        return canvas.width * pos.x / this.WIDTH;
    }
}