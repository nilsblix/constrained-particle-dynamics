class Vector2 {
    constructor(x = 0.0, y = 0.0) {
        this.x = x;
        this.y = y;
    }

    add(v) {
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    static addVectors(a, b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    subtract(v) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    static subtractVectors(a, b) {
        return new Vector2(a.x - b.x, a.y - b.y);
    }

    mult(s) {
        this.x *= s;
        this.y *= s;
        return this;
    }

    static scaleVector(s) {
        return new Vector2(this.x * s, this.y * s);
    }

    sqr_magnitude() {
        return this.x * this.x + this.y * this.y;
    }

    magnitude() {
        if (this.x == 0 && this.y == 0) return 0;
        return Math.sqrt(this.sqr_magnitude());
    }

    dot(v) {
        return this.x * v.x + this.y * v.y;
    }

    toString() {
        return "x: " + this.x + " y: " + this.y;
    }

    equals(v) {
        return this.x == v.x && this.y == v.y;
    }
}

class Vector {

}

class SparseMatrix {

}