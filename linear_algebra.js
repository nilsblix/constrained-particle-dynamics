export class Vector2 {
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

    static scaleVector(v, s) {
        return new Vector2(v.x * s, v.y * s);
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
    constructor(elements) {
        this.elements = elements;
    }
    constructor(num_elements) {
        this.elements = new Float32Array(num_elements);
    }
    constructor() {
        this.elements = new Float32Array(0);
    }

    static add_vectors(a, b) {
        if (a.elements.length != b.elements.length) {
            throw new Error("Vector add: lengths are not the same");
        }
        let vec = new Vector(a.elements);
        for (let i = 0; i < vec.elements.length; i++) {
            vec.elements[i] = a.elements[i] + b.elements[i];
        }
        return vec;
    }

    static sub_vectors(a, b) {
        if (a.elements.length != b.elements.length) {
            throw new Error("Vector add: lengths are not the same");
        }
        let vec = new Vector(a.elements);
        for (let i = 0; i < vec.elements.length; i++) {
            vec.elements[i] = a.elements[i] - b.elements[i];
        }
        return vec;
    }

    static elem_by_elem_mult_vectors(a, b) {
        if (a.elements.length != b.elements.length) {
            throw new Error("Vector add: lengths are not the same");
        }
        let vec = new Vector(a.elements);
        for (let i = 0; i < vec.elements.length; i++) {
            vec.elements[i] = a.elements[i] * b.elements[i];
        }
        return vec;
    }

    static scale_vector(s, x) {
        let vec = new Vector(x.elements.length);
        vec.zeroVector();
        for (let i = 0; i < vec.elements.length; i++) {
            vec.elements[i] = s * x.elements[i];
        }
        return vec;
    }

    static dot(a, b) {
        if (a.elements.length != b.elements.length) {
            throw new Error("Vector add: lengths are not the same");
        }
        let sum = 0;
        for (let i = 0; i < vec.elements.length; i++) {
            sum += a.elements[i] * b.elements[i];
        }
        return sum;
    }

    magnitude() {
        return Math.sqrt(this.sqr_magnitude());
    }

    sqr_magnitude() {
        let sum = 0;
        for (let i = 0; i < this.elements.length; i++) {
            sum += this.elements[i] * this.elements[i];
        }
        return sum;
    }

    zeroVector() {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i] = 0;
        }
    }

    negate() {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i] *= -1;
        }
    }

    static get_negated(x) {
        let b = new Vector(x.elements.length);
        for (let i = 0; i < b.elements.length; i++) {
            b.elements[i] = - x.elements[i];
        }
        return b;
    }

}

export class SparseMatrixBlock {
    constructor(i, j, data) {
        this.i = i;
        this.j = j;
        this.data = data;
    }
}

export class SparseMatrix {
    constructor(i, j, elements) {
        this.elements = elements;
        this.i_length = i;
        this.j_length = j;
    }
    constructor(i, j) {
        this.elements = [];
        this.i_length = i;
        this.j_length = j;
    }
    constructor() {
        this.elements = [];
        this.i_length = 0;
        this.j_length = 0;
    }

    static mat_mult_vec(A, x) {
        if (A.j_length != x.elements.length) {
            throw new Error("mat_mult_vec: lengths are not the same");
        }

        let b = new Vector(A.i_length);
        b.zeroVector();
        for (let idx = 0; idx < A.elements.length; i++) {
            let j_index = A.elements[idx].j;
            let i_index = A.elements[idx].i;

            b.elements[i_index] += A.elements[idx].data * x.elements[j_index];
        }
        return b;
    }

    static matT_mult_vec(A, x) {
        if (A.j_length != x.elements.length) {
            throw new Error("matT_mult_vec: lengths are not the same");
        }

        let b = new Vector(A.i_length);
        b.zeroVector();
        for (let idx = 0; idx < A.elements.length; i++) {
            let j_index = A.elements[idx].j;
            let i_index = A.elements[idx].i;

            b.elements[j_index] += A.elements[idx].data * x.elements[i_index];
        }
        return b;
    }
}