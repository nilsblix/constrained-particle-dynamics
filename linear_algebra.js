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

    static distance(a, b) {
        return (this.subtractVectors(a, b)).magnitude();
    }

    negated() {
        return new Vector2(-this.x, -this.y);
    }

    normalized() {
        const dist = this.magnitude();
        return Vector2.scaleVector(this, 1 / dist);
    }

    toString() {
        return "x: " + this.x + " y: " + this.y;
    }

    equals(v) {
        return this.x == v.x && this.y == v.y;
    }
}

export class Vector {
    constructor(num_elements) {
        this.elements = new Float32Array(num_elements);
    }

    clone() {
        const clonedVector = new Vector(this.elements.length);
        for (let i = 0; i < this.elements.length; i++) {
            clonedVector.elements[i] = this.elements[i];
        }
        return clonedVector;
    }

    static add_vectors(a, b) {
        if (a.elements.length != b.elements.length) {
            throw new Error("Vector add_vectors: lengths are not the same");
        }
        let vec = new Vector(a.elements.length);
        for (let i = 0; i < vec.elements.length; i++) {
            vec.elements[i] = a.elements[i] + b.elements[i];
        }
        return vec;
    }

    static sub_vectors(a, b) {
        if (a.elements.length != b.elements.length) {
            throw new Error("Vector sub_vectors: lengths are not the same");
        }
        let vec = new Vector(a.elements.length);
        for (let i = 0; i < vec.elements.length; i++) {
            vec.elements[i] = a.elements[i] - b.elements[i];
        }
        return vec;
    }

    static elem_by_elem_mult_vec(a, b) {
        if (a.elements.length != b.elements.length) {
            throw new Error("Vector elem_by_elem_mult: lengths are not the same");
        }
        let vec = new Vector(a.elements.length);
        for (let i = 0; i < vec.elements.length; i++) {
            vec.elements[i] = a.elements[i] * b.elements[i];
        }
        return vec;
    }

    static scale_vector(s, x) {
        let vec = new Vector(x.elements.length);
        for (let i = 0; i < vec.elements.length; i++) {
            vec.elements[i] = s * x.elements[i];
        }
        return vec;
    }

    static dot(a, b) {
        if (a.elements.length != b.elements.length) {
            throw new Error("Vector dot: lengths are not the same");
        }
        let sum = 0;
        for (let i = 0; i < a.elements.length; i++) {
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
            sum += this.elements[i] ** 2;
        }
        return sum;
    }

    static get_negated(x) {
        let b = new Vector(x.elements.length);
        for (let i = 0; i < b.elements.length; i++) {
            b.elements[i] = - x.elements[i];
        }
        return b;
    }

    static set_zero_vector(v) {
        let vec = new Vector(v.elements.length);
        for (let i = 0; i < v.elements.length; i++) {
            vec.elements[i] = 0;
        }
        return vec;
    }

    static inv(v) {
        let c = new Vector(v.elements.length);
        for (let i = 0; i < c.elements.length; i++) {
            c.elements[i] = 1 / v.elements[i];
        }
        return c;
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
    constructor(i, j) {
        this.elements = [];
        this.i_length = i;
        this.j_length = j;
    }

    clone() {
        const clonedMatrix = new SparseMatrix(this.i_length, this.j_length);
        clonedMatrix.elements = this.elements.map(block => {
            return new SparseMatrixBlock(block.i, block.j, block.data);
        });
        return clonedMatrix;
    }

    static mat_mult_vec(A, x) {
        if (A.j_length != x.elements.length) {
            throw new Error("mat_mult_vec: lengths are not the same");
        }

        const b = new Vector(A.i_length);
        for (let idx = 0; idx < A.elements.length; idx++) {
            const {i, j, data} = A.elements[idx];
            b.elements[i] += data * x.elements[j];
        }
        return b;
    }

    static matT_mult_vec(A, x) {
        if (A.i_length !== x.elements.length) {
            throw new Error("matT_mult_vec: lengths are not the same");
        }

        const b = new Vector(A.j_length);
        for (let idx = 0; idx < A.elements.length; idx++) {
            const {i, j, data} = A.elements[idx];
            b.elements[j] += data * x.elements[i];
        }
        return b;
    }

}