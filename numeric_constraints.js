import {SparseMatrixBlock} from "./linear_algebra.js";

/*
    When adding a new constraint, 
    these functions must be in them spelled correctly and be returning these specific things:
        --> C_i (the constraint value), return: scalar
        --> C_dot (the time D of C), return: scalar
        --> J_i (the jacobian of C wrt position vectors (q)), return: list of SparseMatrixBlocks
        --> J_i_dot (time D of J_i), return: list of SparseMatrixBlocks
*/

export class FixedYConstraint {
    constructor(id, height) {
        this.p_id = id;
        this.height = height;
    }

    C_i(q) {
        return q.elements[2 * this.p_id + 1] - this.height;
    }

    C_i_dot(q, q_dot) {
        return q_dot.elements[2 * this.p_id + 1];
    }

    J_i(q, ith_row) {
        // last 1 is the data, actual jacobian
        return [new SparseMatrixBlock(ith_row, 2 * this.p_id + 1, 1)];
    }

    J_i_dot(q, q_dot, ith_row) {
        return [new SparseMatrixBlock(ith_row, 2 * this.p_id + 1, 0)];
    }
}

export class FixedXConstraint {
    constructor(id, x0) {
        this.p_id = id;
        this.x0 = x0;
    }

    C_i(q) {
        return q.elements[2 * this.p_id] - this.x0;
    }

    C_i_dot(q, q_dot) {
        return q_dot.elements[2 * this.p_id];
    }

    J_i(q, ith_row) {
        return [new SparseMatrixBlock(ith_row, 2 * this.p_id, 1)];
    }

    J_i_dot(q, q_dot, ith_row) {
        return [new SparseMatrixBlock(ith_row, 2 * this.p_id, 0)];
    }
}

export class LineConstraint {
    constructor(id1, id2, l0) {
        this.id1 = id1;
        this.id2 = id2;
        this.l0 = l0;
    }

    C_i(q) {
        const dx = q.elements[2 * this.id2] - q.elements[2 * this.id1];
        const dy = q.elements[2 * this.id2 + 1] - q.elements[2 * this.id1 + 1];
        return 0.5 * (dx * dx + dy * dy - this.l0 * this.l0);
    }

    C_i_dot(q, q_dot) {
        const dx = q.elements[2 * this.id2] - q.elements[2 * this.id1];
        const dy = q.elements[2 * this.id2 + 1] - q.elements[2 * this.id1 + 1];
        const dvx = q_dot.elements[2 * this.id2] - q_dot.elements[2 * this.id1];
        const dvy = q_dot.elements[2 * this.id2 + 1] - q_dot.elements[2 * this.id1 + 1];
        return dx * dvx + dy * dvy;
    }

    J_i(q, ith_row) {
        const wrt_x1 = new SparseMatrixBlock(ith_row, 2 * this.id1,     -1 * (q.elements[2 * this.id2] - q.elements[2 * this.id1]));
        const wrt_y1 = new SparseMatrixBlock(ith_row, 2 * this.id1 + 1, -1 * (q.elements[2 * this.id2 + 1] - q.elements[2 * this.id1 + 1]));
        const wrt_x2 = new SparseMatrixBlock(ith_row, 2 * this.id2,     1 * (q.elements[2 * this.id2] - q.elements[2 * this.id1]));
        const wrt_y2 = new SparseMatrixBlock(ith_row, 2 * this.id2 + 1, 1 * (q.elements[2 * this.id2 + 1] - q.elements[2 * this.id1 + 1]));
        return [wrt_x1, wrt_y1, wrt_x2, wrt_y2];
    }

    J_i_dot(q, q_dot, ith_row) {
        const wrt_x1 = new SparseMatrixBlock(ith_row, 2 * this.id1,     -1 * (q_dot.elements[2 * this.id2] - q_dot.elements[2 * this.id1]));
        const wrt_y1 = new SparseMatrixBlock(ith_row, 2 * this.id1 + 1, -1 * (q_dot.elements[2 * this.id2 + 1] - q_dot.elements[2 * this.id1 + 1]));
        const wrt_x2 = new SparseMatrixBlock(ith_row, 2 * this.id2,     1 * (q_dot.elements[2 * this.id2] - q_dot.elements[2 * this.id1]));
        const wrt_y2 = new SparseMatrixBlock(ith_row, 2 * this.id2 + 1, 1 * (q_dot.elements[2 * this.id2 + 1] - q_dot.elements[2 * this.id1 + 1]));
        return [wrt_x1, wrt_y1, wrt_x2, wrt_y2];
    }
}