//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


    // STENCIL: reference matrix code has the following functions:
function matrix_multiply(A, B) {
    // A: m x n matrix
    // B: n x p matrix
    // return AB: m x p matrix
    var m = A.length, n = A[0].length, p = B[0].length;
    var C = [];
    for (var i = 0; i < m; i++) {
        C[i] = [];
        for (var j = 0; j < p; j++) {
            C[i][j] = 0;
            for (var k = 0; k < n; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

function matrix_transpose(A) {
    // A: n x n matrix
    // return A^T: n x n matrix
    var n = A.length;
    var Trans = [];
    for (var i = 0; i < n; i++) {
        Trans[i] = [];
        for (var j = 0; j < n; j++) {
            Trans[i][j] = A[j][i];
        }
    }
    return Trans;
}

function matrix_pseudoinverse(R) {
    // R: 3x3 rotation matrix
    // return: R^(-1) = R^(T)
    return matrix_transpose(R);
}

function matrix_invert_affine(T) {
    // T: 4x4 matrix
    // return T^(-1): 4x4 matrix
    var t = [[T[0][3]], [T[1][3]], [T[2][3]]];
    var R = [];
    for (var i = 0; i < 3; i++) {
        R[i] = [];
        for (var j = 0; j <3; j++) {
            R[i][j] = T[i][j];
        }
    }

    var R_inv = matrix_pseudoinverse(R);
    var R_inv_t = matrix_multiply(R_inv, t);

    var Res = generate_identity();
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            Res[i][j] = R_inv[i][j];
        }
    }
    for (i = 0; i < 3; i++) {
        Res[i][3] = R_inv_t[i][0];
    }
    return Res;
}

function vector_normalize(a) {
    // a: 3x1 vector
    // return a / ||a||: 3x1 vector
    var norm = Math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    return [a[0]/norm, a[1]/norm, a[2]/norm];
}

function vector_cross(a, b) {
    // a: 3x1 vector
    // b: 3x1 vector
    // return a x b: 3x1 vector
    return [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]];
}

function generate_identity() {
    // return I: 4x4 matrix
    return [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]];
}

function generate_translation_matrix(tx, ty, tz) {
    // tx, ty, tz: scalar
    // return: 4x4 matrix
    return [[1, 0, 0, tx],
            [0, 1, 0, ty],
            [0, 0, 1, tz],
            [0, 0, 0,  1]];
}

function generate_rotation_matrix_X(theta) {
    // theta: scalar, rad
    // return: 4x4 matrix
    return [[1,               0,                0, 0],
            [0, Math.cos(theta), -Math.sin(theta), 0],
            [0, Math.sin(theta),  Math.cos(theta), 0],
            [0,               0,                0, 1]];
}

function generate_rotation_matrix_Y(theta) {
    // theta: scalar, rad
    // return: 4x4 matrix
    return [[ Math.cos(theta), 0, Math.sin(theta), 0],
            [               0, 1,               0, 0],
            [-Math.sin(theta), 0, Math.cos(theta), 0],
            [               0, 0,               0, 1]];
}

function generate_rotation_matrix_Z(theta) {
    // theta: scalar, rad
    // return: 4x4 matrix
    return [[Math.cos(theta), -Math.sin(theta), 0, 0],
            [Math.sin(theta),  Math.cos(theta), 0, 0],
            [              0,                0, 1, 0],
            [              0,                0, 0, 1]];
}