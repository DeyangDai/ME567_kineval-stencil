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
    var i, j, k;
    var m = A.length, n = A[0].length;
    if (B[1][0] === undefined && n !== 1) {
        var Tmp = [];
        for (i = 0; i < B.length; i++) {
            Tmp[i] = [];
            Tmp[i][0] = B[i];
        }
        B = Tmp;
    }
    var p = B[0].length;
    var C = [];
    for (i = 0; i < m; i++) {
        C[i] = [];
        for (j = 0; j < p; j++) {
            C[i][j] = 0;
            for (k = 0; k < n; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

function matrix_transpose(A) {
    // A: m x n matrix
    // return A^T: n x m matrix
    var i, j;
    var m = A.length;
    var n = A[0].length;
    var Trans = [];
    for (j = 0; j < n; j++) {
        Trans[j] = [];
        for (i = 0; i < m; i++) {
            Trans[j][i] = A[i][j];
        }
    }
    return Trans;
}

function matrix_pseudoinverse(A) {
    // A: m x n rotation matrix
    // return: A^(+) = (A^T A)^(-1) A^T, if m >= n
    //                 A^T (A A^T)^(-1), if m < n
    var m = A.length, n = A[0].length;
    var A_trans = matrix_transpose(A);
    if (m >= n) {
        return matrix_multiply(matrix_inverse(matrix_multiply(A_trans, A)), A_trans);
    } else {
        return matrix_multiply(A_trans, matrix_inverse(matrix_multiply(A, A_trans)));
    }
}

function matrix_invert_affine(T) {
    // T: 4x4 matrix
    // return T^(-1): 4x4 matrix
    var i, j;
    var t = [T[0][3], T[1][3], T[2][3]];
    var R = [];
    for (i = 0; i < 3; i++) {
        R[i] = [];
        for (j = 0; j <3; j++) {
            R[i][j] = T[i][j];
        }
    }

    var R_inv = matrix_transpose(R);
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
    // a: n x 1 vector
    // return a / ||a||: n x 1 vector
    var i;
    var norm = 0;
    for (i = 0; i < a.length; i++) {
        norm += a[i]*a[i];
    }
    norm = Math.sqrt(norm);

    var res = [];
    for (i = 0; i < a.length; i++) {
        res[i] = a[i] / norm;
    }
    return res;
}

function vector_substract(a, b) {
    // a: nx1 vector
    // b: nx1 vector
    // return a - b: nx1 vector
    var res = [];
    for (var i = 0; i < a.length; i++) {
        res.push(a[i] - b[i]);
    }
    return res;
}

function vector_cross(a, b) {
    // a: 3x1 vector
    // b: 3x1 vector
    // return a x b: 3x1 vector
    return [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]];
}

function generate_identity(dim=4) {
    // return I: dim x dim matrix
    var i, j;
    var I = [];
    for (i = 0; i < dim; i++) {
        I[i] = [];
        for (j = 0; j < dim; j++) {
            I[i][j] = (i === j) ? 1 : 0;
        }
    }
    return I;
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

function generate_transformation(xyz, rpy) {
    return matrix_multiply(generate_translation_matrix(xyz[0], xyz[1], xyz[2]),
        matrix_multiply(generate_rotation_matrix_Z(rpy[2]),
            matrix_multiply(generate_rotation_matrix_Y(rpy[1]),
                generate_rotation_matrix_X(rpy[0]))));
}

function rotation_matrix_to_axisangle(R) {
    var R00 = R[0][0], R01 = R[0][1], R02 = R[0][2];
    var R10 = R[1][0], R11 = R[1][1], R12 = R[1][2];
    var R20 = R[2][0], R21 = R[2][1], R22 = R[2][2];

    var thetaX, thetaY, thetaZ;
    if (R02 < 1) {
        if (R02 > -1) {
            thetaY = Math.asin(R02);
            thetaX = Math.atan2(-R12, R22);
            thetaZ = Math.atan2(-R01, R00);
        } else {
            thetaY = -Math.PI / 2;
            thetaX = -Math.atan2(R10, R11);
            thetaZ = 0;
        }
    } else {
        thetaY = Math.PI / 2;
        thetaX = Math.atan2(R10, R11);
        thetaZ = 0;
    }
    return [thetaX, thetaY, thetaZ];
}

///////////// ADVANCED EXTENSIONS //////////////////
function matrix_inverse(A) {
    // A: n x n matrix
    // return A^(-1): n x n matrix
    var i, j, k;
    if (A.length !== A[0].length) {
        return matrix_pseudoinverse(A);
    }

    var PLU = LU_pivoting_decomposition(A);
    var P = PLU.P;
    var L = PLU.L;
    var U = PLU.U;

    var L_inv = generate_identity(L.length);
    var U_inv = generate_identity(U.length);
    for (i = 0; i < A.length; i++) {
        for (k = i + 1; k < A.length; k++) {
            for (j = i; j <= k - 1; j++) {
                L_inv[k][i] -= L[k][j] * L_inv[j][i];
            }
        }
    }
    for (i = 0; i < A.length; i++) {
        U_inv[i][i] = 1 / U[i][i];
        for (k = i - 1; k >= 0; k--) {
            var tmp = 0;
            for (j = k + 1; j <= i; j++) {
                tmp += U[k][j] * U_inv[j][i];
            }
            U_inv[k][i] = -tmp / U[k][k];
        }
    }
    return matrix_multiply(matrix_multiply(U_inv, L_inv), P);
}

function LU_pivoting_decomposition(A) {
    // A: n x n matrix
    // return {"P": P, "L": L, "U": U}
    var n = A.length;
    var P = generate_identity(n);
    var L = generate_identity(n);
    var U = matrix_copy(A);

    for (var i = 0; i < n - 1; i++) {
        var maxIndex = getMaxIndexInColumn(U, i);

        if (i !== maxIndex) {
            for (var j = i; j < n; j++) {
                var tmp = U[i][j];
                U[i][j] = U[maxIndex][j];
                U[maxIndex][j] = tmp;
            }
            for (j = 0; j < i; j++) {
                tmp = L[i][j];
                L[i][j] = L[maxIndex][j];
                L[maxIndex][j] = tmp;
            }
            tmp = P[i];
            P[i] = P[maxIndex];
            P[maxIndex] = tmp;
        }

        for (j = i + 1; j < n; j++) {
            tmp = U[j][i] / U[i][i];
            L[j][i] = tmp;
            for (var k = i; k < n; k++) {
                U[j][k] -= tmp * U[i][k];
            }
        }
    }

    return {"P": P, "L": L, "U": U};

    function getMaxIndexInColumn(U, j) {
        var max = Math.abs(U[j][j]);
        var maxIndex = j;
        for (var i = j + 1; i < U.length; i++) {
            var entry = Math.abs(U[i][j]);
            if (entry > max) {
                max = entry;
                maxIndex = i;
            }
        }
        return maxIndex;
    }
}

function linear_solve(A, b) {
    // return x, where A x = b
    var i, j, sum;
    var PLU = LU_pivoting_decomposition(A);
    var P = PLU.P;
    var L = PLU.L;
    var U = PLU.U;

    var P_b = matrix_multiply(P, b);

    var U_x = [];
    for (i = 0; i < b.length; i++) {
        U_x[i] = [];
        sum = 0;
        for (j = 0; j < i; j++) {
            sum += L[i][j] * U_x[j][0];
        }
        U_x[i][0] = P_b[i][0] - sum;
    }

    var x = [];
    for (i = b.length - 1; i >= 0; i--) {
        sum = 0;
        for (j = i + 1; j < b.length; j++) {
            sum += U[i][j] * x[j - i - 1][0];
        }
        x.unshift([(U_x[i][0] - sum) / U[i][i]]);
    }
    return x;
}