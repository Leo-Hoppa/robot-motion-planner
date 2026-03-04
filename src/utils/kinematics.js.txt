import * as math from 'mathjs';

// Helper function to convert degrees to radians
const deg2rad = (deg) => deg * (Math.PI / 180);

/**
 * Calculates the Forward Kinematics for the Yaskawa Motoman DX1350D
 * @param {number[]} jointAngles - Array of 6 joint angles in degrees: [t1, t2, t3, t4, t5, t6]
 * @returns {number[][]} A 4x4 homogeneous transformation matrix representing the end-effector pose
 */
export function calculateFK(jointAngles) {
  // Apply the -90 degree offset to joint 2 as specified in your Mathematica code,
  // then convert all input angles to radians.
  const theta = [
    deg2rad(jointAngles[0]),
    deg2rad(jointAngles[1] - 90), 
    deg2rad(jointAngles[2]),
    deg2rad(jointAngles[3]),
    deg2rad(jointAngles[4]),
    deg2rad(jointAngles[5])
  ];

  // Constants (Converted to 0-indexed arrays, dropping the Mathematica 'Null' placeholders)
  const alpha = [-90, 180, -90, 90, -90].map(deg2rad);
  const a = [200, 600, 75, 0, 0];
  const d = [480, 0, 0, -550, 0, 0];

  const X = [];
  const Z = [];

  // Build the 5 'X' matrices (representing alpha and a transitions)
  for (let i = 0; i < 5; i++) {
    X.push(math.matrix([
      [1, 0, 0, a[i]],
      [0, Math.cos(alpha[i]), -Math.sin(alpha[i]), 0],
      [0, Math.sin(alpha[i]), Math.cos(alpha[i]), 0],
      [0, 0, 0, 1]
    ]));
  }

  // Build the 6 'Z' matrices (representing theta and d transitions)
  for (let i = 0; i < 6; i++) {
    Z.push(math.matrix([
      [Math.cos(theta[i]), -Math.sin(theta[i]), 0, 0],
      [Math.sin(theta[i]), Math.cos(theta[i]), 0, 0],
      [0, 0, 1, d[i]],
      [0, 0, 0, 1]
    ]));
  }

  // Multiply them together: Z[0] * X[0] * Z[1] * X[1] ... Z[5]
  let Dfinal = Z[0];
  for (let i = 0; i < 5; i++) {
    Dfinal = math.multiply(Dfinal, X[i]);
    Dfinal = math.multiply(Dfinal, Z[i + 1]);
  }

  // Return the matrix as a standard 2D JavaScript array so it's easy to read in the UI
  return Dfinal.toArray(); 
}

// Helper function to wrap angles between -180 and 180 degrees
const wrapAngle = (deg) => {
  let wrapped = deg % 360;
  if (wrapped > 180) wrapped -= 360;
  if (wrapped <= -180) wrapped += 360;
  return wrapped;
};

// Helper function to create Z-axis transformation matrix
const Zfunc = (d, thetaDeg) => {
  const t = deg2rad(thetaDeg);
  return math.matrix([
    [Math.cos(t), -Math.sin(t), 0, 0],
    [Math.sin(t), Math.cos(t), 0, 0],
    [0, 0, 1, d],
    [0, 0, 0, 1]
  ]);
};

// Helper function to create X-axis transformation matrix
const Xfunc = (a, alphaDeg) => {
  const al = deg2rad(alphaDeg);
  return math.matrix([
    [1, 0, 0, a],
    [0, Math.cos(al), -Math.sin(al), 0],
    [0, Math.sin(al), Math.cos(al), 0],
    [0, 0, 0, 1]
  ]);
};

/**
 * Calculates the Inverse Kinematics for the Yaskawa Motoman DX1350D
 * @param {number[][]} T - 4x4 Homogeneous transformation matrix of the desired pose
 * @returns {number[][]} Array of up to 8 possible joint configurations [t1, t2, t3, t4, t5, t6] in degrees
 */
export function calculateIK(T) {
  // Extract Position
  const P = [T[0][3], T[1][3], T[2][3]];

  // Constants
  const a2 = 200, a3 = 600, a4 = 75;
  const d1 = 480, d4 = -550;
  const alpha2 = -90, alpha3 = 180, alpha4 = -90;

  // Initialize arrays to hold the solutions
  const t1 = [], t2 = [], t3 = [], t4 = [], t5 = [], t6 = [];

  // --- Calculate Theta 1 ---
  // Note: JS Math.atan2(y, x) takes y first, unlike Mathematica's ArcTan[x, y]
  t1[0] = Math.atan2(P[1], P[0]) * (180 / Math.PI);
  t1[1] = wrapAngle(t1[0] + 180);

  // --- Calculate Theta 3 ---
  const A = 2 * a3 * a4;
  const B = -2 * a3 * d4;

  // For forwards Theta 1
  const Cnum1 = Math.pow(a3, 2) + Math.pow(a4, 2) + Math.pow(d4, 2) -
    (Math.pow(a2 - Math.sqrt(Math.pow(P[0], 2) + Math.pow(P[1], 2)), 2) + Math.pow(d1 - P[2], 2));
  
  const sqrtVal1 = Math.sqrt(Math.pow(A, 2) + Math.pow(B, 2) - Math.pow(Cnum1, 2));
  t3[0] = wrapAngle(2 * Math.atan2(-B + sqrtVal1, Cnum1 - A) * (180 / Math.PI));
  t3[1] = wrapAngle(2 * Math.atan2(-B - sqrtVal1, Cnum1 - A) * (180 / Math.PI));

  // For backwards Theta 1
  const Cnum2 = Math.pow(a3, 2) + Math.pow(a4, 2) + Math.pow(d4, 2) -
    (Math.pow(a2 + Math.sqrt(Math.pow(P[0], 2) + Math.pow(P[1], 2)), 2) + Math.pow(d1 - P[2], 2));
  
  const sqrtVal2 = Math.sqrt(Math.pow(A, 2) + Math.pow(B, 2) - Math.pow(Cnum2, 2));
  t3[2] = wrapAngle(2 * Math.atan2(-B + sqrtVal2, Cnum2 - A) * (180 / Math.PI));
  t3[3] = wrapAngle(2 * Math.atan2(-B - sqrtVal2, Cnum2 - A) * (180 / Math.PI));

  // --- Calculate Theta 2 ---
  const calcT2 = (theta3, rSign) => {
    const t3Rad = deg2rad(theta3);
    const Acoeff = a3 + a4 * Math.cos(t3Rad) - d4 * Math.sin(t3Rad);
    const Bcoeff = -d4 * Math.cos(t3Rad) - a4 * Math.sin(t3Rad);
    const r = rSign * Math.sqrt(Math.pow(P[0], 2) + Math.pow(P[1], 2)) - a2;
    const z = P[2] - d1;
    return wrapAngle(Math.atan2(Acoeff * r - Bcoeff * z, Bcoeff * r + Acoeff * z) * (180 / Math.PI));
  };

  t2[0] = calcT2(t3[0], 1);  // Forwards
  t2[1] = calcT2(t3[1], 1);  // Forwards
  t2[2] = calcT2(t3[2], -1); // Backwards
  t2[3] = calcT2(t3[3], -1); // Backwards

  // Group the first 3 joints
  const sol123 = [
    [t1[0], t2[0], t3[0]],
    [t1[0], t2[1], t3[1]],
    [t1[1], t2[2], t3[2]],
    [t1[1], t2[3], t3[3]]
  ];

  // --- Calculate Spherical Wrist (Theta 4, 5, 6) ---
  const T_matrix = math.matrix(T);
  const solutions = [];

  for (let i = 0; i < 4; i++) {
    const [th1, th2, th3] = sol123[i];

    // Construct R matrix
    let R = math.multiply(Zfunc(d1, th1), Xfunc(a2, alpha2));
    R = math.multiply(R, Zfunc(0, th2 - 90)); // -90 offset as per your Mathematica code
    R = math.multiply(R, Xfunc(a3, alpha3));
    R = math.multiply(R, Zfunc(0, th3));
    R = math.multiply(R, Xfunc(a4, alpha4));
    R = math.multiply(R, Zfunc(d4, 0));

    // Calculate aMatrix = Inverse[R] * T
    const aMatrix = math.multiply(math.inv(R), T_matrix).toArray();

    // Solutions for Theta 5
    const t5_a = wrapAngle(Math.atan2(Math.sqrt(Math.pow(aMatrix[0][2], 2) + Math.pow(aMatrix[1][2], 2)), aMatrix[2][2]) * (180 / Math.PI));
    const t5_b = wrapAngle(-t5_a);

    // Solutions for Theta 4
    const t4_a = wrapAngle(Math.atan2(aMatrix[1][2], aMatrix[0][2]) * (180 / Math.PI) - 180);
    const t4_b = wrapAngle(t4_a + 180);

    // Solutions for Theta 6
    const t6_a = wrapAngle(Math.atan2(-aMatrix[2][1], aMatrix[2][0]) * (180 / Math.PI));
    const t6_b = wrapAngle(t6_a + 180);

    // Push the two branch solutions for this arm configuration
    solutions.push([th1, th2, th3, t4_a, t5_a, t6_a]);
    solutions.push([th1, th2, th3, t4_b, t5_b, t6_b]);
  }

  return solutions;
}