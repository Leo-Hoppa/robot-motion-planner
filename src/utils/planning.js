import { calculateFK } from './kinematics';
import * as math from 'mathjs';
import { calculateIK } from './kinematics';

// Helper to calculate the Euclidean distance between two joint configurations
const getJointDistance = (q1, q2) => {
  return Math.sqrt(q1.reduce((sum, val, i) => sum + Math.pow(val - q2[i], 2), 0));
};

// Helper to select the IK solution closest to the current joint state
const selectClosestIK = (ikSolutions, currentJoints) => {
  let bestSolution = null;
  let minDistance = Infinity;

  ikSolutions.forEach(solution => {
    // Skip invalid solutions (e.g., if IK returned NaN due to unreachable workspace)
    if (solution.some(isNaN)) return; 
    
    const dist = getJointDistance(solution, currentJoints);
    if (dist < minDistance) {
      minDistance = dist;
      bestSolution = solution;
    }
  });

  return bestSolution;
};

// Helper for SO(3) Matrix Logarithm: Extracts the axis of rotation and the angle
const getAxisAngle = (R) => {
  const trace = R[0][0] + R[1][1] + R[2][2];
  // Clamp trace to avoid NaN from floating point errors
  const clampedTrace = Math.max(-1, Math.min(3, trace)); 
  const theta = Math.acos((clampedTrace - 1) / 2);

  if (theta < 1e-6) {
    return { axis: [0, 0, 0], theta: 0 }; // No rotation
  }

  const multiplier = 1 / (2 * Math.sin(theta));
  const axis = [
    multiplier * (R[2][1] - R[1][2]),
    multiplier * (R[0][2] - R[2][0]),
    multiplier * (R[1][0] - R[0][1])
  ];

  return { axis, theta };
};

// Helper for SO(3) Matrix Exponential using Rodrigues' Formula
const rodriguesExp = (axis, angle) => {
  if (angle < 1e-6) return math.identity(3).toArray();

  const [kx, ky, kz] = axis;
  const K = [
    [0, -kz, ky],
    [kz, 0, -kx],
    [-ky, kx, 0]
  ];
  
  const K_matrix = math.matrix(K);
  const K_sq = math.multiply(K_matrix, K_matrix);

  const I = math.identity(3);
  const term2 = math.multiply(K_matrix, Math.sin(angle));
  const term3 = math.multiply(K_sq, 1 - Math.cos(angle));

  return math.add(math.add(I, term2), term3).toArray();
};

/**
 * Generates a joint-space trajectory (MoveJ) using a synchronized trapezoidal velocity profile.
 * * @param {number[]} qStart - The starting 6 joint angles in degrees.
 * @param {number[]} qGoal - The goal 6 joint angles in degrees.
 * @param {number} duration - Total time for the move in seconds (tf).
 * @param {number} timeStep - Resolution of the interpolated points in seconds.
 * @returns {Object} An object containing the time array, joint trajectory, and cartesian trajectory.
 */
export function planMoveJ(qStart, qGoal, duration, timeStep = 0.01) {
  const trajectory = [];
  const timeArray = [];
  const cartesianPath = []; // For storing the FK pose at each step

  // We use the 1/3 rule: acceleration, cruise, and deceleration each take 1/3 of the total time.
  const tb = duration / 3.0; 

  // Calculate parameters for each of the 6 joints
  const jointParams = qStart.map((q0, i) => {
    const qf = qGoal[i];
    const deltaQ = qf - q0;
    
    // Max velocity required to reach the goal given our blend time
    const vmax = deltaQ / (duration - tb); 
    // Constant acceleration/deceleration rate
    const a = vmax / tb; 

    return { q0, qf, vmax, a };
  });

  // Generate the interpolated points
  for (let t = 0; t <= duration; t += timeStep) {
    const currentJoints = [];

    for (let i = 0; i < 6; i++) {
      const { q0, qf, vmax, a } = jointParams[i];
      let q_t = 0;

      if (t <= tb) {
        // Phase 1: Parabolic Acceleration
        q_t = q0 + 0.5 * a * Math.pow(t, 2);
      } else if (t > tb && t <= duration - tb) {
        // Phase 2: Linear Constant Velocity
        q_t = q0 + 0.5 * a * Math.pow(tb, 2) + vmax * (t - tb);
      } else {
        // Phase 3: Parabolic Deceleration
        // We calculate backward from the final position
        const t_rem = duration - t;
        q_t = qf - 0.5 * a * Math.pow(t_rem, 2);
      }

      currentJoints.push(q_t);
    }

    timeArray.push(t);
    trajectory.push(currentJoints);
    
    // Requirement (b): Use FK to compute Cartesian pose at each interpolated step
    cartesianPath.push(calculateFK(currentJoints));
  }

  // Ensure the final point perfectly matches the exact goal to avoid floating-point errors
  timeArray.push(duration);
  trajectory.push(qGoal);
  cartesianPath.push(calculateFK(qGoal));

  return { time: timeArray, joints: trajectory, poses: cartesianPath };
}

/**
 * Generates a Cartesian-space trajectory (MoveL) using linear interpolation for position
 * and screw-axis interpolation for orientation, driven by a trapezoidal velocity profile.
 * @param {number[][]} TStart - Starting 4x4 homogeneous transformation matrix.
 * @param {number[][]} TGoal - Goal 4x4 homogeneous transformation matrix.
 * @param {number[]} qStart - The starting 6 joint angles (needed for IK selection).
 * @param {number} duration - Total time for the move in seconds.
 * @param {number} timeStep - Resolution of the interpolated points.
 * @returns {Object} An object containing the time array, joint trajectory, and cartesian trajectory.
 */
export function planMoveL(TStart, TGoal, qStart, duration, timeStep = 0.01) {
  const trajectory = [];
  const timeArray = [];
  const cartesianPath = [];

  // Extract Position Vectors
  const P0 = [TStart[0][3], TStart[1][3], TStart[2][3]];
  const Pf = [TGoal[0][3], TGoal[1][3], TGoal[2][3]];

  // Extract 3x3 Rotation Matrices
  const R0 = TStart.slice(0, 3).map(row => row.slice(0, 3));
  const Rf = TGoal.slice(0, 3).map(row => row.slice(0, 3));

  // Compute Relative Rotation: R_rel = R0^T * Rf
  const R0_T = math.transpose(R0);
  const R_rel = math.multiply(R0_T, Rf);

  // Extract axis and total angle for the rotation
  const { axis, theta } = getAxisAngle(R_rel);

  // Motion Planning: Trapezoidal profile for path parameter s(t) from 0 to 1
  const tb = duration / 3.0;
  const sMaxDot = 1 / (duration - tb); // Max velocity of path parameter
  const sAccel = sMaxDot / tb;         // Acceleration of path parameter

  let currentJoints = [...qStart];

  for (let t = 0; t <= duration; t += timeStep) {
    let s = 0;

    // Calculate path parameter s(t) using the 1/3 rule
    if (t <= tb) {
      s = 0.5 * sAccel * Math.pow(t, 2);
    } else if (t > tb && t <= duration - tb) {
      s = 0.5 * sAccel * Math.pow(tb, 2) + sMaxDot * (t - tb);
    } else {
      const t_rem = duration - t;
      s = 1 - 0.5 * sAccel * Math.pow(t_rem, 2);
    }

    // Clamp s to [0, 1] to prevent floating point overshoot
    s = Math.max(0, Math.min(1, s)); 

    // 1. Interpolate Position
    const P_s = [
      P0[0] + s * (Pf[0] - P0[0]),
      P0[1] + s * (Pf[1] - P0[1]),
      P0[2] + s * (Pf[2] - P0[2])
    ];

    // 2. Interpolate Orientation
    const R_s_rel = rodriguesExp(axis, s * theta);
    const R_s = math.multiply(R0, R_s_rel);

    // 3. Construct interpolated 4x4 Pose T(s)
    const T_s = [
      [R_s[0][0], R_s[0][1], R_s[0][2], P_s[0]],
      [R_s[1][0], R_s[1][1], R_s[1][2], P_s[1]],
      [R_s[2][0], R_s[2][1], R_s[2][2], P_s[2]],
      [0, 0, 0, 1]
    ];

    // 4. Calculate IK and select the smoothest solution
    const ikSolutions = calculateIK(T_s);
    const bestJoints = selectClosestIK(ikSolutions, currentJoints);

    if (!bestJoints) {
      console.error(`IK failed at t=${t.toFixed(2)}. Target pose may be out of workspace limits or in singularity.`);
      break; 
    }

    currentJoints = bestJoints;

    timeArray.push(t);
    trajectory.push(currentJoints);
    cartesianPath.push(T_s);
  }

  return { time: timeArray, joints: trajectory, poses: cartesianPath };
}