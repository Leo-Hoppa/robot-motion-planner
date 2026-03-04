import { useState } from 'react';
import Plot from 'react-plotly.js';
import * as math from 'mathjs';
import { planMoveJ, planMoveL } from './utils/planning';
import { calculateFK, calculateIK } from './utils/kinematics'; // <-- Added calculateFK
import RobotCanvas from './components/RobotCanvas'; 

const deg2rad = (deg) => deg * (Math.PI / 180);

// Helper: Builds a 4x4 Transformation Matrix using X-Y-Z Euler Angle Convention
const buildPoseMatrix = (pose) => {
  const { x, y, z, rx, ry, rz } = pose;
  const rotX = math.matrix([[1, 0, 0], [0, Math.cos(deg2rad(rx)), -Math.sin(deg2rad(rx))], [0, Math.sin(deg2rad(rx)), Math.cos(deg2rad(rx))]]);
  const rotY = math.matrix([[Math.cos(deg2rad(ry)), 0, Math.sin(deg2rad(ry))], [0, 1, 0], [-Math.sin(deg2rad(ry)), 0, Math.cos(deg2rad(ry))]]);
  const rotZ = math.matrix([[Math.cos(deg2rad(rz)), -Math.sin(deg2rad(rz)), 0], [Math.sin(deg2rad(rz)), Math.cos(deg2rad(rz)), 0], [0, 0, 1]]);
  
  // CHANGED: X * Y * Z multiplication order
  const R = math.multiply(math.multiply(rotX, rotY), rotZ).toArray();
  
  return [
    [R[0][0], R[0][1], R[0][2], Number(x)],
    [R[1][0], R[1][1], R[1][2], Number(y)],
    [R[2][0], R[2][1], R[2][2], Number(z)],
    [0, 0, 0, 1]
  ];
};

// Helper: Extracts XYZ Euler angles from a 4x4 Matrix
const extractPoseFromMatrix = (T) => {
  const x = T[0][3];
  const y = T[1][3];
  const z = T[2][3];

  // CHANGED: Extract X-Y-Z Euler angles from the rotation matrix
  let ry = Math.asin(Math.max(-1, Math.min(1, T[0][2])));
  let rx, rz;
  
  if (Math.abs(Math.cos(ry)) > 1e-6) {
    rx = Math.atan2(-T[1][2], T[2][2]);
    rz = Math.atan2(-T[0][1], T[0][0]);
  } else {
    // Gimbal lock case (ry = 90 or -90 degrees)
    rx = 0;
    if (ry > 0) {
      rz = Math.atan2(T[1][0], T[1][1]);
    } else {
      rz = -Math.atan2(T[1][0], T[1][1]);
    }
  }

  return {
    x: Number(x.toFixed(2)),
    y: Number(y.toFixed(2)),
    z: Number(z.toFixed(2)),
    rx: Number((rx * 180 / Math.PI).toFixed(2)),
    ry: Number((ry * 180 / Math.PI).toFixed(2)),
    rz: Number((rz * 180 / Math.PI).toFixed(2))
  };
};

export default function App() {
  const [startPose, setStartPose] = useState({ x: 750, y: 0, z: 1155, rx: 0, ry: -90, rz: 0 });
  const [goalPose, setGoalPose] = useState({ x: -375, y: 649.519, z: 1155, rx: 0, ry: -90, rz: 120 });
  const [trajectoryData, setTrajectoryData] = useState({ time: [], joints: [] });
  
  // --- NEW STATE: Tracks manual joint slider values ---
  const [startJoints, setStartJoints] = useState([0, 0, 0, 0, 0, 0]);

  const handleInputChange = (e, poseType, axis) => {
    const val = parseFloat(e.target.value) || 0;
    if (poseType === 'start') setStartPose({ ...startPose, [axis]: val });
    else setGoalPose({ ...goalPose, [axis]: val });
  };

  // --- NEW HANDLER: Triggers when you drag a joint slider ---
  const handleJointChange = (index, value) => {
    const newJoints = [...startJoints];
    newJoints[index] = parseFloat(value);
    setStartJoints(newJoints);

    // 1. Run Forward Kinematics to find the new Cartesian position
    const T = calculateFK(newJoints);
    const newPose = extractPoseFromMatrix(T);
    
    // 2. Overwrite the Start Pose text boxes with the new math
    setStartPose(newPose);

    // 3. Feed the single frame to the 3D viewer to instantly animate it
    setTrajectoryData({ time: [0], joints: [newJoints] });
  };

  const handlePlanMoveJ = () => {
    const TStart = buildPoseMatrix(startPose);
    const TGoal = buildPoseMatrix(goalPose);
    const qStart = calculateIK(TStart)[0];
    const qGoal = calculateIK(TGoal)[0];

    if (!qStart || !qGoal) {
      alert("IK Failed! Start or Goal pose is outside the robot's workspace.");
      return;
    }
    const result = planMoveJ(qStart, qGoal, 5.0, 0.05); 
    setTrajectoryData(result);
  };

  const handlePlanMoveL = () => {
    const TStart = buildPoseMatrix(startPose);
    const TGoal = buildPoseMatrix(goalPose);
    const qStart = calculateIK(TStart)[0];

    if (!qStart) {
      alert("IK Failed for Start Pose!");
      return;
    }
    const result = planMoveL(TStart, TGoal, qStart, 5.0, 0.05);
    setTrajectoryData(result);
  };

  const handleHoldStart = () => {
    const TStart = buildPoseMatrix(startPose);
    const qStart = calculateIK(TStart)[0];
    if (!qStart) {
      alert("IK Failed! Start pose is outside the workspace.");
      return;
    }
    // Update the joint sliders to match the IK solution
    setStartJoints(qStart.map(q => Number(q.toFixed(2))));
    setTrajectoryData({ time: [0], joints: [qStart] });
  };

  const handleHoldGoal = () => {
    const TGoal = buildPoseMatrix(goalPose);
    const qGoal = calculateIK(TGoal)[0];
    if (!qGoal) {
      alert("IK Failed! Goal pose is outside the workspace.");
      return;
    }
    setTrajectoryData({ time: [0], joints: [qGoal] });
  };

  const generatePlotTraces = () => {
    if (!trajectoryData.time.length) return [];
    const colors = ['#e6194B', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4'];
    const traces = [];

    for (let i = 0; i < 6; i++) {
      traces.push({
        x: trajectoryData.time,
        y: trajectoryData.joints.map(step => step[i]),
        mode: trajectoryData.time.length === 1 ? 'markers' : 'lines',
        name: `Theta ${i + 1}`,
        line: { color: colors[i], width: 2 },
        marker: { color: colors[i], size: 8 }
      });
    }
    return traces;
  };

  return (
    <div style={{ padding: '20px', fontFamily: 'sans-serif' }}>
      <h2>DX1350D Motion Planner</h2>
      
      <div style={{ display: 'flex', gap: '40px', marginBottom: '20px', flexWrap: 'wrap' }}>
        
        {/* --- NEW: Forward Kinematics Joint Sliders --- */}
        <div style={{ backgroundColor: '#f5f5f5', padding: '15px', borderRadius: '8px' }}>
          <h3>Start Joints (FK)</h3>
          {[0, 1, 2, 3, 4, 5].map(i => (
            <div key={`joint-${i}`} style={{ display: 'flex', alignItems: 'center', marginBottom: '8px' }}>
              <label style={{ display: 'inline-block', width: '35px', fontWeight: 'bold' }}>θ{i + 1}:</label>
              <input 
                type="range" 
                min="-180" 
                max="180" 
                value={startJoints[i]} 
                onChange={(e) => handleJointChange(i, e.target.value)} 
                style={{ width: '150px' }}
              />
              <span style={{ marginLeft: '10px', width: '50px', textAlign: 'right' }}>{startJoints[i]}°</span>
            </div>
          ))}
        </div>

        <div>
          <h3>Start Pose</h3>
          {['x', 'y', 'z', 'rx', 'ry', 'rz'].map(axis => (
            <div key={`start-${axis}`} style={{ marginBottom: '5px' }}>
              <label style={{ display: 'inline-block', width: '30px', textTransform: 'uppercase' }}>{axis}: </label>
              <input type="number" value={startPose[axis]} onChange={(e) => handleInputChange(e, 'start', axis)} />
            </div>
          ))}
        </div>

        <div>
          <h3>Goal Pose</h3>
          {['x', 'y', 'z', 'rx', 'ry', 'rz'].map(axis => (
            <div key={`goal-${axis}`} style={{ marginBottom: '5px' }}>
              <label style={{ display: 'inline-block', width: '30px', textTransform: 'uppercase' }}>{axis}: </label>
              <input type="number" value={goalPose[axis]} onChange={(e) => handleInputChange(e, 'goal', axis)} />
            </div>
          ))}
        </div>
      </div>

      <div style={{ display: 'flex', gap: '10px', marginBottom: '30px', flexWrap: 'wrap' }}>
        <button onClick={handlePlanMoveJ} style={{ padding: '10px 20px', cursor: 'pointer', backgroundColor: '#e0f7fa', border: '1px solid #00bcd4' }}>Plan MoveJ</button>
        <button onClick={handlePlanMoveL} style={{ padding: '10px 20px', cursor: 'pointer', backgroundColor: '#e0f7fa', border: '1px solid #00bcd4' }}>Plan MoveL</button>
        <button onClick={handleHoldStart} style={{ padding: '10px 20px', cursor: 'pointer', backgroundColor: '#ffebee', border: '1px solid #f44336' }}>Snap to Start IK</button>
        <button onClick={handleHoldGoal} style={{ padding: '10px 20px', cursor: 'pointer', backgroundColor: '#ffebee', border: '1px solid #f44336' }}>Snap to Goal IK</button>
      </div>

      <div style={{ display: 'flex', gap: '20px', flexWrap: 'wrap', alignItems: 'flex-start' }}>
        <div style={{ border: '1px solid #ccc', padding: '10px', backgroundColor: '#fff' }}>
          <Plot
            data={generatePlotTraces()}
            layout={{
              title: 'Joint Angles vs Time',
              xaxis: { title: 'Time (s)' },
              yaxis: { title: 'Joint Angle (degrees)' },
              width: 600, 
              height: 500
            }}
          />
        </div>

        <RobotCanvas trajectoryData={trajectoryData} />
      </div>
    </div>
  );
}