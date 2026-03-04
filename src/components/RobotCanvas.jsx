import { useRef, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Line } from '@react-three/drei';

const RobotArm = ({ trajectoryData }) => {
  const j1 = useRef();
  const j2 = useRef();
  const j3 = useRef();
  const j4 = useRef();
  const j5 = useRef();
  const j6 = useRef();

  const timeRef = useRef(0);

  // --- NEW: Extract the XYZ coordinates from the 4x4 Pose Matrices ---
  const pathPoints = useMemo(() => {
    // Only draw the line if we have an active trajectory
    if (!trajectoryData || !trajectoryData.poses || trajectoryData.poses.length < 2) return null;
    
    // Map through the array of matrices and grab the translation column: T[0][3], T[1][3], T[2][3]
    return trajectoryData.poses.map(pose => [pose[0][3], pose[1][3], pose[2][3]]);
  }, [trajectoryData]);

  useFrame((state, delta) => {
    if (!trajectoryData || !trajectoryData.time || !trajectoryData.time.length) return;

    timeRef.current += delta;
    const maxTime = trajectoryData.time[trajectoryData.time.length - 1];
    if (timeRef.current > maxTime) timeRef.current = 0;

    let idx = trajectoryData.time.findIndex(t => t >= timeRef.current);
    if (idx === -1) idx = trajectoryData.time.length - 1;

    const angles = trajectoryData.joints[idx].map(deg => deg * (Math.PI / 180));

    if (j1.current) j1.current.rotation.z = angles[0];
    if (j2.current) j2.current.rotation.z = angles[1] - (Math.PI / 2);
    if (j3.current) j3.current.rotation.z = angles[2];
    if (j4.current) j4.current.rotation.z = angles[3];
    if (j5.current) j5.current.rotation.z = angles[4];
    if (j6.current) j6.current.rotation.z = angles[5];
  });

  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      
      {/* --- NEW: The Trajectory Line --- */}
      {pathPoints && (
        <Line 
          points={pathPoints}       // Array of [x, y, z] arrays
          color="#FF00FF"           // Bright Magenta so it contrasts with the red tip
          lineWidth={4}             // Thickness of the line
          dashed={false}
        />
      )}

      {/* --- JOINT 1 --- */}
      <group ref={j1}>
        <mesh position={[0, 0, 240]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[45, 50, 480]} />
          <meshStandardMaterial color="#0055A4" />
        </mesh>

        {/* DH Transform 1 -> 2 */}
        <group position={[200, 0, 480]} rotation={[-Math.PI / 2, 0, 0]}>
          <mesh position={[-100, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
            <cylinderGeometry args={[35, 35, 200]} />
            <meshStandardMaterial color="#0055A4" />
          </mesh>

          {/* --- JOINT 2 --- */}
          <group ref={j2}>
            <mesh position={[300, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
              <cylinderGeometry args={[30, 30, 600]} />
              <meshStandardMaterial color="#E6E6E6" />
            </mesh>

            {/* DH Transform 2 -> 3 */}
            <group position={[600, 0, 0]} rotation={[Math.PI, 0, 0]}>

              {/* --- JOINT 3 --- */}
              <group ref={j3}>
                <mesh position={[37.5, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
                  <cylinderGeometry args={[25, 25, 75]} />
                  <meshStandardMaterial color="#0055A4" />
                </mesh>

                {/* DH Transform 3 -> 4 */}
                <group position={[75, 0, 0]} rotation={[-Math.PI / 2, 0, 0]}>
                  <mesh position={[0, 0, -275]} rotation={[Math.PI / 2, 0, 0]}>
                    <cylinderGeometry args={[25, 25, 550]} />
                    <meshStandardMaterial color="#0055A4" />
                  </mesh>

                  <group position={[0, 0, -550]}>
                    
                    {/* --- JOINT 4 --- */}
                    <group ref={j4}>
                      <group rotation={[Math.PI / 2, 0, 0]}>
                        
                        {/* --- JOINT 5 --- */}
                        <group ref={j5}>
                          <mesh>
                            <boxGeometry args={[45, 45, 45]} />
                            <meshStandardMaterial color="#E6E6E6" />
                          </mesh>

                          <group rotation={[-Math.PI / 2, 0, 0]}>
                            
                            {/* --- JOINT 6 --- */}
                            <group ref={j6}>
                              <mesh position={[0, 0, 35]} rotation={[Math.PI / 2, 0, 0]}>
                                <cylinderGeometry args={[10, 15, 70]} />
                                <meshStandardMaterial color="#FF0000" />
                              </mesh>
                            </group>

                          </group>
                        </group>
                      </group>
                    </group>
                  </group>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
};

export default function RobotCanvas({ trajectoryData }) {
  return (
    <div style={{ height: '500px', flex: 1, minWidth: '500px', border: '1px solid #ccc' }}>
      <Canvas camera={{ position: [1500, 1500, 1500], fov: 50, near: 10, far: 20000 }}>
        <ambientLight intensity={0.5} />
        <directionalLight position={[1000, 2000, 1000]} intensity={1} />
        <RobotArm trajectoryData={trajectoryData} />
        <OrbitControls />
        <Grid infiniteGrid fadeDistance={10000} sectionColor="#aaaaaa" cellColor="#dddddd" />
      </Canvas>
    </div>
  );
}