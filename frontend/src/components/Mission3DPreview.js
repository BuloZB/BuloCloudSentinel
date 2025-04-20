import React, { useRef, useEffect } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { Line } from '@react-three/drei';

function MissionPath({ waypoints }) {
  const points = waypoints.map(wp => [wp.longitude, wp.latitude, wp.altitude || 0]);
  return <Line points={points} color="blue" lineWidth={2} />;
}

export default function Mission3DPreview({ waypoints }) {
  const cameraRef = useRef();

  useEffect(() => {
    if (cameraRef.current) {
      cameraRef.current.position.set(0, 0, 100);
      cameraRef.current.lookAt(0, 0, 0);
    }
  }, []);

  return (
    <Canvas camera={{ position: [0, 0, 100], fov: 75 }} style={{ height: '400px', width: '100%' }}>
      <ambientLight />
      <pointLight position={[10, 10, 10]} />
      <MissionPath waypoints={waypoints} />
    </Canvas>
  );
}
