import { useEffect, useRef, useState, useLayoutEffect, useCallback } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';

// --- CONSTANTS ---
const CONSTANTS = {
  G: 0.5,
  DT: 0.2,
  SOFTENING: 2,
  TRAIL_LENGTH: 200,
  PREDICTION_STEPS: 400,
  EARTH_MASS: 10000,
  MAX_SATELLITES: 20,
  MAX_VELOCITY_CONTROL: 8,
  ATMOSPHERE_RADIUS: 250,
  DRAG_COEFFICIENT: 0.02
};

// --- HELPER MATH FUNCTIONS ---
const solveKeplerNewtonRaphson = (M, e, tolerance = 1e-6) => {
  let E = M;
  for (let i = 0; i < 10; i++) {
    const f = E - e * Math.sin(E) - M;
    const df = 1 - e * Math.cos(E);
    const dE = f / df;
    E = E - dE;
    if (Math.abs(dE) < tolerance) return E;
  }
  return E;
};

// --- JOYSTICK COMPONENT ---
const VelocityJoystick = ({ vx, vy, onChange }) => {
  const containerRef = useRef(null);
  const [dragging, setDragging] = useState(false);
  const size = 140;
  const knobSize = 50;
  const center = size / 2;
  const maxRadius = (size / 2) - (knobSize / 2) - 5;

  const velToPos = (v) => (v / CONSTANTS.MAX_VELOCITY_CONTROL) * maxRadius;
  const knobX = center + velToPos(vx) - knobSize / 2;
  const knobY = center + velToPos(vy) - knobSize / 2;

  const handleMovement = useCallback((clientX, clientY) => {
    if (!containerRef.current) return;
    const rect = containerRef.current.getBoundingClientRect();
    let dx = clientX - rect.left - center;
    let dy = clientY - rect.top - center;
    const distance = Math.sqrt(dx * dx + dy * dy);
    if (distance > maxRadius) {
      dx = (dx / distance) * maxRadius;
      dy = (dy / distance) * maxRadius;
    }
    const newVx = (dx / maxRadius) * CONSTANTS.MAX_VELOCITY_CONTROL;
    const newVy = (dy / maxRadius) * CONSTANTS.MAX_VELOCITY_CONTROL;
    onChange(newVx, newVy);
  }, [onChange, maxRadius, center]);

  const onMouseDown = (e) => {
    setDragging(true);
    handleMovement(e.clientX, e.clientY);
  };

  useEffect(() => {
    const onMouseMove = (e) => { if (dragging) handleMovement(e.clientX, e.clientY); };
    const onMouseUp = () => { setDragging(false); };
    if (dragging) {
      window.addEventListener('mousemove', onMouseMove);
      window.addEventListener('mouseup', onMouseUp);
    }
    return () => {
      window.removeEventListener('mousemove', onMouseMove);
      window.removeEventListener('mouseup', onMouseUp);
    };
  }, [dragging, handleMovement]);

  return (
    <div
      ref={containerRef} onMouseDown={onMouseDown}
      style={{
        width: size, height: size, background: 'rgba(15, 23, 42, 0.8)',
        border: '2px solid #334155', borderRadius: '50%', position: 'relative', margin: '10px auto',
        boxShadow: 'inset 0 0 20px rgba(0,0,0,0.5)', cursor: dragging ? 'grabbing' : 'grab'
      }}
    >
      <div style={{ position: 'absolute', top: '50%', left: '10%', right: '10%', height: 1, background: 'rgba(255,255,255,0.1)' }}></div>
      <div style={{ position: 'absolute', left: '50%', top: '10%', bottom: '10%', width: 1, background: 'rgba(255,255,255,0.1)' }}></div>
      <div style={{
        position: 'absolute', left: knobX, top: knobY, width: knobSize, height: knobSize, borderRadius: '50%',
        background: 'radial-gradient(circle at 30% 30%, #facc15, #ca8a04)',
        boxShadow: `0 0 15px ${dragging ? '#facc15' : 'rgba(250, 204, 21, 0.5)'}, inset 0 0 5px rgba(255,255,255,0.3)`,
        border: '2px solid #fde047', transform: `scale(${dragging ? 1.1 : 1})`, transition: dragging ? 'none' : 'transform 0.1s ease-out'
      }}>
        <div style={{ position: 'absolute', top: '50%', left: '50%', transform: 'translate(-50%,-50%)', color: 'rgba(0,0,0,0.5)', fontSize: 20 }}>✛</div>
      </div>
    </div>
  );
};

// --- PHYSICS ENGINE ---
const getAccelerations = (bodies, useDrag = false) => {
  const acc = bodies.map(() => ({ ax: 0, ay: 0 }));
  for (let i = 0; i < bodies.length; i++) {
    if (bodies[i].isFixed) continue;
    for (let j = 0; j < bodies.length; j++) {
      if (i === j) continue;
      const dx = bodies[j].x - bodies[i].x;
      const dy = bodies[j].y - bodies[i].y;
      const distSq = dx * dx + dy * dy;
      const dist = Math.sqrt(distSq + CONSTANTS.SOFTENING);
      const f = (CONSTANTS.G * bodies[j].mass) / (distSq * dist);
      acc[i].ax += f * dx;
      acc[i].ay += f * dy;

      if (useDrag && bodies[j].isFixed && dist < CONSTANTS.ATMOSPHERE_RADIUS) {
        const speed = Math.sqrt(bodies[i].vx ** 2 + bodies[i].vy ** 2);
        const dragForce = -CONSTANTS.DRAG_COEFFICIENT * speed;
        acc[i].ax += dragForce * bodies[i].vx;
        acc[i].ay += dragForce * bodies[i].vy;
      }
    }
  }
  return acc;
};

// Verlet Integrator
const stepVerlet = (bodies, dt, useDrag) => {
  const acc1 = getAccelerations(bodies, useDrag);
  for (let i = 0; i < bodies.length; i++) {
    if (bodies[i].isFixed) continue;
    bodies[i].vx += 0.5 * acc1[i].ax * dt;
    bodies[i].vy += 0.5 * acc1[i].ay * dt;
    bodies[i].x += bodies[i].vx * dt;
    bodies[i].y += bodies[i].vy * dt;
  }
  const acc2 = getAccelerations(bodies, useDrag);
  for (let i = 0; i < bodies.length; i++) {
    if (bodies[i].isFixed) continue;
    bodies[i].vx += 0.5 * acc2[i].ax * dt;
    bodies[i].vy += 0.5 * acc2[i].ay * dt;
  }
};

// RK4 Integrator
const stepRK4 = (bodies, dt, useDrag) => {
  const acc1 = getAccelerations(bodies, useDrag);
  const v1 = bodies.map(b => ({ vx: b.vx, vy: b.vy }));
  const bodiesK2 = bodies.map((b, i) => ({ ...b, x: b.x + v1[i].vx * 0.5 * dt, y: b.y + v1[i].vy * 0.5 * dt }));
  const acc2 = getAccelerations(bodiesK2, useDrag);
  const v2 = bodies.map((b, i) => ({ vx: b.vx + acc1[i].ax * 0.5 * dt, vy: b.vy + acc1[i].ay * 0.5 * dt }));
  const bodiesK3 = bodies.map((b, i) => ({ ...b, x: b.x + v2[i].vx * 0.5 * dt, y: b.y + v2[i].vy * 0.5 * dt }));
  const acc3 = getAccelerations(bodiesK3, useDrag);
  const v3 = bodies.map((b, i) => ({ vx: b.vx + acc2[i].ax * 0.5 * dt, vy: b.vy + acc2[i].ay * 0.5 * dt }));
  const bodiesK4 = bodies.map((b, i) => ({ ...b, x: b.x + v3[i].vx * dt, y: b.y + v3[i].vy * dt }));
  const acc4 = getAccelerations(bodiesK4, useDrag);
  const v4 = bodies.map((b, i) => ({ vx: b.vx + acc3[i].ax * dt, vy: b.vy + acc3[i].ay * dt }));

  for (let i = 0; i < bodies.length; i++) {
    if (bodies[i].isFixed) continue;
    bodies[i].x += (dt / 6) * (v1[i].vx + 2 * v2[i].vx + 2 * v3[i].vx + v4[i].vx);
    bodies[i].y += (dt / 6) * (v1[i].vy + 2 * v2[i].vy + 2 * v3[i].vy + v4[i].vy);
    bodies[i].vx += (dt / 6) * (acc1[i].ax + 2 * acc2[i].ax + 2 * acc3[i].ax + acc4[i].ax);
    bodies[i].vy += (dt / 6) * (acc1[i].ay + 2 * acc2[i].ay + 2 * acc3[i].ay + acc4[i].ay);
  }
};

// Euler Integrator
const stepEuler = (bodies, dt, useDrag) => {
  const acc = getAccelerations(bodies, useDrag);
  for (let i = 0; i < bodies.length; i++) {
    if (bodies[i].isFixed) continue;
    bodies[i].vx += acc[i].ax * dt;
    bodies[i].vy += acc[i].ay * dt;
    bodies[i].x += bodies[i].vx * dt;
    bodies[i].y += bodies[i].vy * dt;
  }
};

const calculateTotalEnergy = (bodies) => {
  let ke = 0, pe = 0;
  for (let i = 0; i < bodies.length; i++) {
    const vSq = bodies[i].vx * bodies[i].vx + bodies[i].vy * bodies[i].vy;
    ke += 0.5 * bodies[i].mass * vSq;
    for (let j = i + 1; j < bodies.length; j++) {
      const dx = bodies[j].x - bodies[i].x;
      const dy = bodies[j].y - bodies[i].y;
      const r = Math.sqrt(dx * dx + dy * dy);
      if (r > 0) pe -= (CONSTANTS.G * bodies[i].mass * bodies[j].mass) / r;
    }
  }
  return ke + pe;
};

// --- MAIN APPLICATION ---
const FullScreenSimulation = () => {
  const canvasRef = useRef(null);
  const bodiesRef = useRef([]);
  const requestRef = useRef();
  const frameCount = useRef(0);
  const logsEndRef = useRef(null);
  const isPausedRef = useRef(false);
  const hoveredBodyIdRef = useRef(null);
  const lastIgnoredRef = useRef({ id: null, time: 0 });
  const initialEnergyRef = useRef(null);

  const emergencyRef = useRef({ show: false, target: null });
  const selectedBodyRef = useRef(null);
  const selectedBodyDataRef = useRef({ vx: 0, vy: 0 });

  const [selectedBodyId, setSelectedBodyId] = useState(null);
  const [selectedBodyData, setSelectedBodyData] = useState({ vx: 0, vy: 0 });
  const [algorithm, setAlgorithm] = useState('VERLET');
  const [useDrag, setUseDrag] = useState(false);
  const [collisionRisk, setCollisionRisk] = useState({ risk: 0, details: [] });
  const [energyData, setEnergyData] = useState([]);
  const [satCount, setSatCount] = useState(0);
  const [keplerInfo, setKeplerInfo] = useState(null);

  const [showEmergencyPopup, setShowEmergencyPopup] = useState(false);
  const [emergencyTarget, setEmergencyTarget] = useState(null);

  const [showAnalysis, setShowAnalysis] = useState(false);
  const [chartData, setChartData] = useState([]);

  const [logs, setLogs] = useState([]);
  const [toasts, setToasts] = useState([]);

  const PANEL_WIDTH = 300;

  useEffect(() => {
    emergencyRef.current = { show: showEmergencyPopup, target: emergencyTarget };
    selectedBodyRef.current = selectedBodyId;
    selectedBodyDataRef.current = selectedBodyData;
  }, [showEmergencyPopup, emergencyTarget, selectedBodyId, selectedBodyData]);

  const showToast = (message) => {
    const id = Date.now() + Math.random();
    setToasts(prev => [...prev, { id, message }]);
    setTimeout(() => {
      setToasts(prev => prev.filter(t => t.id !== id));
    }, 4000);
  };

  const addLog = (message, type = 'danger') => {
    const time = new Date().toLocaleTimeString();
    setLogs(prev => [...prev, { time, message, type }]);
  };

  useEffect(() => { logsEndRef.current?.scrollIntoView({ behavior: "smooth" }); }, [logs]);

  const getUniqueColor = (index) => {
    if (index === 0) return '#3b82f6';
    const hue = (index * 137.508) % 360;
    return `hsl(${hue}, 80%, 60%)`;
  };

  const createSatellite = (idSuffix, colorIndex, centerX, centerY) => {
    const r = 120 + Math.random() * 200;
    const angle = Math.random() * Math.PI * 2;
    const x = centerX + r * Math.cos(angle);
    const y = centerY + r * Math.sin(angle);
    const v = Math.sqrt((CONSTANTS.G * CONSTANTS.EARTH_MASS) / r);
    return {
      id: `SAT-${idSuffix}`,
      x: x, y: y,
      vx: -Math.sin(angle) * v,
      vy: Math.cos(angle) * v,
      mass: 5, radius: 6, color: getUniqueColor(colorIndex), isFixed: false, history: []
    };
  };

  const initSimulation = (w, h) => {
    bodiesRef.current = [
      { id: 'DÜNYA', x: w / 2, y: h / 2, vx: 0, vy: 0, mass: CONSTANTS.EARTH_MASS, radius: 30, color: getUniqueColor(0), isFixed: true, history: [] },
      createSatellite(1, 1, w / 2, h / 2),
      createSatellite(2, 2, w / 2, h / 2),
      createSatellite(3, 3, w / 2, h / 2)
    ];
    setSatCount(3);
    setEnergyData([]);
    initialEnergyRef.current = null;
    setLogs([]);
    setToasts([]);
    setCollisionRisk({ risk: 0, details: [] });
    isPausedRef.current = false;
    setShowEmergencyPopup(false);
    setEmergencyTarget(null);
    setSelectedBodyId(null);
    lastIgnoredRef.current = { id: null, time: 0 };
  };

  useLayoutEffect(() => {
    const handleResize = () => {
      if (canvasRef.current) {
        canvasRef.current.width = window.innerWidth;
        canvasRef.current.height = window.innerHeight;
      }
    };
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  const addSatellite = () => {
    if (bodiesRef.current.length - 1 >= CONSTANTS.MAX_SATELLITES) return;
    const newIdx = bodiesRef.current.length;
    const earth = bodiesRef.current.find(b => b.isFixed);
    bodiesRef.current.push(createSatellite(newIdx, newIdx, earth.x, earth.y));
    setSatCount(bodiesRef.current.length - 1);
  };

  const removeSatellite = () => {
    if (bodiesRef.current.length <= 1) return;
    bodiesRef.current.pop();
    setSatCount(bodiesRef.current.length - 1);
  };

  const resetSim = () => { initSimulation(window.innerWidth, window.innerHeight); };

  // --- COMPARISON ANALYSIS ENGINE ---
  const runComparisonAnalysis = () => {
    if (bodiesRef.current.length < 2) return;

    const initialBodies = JSON.parse(JSON.stringify(bodiesRef.current));
    const steps = 300;

    // Simülasyon clones
    const simEuler = JSON.parse(JSON.stringify(initialBodies));
    const simRK4 = JSON.parse(JSON.stringify(initialBodies));
    const simVerlet = JSON.parse(JSON.stringify(initialBodies));

    const e0 = calculateTotalEnergy(initialBodies);

    const formattedData = [];

    for (let i = 0; i < steps; i++) {
      // Euler
      stepEuler(simEuler, CONSTANTS.DT, false);
      const eE = calculateTotalEnergy(simEuler);
      const valE = Math.abs((eE - e0) / e0) * 100;

      // RK4
      stepRK4(simRK4, CONSTANTS.DT, false);
      const eR = calculateTotalEnergy(simRK4);
      const valR = Math.abs((eR - e0) / e0) * 100;

      // Verlet
      stepVerlet(simVerlet, CONSTANTS.DT, false);
      const eV = calculateTotalEnergy(simVerlet);
      const valV = Math.abs((eV - e0) / e0) * 100;

      // Format data for every 5th step
      if (i % 5 === 0) {
        formattedData.push({
          step: i,
          euler: parseFloat(valE.toFixed(4)),
          rk4: parseFloat(valR.toFixed(4)),
          verlet: parseFloat(valV.toFixed(4))
        });
      }
    }

    setChartData(formattedData);
    setShowAnalysis(true);
    isPausedRef.current = true;
  };

  const closeAnalysis = () => {
    setShowAnalysis(false);
    isPausedRef.current = false;
  };

  const handlePhysicalCollisions = () => {
    const bodies = bodiesRef.current;
    const toRemoveIndices = new Set();
    for (let i = 0; i < bodies.length; i++) {
      if (toRemoveIndices.has(i)) continue;
      for (let j = i + 1; j < bodies.length; j++) {
        if (toRemoveIndices.has(j)) continue;
        const b1 = bodies[i];
        const b2 = bodies[j];
        const dx = b1.x - b2.x;
        const dy = b1.y - b2.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        const minDist = b1.radius + b2.radius;
        if (dist < minDist) {

          const v1Str = `(Vx:${b1.vx.toFixed(2)}, Vy:${b1.vy.toFixed(2)})`;
          const v2Str = `(Vx:${b2.vx.toFixed(2)}, Vy:${b2.vy.toFixed(2)})`;

          if (b1.isFixed || b2.isFixed) {
            const satelliteIndex = b1.isFixed ? j : i;
            const satId = b1.isFixed ? b2.id : b1.id;
            const satV = b1.isFixed ? v2Str : v1Str;
            const earthId = b1.isFixed ? b1.id : b2.id;
            toRemoveIndices.add(satelliteIndex);

            const msg = `💥 ÇARPIŞMA: ${satId} ${satV} -> ${earthId}`;
            addLog(msg, "danger");
            showToast(msg);
          } else {
            toRemoveIndices.add(i);
            toRemoveIndices.add(j);
            const msg = `💥 ÇARPIŞMA: ${b1.id} ${v1Str} <-> ${b2.id} ${v2Str}`;
            addLog(msg, "danger");
            showToast(msg);
          }
          if (selectedBodyId === b1.id || selectedBodyId === b2.id) setSelectedBodyId(null);
        }
      }
    }
    if (toRemoveIndices.size > 0) {
      bodiesRef.current = bodies.filter((_, idx) => !toRemoveIndices.has(idx));
      setSatCount(bodiesRef.current.length - 1);
    }
  };

  const analyzeOrbit = () => {
    if (!selectedBodyId) { setKeplerInfo(null); return; }
    const body = bodiesRef.current.find(b => b.id === selectedBodyId);
    const earth = bodiesRef.current.find(b => b.isFixed);
    if (!body || !earth) return;

    const dx = body.x - earth.x;
    const dy = body.y - earth.y;
    const r = Math.sqrt(dx * dx + dy * dy);
    const v2 = body.vx * body.vx + body.vy * body.vy;

    const epsilon = v2 / 2 - (CONSTANTS.G * CONSTANTS.EARTH_MASS) / r;
    const a = -(CONSTANTS.G * CONSTANTS.EARTH_MASS) / (2 * epsilon);

    if (a > 0) {
      const h = dx * body.vy - dy * body.vx;
      const e_sq = 1 + (2 * epsilon * h * h) / (CONSTANTS.G * CONSTANTS.EARTH_MASS) ** 2;
      const e = Math.sqrt(Math.max(0, e_sq));

      const theta = Math.atan2(dy, dx);
      const M = theta;

      const E = solveKeplerNewtonRaphson(M, e);

      setKeplerInfo({
        eccentricity: e.toFixed(4),
        semiMajorAxis: a.toFixed(1),
        anomalyE: (E * 180 / Math.PI).toFixed(2)
      });
    } else {
      setKeplerInfo({ type: "Hyperbolic/Parabolic", eccentricity: ">1" });
    }
  };

  const calculateRiskDetails = () => {
    if (isPausedRef.current && !showAnalysis) return;

    const ghostBodies = bodiesRef.current.map(b => ({ ...b }));
    let minRatio = Infinity;
    let criticalPairs = [];
    let firstCollider = null;

    for (let step = 0; step < CONSTANTS.PREDICTION_STEPS; step++) {
      stepVerlet(ghostBodies, CONSTANTS.DT * 2, useDrag);

      for (let i = 0; i < ghostBodies.length; i++) {
        for (let j = i + 1; j < ghostBodies.length; j++) {
          const b1 = ghostBodies[i];
          const b2 = ghostBodies[j];
          if (b1.isFixed || b2.isFixed) continue;

          const d = Math.sqrt((b1.x - b2.x) ** 2 + (b1.y - b2.y) ** 2);
          const limit = b1.radius + b2.radius;
          const ratio = d / limit;
          if (ratio < minRatio) minRatio = ratio;
          if (ratio < 4.0) {
            const pairName = `${b1.id} - ${b2.id}`;
            if (!criticalPairs.find(p => p.pair === pairName)) {
              let level = 'DÜŞÜK';
              if (ratio <= 1.2) level = 'ÇARPIŞMA';
              else if (ratio <= 2.5) level = 'YÜKSEK';
              else level = 'ORTA';
              criticalPairs.push({ pair: pairName, dist: d.toFixed(1), riskLevel: level });
              if (ratio <= 2.0 && !firstCollider) {
                const originalB1 = bodiesRef.current.find(b => b.id === b1.id);
                firstCollider = originalB1;
              }
            }
          }
        }
      }
      if (minRatio <= 1.0) break;
    }

    let riskScore = 0;
    if (minRatio <= 1.0) riskScore = 100;
    else if (minRatio > 8.0) riskScore = 0;
    else riskScore = 100 - ((minRatio - 1.0) / 7.0) * 100;

    setCollisionRisk({ risk: riskScore.toFixed(0), details: criticalPairs.slice(0, 3) });

    if (riskScore >= 60 && !isPausedRef.current && firstCollider && !showAnalysis) {
      const now = Date.now();
      const isIgnoredRecently = lastIgnoredRef.current.id === firstCollider.id && (now - lastIgnoredRef.current.time < 4000);
      if (!isIgnoredRecently) {
        isPausedRef.current = true;
        setShowEmergencyPopup(true);
        setEmergencyTarget({
          id: firstCollider.id,
          x: firstCollider.x, y: firstCollider.y, vx: firstCollider.vx, vy: firstCollider.vy
        });
        addLog(`ERKEN UYARI: ${firstCollider.id} riskli rotada!`, "warn");
      }
    }
  };

  const handleEmergencyConfirm = () => {
    const body = bodiesRef.current.find(b => b.id === emergencyTarget.id);
    if (body) {
      body.vx = emergencyTarget.vx;
      body.vy = emergencyTarget.vy;
      body.history = [];
      addLog(`${body.id} manuel düzeltildi.`, "success");
    }
    setEmergencyTarget(null);
    setShowEmergencyPopup(false);
    isPausedRef.current = false;
  };

  const handleEmergencyIgnore = () => {
    lastIgnoredRef.current = { id: emergencyTarget.id, time: Date.now() };
    addLog(`${emergencyTarget.id} uyarısı yoksayıldı.`, "warn");
    setEmergencyTarget(null);
    setShowEmergencyPopup(false);
    isPausedRef.current = false;
  };

  const handleAutoEscape = () => {
    const bodyId = emergencyTarget.id;
    const originalBody = bodiesRef.current.find(b => b.id === bodyId);
    const earth = bodiesRef.current.find(b => b.isFixed);
    if (!originalBody || !earth) return;

    const dx = originalBody.x - earth.x;
    const dy = originalBody.y - earth.y;
    const r = Math.sqrt(dx * dx + dy * dy);
    const escapeVelocity = Math.sqrt(2 * CONSTANTS.G * CONSTANTS.EARTH_MASS / r);
    const currentSpeed = Math.sqrt(originalBody.vx ** 2 + originalBody.vy ** 2);
    const currentAngle = Math.atan2(originalBody.vy, originalBody.vx);

    const candidates = [];
    const speedModifiers = [0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15];
    const angleModifiers = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3];

    speedModifiers.forEach(speedFac => {
      angleModifiers.forEach(angleMod => {
        const newSpeed = currentSpeed * speedFac;
        if (newSpeed > escapeVelocity * 0.9) return;
        const newAngle = currentAngle + angleMod;
        candidates.push({ vx: Math.cos(newAngle) * newSpeed, vy: Math.sin(newAngle) * newSpeed });
      });
    });

    let bestCandidate = null;
    let bestScore = -Infinity;

    candidates.forEach(candidate => {
      const ghostBodies = bodiesRef.current.map(b => ({ ...b }));
      const ghostSelf = ghostBodies.find(b => b.id === bodyId);
      ghostSelf.vx = candidate.vx;
      ghostSelf.vy = candidate.vy;
      let minDistanceGlobal = Infinity;
      let maxRadiusDeviation = 0;
      for (let step = 0; step < 300; step++) {
        stepRK4(ghostBodies, CONSTANTS.DT * 2, useDrag);
        ghostBodies.forEach(other => {
          if (other.id !== bodyId) {
            const d = Math.sqrt((ghostSelf.x - other.x) ** 2 + (ghostSelf.y - other.y) ** 2);
            const limit = ghostSelf.radius + other.radius + 10;
            if (d < minDistanceGlobal) minDistanceGlobal = d;
            if (d < limit) minDistanceGlobal = -1000;
          }
        });
        if (minDistanceGlobal === -1000) break;
        const distToEarth = Math.sqrt((ghostSelf.x - earth.x) ** 2 + (ghostSelf.y - earth.y) ** 2);
        const deviation = Math.abs(distToEarth - r);
        if (deviation > maxRadiusDeviation) maxRadiusDeviation = deviation;
      }
      if (minDistanceGlobal > 0) {
        const score = (minDistanceGlobal * 2) - (maxRadiusDeviation * 0.8);
        if (score > bestScore) {
          bestScore = score;
          bestCandidate = candidate;
        }
      }
    });

    if (bestCandidate) {
      originalBody.vx = bestCandidate.vx;
      originalBody.vy = bestCandidate.vy;
      originalBody.history = [];
      addLog(`${bodyId} için STABİL KAÇIŞ rotası uygulandı.`, "success");
      setEmergencyTarget(null);
      setShowEmergencyPopup(false);
      isPausedRef.current = false;
    } else {
      addLog(`${bodyId} için güvenli rota bulunamadı!`, "danger");
    }
  };

  const updateEmergencyVelocityFromJoystick = (vx, vy) => { setEmergencyTarget(prev => ({ ...prev, vx, vy })); };

  const handleMouseMove = (e) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    let closest = null;
    let minD = Infinity;
    bodiesRef.current.forEach(b => {
      if (b.isFixed) return;
      const d = Math.sqrt((b.x - mx) ** 2 + (b.y - my) ** 2);
      if (d < (b.radius + 20) && d < minD) {
        minD = d;
        closest = b;
      }
    });
    hoveredBodyIdRef.current = closest ? closest.id : null;
    canvas.style.cursor = closest ? 'pointer' : 'default';
  };

  const handleCanvasClick = (e) => {
    if (isPausedRef.current) return;
    if (hoveredBodyIdRef.current) {
      const body = bodiesRef.current.find(b => b.id === hoveredBodyIdRef.current);
      if (body) {
        setSelectedBodyId(body.id);
        setSelectedBodyData({ vx: body.vx, vy: body.vy });
      }
    } else {
      if (selectedBodyId) setSelectedBodyId(null);
    }
  };

  const updateVelocityFromJoystick = (newVx, newVy) => {
    if (!selectedBodyId) return;
    setSelectedBodyData({ vx: newVx, vy: newVy });
    const body = bodiesRef.current.find(b => b.id === selectedBodyId);
    if (body) {
      body.vx = newVx;
      body.vy = newVy;
      body.history = [];
    }
  };

  const loop = () => {
    draw();
    if (isPausedRef.current) {
      requestRef.current = requestAnimationFrame(loop);
      return;
    }

    if (algorithm === 'EULER') stepEuler(bodiesRef.current, CONSTANTS.DT, useDrag);
    else if (algorithm === 'VERLET') stepVerlet(bodiesRef.current, CONSTANTS.DT, useDrag);
    else stepRK4(bodiesRef.current, CONSTANTS.DT, useDrag);

    handlePhysicalCollisions();
    bodiesRef.current.forEach(b => {
      if (!b.isFixed) {
        b.history.push({ x: b.x, y: b.y });
        if (b.history.length > CONSTANTS.TRAIL_LENGTH) b.history.shift();
      }
    });
    if (frameCount.current % 5 === 0) {
      const e = calculateTotalEnergy(bodiesRef.current);
      if (initialEnergyRef.current === null) initialEnergyRef.current = e;
      setEnergyData(prev => {
        const newData = [...prev, e];
        if (newData.length > 100) newData.shift();
        return newData;
      });
      analyzeOrbit();
    }
    if (frameCount.current % 15 === 0) calculateRiskDetails();
    frameCount.current++;
    requestRef.current = requestAnimationFrame(loop);
  };

  const draw = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    if (useDrag) {
      const earth = bodiesRef.current.find(b => b.isFixed);
      if (earth) {
        ctx.beginPath();
        ctx.arc(earth.x, earth.y, CONSTANTS.ATMOSPHERE_RADIUS, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(59, 130, 246, 0.1)';
        ctx.fill();
        ctx.strokeStyle = 'rgba(59, 130, 246, 0.3)';
        ctx.setLineDash([5, 5]);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }

    ctx.strokeStyle = '#1e293b'; ctx.lineWidth = 0.5; ctx.beginPath();
    for (let i = 0; i < canvas.width; i += 50) { ctx.moveTo(i, 0); ctx.lineTo(i, canvas.height); }
    for (let i = 0; i < canvas.height; i += 50) { ctx.moveTo(0, i); ctx.lineTo(canvas.width, i); }
    ctx.stroke();

    const emState = emergencyRef.current;

    bodiesRef.current.forEach(b => {
      if (b.history.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = b.color;
        ctx.lineWidth = 1;
        ctx.setLineDash([]);
        ctx.moveTo(b.history[0].x, b.history[0].y);
        for (let i = 1; i < b.history.length; i++) ctx.lineTo(b.history[i].x, b.history[i].y);
        ctx.stroke();
      }
      const isHovered = (b.id === hoveredBodyIdRef.current);
      const isSelected = (b.id === selectedBodyId);
      const isEmergency = (emState.show && emState.target && b.id === emState.target.id);
      const radiusToDraw = isHovered ? b.radius * 1.8 : b.radius;

      ctx.beginPath();
      ctx.arc(b.x, b.y, radiusToDraw, 0, Math.PI * 2);
      ctx.fillStyle = b.color;

      if (isEmergency) {
        ctx.shadowColor = '#ef4444'; ctx.shadowBlur = 30;
      } else if (isSelected) {
        ctx.shadowColor = '#facc15'; ctx.shadowBlur = 25;
      } else if (isHovered) {
        ctx.shadowColor = 'white'; ctx.shadowBlur = 15;
      } else {
        ctx.shadowColor = b.color; ctx.shadowBlur = 10;
      }
      ctx.fill();
      ctx.shadowBlur = 0;
      if (isSelected || isEmergency) {
        ctx.beginPath();
        ctx.arc(b.x, b.y, radiusToDraw + 6, 0, Math.PI * 2);
        ctx.strokeStyle = isEmergency ? '#ef4444' : '#facc15';
        ctx.lineWidth = 2.5;
        ctx.setLineDash([]);
        ctx.stroke();

        if (isEmergency) {
          ctx.font = '24px sans-serif';
          ctx.textAlign = 'center';
          ctx.textBaseline = 'middle';
          ctx.fillText("⚠️", b.x, b.y - radiusToDraw - 25);
        }
      }
      ctx.fillStyle = 'white';
      ctx.font = isHovered ? 'bold 12px sans-serif' : '10px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(b.id, b.x, b.y - radiusToDraw - 12);
    });

    // --- Manual & Emergency Trajectory Preview (White & Animated) ---
    const selBodyId = selectedBodyRef.current;
    const selBodyData = selectedBodyDataRef.current;

    let previewTargetId = null;
    let previewVx = 0;
    let previewVy = 0;

    if (emState.show && emState.target) {
      previewTargetId = emState.target.id;
      previewVx = emState.target.vx;
      previewVy = emState.target.vy;
    } else if (selBodyId) {
      previewTargetId = selBodyId;
      previewVx = selBodyData.vx;
      previewVy = selBodyData.vy;
    }

    if (previewTargetId) {
      const previewBodies = bodiesRef.current.map(b => ({ ...b }));
      const previewTarget = previewBodies.find(b => b.id === previewTargetId);

      if (previewTarget) {
        previewTarget.vx = previewVx;
        previewTarget.vy = previewVy;

        const dashOffset = (Date.now() / 20) % 20;

        ctx.beginPath();
        ctx.strokeStyle = 'white';
        ctx.lineWidth = 2;
        ctx.setLineDash([8, 8]);
        ctx.lineDashOffset = -dashOffset;

        ctx.moveTo(previewTarget.x, previewTarget.y);

        for (let i = 0; i < 300; i++) {
          stepRK4(previewBodies, CONSTANTS.DT * 2, useDrag);
          ctx.lineTo(previewTarget.x, previewTarget.y);
        }

        ctx.stroke();
        ctx.setLineDash([]);
        ctx.lineDashOffset = 0;

        ctx.fillStyle = 'white';
        ctx.beginPath();
        ctx.arc(previewTarget.x, previewTarget.y, 4, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  };

  // Draw graph on canvas for the left panel
  const drawGraph = (canvas) => {
    if (!canvas || energyData.length < 2) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = 'rgba(15, 23, 42, 0.6)';
    ctx.fillRect(0, 0, w, h);

    const topPadding = 25;
    const bottomPadding = 40;
    const graphH = h - (topPadding + bottomPadding);

    const min = Math.min(...energyData), max = Math.max(...energyData), range = max - min || 1;

    const currentEnergy = energyData[energyData.length - 1];
    const initialE = initialEnergyRef.current || currentEnergy;

    const driftPercent = Math.abs((currentEnergy - initialE) / initialE) * 100;

    let accuracyScore = 100 - (driftPercent * 10);
    if (accuracyScore < 0) accuracyScore = 0;

    let statusColor = '#4ade80';
    let statusText = "SİSTEM STABİL";

    if (accuracyScore < 50) {
      statusColor = '#ef4444';
      statusText = "KRİTİK HATA";
    } else if (accuracyScore < 85) {
      statusColor = '#facc15';
      statusText = "DENGESİZLİK VAR";
    }

    ctx.beginPath();
    ctx.strokeStyle = statusColor;
    ctx.lineWidth = 2;

    energyData.forEach((val, i) => {
      const x = (i / (energyData.length - 1)) * w;
      const normalizedY = (val - min) / range;
      const y = (h - bottomPadding) - (normalizedY * graphH);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();

    ctx.fillStyle = '#e2e8f0';
    ctx.font = 'bold 11px sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText("SİMÜLASYON DOĞRULUĞU", 10, 5);

    ctx.textAlign = 'right';
    ctx.fillStyle = statusColor;
    ctx.fillText(statusText, w - 10, 5);

    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';

    ctx.font = 'bold 16px sans-serif';
    ctx.fillStyle = statusColor;
    ctx.fillText(`%${accuracyScore.toFixed(1)}`, 10, h - 5);

    ctx.font = '10px sans-serif';
    ctx.fillStyle = '#94a3b8';
    ctx.fillText("Fiziksel Gerçekçilik", 10, h - 22);

    ctx.textAlign = 'right';
    ctx.fillStyle = '#64748b';
    ctx.fillText(`Sapma: %${driftPercent.toFixed(1)}`, w - 10, h - 5);
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    initSimulation(window.innerWidth, window.innerHeight);
    requestRef.current = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(requestRef.current);
  }, []);

  const activeSelectedBody = bodiesRef.current.find(b => b.id === selectedBodyId);

  return (
    <div style={{ position: 'relative', width: '100vw', height: '100vh', background: '#020617', overflow: 'hidden', fontFamily: 'Inter, system-ui, sans-serif', color: 'white' }}>
      <canvas ref={canvasRef} onClick={handleCanvasClick} onMouseMove={handleMouseMove} style={{ display: 'block', width: '100%', height: '100%' }} />

      {/* --- Toast Notifications at Top (Increased Z-Index) --- */}
      <div style={{ position: 'absolute', top: 20, left: '50%', transform: 'translateX(-50%)', display: 'flex', flexDirection: 'column', gap: 10, alignItems: 'center', pointerEvents: 'none', zIndex: 10001 }}>
        {toasts.map(toast => (
          <div key={toast.id} style={{
            background: 'rgba(239, 68, 68, 0.95)', color: 'white', padding: '12px 24px', borderRadius: 25,
            fontWeight: 'bold', fontSize: 14, boxShadow: '0 4px 20px rgba(0,0,0,0.5)',
            animation: 'fadeIn 0.3s ease-out', border: '1px solid #fca5a5'
          }}>
            {toast.message}
          </div>
        ))}
      </div>

      {/* --- LEFT PANEL --- */}
      <div style={{ position: 'absolute', top: 20, left: 20, width: PANEL_WIDTH, display: 'flex', flexDirection: 'column', gap: 10 }}>

        {/* Risk */}
        <div style={{ background: 'rgba(30, 41, 59, 0.9)', padding: '10px 20px', borderRadius: 12, borderLeft: `6px solid ${collisionRisk.risk > 50 ? '#ef4444' : '#22c55e'}`, width: '100%', boxSizing: 'border-box', backdropFilter: 'blur(5px)' }}>
          <div style={{ fontSize: 12, color: '#94a3b8' }}>Çarpışma Riski</div>
          <div style={{ fontSize: 24, fontWeight: 'bold', color: collisionRisk.risk > 50 ? '#ef4444' : '#22c55e' }}>%{collisionRisk.risk}</div>
        </div>

        {/* Grafic */}
        <div style={{ width: '100%', height: 80, borderRadius: 8, border: '1px solid #475569', overflow: 'hidden', background: 'rgba(15,23,42,0.8)' }}>
          <canvas width={PANEL_WIDTH} height={80} ref={c => drawGraph(c)} />
        </div>

        {/* KEPLER Analysis */}
        {activeSelectedBody && keplerInfo && !showEmergencyPopup && (
          <div style={{ background: 'rgba(30, 41, 59, 0.9)', padding: '10px 20px', borderRadius: 12, borderLeft: '6px solid #8b5cf6', width: '100%', boxSizing: 'border-box', backdropFilter: 'blur(5px)' }}>
            <div style={{ fontSize: 12, color: '#a78bfa', fontWeight: 'bold', marginBottom: 5 }}>YÖRÜNGE ANALİZİ (Newton-Raphson)</div>
            <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 11, color: '#cbd5e1', marginBottom: 2 }}>
              <span>Eksantriklik (e):</span> <span style={{ fontWeight: 'bold' }}>{keplerInfo.eccentricity}</span>
            </div>
            <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 11, color: '#cbd5e1', marginBottom: 2 }}>
              <span>Yarı Eksen (a):</span> <span style={{ fontWeight: 'bold' }}>{keplerInfo.semiMajorAxis}</span>
            </div>
            <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 11, color: '#cbd5e1' }}>
              <span>Anomali (E°):</span> <span style={{ fontWeight: 'bold' }}>{keplerInfo.anomalyE}°</span>
            </div>
          </div>
        )}

        {/* Logs */}
        <div style={{ width: '100%', maxHeight: 200, background: 'rgba(15, 23, 42, 0.95)', border: '1px solid #334155', borderRadius: 8, display: 'flex', flexDirection: 'column', backdropFilter: 'blur(5px)', boxSizing: 'border-box' }}>
          <div style={{ padding: '8px 12px', borderBottom: '1px solid #334155', fontSize: 12, fontWeight: 'bold', color: '#94a3b8', display: 'flex', justifyContent: 'space-between' }}>
            <span>OLAY GÜNLÜĞÜ</span>
            <span style={{ fontSize: 10, cursor: 'pointer' }} onClick={() => setLogs([])}>Temizle</span>
          </div>
          <div style={{ flex: 1, overflowY: 'auto', padding: 10, display: 'flex', flexDirection: 'column', gap: 6, minHeight: 60 }}>
            {logs.length === 0 && <div style={{ fontSize: 11, color: '#64748b', fontStyle: 'italic', textAlign: 'center', marginTop: 10 }}>Kritik olay yok.</div>}
            {logs.map((log, i) => (
              <div key={i} style={{ fontSize: 10, fontFamily: 'monospace', color: log.type === 'danger' ? '#fca5a5' : log.type === 'warn' ? '#fde047' : '#86efac', borderLeft: log.type === 'danger' ? '2px solid #ef4444' : log.type === 'warn' ? '2px solid #facc15' : '2px solid #22c55e', paddingLeft: 6, lineHeight: 1.4 }}>
                <span style={{ opacity: 0.5 }}>[{log.time}]</span> {log.message}
              </div>
            ))}
            <div ref={logsEndRef} />
          </div>
        </div>
      </div>

      {/* --- RIGHT PANEL --- */}
      <div style={{ position: 'absolute', top: 20, right: 20, width: PANEL_WIDTH, display: 'flex', flexDirection: 'column', gap: 10 }}>

        {/* CONTROL STATION */}
        <div style={{ background: 'rgba(30, 41, 59, 0.9)', padding: 15, borderRadius: 12, border: '1px solid #475569', width: '100%', boxSizing: 'border-box', backdropFilter: 'blur(5px)' }}>
          <h3 style={{ margin: '0 0 10px 0', fontSize: 16, color: '#e2e8f0' }}>Kontrol İstasyonu</h3>
          <div style={{ display: 'flex', gap: 8, marginBottom: 15 }}>
            <button onClick={addSatellite} style={btnStyle('#10b981')}>+ Uydu</button>
            <button onClick={removeSatellite} style={btnStyle('#f43f5e')}>- Çıkar</button>
            <button onClick={resetSim} style={btnStyle('#3b82f6')}>Sıfırla</button>
          </div>
          <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', fontSize: 13, color: '#94a3b8', marginBottom: 10 }}>
            <span>Algoritma:</span>
            <select value={algorithm} onChange={(e) => { setAlgorithm(e.target.value); setEnergyData([]); }}
              style={{ background: '#334155', color: 'white', border: 'none', padding: '4px', borderRadius: 4, cursor: 'pointer', maxWidth: 120 }}>
              <option value="VERLET">Verlet (Simplektik)</option>
              <option value="RK4">Runge-Kutta 4</option>
              <option value="EULER">Euler</option>
            </select>
          </div>
          {/* Atmospheric Drag Setting */}
          <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', fontSize: 13, color: '#94a3b8', marginBottom: 10 }}>
            <span>Atmosfer:</span>
            <label style={{ cursor: 'pointer', display: 'flex', alignItems: 'center', gap: 5 }}>
              <input type="checkbox" checked={useDrag} onChange={(e) => setUseDrag(e.target.checked)} />
              <span style={{ color: useDrag ? '#38bdf8' : '#64748b' }}>{useDrag ? 'AKTİF' : 'PASİF'}</span>
            </label>
          </div>

          <button onClick={runComparisonAnalysis} style={{ width: '100%', padding: 8, background: '#6366f1', color: 'white', border: 'none', borderRadius: 6, cursor: 'pointer', fontWeight: 'bold', fontSize: 12, marginTop: 5 }}>
            📊 ALGORİTMA ANALİZİ BAŞLAT
          </button>

          <div style={{ fontSize: 12, color: '#cbd5e1', marginTop: 10 }}>Aktif Uydular: <strong>{satCount}</strong></div>
        </div>

        {/* Manuel Control */}
        {activeSelectedBody && !showEmergencyPopup && !showAnalysis && (
          <div style={{ background: 'rgba(30, 41, 59, 0.9)', padding: 15, borderRadius: 12, border: '1px solid #facc15', width: '100%', boxSizing: 'border-box', backdropFilter: 'blur(5px)', textAlign: 'center' }}>
            <div style={{ color: '#facc15', fontWeight: 'bold', marginBottom: 10 }}>HEDEF: {activeSelectedBody.id}</div>
            <div style={{ display: 'flex', justifyContent: 'space-around', marginBottom: 10, fontSize: 11, color: '#cbd5e1' }}>
              <div>Vx: <span style={{ fontWeight: 'bold', color: 'white' }}>{selectedBodyData.vx.toFixed(2)}</span></div>
              <div>Vy: <span style={{ fontWeight: 'bold', color: 'white' }}>{selectedBodyData.vy.toFixed(2)}</span></div>
            </div>
            <VelocityJoystick vx={selectedBodyData.vx} vy={selectedBodyData.vy} onChange={updateVelocityFromJoystick} />
          </div>
        )}

        {/* Emergency Popup */}
        {showEmergencyPopup && emergencyTarget && !showAnalysis && (
          <div style={{
            background: 'rgba(30, 41, 59, 0.95)', border: '2px solid #ef4444', borderRadius: 12, padding: 20, width: '100%', boxSizing: 'border-box',
            boxShadow: '0 0 50px rgba(239, 68, 68, 0.4)', textAlign: 'center'
          }}>
            <h2 style={{ margin: '0 0 10px 0', color: '#ef4444', fontSize: 20 }}>⚠️ ÇARPIŞMA UYARISI</h2>
            <div style={{ fontSize: 14, marginBottom: 15, color: '#e2e8f0' }}>
              <strong>{emergencyTarget.id}</strong> tehlikede!
            </div>

            <button onClick={handleAutoEscape}
              style={{ width: '100%', padding: '12px', marginBottom: 15, background: 'linear-gradient(90deg, #3b82f6, #06b6d4)', color: 'white', border: 'none', borderRadius: 8, fontWeight: 'bold', cursor: 'pointer', boxShadow: '0 4px 15px rgba(6, 182, 212, 0.3)' }}>
              GÜVENLİ ROTA BUL
            </button>

            <div style={{ background: '#0f172a', padding: 15, borderRadius: 8, marginBottom: 20 }}>
              <div style={{ fontSize: 11, color: '#94a3b8', marginBottom: 5 }}>MANUEL KONTROL & ÖNİZLEME:</div>
              <div style={{ display: 'flex', justifyContent: 'space-around', marginBottom: 10, fontSize: 11, color: '#cbd5e1' }}>
                <div>Vx: <span style={{ fontWeight: 'bold', color: 'white' }}>{emergencyTarget.vx.toFixed(2)}</span></div>
                <div>Vy: <span style={{ fontWeight: 'bold', color: 'white' }}>{emergencyTarget.vy.toFixed(2)}</span></div>
              </div>
              <VelocityJoystick vx={emergencyTarget.vx} vy={emergencyTarget.vy} onChange={updateEmergencyVelocityFromJoystick} />
            </div>

            <div style={{ display: 'flex', gap: 10 }}>
              <button onClick={handleEmergencyConfirm} style={{ flex: 1, padding: '12px', background: '#ef4444', color: 'white', border: 'none', borderRadius: 6, fontWeight: 'bold', cursor: 'pointer', fontSize: 13 }}>ONAYLA</button>
              <button onClick={handleEmergencyIgnore} style={{ flex: 1, padding: '12px', background: 'transparent', border: '1px solid #64748b', color: '#94a3b8', borderRadius: 6, fontWeight: 'bold', cursor: 'pointer', fontSize: 13 }}>YOKSAY</button>
            </div>
          </div>
        )}

      </div>

      {/* --- Analysis Modal Window  --- */}
      {showAnalysis && chartData.length > 0 && (
        <div style={{
          position: 'absolute', top: 0, left: 0, width: '100%', height: '100%',
          background: 'rgba(0,0,0,0.85)', display: 'flex', alignItems: 'center', justifyContent: 'center', zIndex: 10000
        }}>
          <div style={{
            width: 700, height: 500, background: '#1e293b', borderRadius: 12,
            border: '1px solid #475569', display: 'flex', flexDirection: 'column', padding: 20,
            boxShadow: '0 0 40px rgba(0,0,0,0.5)'
          }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 15 }}>
              <h2 style={{ margin: 0, color: 'white', fontSize: 18 }}>🔍 Sayısal Algoritma Karşılaştırması</h2>
              <button onClick={closeAnalysis} style={{ background: 'transparent', border: 'none', color: '#94a3b8', cursor: 'pointer', fontSize: 20 }}>✕</button>
            </div>

            {/* Recharts Graphic */}
            <div style={{ flex: 1, background: '#0f172a', borderRadius: 8, padding: 10 }}>
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
                  <CartesianGrid strokeDasharray="3 3" stroke="#334155" />

                  <XAxis
                    dataKey="step"
                    stroke="#94a3b8"
                    label={{ value: 'Adım Sayısı', position: 'insideBottomRight', offset: -5, fill: '#94a3b8' }}
                  />
                  <YAxis
                    stroke="#94a3b8"
                    width={80}
                    tickFormatter={(val) => val.toFixed(3)}
                    label={{
                      value: 'Enerji Sapması (%)',
                      angle: -90,
                      position: 'insideLeft',
                      fill: '#94a3b8',
                      style: { textAnchor: 'middle' }
                    }}
                  />

                  <Tooltip
                    contentStyle={{ backgroundColor: '#1e293b', borderColor: '#475569', color: 'white' }}
                    itemStyle={{ color: 'white' }}
                    formatter={(value) => `%${value}`}
                  />
                  <Legend wrapperStyle={{ paddingTop: '10px' }} />
                  <Line type="monotone" dataKey="euler" stroke="#ef4444" name="Euler" strokeWidth={2} dot={false} />
                  <Line type="monotone" dataKey="rk4" stroke="#3b82f6" name="RK4 " strokeWidth={2} dot={false} />
                  <Line type="monotone" dataKey="verlet" stroke="#22c55e" name="Verlet" strokeWidth={2} dot={false} />
                </LineChart>
              </ResponsiveContainer>
            </div>

            <div style={{ marginTop: 15, fontSize: 12, color: '#cbd5e1', lineHeight: 1.5 }}>
              <p>Bu grafik, simülasyonun aynı başlangıç koşulları altında <strong>300 adım</strong> boyunca çalıştırılmasıyla elde edilen <strong>Enerji Sapma (Drift)</strong> oranlarını gösterir.</p>
              <ul style={{ paddingLeft: 20, margin: '5px 0' }}>
                <li style={{ color: '#ef4444' }}><strong>Euler:</strong> Enerji korunmaz, sistem sürekli enerji kazanır (Yörüngeler dışa açılır).</li>
                <li style={{ color: '#3b82f6' }}><strong>RK4:</strong> Hassastır ancak uzun vadede enerji kaybeder (Yörüngeler küçülür).</li>
                <li style={{ color: '#22c55e' }}><strong>Verlet:</strong> Simplektik yapısı sayesinde enerjiyi en iyi koruyan yöntemdir.</li>
              </ul>
            </div>
          </div>
        </div>
      )}

      <style>{`@keyframes fadeIn { from { opacity: 0; transform: translateY(-20px); } to { opacity: 1; transform: translateY(0); } }`}</style>
    </div>
  );
};

const btnStyle = (bg) => ({ background: bg, border: 'none', borderRadius: 6, color: 'white', padding: '6px 12px', cursor: 'pointer', fontWeight: '500', fontSize: 12, flex: 1 });

export default FullScreenSimulation;