const frameCanvas = document.getElementById('frameCanvas');
const frameCtx = frameCanvas.getContext('2d');
const chartCanvas = document.getElementById('chartCanvas');
const chartCtx = chartCanvas.getContext('2d');

const freqDisplay = document.getElementById('freqDisplay');
const periodDisplay = document.getElementById('periodDisplay');
const driftDisplay = document.getElementById('driftDisplay');

let model = null;
let animationId = null;
let groundMotion = null;
let displacementHistory = [];
let running = false;

class Node {
    constructor(x, y, anchored = false) {
        this.x0 = x;
        this.y0 = y;
        this.disp = 0; // horizontal DOF
        this.vel = 0;
        this.acc = 0;
        this.mass = 0;
        this.anchored = anchored;
    }
}

class Spring {
    constructor(i, j, stiffness) {
        this.i = i;
        this.j = j;
        this.k = stiffness;
        this.force = 0;
    }

    update(nodes) {
        const di = nodes[this.i].disp;
        const dj = nodes[this.j].disp;
        const delta = dj - di;
        this.force = this.k * delta;
        return { i: -this.force, j: this.force };
    }
}

function buildModel() {
    const stories = parseInt(document.getElementById('stories').value, 10);
    const bays = parseInt(document.getElementById('bays').value, 10);
    const storyHeight = parseFloat(document.getElementById('storyHeight').value);
    const bayWidth = parseFloat(document.getElementById('bayWidth').value);
    const segments = parseInt(document.getElementById('segments').value, 10);
    const levelMass = parseFloat(document.getElementById('levelMass').value);
    const columnEI = parseFloat(document.getElementById('columnEI').value);
    const beamEI = parseFloat(document.getElementById('beamEI').value);

    const nodes = [];
    const springs = [];

    const totalHeights = stories * storyHeight;
    const totalWidth = bays * bayWidth;

    // Create nodes, subdividing each story and bay
    for (let r = 0; r <= stories * segments; r++) {
        const y = r * (storyHeight / segments);
        const anchored = r === 0;
        for (let c = 0; c <= bays * segments; c++) {
            const x = c * (bayWidth / segments);
            nodes.push(new Node(x, y, anchored));
        }
    }

    const colsPerRow = bays * segments + 1;

    // Build column springs
    for (let r = 0; r < stories * segments; r++) {
        for (let c = 0; c <= bays * segments; c++) {
            const idx1 = r * colsPerRow + c;
            const idx2 = (r + 1) * colsPerRow + c;
            const segHeight = storyHeight / segments;
            const k = lateralStiffness(columnEI, segHeight);
            springs.push(new Spring(idx1, idx2, k));
        }
    }

    // Build beam springs
    for (let r = 0; r <= stories * segments; r++) {
        for (let c = 0; c < bays * segments; c++) {
            const idx1 = r * colsPerRow + c;
            const idx2 = r * colsPerRow + (c + 1);
            const segWidth = bayWidth / segments;
            const k = lateralStiffness(beamEI, segWidth);
            springs.push(new Spring(idx1, idx2, k));
        }
    }

    // Lump masses at story nodes (average of bays)
    const storyNodeIndices = [];
    for (let s = 1; s <= stories; s++) {
        const row = s * segments;
        const indices = [];
        for (let c = 0; c <= bays * segments; c += segments) {
            indices.push(row * colsPerRow + c);
        }
        storyNodeIndices.push(indices);
    }

    storyNodeIndices.forEach(indices => {
        const m = levelMass / indices.length;
        indices.forEach(idx => {
            nodes[idx].mass += m;
        });
    });

    const modelWidth = colsPerRow * (bayWidth / segments);
    return { nodes, springs, totalHeights, totalWidth, colsPerRow };
}

function lateralStiffness(EI, L) {
    // Equivalent lateral stiffness for a prismatic beam/column (cantilever ends fixed): 12*EI/L^3
    return (12 * EI) / Math.pow(L, 3);
}

function assembleMatrices(nodes, springs) {
    const n = nodes.length;
    const M = new Float64Array(n);
    const K = Array.from({ length: n }, () => new Float64Array(n));

    nodes.forEach((node, i) => {
        if (!node.anchored) {
            M[i] = node.mass;
        } else {
            M[i] = Number.POSITIVE_INFINITY; // anchored nodes treated as fixed masses
        }
    });

    springs.forEach(s => {
        const k = s.k;
        K[s.i][s.i] += k;
        K[s.j][s.j] += k;
        K[s.i][s.j] -= k;
        K[s.j][s.i] -= k;
    });

    return { M, K };
}

function estimateNaturalFrequency(M, K) {
    // Solve for smallest eigenvalue using power iteration on inverse power method
    const n = M.length;
    const free = [];
    for (let i = 0; i < n; i++) {
        if (M[i] < Number.POSITIVE_INFINITY / 2) free.push(i);
    }

    const size = free.length;
    const Kf = Array.from({ length: size }, () => new Float64Array(size));
    const Mf = new Float64Array(size);
    free.forEach((idx, i) => {
        Mf[i] = M[idx];
        free.forEach((jdx, j) => {
            Kf[i][j] = K[idx][jdx];
        });
    });

    let v = new Float64Array(size).map(() => Math.random());
    normalize(v);
    let lambda = 0;

    for (let iter = 0; iter < 40; iter++) {
        const w = multiplyMatVec(Kf, v);
        for (let i = 0; i < size; i++) {
            w[i] /= Mf[i];
        }
        const mu = dot(v, w);
        lambda = mu;
        v = w;
        normalize(v);
    }

    const omega = Math.sqrt(lambda);
    const freq = omega / (2 * Math.PI);
    return { freq, period: 1 / freq };
}

function normalize(v) {
    const norm = Math.sqrt(dot(v, v)) || 1;
    for (let i = 0; i < v.length; i++) v[i] /= norm;
}

function dot(a, b) {
    let s = 0;
    for (let i = 0; i < a.length; i++) s += a[i] * b[i];
    return s;
}

function multiplyMatVec(M, v) {
    const res = new Float64Array(v.length);
    for (let i = 0; i < M.length; i++) {
        let s = 0;
        for (let j = 0; j < v.length; j++) s += M[i][j] * v[j];
        res[i] = s;
    }
    return res;
}

function buildGroundMotion(duration, dt) {
    const type = document.getElementById('motionType').value;
    const peak = parseFloat(document.getElementById('peakAccel').value);
    const freq = parseFloat(document.getElementById('motionFreq').value);

    const steps = Math.floor(duration / dt);
    const accel = new Float64Array(steps);

    for (let i = 0; i < steps; i++) {
        const t = i * dt;
        if (type === 'sine') {
            accel[i] = peak * Math.sin(2 * Math.PI * freq * t) * Math.exp(-t / duration);
        } else if (type === 'pulse') {
            const ramp = t < duration / 2 ? t / (duration / 2) : (duration - t) / (duration / 2);
            accel[i] = peak * ramp;
        } else {
            // filtered noise
            const noise = (Math.random() * 2 - 1) * peak;
            accel[i] = noise * Math.exp(-t / duration);
        }
    }

    return { accel, dt };
}

function simulateStep(model, groundMotion, dampingRatio) {
    const { nodes, springs } = model;
    const dt = groundMotion.dt;
    const gamma = 0.5, beta = 0.25;

   springs.forEach(s => s.update(nodes));

   nodes.forEach((node, idx) => {
       if (node.anchored) {
            const aGround = groundMotion.currentAccel || 0;
            node.acc = aGround;
            node.vel += node.acc * dt;
            node.disp += node.vel * dt;
            return;
        }

        let restoring = 0;
        let localStiffness = 0;
        springs.forEach(s => {
            if (s.i === idx) {
                restoring += s.force;
                localStiffness += s.k;
            }
            if (s.j === idx) {
                restoring -= s.force;
                localStiffness += s.k;
            }
        });

        const c = 2 * dampingRatio * Math.sqrt(Math.max(localStiffness, 1e-6) / Math.max(node.mass, 1));

        const effectiveForce = -restoring - c * node.vel;
        const aGround = groundMotion.currentAccel || 0;
        const acc = (effectiveForce - node.mass * aGround) / node.mass;
        node.acc = acc;

        node.vel += (1 - gamma) * node.acc * dt;
        node.disp += node.vel * dt + beta * node.acc * dt * dt;
    });
}

function renderFrame(model) {
    const { nodes, springs, totalHeights, totalWidth, colsPerRow } = model;
    frameCtx.clearRect(0, 0, frameCanvas.width, frameCanvas.height);

    const padding = 60;
    const scaleX = (frameCanvas.width - 2 * padding) / totalWidth;
    const scaleY = (frameCanvas.height - 2 * padding) / totalHeights;

    const maxForce = Math.max(...springs.map(s => Math.abs(s.force))) || 1;

    springs.forEach(s => {
        const n1 = nodes[s.i];
        const n2 = nodes[s.j];
        const x1 = padding + n1.x0 * scaleX + n1.disp * 20;
        const x2 = padding + n2.x0 * scaleX + n2.disp * 20;
        const y1 = frameCanvas.height - (padding + n1.y0 * scaleY);
        const y2 = frameCanvas.height - (padding + n2.y0 * scaleY);

        const ratio = Math.max(-1, Math.min(1, s.force / maxForce));
        const color = ratioToColor(ratio);
        frameCtx.strokeStyle = color;
        frameCtx.lineWidth = 3;
        frameCtx.beginPath();
        frameCtx.moveTo(x1, y1);
        frameCtx.lineTo(x2, y2);
        frameCtx.stroke();
    });

    // Draw nodes
    nodes.forEach(n => {
        const x = padding + n.x0 * scaleX + n.disp * 20;
        const y = frameCanvas.height - (padding + n.y0 * scaleY);
        frameCtx.fillStyle = n.anchored ? '#333' : '#1f4b99';
        frameCtx.beginPath();
        frameCtx.arc(x, y, 4, 0, Math.PI * 2);
        frameCtx.fill();
    });
}

function ratioToColor(ratio) {
    // -1 compression (red), 0 neutral (yellow), +1 tension (green)
    const r = ratio < 0 ? 255 : Math.floor(255 * (1 - ratio));
    const g = ratio > 0 ? 255 : Math.floor(255 * (1 + ratio));
    const b = 40;
    return `rgb(${r},${g},${b})`;
}

function renderChart(model) {
    const { nodes, colsPerRow } = model;
    chartCtx.clearRect(0, 0, chartCanvas.width, chartCanvas.height);

    const stories = Math.floor((nodes.length / colsPerRow) - 1);
    const padding = 40;
    const maxDisp = Math.max(0.01, ...nodes.filter(n => !n.anchored).map(n => Math.abs(n.disp)));

    for (let s = 1; s <= stories; s++) {
        const row = s * (colsPerRow);
        const storyNodes = nodes.slice(row, row + colsPerRow);
        const avgDisp = storyNodes.reduce((sum, n) => sum + n.disp, 0) / storyNodes.length;
        const x = padding + (avgDisp / maxDisp) * (chartCanvas.width - 2 * padding) / 2 + (chartCanvas.width / 2);
        const y = chartCanvas.height - padding - (s / stories) * (chartCanvas.height - 2 * padding);

        chartCtx.fillStyle = '#0b7a75';
        chartCtx.beginPath();
        chartCtx.arc(x, y, 6, 0, Math.PI * 2);
        chartCtx.fill();

        if (s > 1) {
            const prevRow = (s - 1) * (colsPerRow);
            const prevNodes = nodes.slice(prevRow, prevRow + colsPerRow);
            const prevDisp = prevNodes.reduce((sum, n) => sum + n.disp, 0) / prevNodes.length;
            const px = padding + (prevDisp / maxDisp) * (chartCanvas.width - 2 * padding) / 2 + (chartCanvas.width / 2);
            const py = chartCanvas.height - padding - ((s - 1) / stories) * (chartCanvas.height - 2 * padding);
            chartCtx.strokeStyle = '#555';
            chartCtx.beginPath();
            chartCtx.moveTo(px, py);
            chartCtx.lineTo(x, y);
            chartCtx.stroke();
        }
    }

    chartCtx.strokeStyle = '#888';
    chartCtx.beginPath();
    chartCtx.moveTo(chartCanvas.width / 2, padding);
    chartCtx.lineTo(chartCanvas.width / 2, chartCanvas.height - padding + 10);
    chartCtx.stroke();
}

function updateStats(model) {
    const { M, K } = assembleMatrices(model.nodes, model.springs);
    const { freq, period } = estimateNaturalFrequency(M, K);
    freqDisplay.textContent = `${freq.toFixed(2)} Hz`;
    periodDisplay.textContent = `${period.toFixed(2)} s`;

    const { nodes, colsPerRow } = model;
    const stories = Math.floor((nodes.length / colsPerRow) - 1);
    let peak = 0;
    for (let s = 1; s <= stories; s++) {
        const row = s * colsPerRow;
        const storyNodes = nodes.slice(row, row + colsPerRow);
        const avg = storyNodes.reduce((sum, n) => sum + n.disp, 0) / storyNodes.length;
        peak = Math.max(peak, Math.abs(avg));
    }
    driftDisplay.textContent = `${(peak * 1000).toFixed(1)} mm`;
}

function step() {
    if (!running || !model) return;
    const dt = groundMotion.dt;

    if (groundMotion.index >= groundMotion.accel.length) {
        running = false;
        return;
    }

    groundMotion.currentAccel = groundMotion.accel[groundMotion.index];
    groundMotion.index += 1;

    simulateStep(model, groundMotion, parseFloat(document.getElementById('damping').value) / 100);
    renderFrame(model);
    renderChart(model);
    updateStats(model);

    displacementHistory.push(model.nodes.map(n => n.disp));
    if (displacementHistory.length > 600) displacementHistory.shift();

    animationId = requestAnimationFrame(step);
}

function resetSimulation() {
    if (!model) return;
    model.nodes.forEach(n => {
        n.disp = 0;
        n.vel = 0;
        n.acc = 0;
    });
    groundMotion.index = 0;
    displacementHistory = [];
    renderFrame(model);
    renderChart(model);
    updateStats(model);
}

function init() {
    model = buildModel();
    groundMotion = buildGroundMotion(parseFloat(document.getElementById('duration').value), 1 / 60);
    groundMotion.index = 0;
    renderFrame(model);
    renderChart(model);
    updateStats(model);
}

function bindUI() {
    document.getElementById('build').addEventListener('click', () => {
        model = buildModel();
        groundMotion = buildGroundMotion(parseFloat(document.getElementById('duration').value), 1 / 60);
        groundMotion.index = 0;
        renderFrame(model);
        renderChart(model);
        updateStats(model);
    });

    document.getElementById('play').addEventListener('click', () => {
        if (!model) init();
        running = true;
        cancelAnimationFrame(animationId);
        animationId = requestAnimationFrame(step);
    });

    document.getElementById('pause').addEventListener('click', () => {
        running = false;
        cancelAnimationFrame(animationId);
    });

    document.getElementById('reset').addEventListener('click', () => {
        running = false;
        cancelAnimationFrame(animationId);
        resetSimulation();
    });
}

bindUI();
init();
