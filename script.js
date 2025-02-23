const canvas = document.getElementById('beamCanvas');
const ctx = canvas.getContext('2d');
let loads = [];

function addLoad() {
    const type = document.getElementById('loadType').value;
    const magnitude = parseFloat(document.getElementById('loadMagnitude').value);
    const position = parseFloat(document.getElementById('loadPosition').value);
    const spanLength = parseFloat(document.getElementById('spanLength').value);
    
    if (position > spanLength) {
        alert('Load position cannot exceed span length.');
        return;
    }
    
    loads.push({ type, magnitude, position });
    updateLoadList();
}

function updateLoadList() {
    const loadList = document.getElementById('loadList');
    loadList.innerHTML = '';
    loads.forEach((load, index) => {
        const li = document.createElement('li');
        li.textContent = `${load.type} Load: ${load.magnitude} at ${load.position} ft`;
        loadList.appendChild(li);
    });
}

function analyzeBeam() {
    loads = []; // Reset loads for simplicity; in practice, manage multi-span loads
    addLoad();  // Add the current load input
    
    const spanLength = parseFloat(document.getElementById('spanLength').value);
    const depth = parseFloat(document.getElementById('depth').value);
    const height = parseFloat(document.getElementById('height').value);
    const flangeThickness = parseFloat(document.getElementById('flangeThickness').value);
    const webThickness = parseFloat(document.getElementById('webThickness').value);
    const isSimple = document.getElementById('simple').checked;
    const numSpans = isSimple ? 1 : parseInt(document.getElementById('numSpans').value);

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Scale factors
    const scaleX = 600 / (spanLength * numSpans); // 600px for beam length
    const scaleY = 100 / Math.max(depth, height); // 100px for beam height

    // Draw beam cross-section (top left)
    drawCrossSection(10, 10, depth, height, flangeThickness, webThickness, scaleY);

    // Draw beam elevation
    drawBeamElevation(100, 200, spanLength, numSpans, scaleX, depth, scaleY);

    // Analyze and draw diagrams (simple span only for simplicity)
    if (isSimple) {
        const { shear, moment } = calculateDiagrams(spanLength, loads);
        drawShearDiagram(100, 350, shear, spanLength, scaleX);
        drawMomentDiagram(100, 500, moment, spanLength, scaleX);
    } else {
        ctx.fillText('Multi-span analysis not implemented in this demo.', 100, 350);
    }
}

function drawCrossSection(x, y, depth, height, ft, wt, scale) {
    ctx.beginPath();
    ctx.rect(x, y, wt * scale, height * scale); // Web
    ctx.rect(x - (ft * scale), y, (wt + 2 * ft) * scale, ft * scale); // Top flange
    ctx.rect(x - (ft * scale), y + (height - ft) * scale, (wt + 2 * ft) * scale, ft * scale); // Bottom flange
    ctx.stroke();
}

function drawBeamElevation(x, y, spanLength, numSpans, scaleX, depth, scaleY) {
    const totalLength = spanLength * numSpans;
    ctx.beginPath();
    ctx.rect(x, y, totalLength * scaleX, depth * scaleY);
    ctx.stroke();
    
    // Draw supports
    for (let i = 0; i <= numSpans; i++) {
        ctx.beginPath();
        ctx.moveTo(x + i * spanLength * scaleX, y + depth * scaleY);
        ctx.lineTo(x + i * spanLength * scaleX - 10, y + depth * scaleY + 20);
        ctx.lineTo(x + i * spanLength * scaleX + 10, y + depth * scaleY + 20);
        ctx.fill();
    }
    
    // Draw loads
    loads.forEach(load => {
        const posX = x + load.position * scaleX;
        if (load.type === 'point') {
            ctx.beginPath();
            ctx.moveTo(posX, y - 20);
            ctx.lineTo(posX, y);
            ctx.stroke();
            ctx.fillText(`${load.magnitude} k`, posX - 10, y - 25);
        } else {
            ctx.fillRect(posX - 10, y - 10, 20, 10);
            ctx.fillText(`${load.magnitude} k/ft`, posX - 15, y - 15);
        }
    });
}

function calculateDiagrams(spanLength, loads) {
    const shear = [];
    const moment = [];
    const step = spanLength / 100;
    
    let V = 0;
    let M = 0;
    
    for (let x = 0; x <= spanLength; x += step) {
        V = 0;
        loads.forEach(load => {
            if (load.type === 'point' && x >= load.position) {
                V -= load.magnitude;
            } else if (load.type === 'uniform') {
                if (x >= load.position) V -= load.magnitude * (x - load.position);
            }
        });
        shear.push({ x, V });
        
        M += V * step;
        moment.push({ x, M });
    }
    
    // Adjust for reaction at supports (simple span)
    const R = loads.reduce((sum, load) => sum + (load.type === 'point' ? load.magnitude : load.magnitude * spanLength), 0) / 2;
    shear.forEach(point => point.V += R);
    shear.unshift({ x: 0, V: R });
    
    return { shear, moment };
}

function drawShearDiagram(x, y, shear, spanLength, scaleX) {
    ctx.beginPath();
    ctx.moveTo(x, y);
    const maxShear = Math.max(...shear.map(s => Math.abs(s.V)));
    const scaleY = 100 / maxShear;
    
    shear.forEach(point => {
        ctx.lineTo(x + point.x * scaleX, y - point.V * scaleY);
    });
    ctx.stroke();
    ctx.fillText('Shear Diagram (kips)', x, y + 20);
}

function drawMomentDiagram(x, y, moment, spanLength, scaleX) {
    ctx.beginPath();
    ctx.moveTo(x, y);
    const maxMoment = Math.max(...moment.map(m => Math.abs(m.M)));
    const scaleY = 100 / maxMoment;
    
    moment.forEach(point => {
        ctx.lineTo(x + point.x * scaleX, y - point.M * scaleY);
    });
    ctx.stroke();
    ctx.fillText('Moment Diagram (kip-ft)', x, y + 20);
}

// Toggle continuous span options
document.getElementById('continuous').addEventListener('change', () => {
    document.getElementById('continuousOptions').style.display = 'block';
});
document.getElementById('simple').addEventListener('change', () => {
    document.getElementById('continuousOptions').style.display = 'none';
});
