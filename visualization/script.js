// Canvas setup
const canvas = document.getElementById('robotCanvas');
const ctx = canvas.getContext('2d');

// Robot state
let robotState = {
    x: 0,
    y: 0,
    theta: 0,
    linear: 0,
    angular: 0,
    cmdType: 'STOP'
};

// Trail tracking
let trail = [];
const MAX_TRAIL_POINTS = 200;

// Canvas parameters
let scale = 200; // pixels per meter
let offsetX = 0;
let offsetY = 0;

// Resize canvas
function resizeCanvas() {
    canvas.width = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;
    offsetX = canvas.width / 2;
    offsetY = canvas.height / 2;
    draw();
}

window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Draw grid
function drawGrid() {
    ctx.strokeStyle = '#1a1a1a';
    ctx.lineWidth = 1;

    const gridSize = 1; // 1 meter grid
    const startX = Math.floor(-offsetX / scale / gridSize) * gridSize;
    const startY = Math.floor(-offsetY / scale / gridSize) * gridSize;
    const endX = Math.ceil((canvas.width - offsetX) / scale / gridSize) * gridSize;
    const endY = Math.ceil((canvas.height - offsetY) / scale / gridSize) * gridSize;

    // Vertical lines
    for (let x = startX; x <= endX; x += gridSize) {
        const px = offsetX + x * scale;
        ctx.beginPath();
        ctx.moveTo(px, 0);
        ctx.lineTo(px, canvas.height);
        ctx.stroke();
    }

    // Horizontal lines
    for (let y = startY; y <= endY; y += gridSize) {
        const py = offsetY - y * scale;
        ctx.beginPath();
        ctx.moveTo(0, py);
        ctx.lineTo(canvas.width, py);
        ctx.stroke();
    }

    // Draw axes (thicker)
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 2;
    
    // X-axis
    ctx.beginPath();
    ctx.moveTo(0, offsetY);
    ctx.lineTo(canvas.width, offsetY);
    ctx.stroke();
    
    // Y-axis
    ctx.beginPath();
    ctx.moveTo(offsetX, 0);
    ctx.lineTo(offsetX, canvas.height);
    ctx.stroke();

    // Axis labels
    ctx.fillStyle = '#666';
    ctx.font = '12px monospace';
    ctx.fillText('X', canvas.width - 20, offsetY - 10);
    ctx.fillText('Y', offsetX + 10, 20);
    ctx.fillText('0', offsetX + 5, offsetY - 5);
}

// Draw trail
function drawTrail() {
    if (trail.length < 2) return;

    ctx.strokeStyle = '#ff6b9d';
    ctx.lineWidth = 2;
    ctx.beginPath();

    for (let i = 0; i < trail.length; i++) {
        const point = trail[i];
        const px = offsetX + point.x * scale;
        const py = offsetY - point.y * scale;

        if (i === 0) {
            ctx.moveTo(px, py);
        } else {
            ctx.lineTo(px, py);
        }
    }

    ctx.stroke();

    // Draw trail points
    ctx.fillStyle = '#ff6b9d';
    trail.forEach((point, i) => {
        if (i % 5 === 0) { // Draw every 5th point
            const px = offsetX + point.x * scale;
            const py = offsetY - point.y * scale;
            ctx.beginPath();
            ctx.arc(px, py, 2, 0, Math.PI * 2);
            ctx.fill();
        }
    });
}

// Draw robot
function drawRobot() {
    const px = offsetX + robotState.x * scale;
    const py = offsetY - robotState.y * scale;
    const robotRadius = 0.3 * scale; // 30cm radius

    // Robot body (circle)
    ctx.fillStyle = '#4a9eff';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(px, py, robotRadius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Direction arrow
    ctx.strokeStyle = '#ffffff';
    ctx.fillStyle = '#ffffff';
    ctx.lineWidth = 3;
    
    const arrowLength = robotRadius * 1.2;
    const arrowEndX = px + Math.cos(robotState.theta) * arrowLength;
    const arrowEndY = py - Math.sin(robotState.theta) * arrowLength;
    
    // Arrow line
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(arrowEndX, arrowEndY);
    ctx.stroke();
    
    // Arrow head
    const headSize = 10;
    const angle1 = robotState.theta - Math.PI + Math.PI / 6;
    const angle2 = robotState.theta - Math.PI - Math.PI / 6;
    
    ctx.beginPath();
    ctx.moveTo(arrowEndX, arrowEndY);
    ctx.lineTo(
        arrowEndX + Math.cos(angle1) * headSize,
        arrowEndY - Math.sin(angle1) * headSize
    );
    ctx.moveTo(arrowEndX, arrowEndY);
    ctx.lineTo(
        arrowEndX + Math.cos(angle2) * headSize,
        arrowEndY - Math.sin(angle2) * headSize
    );
    ctx.stroke();

    // Robot position text
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 12px monospace';
    ctx.fillText(
        `(${robotState.x.toFixed(2)}, ${robotState.y.toFixed(2)})`,
        px + robotRadius + 10,
        py - robotRadius - 10
    );
}

// Main draw function
function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawGrid();
    drawTrail();
    drawRobot();
}

// Add point to trail
function addTrailPoint(x, y) {
    trail.push({ x, y });
    if (trail.length > MAX_TRAIL_POINTS) {
        trail.shift();
    }
}

// Clear trail button
document.getElementById('clearTrail').addEventListener('click', () => {
    trail = [];
    draw();
});

// Update panels
function updatePanels() {
    // Panel 1: ROS2 Command
    document.getElementById('cmd_linear').textContent = `${robotState.linear.toFixed(2)} m/s`;
    document.getElementById('cmd_angular').textContent = `${robotState.angular.toFixed(2)} rad/s`;
    document.getElementById('cmd_type').textContent = getCmdTypeIcon(robotState.cmdType);

    // Panel 3: Bridge (static for now)
    document.getElementById('bridge_status').textContent = '✅ Active';
    document.getElementById('bridge_info').textContent = 'Humble → Melodic';

    // Panel 4: Robot State
    document.getElementById('pos_x').textContent = `${robotState.x.toFixed(2)} m`;
    document.getElementById('pos_y').textContent = `${robotState.y.toFixed(2)} m`;
    document.getElementById('pos_theta').textContent = `${(robotState.theta * 180 / Math.PI).toFixed(1)}°`;
    document.getElementById('robot_executing').textContent = getCmdTypeIcon(robotState.cmdType);
}

// Get command type icon
function getCmdTypeIcon(cmdType) {
    // Obsługa nowych komend z kwadratu
    if (cmdType.includes('BÓK')) {
        return `⬆️ ${cmdType}`;
    }
    if (cmdType.includes('OBRÓT')) {
        return `🔄 ${cmdType}`;
    }
    
    // Stare komendy (fallback)
    const icons = {
        'FORWARD': '⬆️ PRZÓD',
        'BACKWARD': '⬇️ TYŁ',
        'LEFT': '⬅️ LEWO',
        'RIGHT': '➡️ PRAWO',
        'STOP': '⏸️ STOP',
        'IDLE': '⏸️ IDLE'
    };
    return icons[cmdType] || `❓ ${cmdType}`;
}

// Fetch robot state from API
async function fetchRobotState() {
    try {
        const response = await fetch('http://localhost:5001/api/robot_state');
        const data = await response.json();
        
        if (data.success) {
            const oldX = robotState.x;
            const oldY = robotState.y;
            
            robotState.x = data.x || 0;
            robotState.y = data.y || 0;
            robotState.theta = data.theta || 0;
            
            // Detect movement for trail
            const distance = Math.sqrt(
                Math.pow(robotState.x - oldX, 2) + 
                Math.pow(robotState.y - oldY, 2)
            );
            
            if (distance > 0.01) { // 1cm threshold
                addTrailPoint(robotState.x, robotState.y);
            }
            
            // Infer command type from movement (simplified)
            if (Math.abs(data.theta_deg) < 5) {
                robotState.cmdType = data.x > oldX ? 'FORWARD' : (data.x < oldX ? 'BACKWARD' : 'IDLE');
            } else {
                robotState.cmdType = 'TURNING';
            }
            
            // Update status
            document.getElementById('apiStatus').className = 'status-dot connected';
            document.getElementById('statusText').textContent = 'Connected to Robot';
            document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
            
            updatePanels();
            draw();
        }
    } catch (error) {
        console.error('Error fetching robot state:', error);
        document.getElementById('apiStatus').className = 'status-dot error';
        document.getElementById('statusText').textContent = 'Connection Error';
    }
}

// Start polling
setInterval(fetchRobotState, 100); // 10Hz update rate
fetchRobotState(); // Initial fetch