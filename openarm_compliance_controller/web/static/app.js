/* 
================================================================
Agent-L: Robotic Lecturer Dashboard Core JS Logic
Author: Antigravity AI
================================================================
*/

document.addEventListener('DOMContentLoaded', () => {
    initTabRouting();
    initSettingsModal();
    initChatSystem();
    initPhysicsSimulator();
    initDocViewer();
    checkServerStatus();
});

// 1. Fluid Tab Routing
function initTabRouting() {
    const navButtons = document.querySelectorAll('.nav-btn[data-tab]');
    const tabPanels = document.querySelectorAll('.tab-panel');
    const headerTitle = document.getElementById('current-tab-title');
    const headerDesc = document.getElementById('current-tab-desc');

    const tabMetadata = {
        chat: { title: 'AI Chat with Dr. L', desc: 'Robotic Kinematics & Controls Knowledge Base' },
        graphs: { title: 'System Architecture Graphs', desc: 'Full architectural maps of our OpenArm V10 system' },
        simulator: { title: 'Variable Impedance Physics Simulator', desc: 'Interactive mass-spring-damper parameter tuning' },
        docs: { title: 'Repository System Database', desc: 'Active system documentation files' }
    };

    navButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const tabName = btn.getAttribute('data-tab');
            
            // Toggle active button
            navButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            // Toggle active panel
            tabPanels.forEach(p => p.classList.remove('active'));
            document.getElementById(`tab-${tabName}`).classList.add('active');

            // Update header info
            if (tabMetadata[tabName]) {
                headerTitle.textContent = tabMetadata[tabName].title;
                headerDesc.textContent = tabMetadata[tabName].desc;
            }

            // Specific tab initializations
            if (tabName === 'simulator') {
                resizeCanvas();
            }
        });
    });

    // Handle static graph carousel switching
    const carouselBtns = document.querySelectorAll('.carousel-btn');
    const carouselImg = document.getElementById('carousel-image-view');
    const imagePaths = {
        graph: 'assets/openarm_node_graph.png',
        arch: 'assets/ros2_control_architecture.png',
        paradox: 'assets/latency_throughput_paradox.png'
    };

    carouselBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            carouselBtns.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            const imgKey = btn.getAttribute('data-img');
            if (imagePaths[imgKey]) {
                carouselImg.src = imagePaths[imgKey];
            }
        });
    });
}

// 2. Settings Modal & LocalStorage API Key Management
function initSettingsModal() {
    const modal = document.getElementById('settings-modal');
    const openBtn = document.getElementById('open-settings');
    const closeBtn = document.getElementById('close-settings');
    const saveBtn = document.getElementById('save-key-btn');
    const clearBtn = document.getElementById('clear-key-btn');
    const apiKeyInput = document.getElementById('gemini-api-key-input');
    const keyStatusText = document.getElementById('key-status-text');

    // Load existing key from localStorage
    const savedKey = localStorage.getItem('GEMINI_API_KEY');
    if (savedKey) {
        apiKeyInput.value = savedKey;
        keyStatusText.textContent = "Saved key detected in local browser storage.";
        keyStatusText.style.color = "var(--accent-green)";
    } else {
        keyStatusText.textContent = "No saved key. Add your key or configure GEMINI_API_KEY on the server.";
        keyStatusText.style.color = "var(--text-secondary)";
    }

    openBtn.addEventListener('click', () => {
        modal.classList.add('active');
        checkServerStatus(); // refresh backend status indicators
    });
    closeBtn.addEventListener('click', () => modal.classList.remove('active'));
    modal.addEventListener('click', (e) => {
        if (e.target === modal) modal.classList.remove('active');
    });

    saveBtn.addEventListener('click', () => {
        const key = apiKeyInput.value.trim();
        if (key) {
            localStorage.setItem('GEMINI_API_KEY', key);
            keyStatusText.textContent = "API Key saved successfully!";
            keyStatusText.style.color = "var(--accent-green)";
            setTimeout(() => modal.classList.remove('active'), 1000);
        } else {
            alert("Please enter a valid API key.");
        }
    });

    clearBtn.addEventListener('click', () => {
        localStorage.removeItem('GEMINI_API_KEY');
        apiKeyInput.value = '';
        keyStatusText.textContent = "Saved key cleared.";
        keyStatusText.style.color = "var(--text-secondary)";
    });
}

// 3. Conversational AI Chat System
function initChatSystem() {
    const chatInput = document.getElementById('chat-input-field');
    const sendBtn = document.getElementById('send-chat-btn');
    const chatMessagesBox = document.getElementById('chat-messages-box');
    
    let chatHistory = [];

    function addMessage(role, content) {
        const msgDiv = document.createElement('div');
        msgDiv.className = `chat-msg ${role === 'user' ? 'user' : 'system'}`;
        
        const avatar = document.createElement('div');
        avatar.className = 'avatar';
        avatar.innerHTML = role === 'user' ? '<i class="fa-solid fa-user"></i>' : '<i class="fa-solid fa-user-tie"></i>';
        
        const bubble = document.createElement('div');
        bubble.className = 'bubble';
        
        // Parse markdown formatting using marked library
        if (role === 'system') {
            bubble.innerHTML = marked.parse(content);
        } else {
            bubble.textContent = content;
        }

        msgDiv.appendChild(avatar);
        msgDiv.appendChild(bubble);
        chatMessagesBox.appendChild(msgDiv);
        
        // Auto scroll to bottom
        chatMessagesBox.scrollTop = chatMessagesBox.scrollHeight;
    }

    async function handleSend() {
        const query = chatInput.value.trim();
        if (!query) return;

        // Clear input field
        chatInput.value = '';
        
        // Add user message to UI
        addMessage('user', query);

        // Add thinking loader bubble from Dr. L
        const loaderDiv = document.createElement('div');
        loaderDiv.className = 'chat-msg system loader-msg';
        loaderDiv.innerHTML = `
            <div class="avatar"><i class="fa-solid fa-spinner fa-spin"></i></div>
            <div class="bubble"><p><em>Dr. L is consulting the repository files...</em></p></div>
        `;
        chatMessagesBox.appendChild(loaderDiv);
        chatMessagesBox.scrollTop = chatMessagesBox.scrollHeight;

        // Retrieve local storage key
        const apiKey = localStorage.getItem('GEMINI_API_KEY') || "";

        try {
            const response = await fetch('/api/chat', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    message: query,
                    apiKey: apiKey,
                    history: chatHistory
                })
            });

            // Remove loading bubble
            const loaders = document.querySelectorAll('.loader-msg');
            loaders.forEach(l => l.remove());

            if (!response.ok) {
                const errorData = await response.json();
                addMessage('system', `⚠️ **Error ${response.status}:** ${errorData.detail || 'Failed to get a response.'}`);
                return;
            }

            const data = await response.json();
            
            // Add system response to UI
            addMessage('system', data.response);
            
            // Track history (keep history compact for tokens)
            chatHistory.push({ role: 'user', content: query });
            chatHistory.push({ role: 'model', content: data.response });
            if (chatHistory.length > 10) chatHistory = chatHistory.slice(-10);

        } catch (e) {
            // Remove loading bubble
            const loaders = document.querySelectorAll('.loader-msg');
            loaders.forEach(l => l.remove());
            addMessage('system', `❌ **Network Connection Error:** Could not connect to the local FastAPI server. Please verify uvicorn is running.`);
            console.error(e);
        }
    }

    sendBtn.addEventListener('click', handleSend);
    chatInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') handleSend();
    });
}

// 4. Live Physics Joint Simulator (Mass-Spring-Damper Euler Integrator)
let resizeCanvas = () => {}; // global hook to bind on tab changes
function initPhysicsSimulator() {
    const canvas = document.getElementById('physics-canvas');
    const ctx = canvas.getContext('2d');
    
    // Sliders & UI Elements
    const kpSlider = document.getElementById('sim-kp');
    const kdSlider = document.getElementById('sim-kd');
    const massSlider = document.getElementById('sim-mass');
    
    const kpVal = document.getElementById('kp-val');
    const kdVal = document.getElementById('kd-val');
    const massVal = document.getElementById('mass-val');
    
    const dampingStatus = document.getElementById('damping-status');
    
    // Presets
    const transitBtn = document.getElementById('preset-stiff');
    const contactBtn = document.getElementById('preset-soft');
    const teachBtn = document.getElementById('preset-teach');

    // Simulation Parameters
    let kp = parseFloat(kpSlider.value);
    let kd = parseFloat(kdSlider.value);
    let mass = parseFloat(massSlider.value);
    
    // Physical state variables (Joint angle in radians)
    let q = 0.0;          // Actual Position
    let dq = 0.0;         // Actual Velocity
    let q_des = 0.0;      // Target Position
    let dq_des = 0.0;     // Target Velocity
    
    // Joint drawing configuration
    let centerX, centerY, linkLength;
    
    // Interactivity
    let isDragging = false;
    let dragAngle = 0.0;

    resizeCanvas = () => {
        const rect = canvas.parentElement.getBoundingClientRect();
        canvas.width = rect.width;
        canvas.height = rect.height;
        centerX = canvas.width / 2;
        centerY = canvas.height / 2;
        linkLength = Math.min(canvas.width, canvas.height) * 0.35;
    };
    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();

    // Event listeners to tune values in real-time
    kpSlider.addEventListener('input', () => {
        kp = parseFloat(kpSlider.value);
        kpVal.textContent = kp.toFixed(1);
        updateDampingStatus();
        clearPresetActives();
    });

    kdSlider.addEventListener('input', () => {
        kd = parseFloat(kdSlider.value);
        kdVal.textContent = kd.toFixed(1);
        updateDampingStatus();
        clearPresetActives();
    });

    massSlider.addEventListener('input', () => {
        mass = parseFloat(massSlider.value);
        massVal.textContent = `${mass.toFixed(1)} kg`;
        updateDampingStatus();
        clearPresetActives();
    });

    function clearPresetActives() {
        [transitBtn, contactBtn, teachBtn].forEach(b => b.classList.remove('active'));
    }

    // Set presets
    transitBtn.addEventListener('click', () => {
        setPresetValues(120, 6.0, 1.0);
        transitBtn.classList.add('active');
    });
    contactBtn.addEventListener('click', () => {
        setPresetValues(30, 2.0, 1.0);
        contactBtn.classList.add('active');
    });
    teachBtn.addEventListener('click', () => {
        setPresetValues(3, 0.5, 1.0);
        teachBtn.classList.add('active');
    });

    function setPresetValues(p_kp, p_kd, p_m) {
        clearPresetActives();
        
        kpSlider.value = p_kp;
        kpVal.textContent = p_kp.toFixed(1);
        kp = p_kp;

        kdSlider.value = p_kd;
        kdVal.textContent = p_kd.toFixed(1);
        kd = p_kd;

        massSlider.value = p_m;
        massVal.textContent = `${p_m.toFixed(1)} kg`;
        mass = p_m;

        updateDampingStatus();
    }

    // Classification of damping category
    function updateDampingStatus() {
        if (kp === 0) {
            dampingStatus.textContent = "Teach Mode (Floating)";
            dampingStatus.className = "badge";
            return;
        }
        
        // Critical Damping ratio: ζ = c / (2 * sqrt(m * k))
        const criticalDamping = 2 * Math.sqrt(mass * kp);
        const ratio = kd / criticalDamping;

        if (ratio < 0.05) {
            dampingStatus.textContent = "Undamped / Unstable";
            dampingStatus.className = "badge warning";
        } else if (ratio < 0.9) {
            dampingStatus.textContent = "Underdamped (Overshoots)";
            dampingStatus.className = "badge warning";
        } else if (ratio < 1.1) {
            dampingStatus.textContent = "Critically Damped (Optimal)";
            dampingStatus.className = "badge";
        } else {
            dampingStatus.textContent = "Overdamped (Sluggish)";
            dampingStatus.className = "badge";
        }
    }

    // Capture mouse/touch interactions to physically pull the joint
    function getMouseAngle(e) {
        const rect = canvas.getBoundingClientRect();
        const clientX = (e.touches ? e.touches[0].clientX : e.clientX) - rect.left;
        const clientY = (e.touches ? e.touches[0].clientY : e.clientY) - rect.top;
        return Math.atan2(clientY - centerY, clientX - centerX);
    }

    function startDrag(e) {
        const rect = canvas.getBoundingClientRect();
        const clientX = (e.touches ? e.touches[0].clientX : e.clientX) - rect.left;
        const clientY = (e.touches ? e.touches[0].clientY : e.clientY) - rect.top;
        
        // Only trigger drag if clicking near the end effector tip
        const tipX = centerX + linkLength * Math.cos(q);
        const tipY = centerY + linkLength * Math.sin(q);
        const dist = Math.hypot(clientX - tipX, clientY - tipY);
        
        if (dist < 40) {
            isDragging = true;
            dragAngle = getMouseAngle(e);
            e.preventDefault();
        }
    }

    function doDrag(e) {
        if (!isDragging) return;
        dragAngle = getMouseAngle(e);
        q = dragAngle;
        dq = 0; // stop moving while holding
        e.preventDefault();
    }

    function endDrag() {
        isDragging = false;
    }

    canvas.addEventListener('mousedown', startDrag);
    canvas.addEventListener('mousemove', doDrag);
    window.addEventListener('mouseup', endDrag);

    canvas.addEventListener('touchstart', startDrag);
    canvas.addEventListener('touchmove', doDrag);
    window.addEventListener('touchend', endDrag);

    // Dynamic path animation for target pose (sine wave target)
    let time = 0;

    // Core Physics Update loop (60Hz Euler integration)
    function updatePhysics() {
        time += 0.016;

        // Command desired position to follow a gentle sinusoidal trajectory
        // to show tracking compliance.
        q_des = 0.8 * Math.sin(time * 1.5);
        dq_des = 1.2 * Math.cos(time * 1.5);

        if (!isDragging) {
            // Apply the exact impedance controller law
            const tau_ff = 0.5 * Math.sin(q); // mock gravity compensation feedforward
            const tau_fb = kp * (q_des - q) + kd * (dq_des - dq);
            const tau_cmd = tau_ff + tau_fb;

            // Simple arm dynamics: Accel = (Torque - Viscous Joint Friction - Gravity) / Mass
            const viscous_friction = 0.8 * dq;
            const physical_gravity = 0.5 * Math.sin(q);
            const accel = (tau_cmd - viscous_friction - physical_gravity) / mass;

            // Euler integration
            dq += accel * 0.016;
            q += dq * 0.016;
        }

        renderSimulation();
        requestAnimationFrame(updatePhysics);
    }

    // Render joint, spring, and damper on canvas
    function renderSimulation() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // 1. Draw Grid Background
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.015)';
        ctx.lineWidth = 1;
        const gridSize = 40;
        for (let x = 0; x < canvas.width; x += gridSize) {
            ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
        }
        for (let y = 0; y < canvas.height; y += gridSize) {
            ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
        }

        // 2. Draw Desired Target position vector
        const desX = centerX + linkLength * Math.cos(q_des);
        const desY = centerY + linkLength * Math.sin(q_des);

        ctx.strokeStyle = 'rgba(192, 132, 252, 0.2)';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(desX, desY);
        ctx.stroke();
        ctx.setLineDash([]);

        // Target marker dot
        ctx.fillStyle = 'var(--accent-purple)';
        ctx.shadowColor = 'var(--accent-purple)';
        ctx.shadowBlur = 10;
        ctx.beginPath();
        ctx.arc(desX, desY, 7, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0;

        // 3. Draw Actual Compliant Joint Link
        const actX = centerX + linkLength * Math.cos(q);
        const actY = centerY + linkLength * Math.sin(q);

        ctx.strokeStyle = 'var(--accent-cyan)';
        ctx.lineWidth = 6;
        ctx.lineCap = 'round';
        ctx.shadowColor = 'var(--accent-cyan)';
        ctx.shadowBlur = 15;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(actX, actY);
        ctx.stroke();
        ctx.shadowBlur = 0;

        // Actual End effector cap
        ctx.fillStyle = '#000';
        ctx.strokeStyle = 'var(--accent-cyan)';
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.arc(actX, actY, 12, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // 4. Draw Center Rotary Actuator housing
        ctx.fillStyle = 'var(--bg-dark)';
        ctx.strokeStyle = 'var(--border-glow)';
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.arc(centerX, centerY, 30, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Draw inner gear patterns to represent DaMiao actuator
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
        ctx.lineWidth = 2;
        for (let a = 0; a < Math.PI * 2; a += Math.PI / 6) {
            ctx.beginPath();
            ctx.moveTo(centerX + 15 * Math.cos(a), centerY + 15 * Math.sin(a));
            ctx.lineTo(centerX + 25 * Math.cos(a), centerY + 25 * Math.sin(a));
            ctx.stroke();
        }

        // Inner axle cap
        ctx.fillStyle = 'var(--accent-cyan)';
        ctx.beginPath();
        ctx.arc(centerX, centerY, 8, 0, Math.PI * 2);
        ctx.fill();
    }

    // Boot Simulator
    setPresetValues(50, 2.5, 1.0);
    requestAnimationFrame(updatePhysics);
}

// 5. System Markdown Documentation Viewer
function initDocViewer() {
    const docButtons = document.querySelectorAll('.doc-link-btn[data-file]');
    const docRenderArea = document.getElementById('doc-render-area');

    async function loadMarkdownFile(fileName) {
        docRenderArea.innerHTML = `<p class="loader"><i class="fa-solid fa-spinner fa-spin"></i> Loading context repository file: ${fileName}...</p>`;
        
        try {
            // Read files from backend API context loading
            const response = await fetch('/api/status');
            if (!response.ok) throw new Error("Backend offline");

            // Build dynamic path reference mapping
            const fileMapping = {
                architecture: 'ARCHITECTURE.md',
                lecturer: 'AGENT_L_LECTURER.md',
                complete_node_graph: 'COMPLETE_NODE_GRAPH.md',
                proprioceptive_force: 'PROPRIOCEPTIVE_FORCE.md',
                teach_mode_guide: 'Enable_Teaching_Biarm.md'
            };

            const realName = fileMapping[fileName] || 'ARCHITECTURE.md';
            
            // To make this simple and robust, we can query the backend which parses the files
            // or fetch the static version directly. Since we preloaded system context in server.py,
            // we can retrieve it by matching lines. Let's read from the local server.
            const docPaths = {
                architecture: '/home/user/ros2_ws/src/ARCHITECTURE.md',
                lecturer: '/home/user/ros2_ws/src/impedance_control/openarm_compliance_controller/AGENT_L_LECTURER.md',
                complete_node_graph: '/home/user/.gemini/antigravity/brain/a7d60202-aa60-4271-9c6b-0407bf12d883/openarm_complete_node_graph.md',
                proprioceptive_force: '/home/user/ros2_ws/src/impedance_control/openarm_compliance_controller/PROPRIOCEPTIVE_FORCE.md',
                teach_mode_guide: '/home/user/ros2_ws/src/impedance_control/openarm_compliance_controller/Enable_Teaching_Biarm.md'
            };

            // To fetch a local file robustly without exposing file reading on arbitrary paths,
            // we can fall back to standard HTML pages or preloaded assets, or read them directly.
            // Let's implement a direct mock fetch or render a clean descriptive system page.
            // Wait, we can fetch these files since FastAPI has access to directories! Let's expose an endpoint or simply render a beautiful
            // professor summary of each file if the backend can't read it. But wait, server.py loads SYSTEM_CONTEXT! We can query uvicorn.
            // Let's write a quick doc fetcher or render structured guides elegantly.
            
            // Let's query uvicorn by matching the preloaded context!
            // To make this absolutely bulletproof and clean, let's look up if we have standard documentation.
            // We will render standard, beautiful HTML parsed from the actual files or fallback to clear summaries.
            
            // Let's implement a lightweight route in server.py or just fetch the file if it's placed in static.
            // Wait, since we are using Python to write files, we can easily add a quick API route `/api/docs/{name}` in server.py!
            // Let's modify server.py to support this, or since uvicorn is running, let's keep it simple.
            // Let's fetch the file content from a simple document fetcher. Since we didn't add it yet, let's write a simple route.
            // Actually, we can fetch them if we mount them or let's add a `/api/docs` route! Let's do a multi_replace edit on server.py
            // to support `/api/docs/{fileName}`. That is extremely clean and reliable!
            const docRes = await fetch(`/api/docs?file=${fileName}`);
            if (!docRes.ok) throw new Error("Doc fetch failed");
            
            const docData = await docRes.json();
            docRenderArea.innerHTML = marked.parse(docData.content);

        } catch (e) {
            console.error(e);
            docRenderArea.innerHTML = `
                <div style="padding: 40px; text-align: center;">
                    <i class="fa-solid fa-triangle-exclamation" style="font-size: 40px; color: var(--accent-red); margin-bottom: 20px;"></i>
                    <h4>Documentation Sync Required</h4>
                    <p style="margin-top: 10px; font-size:14px; color: var(--text-secondary);">Could not load the markdown file from the local workspace. Please verify uvicorn is running or set up the API docs route.</p>
                </div>
            `;
        }
    }

    docButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            docButtons.forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            const fileKey = btn.getAttribute('data-file');
            loadMarkdownFile(fileKey);
        });
    });

    // Load initial file
    loadMarkdownFile('architecture');
}

// 6. Server Health Checking
async function checkServerStatus() {
    const statusDot = document.querySelector('.pulse-dot');
    const statusText = document.getElementById('system-status-text');
    const keyStatusText = document.getElementById('key-status-text');

    try {
        const response = await fetch('/api/status');
        if (response.ok) {
            const data = await response.json();
            statusDot.className = "pulse-dot green";
            statusText.textContent = "Server: Online";
            
            // Check if server already has GEMINI_API_KEY set in environment
            if (data.is_api_key_env_set) {
                keyStatusText.textContent = "Server environment key (GEMINI_API_KEY) detected and active.";
                keyStatusText.style.color = "var(--accent-green)";
            }
        } else {
            throw new Error("Server error");
        }
    } catch (e) {
        statusDot.className = "pulse-dot";
        statusText.textContent = "Server: Offline";
        keyStatusText.textContent = "FastAPI server is unreachable. Please run 'python3 server.py' in the web directory.";
        keyStatusText.style.color = "var(--accent-red)";
    }
}
