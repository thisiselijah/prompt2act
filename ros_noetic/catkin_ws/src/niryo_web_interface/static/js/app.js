// Enhanced Desktop UI JavaScript for Niryo Robot Control

// Toast notification system
function showToast(message, type = 'info', duration = 3000) {
    const container = document.getElementById('toast-container');
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.innerHTML = `
        <div style="display: flex; justify-content: space-between; align-items: center;">
            <span>${message}</span>
            <button onclick="this.parentElement.parentElement.remove()" style="background: none; border: none; font-size: 1.2rem; cursor: pointer; color: #666;">×</button>
        </div>
    `;
    
    container.appendChild(toast);
    
    // Trigger animation
    setTimeout(() => toast.classList.add('show'), 100);
    
    // Auto remove
    setTimeout(() => {
        if (toast.parentElement) {
            toast.classList.remove('show');
            setTimeout(() => toast.remove(), 300);
        }
    }, duration);
}

// Enhanced API call wrapper
async function apiCall(url, options = {}) {
    try {
        const response = await fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            ...options
        });
        
        if (!response.ok) {
            const errorData = await response.json().catch(() => ({}));
            throw new Error(errorData.message || `HTTP ${response.status}: ${response.statusText}`);
        }
        
        return await response.json();
    } catch (error) {
        console.error(`API call failed for ${url}:`, error);
        throw error;
    }
}

// Robot control functions
function open_and_close(direction) {
    const actionText = direction === 'open' ? '打開夾爪' : '夾取物體';
    
    apiCall('/open_and_close', {
        body: JSON.stringify({ direction: direction })
    })
    .then(data => {
        showToast(`✅ ${actionText}指令已發送`, 'info');
        console.log(data);
    })
    .catch(error => {
        showToast(`❌ ${actionText}失敗: ${error.message}`, 'error');
    });
}

function setLearningMode() {
    apiCall('/learning_mode')
    .then(data => {
        showToast('🎯 已切換至 Learning Mode', 'info');
        console.log(data);
    })
    .catch(error => {
        showToast(`❌ Learning Mode 切換失敗: ${error.message}`, 'error');
    });
}

function pickObject(color) {
    const colorText = color === 'blue_block' ? '藍色' : '紅色';
    
    apiCall('/pick_object', {
        body: JSON.stringify({ label: color })
    })
    .then(data => {
        showToast(`🤖 正在抓取${colorText}方塊...`, 'info');
        console.log(data);
    })
    .catch(error => {
        showToast(`❌ 抓取${colorText}方塊失敗: ${error.message}`, 'error');
    });
}

function moveToHome() {
    apiCall('/move_to_home')
    .then(data => {
        showToast('🏠 機械手臂正在回到起始位置', 'info');
        console.log(data);
    })
    .catch(error => {
        showToast(`❌ 回到起始位置失敗: ${error.message}`, 'error');
    });
}

// Enhanced camera reconnection
function reconnectCamera() {
    const btn = document.getElementById('camera-reconnect-btn');
    const originalText = btn.textContent;
    
    // Update button state
    btn.textContent = '🔄 連接中...';
    btn.disabled = true;
    btn.style.opacity = '0.6';
    
    showToast('📷 正在重新連接攝影機...', 'info');
    
    apiCall('/reconnect_camera')
    .then(data => {
        showToast('✅ 攝影機重新連接成功', 'info');
        
        // Refresh camera stream with cache busting
        const img = document.getElementById('camera-stream');
        const src = img.getAttribute('src').split('?')[0];
        img.src = `${src}?t=${Date.now()}`;
        
        // Update behavior tree visualization
        updateBehaviorTreeStatus('camera-connected');
    })
    .catch(error => {
        showToast(`❌ 攝影機重新連接失敗: ${error.message}`, 'error');
        updateBehaviorTreeStatus('camera-disconnected');
    })
    .finally(() => {
        // Restore button state
        btn.textContent = originalText;
        btn.disabled = false;
        btn.style.opacity = '1';
        
        // Immediate status check
        checkCameraStatus();
    });
}

// Enhanced camera status checking
function checkCameraStatus() {
    const statusIndicator = document.getElementById('status-indicator');
    const statusText = document.getElementById('status-text');

    // Set checking state
    statusIndicator.className = 'status-indicator checking';
    statusText.textContent = '檢查中...';

    fetch('/camera_status')
    .then(response => response.json())
    .then(data => {
        if (data.camera_available) {
            statusIndicator.className = 'status-indicator connected';
            statusText.textContent = '攝影機已連接';
            updateBehaviorTreeStatus('camera-connected');
        } else {
            statusIndicator.className = 'status-indicator disconnected';
            statusText.textContent = '攝影機未連接';
            updateBehaviorTreeStatus('camera-disconnected');
        }
    })
    .catch(err => {
        statusIndicator.className = 'status-indicator disconnected';
        statusText.textContent = '狀態檢查失敗';
        updateBehaviorTreeStatus('error');
        console.error('Camera status check failed:', err);
    });
}

// Behavior tree visualization update
function updateBehaviorTreeStatus(status) {
    fetchBehaviorTreeData().then(treeData => {
        if (treeData && treeData.has_data) {
            renderBehaviorTree(treeData);
        } else {
            // Fallback to basic status update
            const nodes = document.querySelectorAll('.tree-node');
            nodes.forEach(node => node.classList.remove('active'));
            
            switch(status) {
                case 'camera-connected':
                    const visionNode = Array.from(nodes).find(n => n.textContent.includes('視覺處理'));
                    if (visionNode) visionNode.classList.add('active');
                    break;
            }
        }
    });
}

// Fetch live behavior tree data
async function fetchBehaviorTreeData() {
    try {
        const response = await fetch('/behavior_tree_status');
        if (response.ok) {
            return await response.json();
        }
    } catch (error) {
        console.error('Failed to fetch behavior tree data:', error);
    }
    return null;
}

// Render live behavior tree structure
function renderBehaviorTree(treeData) {
    const container = document.querySelector('.behavior-tree-display');
    if (!container || !treeData.structure) return;
    
    // Clear existing content
    container.innerHTML = '';
    
    // Render the tree structure
    const treeHTML = renderTreeNode(treeData.structure, treeData.status);
    container.appendChild(treeHTML);
}

// Recursive function to render tree nodes
function renderTreeNode(node, statusData) {
    const nodeElement = document.createElement('div');
    nodeElement.className = 'tree-node-container';
    
    // Get node status
    const nodeStatus = getNodeStatus(node.id, statusData);
    const statusClass = getStatusClass(nodeStatus);
    
    // Create node display
    const nodeDisplay = document.createElement('div');
    nodeDisplay.className = `tree-node ${statusClass}`;
    nodeDisplay.textContent = `${node.name} (${node.type})`;
    nodeDisplay.title = `ID: ${node.id}, Status: ${nodeStatus}, Type: ${node.composite_type || 'behavior'}`;
    
    nodeElement.appendChild(nodeDisplay);
    
    // Render children if they exist
    if (node.children && node.children.length > 0) {
        const childrenContainer = document.createElement('div');
        childrenContainer.className = 'tree-children';
        
        node.children.forEach(child => {
            const childElement = renderTreeNode(child, statusData);
            childrenContainer.appendChild(childElement);
        });
        
        nodeElement.appendChild(childrenContainer);
    }
    
    return nodeElement;
}

// Get status for a specific node
function getNodeStatus(nodeId, statusData) {
    if (!statusData || !statusData.nodes) return 'INVALID';
    
    const nodeStatus = statusData.nodes.find(n => n.id === nodeId);
    return nodeStatus ? nodeStatus.status : 'INVALID';
}

// Convert status to CSS class
function getStatusClass(status) {
    switch(status) {
        case 'SUCCESS': return 'status-success';
        case 'RUNNING': return 'status-running';
        case 'FAILURE': return 'status-failure';
        default: return 'status-invalid';
    }
}

// Voice input placeholder (implement based on your speech recognition setup)
function startVoiceInput() {
    showToast('🎤 語音輸入功能開發中...', 'warning');
    
    // Placeholder for voice recognition implementation
    // This would integrate with your speech recognition system
    console.log('Voice input requested');
}

// Enhanced initialization
document.addEventListener('DOMContentLoaded', function() {
    // Initial camera status check
    checkCameraStatus();
    
    // Periodic status updates (every 5 seconds)
    setInterval(checkCameraStatus, 5000);
    
    // Initialize behavior tree display
    updateBehaviorTreeStatus('camera-disconnected');
    
    // Show welcome message
    setTimeout(() => {
        showToast('🤖 Niryo 控制台已就緒', 'info');
    }, 1000);
    
    // Initialize behavior tree polling
    setInterval(async () => {
        const treeData = await fetchBehaviorTreeData();
        if (treeData && treeData.has_data) {
            renderBehaviorTree(treeData);
        }
    }, 1000); // Update every second
    
    // Initial behavior tree load
    fetchBehaviorTreeData().then(treeData => {
        if (treeData && treeData.has_data) {
            renderBehaviorTree(treeData);
        }
    });
    
    // Add keyboard shortcuts
    document.addEventListener('keydown', function(e) {
        if (e.ctrlKey || e.metaKey) {
            switch(e.key) {
                case 'h':
                    e.preventDefault();
                    moveToHome();
                    break;
                case 'r':
                    e.preventDefault();
                    reconnectCamera();
                    break;
                case 'l':
                    e.preventDefault();
                    setLearningMode();
                    break;
            }
        }
    });
    
    console.log('🚀 Niryo Desktop Control Interface Initialized');
    console.log('Keyboard shortcuts: Ctrl+H (Home), Ctrl+R (Reconnect), Ctrl+L (Learning Mode)');
});



