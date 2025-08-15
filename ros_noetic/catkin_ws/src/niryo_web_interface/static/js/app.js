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

// D3.js Tree Visualization
let treeData = null;
let svg, g, tree, root;
let currentTransform = d3.zoomIdentity;
let nodeRadius = 20;

// Initialize D3 tree visualization
function initializeD3Tree() {
    const container = document.getElementById('d3-tree-container');
    if (!container) return;
    
    // Clear existing SVG
    d3.select('#tree-svg').selectAll('*').remove();
    
    const width = container.clientWidth;
    const height = container.clientHeight;
    
    svg = d3.select('#tree-svg')
        .attr('width', width)
        .attr('height', height);
    
    // Add zoom behavior
    const zoom = d3.zoom()
        .scaleExtent([0.1, 3])
        .on('zoom', (event) => {
            currentTransform = event.transform;
            g.attr('transform', event.transform);
        });
    
    svg.call(zoom);
    
    g = svg.append('g');
    
    // Create tree layout
    tree = d3.tree()
        .size([height - 100, width - 200])
        .separation((a, b) => (a.parent === b.parent ? 1 : 1.2));
    
    showTreeLoading();
}

// Show loading state
function showTreeLoading() {
    const container = document.getElementById('d3-tree-container');
    if (!container) return;
    
    const existing = container.querySelector('.tree-loading');
    if (existing) return;
    
    const loading = document.createElement('div');
    loading.className = 'tree-loading';
    loading.textContent = '等待行為樹數據...';
    container.appendChild(loading);
}

// Hide loading state
function hideTreeLoading() {
    const container = document.getElementById('d3-tree-container');
    if (!container) return;
    
    const loading = container.querySelector('.tree-loading');
    if (loading) loading.remove();
}

// Enhanced tree rendering with D3.js
function renderBehaviorTree(data) {
    if (!data || !data.structure) {
        showTreeLoading();
        return;
    }
    
    // Validate tree structure to prevent infinite recursion
    if (!validateTreeStructure(data.structure)) {
        showToast('❌ 無效的樹狀結構', 'error');
        return;
    }
    
    hideTreeLoading();
    treeData = data;
    
    if (!svg || !g) {
        initializeD3Tree();
    }
    
    try {
        // Convert tree structure to D3 hierarchy
        root = d3.hierarchy(data.structure);
        
        // Calculate tree layout
        tree(root);
        
        // Center the tree
        const centerX = (svg.attr('width') - 200) / 2;
        const centerY = 50;
        
        root.descendants().forEach(d => {
            d.x += centerX;
            d.y += centerY;
        });
        
        // Update visualization
        updateTreeVisualization(data.status);
        
        // Show info about the tree
        if (data.test_mode) {
            showToast(`🧪 測試二元樹已載入 (${data.node_count} 個節點)`, 'info', 2000);
        }
    } catch (error) {
        console.error('Error rendering tree:', error);
        showToast(`❌ 樹狀圖渲染失敗: ${error.message}`, 'error');
    }
}

// Validate tree structure to prevent circular references
function validateTreeStructure(node, visited = new Set()) {
    if (!node || !node.id) {
        console.error('Invalid node: missing id');
        return false;
    }
    
    if (visited.has(node.id)) {
        console.error('Circular reference detected:', node.id);
        return false;
    }
    
    visited.add(node.id);
    
    if (node.children && Array.isArray(node.children)) {
        for (let child of node.children) {
            if (!validateTreeStructure(child, new Set(visited))) {
                return false;
            }
        }
    }
    
    return true;
}

// Update tree visualization with current data
function updateTreeVisualization(statusData) {
    if (!root) return;
    
    // Update links
    const links = g.selectAll('.tree-link')
        .data(root.links(), d => d.target.data.id);
    
    links.exit().remove();
    
    const linkEnter = links.enter()
        .append('path')
        .attr('class', 'tree-link')
        .attr('d', d3.linkVertical()
            .x(d => d.x)
            .y(d => d.y));
    
    links.merge(linkEnter)
        .attr('d', d3.linkVertical()
            .x(d => d.x)
            .y(d => d.y))
        .attr('class', d => {
            const status = getNodeStatus(d.target.data.id, statusData);
            return `tree-link ${getStatusClass(status).replace('status-', '')}`;
        });
    
    // Update nodes
    const nodes = g.selectAll('.tree-node-group')
        .data(root.descendants(), d => d.data.id);
    
    nodes.exit().remove();
    
    const nodeEnter = nodes.enter()
        .append('g')
        .attr('class', 'tree-node-group')
        .attr('transform', d => `translate(${d.x},${d.y})`);
    
    // Add shapes for nodes based on type
    const nodeShapes = nodeEnter.append('g')
        .attr('class', 'node-shape');
    
    // Add different shapes based on node type
    nodeShapes.each(function(d) {
        const shape = d3.select(this);
        const nodeType = getNodeTypeClass(d.data);
        
        switch(nodeType) {
            case 'selector':
                // Octagon for Selector nodes
                const octagonPoints = [];
                for (let i = 0; i < 8; i++) {
                    const angle = (i * 2 * Math.PI) / 8;
                    const x = nodeRadius * Math.cos(angle);
                    const y = nodeRadius * Math.sin(angle);
                    octagonPoints.push([x, y]);
                }
                shape.append('polygon')
                    .attr('class', 'tree-node-shape')
                    .attr('points', octagonPoints.map(p => `${p[0]},${p[1]}`).join(' '));
                break;
                
            case 'sequence':
                // Rectangle for Sequence nodes
                shape.append('rect')
                    .attr('class', 'tree-node-shape')
                    .attr('x', -nodeRadius)
                    .attr('y', -nodeRadius)
                    .attr('width', nodeRadius * 2)
                    .attr('height', nodeRadius * 2)
                    .attr('rx', 3);
                break;
                
            case 'parallel':
                // Parallelogram for Parallel nodes
                const parallelogramPoints = [
                    [-nodeRadius + 5, -nodeRadius],
                    [nodeRadius, -nodeRadius],
                    [nodeRadius - 5, nodeRadius],
                    [-nodeRadius, nodeRadius]
                ];
                shape.append('polygon')
                    .attr('class', 'tree-node-shape')
                    .attr('points', parallelogramPoints.map(p => `${p[0]},${p[1]}`).join(' '));
                break;
                
            default:
                // Circle/Ellipse for Behaviour nodes
                shape.append('circle')
                    .attr('class', 'tree-node-shape')
                    .attr('r', nodeRadius);
                break;
        }
        
        // Add click and hover events to the shape
        shape.select('.tree-node-shape')
            .on('click', handleNodeClick)
            .on('mouseover', handleNodeMouseOver)
            .on('mouseout', handleNodeMouseOut);
    });
    
    // Add node labels
    nodeEnter.append('text')
        .attr('class', 'node-text')
        .attr('dy', '0.3em')
        .text(d => {
            const name = d.data.name;
            return name.length > 8 ? name.substring(0, 8) + '...' : name;
        });
    
    // Add node type labels
    nodeEnter.append('text')
        .attr('class', 'node-type-text')
        .attr('dy', '2.2em')
        .text(d => d.data.type);
    
    // Update existing nodes
    const nodeUpdate = nodes.merge(nodeEnter);
    
    nodeUpdate
        .transition()
        .duration(500)
        .attr('transform', d => `translate(${d.x},${d.y})`);
    
    nodeUpdate.select('.tree-node-shape')
        .attr('class', d => {
            const status = getNodeStatus(d.data.id, statusData);
            const nodeType = getNodeTypeClass(d.data);
            return `tree-node-shape node-${nodeType} ${getStatusClass(status).replace('status-', 'node-')}`;
        });
}

// Get node type class
function getNodeTypeClass(nodeData) {
    if (nodeData.composite_type) {
        return nodeData.composite_type.toLowerCase();
    }
    return 'behavior';
}

// Handle node click events
function handleNodeClick(event, d) {
    event.stopPropagation();
    
    const nodeInfo = `
        節點: ${d.data.name}
        類型: ${d.data.type}
        ID: ${d.data.id}
        ${d.data.composite_type ? `複合類型: ${d.data.composite_type}` : ''}
        子節點: ${d.children ? d.children.length : 0}個
    `;
    
    showToast(nodeInfo, 'info', 4000);
}

// Handle node mouse over
function handleNodeMouseOver(event, d) {
    // Create tooltip
    const tooltip = d3.select('body').append('div')
        .attr('class', 'tree-tooltip')
        .style('opacity', 0);
    
    const status = getNodeStatus(d.data.id, treeData.status);
    
    tooltip.html(`
        <strong>${d.data.name}</strong><br/>
        類型: ${d.data.type}<br/>
        狀態: ${status}<br/>
        ID: ${d.data.id}
        ${d.data.composite_type ? `<br/>複合類型: ${d.data.composite_type}` : ''}
    `)
    .style('left', (event.pageX + 10) + 'px')
    .style('top', (event.pageY - 10) + 'px')
    .transition()
    .duration(200)
    .style('opacity', 1);
}

// Handle node mouse out
function handleNodeMouseOut() {
    d3.selectAll('.tree-tooltip').remove();
}

// Tree control functions
function expandTree() {
    if (!root) return;
    
    // Expand all collapsed nodes
    root.descendants().forEach(d => {
        if (d._children) {
            d.children = d._children;
            d._children = null;
        }
    });
    
    updateTreeVisualization(treeData?.status);
    showToast('🌳 樹狀圖已完全展開', 'info');
}

function collapseTree() {
    if (!root) return;
    
    // Collapse nodes beyond level 2
    root.descendants().forEach(d => {
        if (d.depth > 1 && d.children) {
            d._children = d.children;
            d.children = null;
        }
    });
    
    updateTreeVisualization(treeData?.status);
    showToast('📊 樹狀圖已收縮', 'info');
}

function centerTree() {
    if (!svg || !root) return;
    
    const bounds = g.node().getBBox();
    const width = svg.attr('width');
    const height = svg.attr('height');
    
    const centerX = width / 2 - bounds.width / 2 - bounds.x;
    const centerY = height / 2 - bounds.height / 2 - bounds.y;
    
    const transform = d3.zoomIdentity.translate(centerX, centerY);
    
    svg.transition()
        .duration(750)
        .call(d3.zoom().transform, transform);
    
    showToast('🎯 樹狀圖已置中', 'info');
}

// Enhanced behavior tree update function
function updateBehaviorTreeStatus(status) {
    fetchBehaviorTreeData().then(treeData => {
        if (treeData) {
            renderBehaviorTree(treeData);
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
        case 'INVALID': return 'status-invalid';
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
    // Initialize D3 tree visualization
    initializeD3Tree();
    
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
        if (e.ctrlKey) {
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
                case 'e':
                    e.preventDefault();
                    expandTree();
                    break;
                case 'c':
                    e.preventDefault();
                    collapseTree();
                    break;
            }
        }
    });
    
    // Handle window resize for D3 tree
    window.addEventListener('resize', () => {
        setTimeout(initializeD3Tree, 100);
    });
    
    console.log('🚀 Niryo Desktop Control Interface Initialized with D3.js Tree');
    console.log('Keyboard shortcuts: Ctrl+H (Home), Ctrl+R (Reconnect), Ctrl+L (Learning Mode), Ctrl+E (Expand Tree), Ctrl+C (Collapse Tree)');
});



