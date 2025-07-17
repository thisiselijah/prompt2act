function moveArm(direction) {
    fetch('/move_arm', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ direction: direction })
    })
    .then(response => response.json())
    .then(data => {
        console.log(data);
        alert(data.message);
    })
    .catch(error => {
        console.error('Error:', error);
    });
}

function setLearningMode() {
    fetch('/learning_mode', {
        method: 'POST'
    })
    .then(response => response.json())
    .then(data => {
        console.log(data);
        alert(data.message);
    })
    .catch(error => {
        console.error('Error:', error);
    });
}

function pickObject() {
    fetch('/pick_object', {
        method: 'POST'
    })
    .then(res => res.json())
    .then(data => alert(data.message))
    .catch(err => console.error(err));
}

function moveToHome() {
    fetch('/move_to_home', { method: 'POST' })
    .then(response => response.json())
    .then(data => {
        alert(data.message || data.error);
    })
    .catch(err => {
        alert('錯誤: ' + err);
    });
}



