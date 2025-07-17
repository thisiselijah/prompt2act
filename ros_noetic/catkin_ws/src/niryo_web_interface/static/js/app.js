function open_and_close(direction) {   
    fetch('/open_and_close', {
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

function pickObject(color) {
    fetch('/pick_object', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ label: color })
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



