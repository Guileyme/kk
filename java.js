const mapNameElement = document.getElementById('map-name');
const updateButton = document.querySelector('button')
let countdownInterval = null;
const maps = ["Olympus", "Kings Canyon", "E-District"];

async function getRealMapData() {
    try {
        console.log(`Запрашиваю данные для режима: ${currentMode}`);
        
        const API_KEY = "YOUR_API_KEY";
        const url = `https://api.mozambiquehe.re/maprotation?version=2&auth=${API_KEY}`;

        const response = await fetch(url);
        
        if (!response.ok){
            throw new Error(`Ошибка API: ${response.status}`);
        }
        
        const data = await response.json();
        
        let targetData;
        if (data[currentMode] && data[currentMode].current) {
            targetData = data[currentMode];
        } else {
            console.log(`Режим ${currentMode} недоступен, использую ranked`);
            targetData = data.ranked;
            currentMode = 'ranked';
            switchMode('ranked');
        }
        
        const currentMap = targetData.current;
        const nextMap = targetData.next;
        
        return {
            map: currentMap.map,
            timer: currentMap.remainingTimer,
            remainingSecs: currentMap.remainingSecs,
            image: currentMap.asset,
            nextMap: nextMap ? nextMap.map : null,
            mode: getModeTitle(currentMode)
        };
        
    } catch (error) {
        console.error("Ошибка:", error);
        return null;
    }
}

let currentMode = 'ranked';

function switchMode(mode) {
    console.log("Переключаем на режим:", mode);
    
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.classList.remove('active');
        if (btn.dataset.mode === mode) {
            btn.classList.add('active');
        }
    });
    
    currentMode = mode;
    updateMap();
}

function getModeTitle(mode) {
    const modeTitles = {
        ranked: "Рейтинговый режим",
        battle_royale: "Публичные матчи"
    };
    return modeTitles[mode] || mode;
}

function formatTime(seconds) {
    if (isNaN(seconds) || seconds < 0) {
        return "00:00:00";
    }
    
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;
    
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}
function startCountdown(seconds){
    const timerElement = document.getElementById('timer');

    if (countdownInterval){
        clearInterval(countdownInterval);
    }
    let timeLeft = seconds;
    timerElement.textContent = `До смены: ${formatTime(timeLeft)}`;

    countdownInterval = setInterval(() => {
        timeLeft--;
        if (timeLeft <= 0) {
            clearInterval(countdownInterval);
            timerElement.textContent = "Карта сменилась";
            timerElement.style.color = "#ff4005";
            setTimeout(updateMap, 5000);
        } else {
            timerElement.textContent = `До смены: ${formatTime(timeLeft)}`;
        }
        if (timeLeft < 300) {
            timerElement.style.color = "#ff4655";
        } else {
            timerElement.style.color = "#90dfaa";
        }
    }, 1000);
}

async function updateMap() {
    document.querySelector('h2').textContent = getModeTitle(currentMode);
    console.log("🔄 Обновление данных...");
    const timerElement = document.getElementById('timer');
    const mapImageElement = document.getElementById('map-image');
    const nextMapElement = document.getElementById('next-map-name');
    
    mapNameElement.textContent = "Загрузка...";
    mapNameElement.style.color = "#7a8ca5";
    timerElement.textContent = "Обновляем данные...";
    mapImageElement.style.display = "none";

    if (nextMapElement) {
        nextMapElement.textContent = "...";
    }

    const data = await getRealMapData();    
    
    if (data) {
        document.querySelector('h2').textContent = data.mode;
        
        mapNameElement.textContent = data.map;
        
        startCountdown(data.remainingSecs);
        
        if (data.image) {
            mapImageElement.src = data.image;
            mapImageElement.style.display = "block";
            mapImageElement.alt = `Карта ${data.map}`;
        }
        
         if (nextMapElement && data.nextMap) {
            nextMapElement.textContent = data.nextMap;
        }

        const colors = ['#ff4655', '#e7a011', '#ffd43b'];
        const randomColor = colors[Math.floor(Math.random() * colors.length)];
        mapNameElement.style.color = randomColor;
        
        const buttonColors = ['#e95216ff', '#ff6e0dff'];
        const randomColorBackground = buttonColors[Math.floor(Math.random() * buttonColors.length)];
        updateButton.style.backgroundColor = randomColorBackground;
        
    } else {
        mapNameElement.textContent = "Ошибка загрузки";
        mapNameElement.style.color = "#ff0000";
        timerElement.textContent = "Проверь соединение";
    }
}

document.addEventListener('DOMContentLoaded', function() {
    const modeButtons = document.querySelectorAll('.mode-btn');
    modeButtons.forEach(btn => {
        btn.addEventListener('click', function() {
            switchMode(this.dataset.mode);
        });
    });
    
    document.querySelector('h2').textContent = getModeTitle(currentMode);
});

updateButton.addEventListener('click', updateMap);
updateMap();