window.addEventListener('DOMContentLoaded', () => {
  // ANIMACIONES DE ENTRADA
  gsap.to('.widget', {
    opacity: 1,
    y: 0,
    duration: 1.2,
    stagger: 0.2,
    ease: 'power3.out'
  });

  gsap.to('.center-panel', {
    opacity: 1,
    y: 0,
    duration: 1.4,
    ease: 'power2.out',
    delay: 0.5
  });

  const navbar = document.getElementById('navbar');
  document.addEventListener('mousemove', (e) => {
    navbar.classList.toggle('visible', e.clientY < 60);
  });

  // THREE.js TEMPORAL
  const container = document.getElementById('rover-model-container');
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111111);

  const camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 1000);
  camera.position.set(0, 0, 3);

  const renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.setPixelRatio(window.devicePixelRatio);
  container.appendChild(renderer.domElement);

  const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
  scene.add(ambientLight);
  const directionalLight = new THREE.DirectionalLight(0xbb86fc, 1);
  directionalLight.position.set(5, 5, 5);
  scene.add(directionalLight);

  const geometry = new THREE.BoxGeometry();
  const material = new THREE.MeshStandardMaterial({ color: 0xbb86fc });
  const cube = new THREE.Mesh(geometry, material);
  scene.add(cube);

  function animate() {
    requestAnimationFrame(animate);
    cube.rotation.y += 0.01;
    cube.rotation.x += 0.005;
    renderer.render(scene, camera);
  }
  animate();


  // Radar de obstáculos
  function updateRadar(usDistance) {
    const radar = document.getElementById('radar-scope');
    if (!radar) return;

    // Borra obstáculos anteriores
    radar.querySelectorAll('.radar-object').forEach(o => o.remove());

    // Normaliza la distancia (simula máximo 100cm)
    const maxDistance = 100;
    const radius = radar.offsetWidth / 2;
    const clamped = Math.min(usDistance, maxDistance);
    const normalized = clamped / maxDistance;

    // Genera una posición aleatoria dentro del rango visible
    const angle = Math.random() * 2 * Math.PI;
    const r = radius * normalized;

    const x = radius + r * Math.cos(angle);
    const y = radius + r * Math.sin(angle);

    const dot = document.createElement('div');
    dot.className = 'radar-object';
    dot.style.left = `${x}px`;
    dot.style.top = `${y}px`;

    radar.appendChild(dot);
  }


  window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  });

  container.addEventListener('mouseenter', () => container.style.filter = 'drop-shadow(0 0 10px #bb86fc)');
  container.addEventListener('mouseleave', () => container.style.filter = 'none');

  const overlay = document.getElementById('sensor-detail-overlay');
  const content = document.getElementById('sensor-detail-content');

  const showAlert = (msg, level = 'info') => {
    const alert = document.createElement('div');
    alert.className = `floating-alert alert-${level}`;
    alert.textContent = msg;
    document.body.appendChild(alert);
    setTimeout(() => alert.remove(), 2000);
  };

  const sensorData = {
    voltaje: () => {
      const batteryLevel = 63; // Simulado; reemplazar con datos reales
      const status = batteryLevel > 90 ? 'Batería óptima' : batteryLevel < 20 ? 'Nivel bajo' : 'Nivel medio';
      const color = batteryLevel > 50 ? '#4caf50' : batteryLevel > 20 ? '#ff9800' : '#f44336';
      if (batteryLevel < 20) showAlert(`⚠️ Batería baja: ${batteryLevel}%`, batteryLevel < 10 ? 'danger' : 'warning');
      return `
        <h5>Voltaje</h5>
        <div class="battery-display" style="--battery:${batteryLevel}%;--battery-color:${color}">
          <div class="battery-shell">
            <div class="battery-level"></div>
            <span class="battery-tip"></span>
          </div>
          <div class="battery-info">
            <strong>${batteryLevel}%</strong><br><small>${status}</small>
          </div>
        </div>
        <p class="battery-connection-status">🔋 Batería conectada</p>
      `;
    },
orientacion: () => {
  return `
    <h5>Sistema de Navegación</h5>
    <div class="navigation-container">
      <div class="nav-compass-ring">
        <div class="nav-directions">
          <span>N</span><span>E</span><span>S</span><span>O</span>
        </div>
        <div class="nav-needle"></div>
        <div class="nav-center-dot"></div>
      </div>
      <p class="nav-readout">Dirección: <span id="nav-direction-text">N</span></p>
    </div>
  `;
},
  color: () => {
    const detectedColor = 'Rojo'; // Simulado, reemplazar con variable si se conecta a sensor real
    const rgb = 'rgb(255, 0, 0)';

    return `
      <h5>Sensor de Color</h5>
      <div class="color-scanner-container">
        <div class="color-sphere" style="background: ${rgb}; box-shadow: 0 0 20px ${rgb};"></div>
        <div class="color-ring"></div>
      </div>
      <div class="color-info-text">
        <p><strong>${detectedColor}</strong></p>
        <p>${rgb}</p>
      </div>
    `;
  },
    obstaculos: () => {
      return `
        <h5>Radar de Obstáculos</h5>
        <div class="radar" id="radar-scope">
          <!-- Objetos se agregarán dinámicamente -->
        </div>
        <p>Escaneo activo</p>
      `;
    }
  };

  document.querySelectorAll('.widget').forEach(widget => {
    widget.classList.add('clickable');
    const sensor = widget.dataset.sensor;
    widget.addEventListener('click', () => {
      const detail = sensorData[sensor]?.() || '<p>Sin datos</p>';
      content.innerHTML = detail;
      overlay.classList.remove('hidden');
      gsap.fromTo('.sensor-detail-box', { scale: 0.85, opacity: 0 }, { scale: 1, opacity: 1, duration: 0.3 });
    });
  });

  overlay.addEventListener('click', (e) => {
    if (e.target.id === 'sensor-detail-overlay' || e.target.id === 'close-detail') {
      overlay.classList.add('hidden');
    }
  });

  const socket = new WebSocket('ws://<IP_DEL_ESP32>:81');
  socket.addEventListener('open', () => {
    console.log('🔌 Conectado al WebSocket de ESP32');
    document.querySelector('.status-indicator').innerHTML = `<i class="bi bi-broadcast-pin"></i> ESP32 Connected`;
  });
  socket.addEventListener('message', (event) => {
    try {
      const data = JSON.parse(event.data);
      if (data.voltaje !== undefined) {
        document.querySelector('[data-sensor="voltaje"] .widget-data').textContent = `${data.voltaje.toFixed(2)} V`;
      }
      if (data.orientacion !== undefined) {
        const dir = data.orientacion;
        document.querySelector('[data-sensor="orientacion"] .widget-data').textContent = `Brújula: ${dir}`;
        const angle = { N: 0, E: 90, S: 180, O: 270 }[dir] ?? 0;
        
        const needle = document.querySelector('.nav-needle');
        if (needle) needle.style.transform = `rotate(${angle}deg)`;
        
        const readout = document.getElementById('nav-direction-text');
        if (readout) readout.textContent = dir;
      }
      if (data.obstaculos !== undefined) {
        const ir = data.obstaculos.ir;
        const us = data.obstaculos.us;
        document.querySelector('[data-sensor="obstaculos"] .widget-data').innerHTML = `IR: ${ir}<br>US: ${us} cm`;

        updateRadar(us); // Actualiza el radar visual
      }
      if (data.color !== undefined) {
        document.querySelector('[data-sensor="color"] .widget-data').textContent = `${data.color}`;
      }
    } catch (err) {
      console.error("❌ Error al procesar datos del WebSocket:", err);
    }
  });
  socket.addEventListener('close', () => {
    console.warn('⚠️ Conexión cerrada con el ESP32');
    document.querySelector('.status-indicator').innerHTML = `<i class="bi bi-exclamation-triangle"></i> ESP32 Desconectado`;
  });
  socket.addEventListener('error', (err) => console.error('❌ Error en WebSocket:', err));
});