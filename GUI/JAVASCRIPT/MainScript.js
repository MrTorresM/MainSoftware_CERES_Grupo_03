window.addEventListener('DOMContentLoaded', () => {
  // ANIMACIONES DE ENTRADA
  gsap.to('.widget', {
    opacity: 1,
    y: 0,
    duration: 1.2,
    stagger: 0.2,
    ease: 'power3.out'
  });

  gsap.to('.model-panel', {
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

  const container = document.getElementById('rover-model-container');
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111111);

  const camera = new THREE.PerspectiveCamera(
    35,
    container.clientWidth / container.clientHeight,
    0.1,
    1000
  );
  camera.position.set(0, 0.2, 2);

  const renderer = new THREE.WebGLRenderer({
    alpha: true,
    antialias: true,
    powerPreference: "high-performance"
  });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.setPixelRatio(window.devicePixelRatio);
  container.appendChild(renderer.domElement);

  // Luces
  scene.add(new THREE.AmbientLight(0xffffff, 0.8));
  const directionalLight1 = new THREE.DirectionalLight(0xbb86fc, 1);
  directionalLight1.position.set(5, 5, 5);
  scene.add(directionalLight1);
  const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.5);
  directionalLight2.position.set(-5, 3, 2);
  scene.add(directionalLight2);

  const loader = new THREE.GLTFLoader();
  let roverModel = null;
  let currentYRotation = 0;
  let targetYRotation = 0;

  // Control manual
  let isDragging = false;
  let previousMouseX = 0;
  let useManualControl = false;

  loader.load('../ASSETS/ROVER/rover_model.glb', (gltf) => {
    const model = gltf.scene;
    model.scale.set(0.25, 0.25, 0.25);
    model.position.y = -0.1;
    scene.add(model);
    roverModel = model;

    gsap.from(model.scale, {
      x: 0.1,
      y: 0.1,
      z: 0.1,
      duration: 1.2,
      ease: 'elastic.out(1, 0.5)'
    });

    camera.position.set(0, 0.3, 2.5);
    camera.lookAt(0, 0, 0);
  }, undefined, (error) => {
    console.error('❌ Error al cargar modelo:', error);
    const geometry = new THREE.BoxGeometry();
    const material = new THREE.MeshStandardMaterial({ color: 0xbb86fc });
    const cube = new THREE.Mesh(geometry, material);
    scene.add(cube);
    roverModel = cube;
  });

  function animate() {
    requestAnimationFrame(animate);
    if (roverModel) {
      if (!useManualControl) {
        // Animación de rotación automática basada en el dato del MPU
        currentYRotation += (targetYRotation - currentYRotation) * 0.05;
        roverModel.rotation.y = currentYRotation;
      }
    }
    renderer.render(scene, camera);
  }
  animate();

  // Resize
  window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  });

  // Efecto de hover visual
  container.addEventListener('mouseenter', () => {
    container.style.filter = 'drop-shadow(0 0 15px #bb86fc)';
    useManualControl = true;
  });

  container.addEventListener('mouseleave', () => {
    container.style.filter = 'none';
    useManualControl = false;
  });

  // Control de rotación tipo OrbitControls
  container.addEventListener('mousedown', (e) => {
    isDragging = true;
    previousMouseX = e.clientX;
  });

  container.addEventListener('mouseup', () => {
    isDragging = false;
  });

let previousMouseY = 0;

container.addEventListener('mousedown', (e) => {
  isDragging = true;
  previousMouseX = e.clientX;
  previousMouseY = e.clientY;
});

container.addEventListener('mouseup', () => {
  isDragging = false;
});

container.addEventListener('mousemove', (e) => {
  if (isDragging && useManualControl && roverModel) {
    const deltaX = e.clientX - previousMouseX;
    const deltaY = e.clientY - previousMouseY;

    previousMouseX = e.clientX;
    previousMouseY = e.clientY;

    roverModel.rotation.y += deltaX * 0.01;
    roverModel.rotation.x += deltaY * 0.01;

    // Limita la rotación en X para evitar que el modelo se voltee completamente
    roverModel.rotation.x = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, roverModel.rotation.x));

    currentYRotation = roverModel.rotation.y;
  }
});


container.addEventListener('mouseleave', () => {
  container.style.filter = 'none';
  useManualControl = false;

  if (roverModel) {
    gsap.to(roverModel.rotation, {
      x: 0,
      y: 0,
      duration: 1.2,
      ease: 'power3.out'
    });

    // Actualiza variable para seguir desde ahí si vuelve a rotarse automáticamente
    currentYRotation = 0;
  }
});

  // WebSocket para recibir datos del MPU y otros sensores
  const socket = new WebSocket('ws://172.20.10.2:81');
  
  //python -m http.server 8000
  //http://localhost:8000/GUI/HTML/MainHUD.html

  socket.addEventListener('open', () => {
    console.log('🔌 Conectado al WebSocket de ESP32');
    document.querySelector('.status-indicator').innerHTML =
      '<i class="bi bi-broadcast-pin"></i> ESP32 Connected';
  });

socket.addEventListener('message', (event) => {
  try {
    const data = JSON.parse(event.data);

    if (data.bateria !== undefined) {
      const batteryLevel = Math.min(Math.max(data.bateria, 0), 100);
      const status = batteryLevel > 90 ? 'Batería óptima' : batteryLevel < 20 ? 'Nivel bajo' : 'Nivel medio';
      const color = batteryLevel > 50 ? '#4caf50' : batteryLevel > 20 ? '#ff9800' : '#f44336';

      const batteryLevelElement = document.querySelector('[data-sensor="voltaje"] .battery-level');
      const batteryInfoElement = document.querySelector('[data-sensor="voltaje"] .battery-info');

      if (batteryLevelElement && batteryInfoElement) {
        batteryLevelElement.style.height = `${batteryLevel}%`;
        batteryLevelElement.style.backgroundColor = color;
        batteryInfoElement.innerHTML = `<strong>${batteryLevel}%</strong><br><small>${status}</small>`;
      }
    }

    if (data.orientacion !== undefined) {
      const dir = data.orientacion;
      //const angleMap = { N: 0, E: 90, S: 180, O: 270 };
      //const angle = angleMap[dir] ?? 0;
      const angle = dir;

      const needle = document.querySelector('.nav-needle');
      if (needle) needle.style.transform = `rotate(${angle}deg)`;

      const readout = document.getElementById('nav-direction-text');
      if (readout) readout.textContent = dir;

      if (!useManualControl) {
        targetYRotation = THREE.MathUtils.degToRad(angle);
      }
    }

    if (data.obstaculos !== undefined) {
      const ir = data.obstaculos.ir;
      const us = data.obstaculos.us;
      const obstacleDataElement = document.querySelector('[data-sensor="obstaculos"] .obstacle-data');

      if (obstacleDataElement) {
        obstacleDataElement.innerHTML = `IR: ${ir}<br>US: ${us} cm`;
      }
      updateRadar(us);
    }

    if (data.colorNombre && data.colorRGB) {
      const colorSphere = document.querySelector('[data-sensor="color"] .color-sphere');
      const colorInfoText = document.querySelector('[data-sensor="color"] .color-info-text');

      if (colorSphere && colorInfoText) {
        colorSphere.style.background = data.colorRGB;
        colorSphere.style.boxShadow = `0 0 20px ${data.colorRGB}`;
        colorInfoText.innerHTML = `<p><strong>${data.colorNombre}</strong></p><p>${data.colorRGB}</p>`;
      }
    }

  } catch (err) {
    console.error("❌ Error al procesar datos del WebSocket:", err);
  }
});


  socket.addEventListener('close', () => {
    console.warn('⚠️ Conexión cerrada con el ESP32');
    document.querySelector('.status-indicator').innerHTML =
      '<i class="bi bi-exclamation-triangle"></i> ESP32 Desconectado';
  });

  socket.addEventListener('error', (err) => console.error('❌ Error en WebSocket:', err));

const maxRadarDistance = 50; // cm
const radarWidth = 200;      // px
const radarHeight = 300;     // px

function updateRadar(usDistance) {
  const radar = document.getElementById('radar-scope');
  if (!radar) return;

  // Elimina cualquier punto previo
  radar.querySelectorAll('.radar-object-rect').forEach(el => el.remove());

  // Si la distancia es válida, muestra el punto
  if (usDistance > 0 && usDistance <= maxRadarDistance) {
    const clamped = Math.min(usDistance, maxRadarDistance);
    const normalized = clamped / maxRadarDistance;
    const y = radarHeight - normalized * radarHeight;
    const centerX = radarWidth / 2;

    const dot = document.createElement('div');
    dot.className = 'radar-object-rect';
    dot.style.left = `${centerX}px`;
    dot.style.top = `${y}px`;
    radar.appendChild(dot);
  }
}



});