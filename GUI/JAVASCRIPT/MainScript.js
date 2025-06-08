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

  // THREE.js CON ROVER MODEL
  const container = document.getElementById('rover-model-container');
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111111);

// En la configuración inicial de la cámara:
const camera = new THREE.PerspectiveCamera(
  35, // Reducir el FOV (field of view) para menos distorsión
  container.clientWidth / container.clientHeight,
  0.1,
  1000
);
camera.position.set(0, 0.2, 2); // Posición más cercana y centrada

// En el evento de resize, ajusta el aspect ratio:
window.addEventListener('resize', () => {
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
  
  // Ajusta la posición de la cámara en el resize si es necesario
  if (container.clientWidth < 600) {
    camera.position.z = 2.2; // Más cerca en móviles
  } else {
    camera.position.z = 2.5; // Valor por defecto
  }
});

  const renderer = new THREE.WebGLRenderer({ 
    alpha: true, 
    antialias: true,
    powerPreference: "high-performance"
  });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.setPixelRatio(window.devicePixelRatio);
  container.appendChild(renderer.domElement);

  // Iluminación mejorada
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
  scene.add(ambientLight);
  
  const directionalLight1 = new THREE.DirectionalLight(0xbb86fc, 1);
  directionalLight1.position.set(5, 5, 5);
  scene.add(directionalLight1);
  
  const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.5);
  directionalLight2.position.set(-5, 3, 2);
  scene.add(directionalLight2);

  // Cargar modelo del rover
  const loader = new THREE.GLTFLoader();
  let roverModel = null;

  loader.load(
    '../ASSETS/ROVER/rover_model.glb',
    (gltf) => {
    const model = gltf.scene;
    // Ajusta estos valores para hacerlo más pequeño (valores menores a 1)
model.scale.set(0.4, 0.4, 0.4); // Más pequeño
model.position.y = -0.2; // Menos desplazamiento vertical
    scene.add(model);
    roverModel = model;
    
    // Ajusta la posición de la cámara para encuadrar mejor
    camera.position.set(0, 0.3, 2.5); // Más cerca y ligeramente más bajo
    camera.lookAt(0, 0, 0);
    
    // Animación opcional de escalado suave
    gsap.from(model.scale, {
      x: 0.1,
      y: 0.1,
      z: 0.1,
      duration: 1.2,
      ease: 'elastic.out(1, 0.5)'
    });
  },
    undefined,
    (error) => {
      console.error('❌ Error al cargar el modelo del rover:', error);
      
      // Crear cubo de respaldo en caso de error
      const geometry = new THREE.BoxGeometry();
      const material = new THREE.MeshStandardMaterial({ color: 0xbb86fc });
      const cube = new THREE.Mesh(geometry, material);
      scene.add(cube);
      roverModel = cube;
    }
  );

  // Animación continua
  function animate() {
    requestAnimationFrame(animate);
    
    if (roverModel) {
      roverModel.rotation.y += 0.005;
    }
    
    renderer.render(scene, camera);
  }
  animate();

  // Radar de obstáculos
  function updateRadar(usDistance) {
    const radar = document.getElementById('radar-scope');
    if (!radar) return;

    radar.querySelectorAll('.radar-object').forEach(o => o.remove());

    const maxDistance = 100;
    const radius = radar.offsetWidth / 2;
    const clamped = Math.min(usDistance, maxDistance);
    const normalized = clamped / maxDistance;

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

  container.addEventListener('mouseenter', () => {
    container.style.filter = 'drop-shadow(0 0 15px #bb86fc)';
    
    // Animación al pasar el mouse
    if (roverModel) {
      gsap.to(roverModel.scale, {
        x: 0.85,
        y: 0.85,
        z: 0.85,
        duration: 0.3,
        ease: 'power2.out'
      });
    }
  });
  
  container.addEventListener('mouseleave', () => {
    container.style.filter = 'none';
    
    if (roverModel) {
      gsap.to(roverModel.scale, {
        x: 0.8,
        y: 0.8,
        z: 0.8,
        duration: 0.5,
        ease: 'elastic.out(1, 0.5)'
      });
    }
  });

  const socket = new WebSocket('ws://<IP_DEL_ESP32>:81');
  socket.addEventListener('open', () => {
    console.log('🔌 Conectado al WebSocket de ESP32');
    document.querySelector('.status-indicator').innerHTML = '<i class="bi bi-broadcast-pin"></i> ESP32 Connected';
  });
  
  socket.addEventListener('message', (event) => {
    try {
      const data = JSON.parse(event.data);
      
      if (data.voltaje !== undefined) {
        const batteryLevel = Math.min(Math.max(data.voltaje * 10, 0), 100);
        const status = batteryLevel > 90 ? 'Batería óptima' : batteryLevel < 20 ? 'Nivel bajo' : 'Nivel medio';
        const color = batteryLevel > 50 ? '#4caf50' : batteryLevel > 20 ? '#ff9800' : '#f44336';
        
        const batteryLevelElement = document.querySelector('[data-sensor="voltaje"] .battery-level');
        const batteryInfoElement = document.querySelector('[data-sensor="voltaje"] .battery-info');
        
        if (batteryLevelElement && batteryInfoElement) {
          batteryLevelElement.style.width = `${batteryLevel}%`;
          batteryLevelElement.style.backgroundColor = color;
          batteryInfoElement.innerHTML = `<strong>${batteryLevel}%</strong><br><small>${status}</small>`;
        }
      }
      
      if (data.orientacion !== undefined) {
        const dir = data.orientacion;
        const angle = { N: 0, E: 90, S: 180, O: 270 }[dir] ?? 0;
        
        const needle = document.querySelector('.nav-needle');
        if (needle) needle.style.transform = `rotate(${angle}deg)`;
        
        const readout = document.getElementById('nav-direction-text');
        if (readout) readout.textContent = dir;
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
      
      if (data.color !== undefined) {
        const colorMap = {
          'Rojo': 'rgb(255, 0, 0)',
          'Verde': 'rgb(0, 255, 0)',
          'Azul': 'rgb(0, 0, 255)',
          'Blanco': 'rgb(255, 255, 255)',
          'Negro': 'rgb(0, 0, 0)'
        };
        
        const rgb = colorMap[data.color] || 'rgb(255, 255, 255)';
        const colorSphere = document.querySelector('[data-sensor="color"] .color-sphere');
        const colorInfoText = document.querySelector('[data-sensor="color"] .color-info-text');
        
        if (colorSphere && colorInfoText) {
          colorSphere.style.background = rgb;
          colorSphere.style.boxShadow = `0 0 20px ${rgb}`;
          colorInfoText.innerHTML = `<p><strong>${data.color}</strong></p><p>${rgb}</p>`;
        }
      }
    } catch (err) {
      console.error("❌ Error al procesar datos del WebSocket:", err);
    }
  });
  
  socket.addEventListener('close', () => {
    console.warn('⚠️ Conexión cerrada con el ESP32');
    document.querySelector('.status-indicator').innerHTML = '<i class="bi bi-exclamation-triangle"></i> ESP32 Desconectado';
  });
  
  socket.addEventListener('error', (err) => console.error('❌ Error en WebSocket:', err));
});