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

  // NAVBAR VISIBLE AL ACERCAR EL MOUSE
  const navbar = document.getElementById('navbar');
  document.addEventListener('mousemove', (e) => {
    if (e.clientY < 60) {
      navbar.classList.add('visible');
    } else {
      navbar.classList.remove('visible');
    }
  });

  // THREE.JS - CUBO TEMPORAL
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

  window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  });

  // INTERACCIÓN MODELO
  container.addEventListener('mouseenter', () => {
    container.style.filter = 'drop-shadow(0 0 10px #bb86fc)';
  });
  container.addEventListener('mouseleave', () => {
    container.style.filter = 'none';
  });

  // DETALLE SENSOR
  const overlay = document.createElement('div');
  overlay.id = 'sensor-detail-overlay';
  overlay.classList.add('hidden');
  document.body.appendChild(overlay);

  overlay.innerHTML = `
    <div class="sensor-detail-box">
      <button class="close-detail" id="close-detail">&times;</button>
      <div id="sensor-detail-content"></div>
    </div>
  `;

  const sensorData = {
    voltaje: "<h5>Voltaje</h5><p>12.6 V - Estado óptimo<br>Batería al 95%</p>",
    orientacion: "<h5>Orientación</h5><p>Brújula: Norte<br>MPU: Estable</p>",
    obstaculos: "<h5>Obstáculos</h5><p>IR: Libre<br>Ultrasonido: 35 cm<br>Sin colisiones detectadas</p>",
    color: "<h5>Color</h5><p>Rojo detectado<br>Posible bandera u obstáculo rojo</p>"
  };

  document.querySelectorAll('.widget').forEach(widget => {
    widget.classList.add('clickable');
    widget.setAttribute('data-sensor', widget.querySelector('.widget-title').innerText.toLowerCase());

    widget.addEventListener('click', () => {
      const sensor = widget.getAttribute('data-sensor');
      const content = document.getElementById('sensor-detail-content');
      content.innerHTML = sensorData[sensor] || "<p>Sin datos</p>";
      overlay.classList.remove('hidden');
      gsap.fromTo('.sensor-detail-box', { scale: 0.85, opacity: 0 }, { scale: 1, opacity: 1, duration: 0.3 });
    });
  });

  overlay.addEventListener('click', (e) => {
    if (e.target === overlay || e.target.id === 'close-detail') {
      overlay.classList.add('hidden');
    }
  });
});