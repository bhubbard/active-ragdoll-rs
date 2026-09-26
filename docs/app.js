// active-ragdoll-rs: Interactive 3D WebGL Sandbox
// Port of the Active Ragdoll physics algorithm into real-time WebGL

import * as THREE from 'https://unpkg.com/three@0.160.0/build/three.module.js';

class ActiveRagdollSimulation {
  constructor() {
    this.canvas = document.getElementById('ragdollCanvas');
    this.container = this.canvas.parentElement;

    // Physics parameters (mirrors Rust AnimationFollowerConfig & SlaveControllerConfig)
    this.config = {
      pForce: 12.0,
      dForce: 0.02,
      maxForce: 15.0,
      maxTorque: 2000.0,
      jointDamping: 0.6,
      looseStrengthLerp: 2.0,
      gainStrengthLerp: 0.5,
      minContactForce: 0.1,
      minContactTorque: 0.1,
      deadTime: 3.0,
    };

    // State machine
    this.state = 'FollowingAnimation';
    this.currentStrength = 1.0;
    this.forceCoefficient = 1.0;
    this.torqueCoefficient = 1.0;
    this.isAlive = true;
    this.currentCollisions = 0;
    this.deadTimer = 0.0;
    this.locomotionMode = 'walk'; // 'idle', 'walk', 'run'

    // Scene & Three.js setup
    this.initThree();
    this.buildRagdoll();
    this.initProjectiles();
    this.bindEvents();

    this.clock = new THREE.Clock();
    this.simTime = 0.0;
    this.fpsCounter = 0;
    this.lastFpsUpdate = 0;

    this.animate = this.animate.bind(this);
    requestAnimationFrame(this.animate);
  }

  initThree() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x07080c);
    this.scene.fog = new THREE.FogExp2(0x07080c, 0.035);

    const width = this.container.clientWidth;
    const height = this.container.clientHeight;

    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    this.camera.position.set(0, 2.2, 5.0);

    this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // Lighting
    const ambient = new THREE.AmbientLight(0x94a3b8, 0.8);
    this.scene.add(ambient);

    const dirLight = new THREE.DirectionalLight(0xffffff, 2.0);
    dirLight.position.set(5, 12, 7);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.5;
    dirLight.shadow.camera.far = 25;
    dirLight.shadow.camera.left = -5;
    dirLight.shadow.camera.right = 5;
    dirLight.shadow.camera.top = 5;
    dirLight.shadow.camera.bottom = -5;
    dirLight.shadow.bias = -0.0005;
    this.scene.add(dirLight);

    const rimLight = new THREE.DirectionalLight(0x06b6d4, 1.5);
    rimLight.position.set(-6, 3, -6);
    this.scene.add(rimLight);

    // Ground Plane with Grid
    const groundGeo = new THREE.PlaneGeometry(60, 60);
    const groundMat = new THREE.MeshStandardMaterial({
      color: 0x0f1118,
      roughness: 0.8,
      metalness: 0.2,
    });
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);

    const grid = new THREE.GridHelper(60, 60, 0x6366f1, 0x1e2433);
    grid.position.y = 0.002;
    this.scene.add(grid);

    // Camera Orbit Controls (simple mouse drag)
    this.isDragging = false;
    this.prevMouse = { x: 0, y: 0 };
    this.cameraAngle = { yaw: 0, pitch: 0.3, distance: 4.8 };
    this.targetLookAt = new THREE.Vector3(0, 1.1, 0);
  }

  buildRagdoll() {
    this.limbs = [];
    const matLimb = new THREE.MeshStandardMaterial({
      color: 0x6366f1,
      roughness: 0.35,
      metalness: 0.65,
    });
    const matJoint = new THREE.MeshStandardMaterial({
      color: 0x06b6d4,
      roughness: 0.2,
      metalness: 0.8,
    });

    // 11 Biomechanical Humanoid Limbs
    const defs = [
      { name: 'Hips', size: [0.34, 0.22, 0.2], pos: [0, 1.0, 0], mass: 14.0 },
      { name: 'LeftUpLeg', size: [0.14, 0.42, 0.14], pos: [-0.12, 0.68, 0], mass: 8.5 },
      { name: 'LeftLeg', size: [0.12, 0.4, 0.12], pos: [-0.12, 0.22, 0], mass: 4.5 },
      { name: 'RightUpLeg', size: [0.14, 0.42, 0.14], pos: [0.12, 0.68, 0], mass: 8.5 },
      { name: 'RightLeg', size: [0.12, 0.4, 0.12], pos: [0.12, 0.22, 0], mass: 4.5 },
      { name: 'Spine', size: [0.32, 0.35, 0.22], pos: [0, 1.32, 0], mass: 18.0 },
      { name: 'LeftArm', size: [0.11, 0.32, 0.11], pos: [-0.28, 1.32, 0], mass: 3.5 },
      { name: 'LeftForeArm', size: [0.09, 0.3, 0.09], pos: [-0.28, 0.98, 0], mass: 2.0 },
      { name: 'Head', size: [0.22, 0.24, 0.22], pos: [0, 1.65, 0], mass: 4.5 },
      { name: 'RightArm', size: [0.11, 0.32, 0.11], pos: [0.28, 1.32, 0], mass: 3.5 },
      { name: 'RightForeArm', size: [0.09, 0.3, 0.09], pos: [0.28, 0.98, 0], mass: 2.0 },
    ];

    this.group = new THREE.Group();

    defs.forEach((def, index) => {
      const geo = new THREE.BoxGeometry(...def.size);
      const mesh = new THREE.Mesh(geo, matLimb);
      mesh.castShadow = true;
      mesh.receiveShadow = true;

      const jointGeo = new THREE.SphereGeometry(Math.min(...def.size) * 0.4, 16, 16);
      const jointMesh = new THREE.Mesh(jointGeo, matJoint);
      mesh.add(jointMesh);

      this.scene.add(mesh);

      this.limbs.push({
        name: def.name,
        mesh,
        basePos: new THREE.Vector3(...def.pos),
        currentPos: new THREE.Vector3(...def.pos),
        targetPos: new THREE.Vector3(...def.pos),
        velocity: new THREE.Vector3(0, 0, 0),
        angularVelocity: new THREE.Vector3(0, 0, 0),
        lastError: new THREE.Vector3(0, 0, 0),
        size: def.size,
        mass: def.mass,
        isRoot: index === 0,
      });
    });

    // Ghost Master Mesh representation (wireframe indicator)
    this.masterMarker = new THREE.Group();
    defs.forEach((def) => {
      const wire = new THREE.LineSegments(
        new THREE.EdgesGeometry(new THREE.BoxGeometry(...def.size)),
        new THREE.LineBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.25 })
      );
      wire.position.set(...def.pos);
      this.masterMarker.add(wire);
    });
    this.scene.add(this.masterMarker);
  }

  initProjectiles() {
    this.projectiles = [];
    this.projGeo = new THREE.SphereGeometry(0.18, 24, 24);
    this.projMat = new THREE.MeshStandardMaterial({
      color: 0xf59e0b,
      metalness: 0.9,
      roughness: 0.15,
      emissive: 0x78350f,
    });
  }

  fireCannonball() {
    const ball = new THREE.Mesh(this.projGeo, this.projMat);
    ball.castShadow = true;

    // Spawn 3.5m in front of character at chest height
    const hips = this.limbs[0].currentPos;
    ball.position.set(
      hips.x + (Math.random() - 0.5) * 0.3,
      hips.y + 0.35 + (Math.random() - 0.5) * 0.2,
      hips.z + 3.8
    );

    // High velocity straight into chest
    const vel = new THREE.Vector3(0, 0.4, -22.0);
    this.scene.add(ball);
    this.projectiles.push({ mesh: ball, vel, radius: 0.18, life: 3.5 });

    // Toast indication
    this.showFloatingText('BALL FIRED!', '#f59e0b');
  }

  knockout() {
    this.isAlive = false;
    this.state = 'Dead';
    this.deadTimer = 0.0;
    this.forceCoefficient = 0.0;
    this.torqueCoefficient = 0.0;
    this.currentStrength = 0.0;
    this.showFloatingText('KNOCKOUT (DIE)', '#f43f5e');
  }

  revive() {
    this.isAlive = true;
    this.deadTimer = this.config.deadTime;
    this.showFloatingText('REVIVING...', '#10b981');
  }

  resetForces() {
    this.forceCoefficient = 0.0;
    this.torqueCoefficient = 0.0;
    this.currentStrength = 0.0;
    this.showFloatingText('FORCES RESET (0%)', '#06b6d4');
  }

  showFloatingText(msg, color) {
    const toast = document.createElement('div');
    toast.textContent = msg;
    toast.style.position = 'absolute';
    toast.style.top = '1.5rem';
    toast.style.right = '1.5rem';
    toast.style.background = 'rgba(15, 17, 26, 0.9)';
    toast.style.border = `1px solid ${color}`;
    toast.style.color = color;
    toast.style.fontWeight = '700';
    toast.style.fontSize = '0.85rem';
    toast.style.padding = '0.4rem 0.85rem';
    toast.style.borderRadius = '0.5rem';
    toast.style.backdropFilter = 'blur(8px)';
    toast.style.transition = 'transform 300ms ease, opacity 300ms ease';
    toast.style.transform = 'translateY(0)';
    toast.style.opacity = '1';
    toast.style.zIndex = '100';
    this.container.appendChild(toast);

    setTimeout(() => {
      toast.style.transform = 'translateY(-10px)';
      toast.style.opacity = '0';
      setTimeout(() => toast.remove(), 300);
    }, 1200);
  }

  bindEvents() {
    // Canvas Mouse Orbit Controls
    this.canvas.addEventListener('mousedown', (e) => {
      this.isDragging = true;
      this.prevMouse = { x: e.clientX, y: e.clientY };
    });
    window.addEventListener('mouseup', () => (this.isDragging = false));
    window.addEventListener('mousemove', (e) => {
      if (!this.isDragging) return;
      const dx = e.clientX - this.prevMouse.x;
      const dy = e.clientY - this.prevMouse.y;
      this.prevMouse = { x: e.clientX, y: e.clientY };

      this.cameraAngle.yaw -= dx * 0.006;
      this.cameraAngle.pitch = Math.max(0.05, Math.min(1.4, this.cameraAngle.pitch + dy * 0.006));
    });

    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.cameraAngle.distance = Math.max(2.0, Math.min(10.0, this.cameraAngle.distance + e.deltaY * 0.005));
    }, { passive: false });

    // Buttons
    document.getElementById('btnStrike')?.addEventListener('click', () => this.fireCannonball());
    document.getElementById('btnDie')?.addEventListener('click', () => this.knockout());
    document.getElementById('btnRevive')?.addEventListener('click', () => this.revive());
    document.getElementById('btnReset')?.addEventListener('click', () => this.resetForces());

    // Locomotion Selectors
    document.querySelectorAll('.btn-loco').forEach((btn) => {
      btn.addEventListener('click', (e) => {
        document.querySelectorAll('.btn-loco').forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        this.locomotionMode = btn.dataset.loco;
      });
    });

    // Slider Listeners
    const bindSlider = (id, valId, prop, scale = 1) => {
      const el = document.getElementById(id);
      const valEl = document.getElementById(valId);
      if (el && valEl) {
        el.addEventListener('input', () => {
          this.config[prop] = parseFloat(el.value) * scale;
          valEl.textContent = el.value;
        });
      }
    };

    bindSlider('sliderPForce', 'valPForce', 'pForce');
    bindSlider('sliderDForce', 'valDForce', 'dForce');
    bindSlider('sliderTorque', 'valTorque', 'maxTorque');
    bindSlider('sliderDamping', 'valDamping', 'jointDamping');

    window.addEventListener('resize', () => {
      const w = this.container.clientWidth;
      const h = this.container.clientHeight;
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(w, h);
    });
  }

  updateMasterPoses(dt) {
    this.simTime += dt;
    const speed = this.locomotionMode === 'run' ? 8.0 : this.locomotionMode === 'walk' ? 4.0 : 0.0;
    const stride = this.locomotionMode === 'run' ? 0.35 : this.locomotionMode === 'walk' ? 0.22 : 0.02;

    const cycle = this.simTime * speed;
    const hipBob = Math.sin(cycle * 2.0) * 0.04;
    const legPhase = Math.sin(cycle);

    // Compute target positions for all 11 bones
    this.limbs.forEach((limb, i) => {
      const target = limb.targetPos.copy(limb.basePos);

      if (this.locomotionMode !== 'idle') {
        target.y += hipBob;

        // Legs gait swing
        if (limb.name === 'LeftUpLeg') {
          target.z += legPhase * stride;
        } else if (limb.name === 'LeftLeg') {
          target.z += legPhase * stride * 0.8;
          target.y += Math.max(0, -legPhase) * 0.06;
        } else if (limb.name === 'RightUpLeg') {
          target.z -= legPhase * stride;
        } else if (limb.name === 'RightLeg') {
          target.z -= legPhase * stride * 0.8;
          target.y += Math.max(0, legPhase) * 0.06;
        }

        // Arm counter-swing
        if (limb.name === 'LeftArm' || limb.name === 'LeftForeArm') {
          target.z -= legPhase * stride * 0.9;
        } else if (limb.name === 'RightArm' || limb.name === 'RightForeArm') {
          target.z += legPhase * stride * 0.9;
        }
      }

      // Update ghost master indicator
      if (this.masterMarker.children[i]) {
        this.masterMarker.children[i].position.copy(target);
      }
    });
  }

  updatePhysics(dt) {
    const fixedDt = Math.min(dt, 0.033);

    // 1. Advance SlaveController State Machine
    if (!this.isAlive) {
      this.state = 'Dead';
      this.deadTimer += fixedDt;
      if (this.deadTimer >= this.config.deadTime) {
        this.isAlive = true;
      }
    } else if (this.currentCollisions > 0) {
      this.state = 'LoosingStrength';
    } else if (this.currentStrength < 1.0) {
      this.state = 'GainingStrength';
    } else {
      this.state = 'FollowingAnimation';
    }

    // Strength attenuation / recovery
    if (this.state === 'LoosingStrength') {
      this.currentStrength = Math.max(0, this.currentStrength - this.config.looseStrengthLerp * fixedDt);
    } else if (this.state === 'GainingStrength') {
      this.currentStrength = Math.min(1.0, this.currentStrength + this.config.gainStrengthLerp * fixedDt);
    } else if (this.state === 'FollowingAnimation') {
      this.currentStrength = 1.0;
    }

    // Interpolate force & torque coefficients
    const r = this.currentStrength;
    this.forceCoefficient = this.config.minContactForce + (1.0 - this.config.minContactForce) * r;
    this.torqueCoefficient = this.config.minContactTorque + (1.0 - this.config.minContactTorque) * r;

    // 2. Active Ragdoll PD Controller
    this.limbs.forEach((limb) => {
      if (!this.isAlive) {
        // Limp ragdoll under gravity
        limb.velocity.y -= 9.81 * fixedDt;
        limb.currentPos.addScaledVector(limb.velocity, fixedDt);

        // Ground floor collision
        const floorY = limb.size[1] * 0.5;
        if (limb.currentPos.y < floorY) {
          limb.currentPos.y = floorY;
          limb.velocity.y *= -0.15;
          limb.velocity.x *= 0.7;
          limb.velocity.z *= 0.7;
        }

        limb.mesh.position.copy(limb.currentPos);
        limb.mesh.rotation.x += limb.angularVelocity.x * fixedDt;
        limb.mesh.rotation.z += limb.angularVelocity.z * fixedDt;
        return;
      }

      // Linear COM PD Tracking: signal = P * (error + D * (error - lastError) / dt)
      const error = new THREE.Vector3().subVectors(limb.targetPos, limb.currentPos);
      const derivative = new THREE.Vector3().subVectors(error, limb.lastError).divideScalar(fixedDt);
      limb.lastError.copy(error);

      const pdSignal = new THREE.Vector3()
        .addScaledVector(error, this.config.pForce)
        .addScaledVector(derivative, this.config.pForce * this.config.dForce);

      // Clamp linear force
      const maxLimbForce = this.config.maxForce * this.forceCoefficient;
      if (pdSignal.length() > maxLimbForce) {
        pdSignal.setLength(maxLimbForce);
      }

      // Apply as VelocityChange impulse
      limb.velocity.addScaledVector(pdSignal, fixedDt);
      limb.velocity.multiplyScalar(0.92); // Damping drag

      limb.currentPos.addScaledVector(limb.velocity, fixedDt);
      limb.mesh.position.copy(limb.currentPos);

      // Slerp orientation toward neutral upright orientation
      limb.mesh.rotation.x *= 0.85;
      limb.mesh.rotation.y *= 0.85;
      limb.mesh.rotation.z *= 0.85;
    });

    // 3. Update Ballistic Projectiles and Collisions
    let hitCount = 0;
    for (let i = this.projectiles.length - 1; i >= 0; i--) {
      const p = this.projectiles[i];
      p.life -= fixedDt;
      p.vel.y -= 9.81 * 0.3 * fixedDt;
      p.mesh.position.addScaledVector(p.vel, fixedDt);

      // Check collision against all ragdoll limbs
      for (const limb of this.limbs) {
        const dist = p.mesh.position.distanceTo(limb.currentPos);
        if (dist < p.radius + limb.size[0] * 0.6) {
          hitCount++;
          // Impart momentum into ragdoll limb
          limb.velocity.addScaledVector(p.vel, 0.45);
          limb.angularVelocity.set((Math.random() - 0.5) * 5, 0, (Math.random() - 0.5) * 5);
          p.vel.multiplyScalar(-0.3); // Bounce
          break;
        }
      }

      if (p.life <= 0 || p.mesh.position.y < 0.1) {
        this.scene.remove(p.mesh);
        this.projectiles.splice(i, 1);
      }
    }
    this.currentCollisions = hitCount;
  }

  updateHUD() {
    // Badge state
    const badgeEl = document.getElementById('badgeState');
    if (badgeEl) {
      badgeEl.className = 'badge-state';
      if (this.state === 'FollowingAnimation') {
        badgeEl.textContent = 'Following Animation';
        badgeEl.classList.add('state-following');
      } else if (this.state === 'LoosingStrength') {
        badgeEl.textContent = 'Loosing Strength';
        badgeEl.classList.add('state-loosing');
      } else if (this.state === 'GainingStrength') {
        badgeEl.textContent = 'Gaining Strength';
        badgeEl.classList.add('state-gaining');
      } else {
        badgeEl.textContent = 'Dead / Knocked Out';
        badgeEl.classList.add('state-dead');
      }
    }

    // Strength Progress Bar
    const strengthBar = document.getElementById('strengthProgress');
    const strengthVal = document.getElementById('valStrength');
    if (strengthBar && strengthVal) {
      const pct = Math.round(this.currentStrength * 100);
      strengthBar.style.width = `${pct}%`;
      strengthVal.textContent = `${pct}%`;
    }

    // Force & Torque Coeffs
    const forceEl = document.getElementById('valForceCoeff');
    if (forceEl) forceEl.textContent = this.forceCoefficient.toFixed(2);

    const torqueEl = document.getElementById('valTorqueCoeff');
    if (torqueEl) torqueEl.textContent = this.torqueCoefficient.toFixed(2);

    // Active Contacts
    const contactsEl = document.getElementById('valContacts');
    if (contactsEl) contactsEl.textContent = this.currentCollisions.toString();

    // FPS
    this.fpsCounter++;
    const now = performance.now();
    if (now - this.lastFpsUpdate >= 500) {
      const fps = Math.round((this.fpsCounter * 1000) / (now - this.lastFpsUpdate));
      const fpsEl = document.getElementById('valFps');
      if (fpsEl) fpsEl.textContent = `${fps} FPS`;
      this.fpsCounter = 0;
      this.lastFpsUpdate = now;
    }
  }

  updateCamera() {
    const x = this.targetLookAt.x + this.cameraAngle.distance * Math.sin(this.cameraAngle.yaw) * Math.cos(this.cameraAngle.pitch);
    const y = this.targetLookAt.y + this.cameraAngle.distance * Math.sin(this.cameraAngle.pitch);
    const z = this.targetLookAt.z + this.cameraAngle.distance * Math.cos(this.cameraAngle.yaw) * Math.cos(this.cameraAngle.pitch);

    this.camera.position.set(x, y, z);
    this.camera.lookAt(this.targetLookAt);
  }

  animate() {
    requestAnimationFrame(this.animate);
    const dt = this.clock.getDelta();

    this.updateMasterPoses(dt);
    this.updatePhysics(dt);
    this.updateCamera();
    this.updateHUD();

    this.renderer.render(this.scene, this.camera);
  }
}

// Instantiate on load
window.addEventListener('DOMContentLoaded', () => {
  new ActiveRagdollSimulation();
});
