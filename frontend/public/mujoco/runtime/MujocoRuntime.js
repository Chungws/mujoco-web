import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { downloadExampleScenesFolder, getPosition, getQuaternion, loadSceneFromURL } from './utils/mujocoScene.js';
import { ONNXModule } from './utils/onnxHelper.js';
import { TrajectoryActionManager } from './managers/actions/TrajectoryActionManager.js';

const DEFAULT_CONTAINER_ID = 'mujoco-container';

export class MujocoRuntime {
  constructor(mujoco, options = {}) {
    this.mujoco = mujoco;
    const workingPath = '/working';
    try {
      mujoco.FS.mkdir(workingPath);
    } catch (error) {
      if (error?.code !== 'EEXIST') {
        console.warn('Failed to create /working directory:', error);
      }
    }
    try {
      mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, workingPath);
    } catch (error) {
      if (error?.code !== 'EEXIST' && error?.code !== 'EBUSY') {
        console.warn('Failed to mount MEMFS at /working:', error);
      }
    }
    this.options = options;
    this.container = document.getElementById(options.containerId || DEFAULT_CONTAINER_ID);
    if (!this.container) {
      throw new Error(`Failed to find container element with id ${options.containerId || DEFAULT_CONTAINER_ID}`);
    }

    this.commandManager = options.commandManager || null;
    this.actionManager = options.actionManager || null;
    this.observationManagers = options.observationManagers || [];
    this.envManagers = options.envManagers || [];

    this.services = new Map();

    this.params = {
      paused: true,
      help: false,
      command_vel_x: 0.0,
      impedance_kp: 24.0,
      use_setpoint: true,
      impulse_remain_time: 0.0,
      compliant_mode: false,
    };

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.001, 1000);
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5);

    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.1);
    this.ambientLight.name = 'AmbientLight';
    this.scene.add(this.ambientLight);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.2, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));
    this.renderer.setAnimationLoop(this.render.bind(this));

    this.lastSimState = {
      bodies: new Map(),
      lights: new Map(),
      tendons: {
        numWraps: 0,
        matrix: new THREE.Matrix4()
      }
    };

    this.mjModel = null;
    this.mjData = null;
    this.policy = null;
    this.inputDict = null;
    this.loopHandle = null;
    this.running = false;
    this.alive = false;
    this.actionContext = {};
    this.assetMetadata = null;

    this.attachManagers();
  }

  attachManagers() {
    if (this.commandManager) {
      this.commandManager.attachRuntime(this);
    }
    if (this.actionManager) {
      this.actionManager.attachRuntime(this);
    }
    for (const manager of this.observationManagers) {
      manager.attachRuntime(this);
    }
    for (const manager of this.envManagers) {
      manager.attachRuntime(this);
    }
  }

  registerService(name, service) {
    this.services.set(name, service);
  }

  unregisterService(name) {
    this.services.delete(name);
  }

  getService(name) {
    return this.services.get(name);
  }

  async init(initialConfig = {}) {
    if (this.commandManager && typeof this.commandManager.onInit === 'function') {
      await this.commandManager.onInit();
    }
    if (this.actionManager && typeof this.actionManager.onInit === 'function') {
      await this.actionManager.onInit();
    }
    for (const manager of [...this.observationManagers, ...this.envManagers]) {
      if (typeof manager.onInit === 'function') {
        await manager.onInit();
      }
    }

    if (initialConfig.scenePath) {
      await this.loadEnvironment(initialConfig);
    }
  }

  async loadEnvironment({ scenePath, metaPath, policyPath }) {
    await this.stop();
    await downloadExampleScenesFolder(this.mujoco, scenePath);
    await this.loadScene(scenePath, metaPath);

    if (policyPath) {
      await this.loadPolicy(policyPath);
    } else {
      await this.clearPolicy();
    }
    this.alive = true;
    this.running = true;
    this.startLoop();
  }

  async clearPolicy() {
    this.policy = null;
    this.policyConfig = null;
    this.inputDict = null;
    this.isInferencing = false;

    if (this.actionManager && typeof this.actionManager.onPolicyCleared === 'function') {
      await this.actionManager.onPolicyCleared();
    }
    for (const manager of this.observationManagers) {
      if (typeof manager.onPolicyCleared === 'function') {
        await manager.onPolicyCleared();
      }
    }
  }

  async loadScene(mjcfPath, metaPath) {
    if (this.loadingScene) {
      await this.loadingScene;
    }
    this.loadingScene = (async () => {
      this.scene.remove(this.scene.getObjectByName('MuJoCo Root'));

      [this.mjModel, this.mjData, this.bodies, this.lights] =
        await loadSceneFromURL(this.mujoco, mjcfPath, this);

      if (!this.mjModel || this.mjModel.deleted) {
        console.warn("MjModel is invalid after loadSceneFromURL");
        return;
      }

      let assetMeta = null;
      if (metaPath && metaPath !== 'null') {
        const response = await fetch(metaPath);
        assetMeta = await response.json();
      }

      // Camera condfig from asset metadata (if present)
      try {
        this.applyCameraFromMetadata(assetMeta);
      } catch (e) {
        console.warn('[MujocoRuntime] Failed to apply camera from metadata:', e);
      }

      // safe guard
      if (!this.mjModel || !this.mjModel.opt) {
        throw new Error("mjModel is invalid or deleted before accessing opt");
      }

      this.timestep = this.mjModel.opt.timestep;
      this.decimation = Math.round(0.02 / this.timestep);
      this.mujoco_time = 0.0;
      this.simStepCount = 0;
      this.inferenceStepCount = 0;
      this.assetMetadata = assetMeta;

      if (this.actionManager?.onSceneLoaded) {
        await this.actionManager.onSceneLoaded({
          mjModel: this.mjModel,
          mjData: this.mjData,
          assetMeta,
        });
      }
      for (const manager of this.envManagers) {
        if (manager?.onSceneLoaded) {
          await manager.onSceneLoaded({
            mjModel: this.mjModel,
            mjData: this.mjData,
            assetMeta,
          });
        }
      }

      this.observationContext = { assetMeta };
      this.loadingScene = null;
    })();

    await this.loadingScene;
  }

  // Reset to default camera, then apply overrides from the scene's asset metadata.
  // Supports the following (all optional):
  // - assetMeta.camera.pos: [x, y, z]
  // - assetMeta.camera.target: [x, y, z]
  // - assetMeta.camera.fov: number
  applyCameraFromMetadata(assetMeta) {
    // Always reset to defaults on scene/policy change
    this.resetDefaultCamera();

    if (!assetMeta || typeof assetMeta !== 'object') {
      return; // no overrides provided
    }

    const cameraMeta = assetMeta.camera && typeof assetMeta.camera === 'object'
      ? assetMeta.camera
      : {};

    const pickVec3 = (v) => (Array.isArray(v) && v.length >= 3 && v.every(n => typeof n === 'number'))
      ? [v[0], v[1], v[2]]
      : null;

    const pos = pickVec3(cameraMeta.pos ?? assetMeta.camera_pos);
    const target = pickVec3(cameraMeta.target ?? assetMeta.camera_target);
    const fov = typeof (cameraMeta.fov ?? assetMeta.camera_fov) === 'number'
      ? (cameraMeta.fov ?? assetMeta.camera_fov)
      : null;

    // Apply FOV if provided
    if (typeof fov === 'number' && isFinite(fov) && fov > 0) {
      this.camera.fov = fov;
      this.camera.updateProjectionMatrix();
    }

    // Apply target before position/angle calculations
    if (target) {
      this.camera.lookAt(target[0], target[1], target[2]);
      this.controls.target.set(target[0], target[1], target[2]);
    }

    // If absolute position provided, use it directly
    if (pos) {
      this.camera.position.set(pos[0], pos[1], pos[2]);
      this.controls.update();
      return;
    }
  }

  // Re-apply the runtime's default camera configuration.
  resetDefaultCamera() {
    // Defaults must match the constructor
    this.camera.fov = 45;
    this.camera.position.set(2.0, 1.7, 1.7);
    this.controls.target.set(0, 0.2, 0);
    this.controls.update();
    this.camera.updateProjectionMatrix();
  }

  async loadPolicy(policyPath) {
    while (this.isInferencing) {
      await new Promise(resolve => setTimeout(resolve, 10));
    }

    let config;
    try {
      const response = await fetch(policyPath);
      config = await response.json();

      console.log('[MujocoRuntime] Loading policy from config:', config);
      this.policyConfig = config;
      this.policy = new ONNXModule(config.onnx);
      console.log('[MujocoRuntime] ONNXModule created, calling init...');
      await this.policy.init();
      console.log('[MujocoRuntime] Policy initialized successfully, session:', this.policy.session);
    } catch (error) {
      console.error('[MujocoRuntime] Failed to load policy:', error);
      throw error;
    }

    if (this.actionManager && typeof this.actionManager.onPolicyLoaded === 'function') {
      await this.actionManager.onPolicyLoaded({ config });
    }
    for (const manager of this.observationManagers) {
      if (typeof manager.onPolicyLoaded === 'function') {
        await manager.onPolicyLoaded({
          config,
          mjModel: this.mjModel,
          mjData: this.mjData,
          assetMeta: this.observationContext?.assetMeta,
        });
      }
    }
    for (const manager of this.envManagers) {
      if (typeof manager.onPolicyLoaded === 'function') {
        await manager.onPolicyLoaded({
          config,
          mjModel: this.mjModel,
          mjData: this.mjData,
          assetMeta: this.observationContext?.assetMeta,
        });
      }
    }

    this.mujoco.mj_resetData(this.mjModel, this.mjData);
    this.mujoco.mj_forward(this.mjModel, this.mjData);

    this.adapt_hx = new Float32Array(128);
    this.rpy = new THREE.Euler();
    this.quat = new THREE.Quaternion();

    this.inputDict = this.policy.initInput();
    this.isInferencing = false;
  }

  async startLoop() {
    if (this.loopPromise) {
      return this.loopPromise;
    }
    this.running = true;
    this.loopPromise = this.mainLoop();
    return this.loopPromise;
  }

  async stop() {
    this.running = false;
    const pending = this.loopPromise;
    if (pending) {
      await pending;
    }
    this.loopPromise = null;
    this.alive = false;
  }

  async mainLoop() {
    this.inputDict = this.inputDict || (this.policy ? this.policy.initInput() : {});
    while (this.running) {
      const loopStart = performance.now();
      const ready = !this.params.paused && this.mjModel && this.mjData;
      if (ready) {
        if (this.actionManager instanceof TrajectoryActionManager) {
          const obsTensors = await this.collectObservations();
          const action = await this.actionManager.generateAction(obsTensors);
          this.applyAction(action);
          await this.executeSimulationSteps();
          this.updateCachedState();
        } else if (this.policy) {
          let time_start = performance.now();
          const quat = this.mjData.qpos.subarray(3, 7);
          this.quat.set(quat[1], quat[2], quat[3], quat[0]);
          this.rpy.setFromQuaternion(this.quat);

          const obsTensors = await this.collectObservations();
          Object.assign(this.inputDict, obsTensors);

          try {
            await this.runInference();
          } catch (e) {
            console.error('Inference error in main loop:', e);
            this.running = false;
            break;
          }

          let time_end = performance.now();
          const policy_inference_time = time_end - time_start;
          time_start = time_end;

          await this.executeSimulationSteps();

          time_end = performance.now();
          const sim_step_time = time_end - time_start;
          time_start = time_end;

          this.updateCachedState();

          time_end = performance.now();
          const update_render_time = time_end - time_start;
        } else {
          await this.executeSimulationSteps();
          this.updateCachedState();
        }
      }

      const loopEnd = performance.now();
      const elapsed = (loopEnd - loopStart) / 1000;
      const sleepTime = Math.max(0, this.timestep * this.decimation - elapsed);
      await new Promise(resolve => setTimeout(resolve, sleepTime * 1000));
    }
    this.loopPromise = null;
  }

  applyAction(action) {
    if (!this.mjData || !this.mjData.ctrl) {
      return;
    }
    const ctrl = this.mjData.ctrl;
    if (!action || typeof action.length !== 'number') {
      ctrl.fill(0);
      return;
    }
    const length = Math.min(action.length, ctrl.length);
    for (let i = 0; i < length; i++) {
      ctrl[i] = action[i];
    }
    for (let i = length; i < ctrl.length; i++) {
      ctrl[i] = 0;
    }
  }

  async executeSimulationSteps() {
    for (let substep = 0; substep < this.decimation; substep++) {
      const stepContext = {
        mjModel: this.mjModel,
        mjData: this.mjData,
        timestep: this.timestep,
        substep,
      };
      if (this.actionManager && typeof this.actionManager.beforeSimulationStep === 'function') {
        this.actionManager.beforeSimulationStep(stepContext);
      }
      for (const manager of this.envManagers) {
        if (typeof manager.beforeSimulationStep === 'function') {
          manager.beforeSimulationStep(stepContext);
        }
      }

      this.mujoco.mj_step(this.mjModel, this.mjData);
      this.mujoco_time += this.timestep * 1000.0;
      this.simStepCount += 1;

      if (this.actionManager && typeof this.actionManager.afterSimulationStep === 'function') {
        this.actionManager.afterSimulationStep(stepContext);
      }
      for (const manager of this.envManagers) {
        if (typeof manager.afterSimulationStep === 'function') {
          manager.afterSimulationStep(stepContext);
        }
      }
    }
  }

  async collectObservations() {
    if (!this.observationManagers.length) {
      return {};
    }
    const tensors = {};
    for (const manager of this.observationManagers) {
      if (typeof manager.collect === 'function') {
        const result = manager.collect({
          mjModel: this.mjModel,
          mjData: this.mjData,
          policyConfig: this.policyConfig,
          params: this.params,
        });
        Object.assign(tensors, result);
      }
    }
    return tensors;
  }

  async runInference() {
    if (!this.policy || this.isInferencing) {
      return;
    }
    this.isInferencing = true;
    this.inferenceStepCount += 1;
    try {
      if (!this.policy.inKeys) {
        console.error('[MujocoRuntime] Policy inKeys is undefined!', {
          policy: this.policy,
          inKeys: this.policy.inKeys,
          session: this.policy.session,
          metaData: this.policy.metaData
        });
      }
      const [result, carry] = await this.policy.runInference(this.inputDict);
      if (this.actionManager && typeof this.actionManager.onPolicyOutput === 'function') {
        this.actionManager.onPolicyOutput(result);
      }
      this.inputDict = carry;
    } finally {
      this.isInferencing = false;
    }
  }

  updateCachedState() {
    if (!this.mjModel || !this.mjData) {
      return;
    }
    for (let b = 0; b < this.mjModel.nbody; b++) {
      if (this.bodies[b]) {
        if (!this.lastSimState.bodies.has(b)) {
          this.lastSimState.bodies.set(b, {
            position: new THREE.Vector3(),
            quaternion: new THREE.Quaternion()
          });
        }
        getPosition(this.mjData.xpos, b, this.lastSimState.bodies.get(b).position);
        getQuaternion(this.mjData.xquat, b, this.lastSimState.bodies.get(b).quaternion);
      }
    }

    for (let l = 0; l < this.mjModel.nlight; l++) {
      if (this.lights[l]) {
        if (!this.lastSimState.lights.has(l)) {
          this.lastSimState.lights.set(l, {
            position: new THREE.Vector3(),
            direction: new THREE.Vector3()
          });
        }
        getPosition(this.mjData.light_xpos, l, this.lastSimState.lights.get(l).position);
        getPosition(this.mjData.light_xdir, l, this.lastSimState.lights.get(l).direction);
      }
    }

    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      let numWraps = 0;
      const mat = this.lastSimState.tendons.matrix;

      for (let t = 0; t < this.mjModel.ntendon; t++) {
        let startW = this.mjData.ten_wrapadr[t];
        let r = this.mjModel.tendon_width[t];
        for (let w = startW; w < startW + this.mjData.ten_wrapnum[t] - 1; w++) {
          let tendonStart = getPosition(this.mjData.wrap_xpos, w, new THREE.Vector3());
          let tendonEnd = getPosition(this.mjData.wrap_xpos, w + 1, new THREE.Vector3());
          let tendonAvg = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

          let validStart = tendonStart.length() > 0.01;
          let validEnd = tendonEnd.length() > 0.01;

          if (validStart) { this.mujocoRoot.spheres.setMatrixAt(numWraps, mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validEnd) { this.mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validStart && validEnd) {
            mat.compose(tendonAvg,
              new THREE.Quaternion().setFromUnitVectors(
                new THREE.Vector3(0, 1, 0),
                tendonEnd.clone().sub(tendonStart).normalize()
              ),
              new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r)
            );
            this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
            numWraps++;
          }
        }
      }
      this.lastSimState.tendons.numWraps = numWraps;
    }
  }

  render() {
    if (!this.mjModel || !this.mjData) {
      return;
    }

    this.controls.update();

    for (const [b, state] of this.lastSimState.bodies) {
      if (this.bodies[b]) {
        this.bodies[b].position.copy(state.position);
        this.bodies[b].quaternion.copy(state.quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    for (const [l, state] of this.lastSimState.lights) {
      if (this.lights[l]) {
        this.lights[l].position.copy(state.position);
        this.lights[l].lookAt(state.direction.add(this.lights[l].position));
      }
    }

    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      const numWraps = this.lastSimState.tendons.numWraps;
      this.mujocoRoot.cylinders.count = numWraps;
      this.mujocoRoot.spheres.count = numWraps > 0 ? numWraps + 1 : 0;
      this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
      this.mujocoRoot.spheres.instanceMatrix.needsUpdate = true;
    }

    this.renderer.render(this.scene, this.camera);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  pause() {
    this.params.paused = true;
  }

  resume() {
    this.params.paused = false;
  }

  async reset() {
    if (!this.mjData) {
      return;
    }
    this.params.paused = true;
    this.mujoco.mj_resetData(this.mjModel, this.mjData);
    this.mujoco.mj_forward(this.mjModel, this.mjData);
    this.params.paused = false;
    if (this.commandManager && typeof this.commandManager.reset === 'function') {
      this.commandManager.reset();
    }
    for (const manager of this.envManagers) {
      if (typeof manager.reset === 'function') {
        manager.reset();
      }
    }
  }

  dispose() {
    // Stop the simulation loop first
    this.stop();

    // Clear policy and ONNX session
    if (this.policy && this.policy.session) {
      try {
        this.policy.session.release();
      } catch (e) {
        console.warn('Failed to release ONNX session:', e);
      }
    }
    this.policy = null;
    this.inputDict = null;

    // Free WebAssembly objects in correct order
    if (this.mjData) {
      try {
        this.mjData.delete();
      } catch (e) {
        console.warn('Failed to delete mjData:', e);
      }
      this.mjData = null;
    }
    if (this.mjModel) {
      try {
        this.mjModel.delete();
      } catch (e) {
        console.warn('Failed to delete mjModel:', e);
      }
      this.mjModel = null;
    }

    // Dispose Three.js scene objects
    this.disposeThreeJSResources();

    // Remove event listeners and dispose renderer
    window.removeEventListener('resize', this.onWindowResize);
    if (this.controls) {
      this.controls.dispose();
    }
    this.renderer.setAnimationLoop(null);
    this.renderer.dispose();

    // Dispose managers
    if (this.commandManager && typeof this.commandManager.dispose === 'function') {
      this.commandManager.dispose();
    }
    if (this.actionManager && typeof this.actionManager.dispose === 'function') {
      this.actionManager.dispose();
    }
    for (const manager of [...this.observationManagers, ...this.envManagers]) {
      if (typeof manager.dispose === 'function') {
        manager.dispose();
      }
    }

    // Clear references
    this.bodies = null;
    this.lights = null;
    this.mujocoRoot = null;
    this.lastSimState = null;
    this.services.clear();
  }

  disposeThreeJSResources() {
    if (this.scene) {
      // Recursively dispose all objects in the scene
      this.scene.traverse((object) => {
        if (object.geometry) {
          object.geometry.dispose();
        }
        if (object.material) {
          if (Array.isArray(object.material)) {
            object.material.forEach(material => {
              this.disposeMaterial(material);
            });
          } else {
            this.disposeMaterial(object.material);
          }
        }
      });

      // Clear the scene
      while (this.scene.children.length > 0) {
        this.scene.remove(this.scene.children[0]);
      }
    }
  }

  disposeMaterial(material) {
    if (material) {
      // Dispose textures
      Object.keys(material).forEach(prop => {
        const value = material[prop];
        if (value && typeof value === 'object' && value.isTexture) {
          value.dispose();
        }
      });

      // Dispose the material itself
      material.dispose();
    }
  }
}
