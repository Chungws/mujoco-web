---
**⚠️ ARCHIVED SPECIFICATION**

This document represents the original Phase 1 (Visualization only) approach and has been **superseded by the integrated MVP specification** (2025-01-04).

**Why Archived:**
- Original plan separated visualization (Phase 1) from comparison (Phase 2)
- New specification integrates both into unified MVP approach
- Benchmark results (283ms VLA inference) confirmed all server-side architecture
- lmarena-clone structure adopted for faster development

**Current Specification:** See [../ROADMAP.md](../ROADMAP.md) and [../00_PROJECT.md](../00_PROJECT.md)

**Useful Content:**
- Plugin architecture concepts (adapted to MVP)
- MuJoCo integration details remain relevant
- Technical specifications still applicable

**Date Archived:** 2025-01-04
---

# Feature: VLA Visualization (Simulator-based) - Phase 1

**Status:** ~~Not Started~~ ARCHIVED
**Priority:** High (MVP Core Feature)
**Estimated Time:** 3-4 weeks

---

## Overview

Build a web-based visualization system for Vision-Language-Action (VLA) models using MuJoCo physics simulation. Users can load environments, provide natural language instructions, and observe VLA policy execution in real-time within a browser-based 3D viewer.

**Goals:**
- Real-time VLA policy visualization in simulated environments
- Interactive 3D viewer with MuJoCo WebAssembly
- Instruction-based policy control
- Support multiple robot environments (arms, humanoids, quadrupeds)
- Browser-based execution (no server-side computation for Phase 1)

---

## Architecture

Based on **[ADR-001: Plugin-based Architecture](../ARCHITECTURE/ADR_001-Plugin_Architecture.md)**, this feature uses a modular, composable design with Robot-First selection and Tag-based compatibility filtering.

### High-Level Architecture

```
┌─────────────────────────────────────────────────┐
│              User Interface (Next.js)            │
│                                                   │
│  Step 1: [Robot Selector]                       │
│    → Franka Panda, UR5, Humanoid (no filter)    │
│                                                   │
│  Step 2: [Scene Selector] (filtered by robot)   │
│    → Kitchen, Table (only compatible scenes)    │
│                                                   │
│  Step 3: [VLA Model Selector] (filtered)        │
│    → RT-1, OpenVLA (only compatible models)     │
│                                                   │
│  Step 4: [Physics Engine Selector]              │
│    → ● Local (WASM)  ○ Remote (Phase 2)        │
│                                                   │
│  [Instruction Input] + [Playback Controls]      │
└──────────────┬──────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────┐
│         Configuration & Compatibility Layer      │
│                                                    │
│  Robot Config:  tags: [7dof-arm, gripper]        │
│  Scene Config:  compatible_robots: [7dof-arm]    │
│  VLA Config:    compatible_with: [7dof-arm]      │
│  Physics Config: type: 'local'                   │
│                                                    │
│  Tag Matcher: Filter scenes/VLA by robot tags    │
└──────────────┬───────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────┐
│              Plugin Factory                       │
│                                                    │
│  createPhysicsPlugin(config) → LocalMuJoCoPlugin │
│  createVLAPlugin(config)     → LocalVLAPlugin    │
└──────────────┬───────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────┐
│            VLA Executor (Episode Loop)            │
│                                                    │
│  ┌──────────────┐        ┌──────────────┐       │
│  │   Physics    │        │     VLA      │       │
│  │   Plugin     │◄──────►│   Plugin     │       │
│  │              │ Adapter│              │       │
│  │ (MuJoCo WASM)│        │(transformers)│       │
│  └──────────────┘        └──────────────┘       │
│                                                    │
│  Loop:                                            │
│    1. observation = physics.getObservation()     │
│    2. resized = adapter.resize(observation)      │
│    3. action = vla.predict(resized, instruction) │
│    4. physics.step(action)                       │
│    5. Render state in MuJoCo Viewer              │
└──────────────────────────────────────────────────┘
```

### Key Components

1. **Component Selectors** (UI)
   - Robot Selector (no filtering - user picks first)
   - Scene Selector (filtered by robot compatibility tags)
   - VLA Model Selector (filtered by robot compatibility tags)
   - Physics Engine Selector (Local WASM / Remote API)

2. **Configuration System**
   - Robot configs with capability tags
   - Scene configs with compatible robot tags
   - VLA configs with compatible robot tags
   - Tag-based filtering to prevent invalid combinations

3. **Plugin System**
   - `PhysicsEnginePlugin` interface (LocalMuJoCoPlugin for Phase 1)
   - `VLAPolicyPlugin` interface (LocalVLAPlugin for Phase 1)
   - Factory functions to instantiate plugins from configs

4. **VLA Executor**
   - Episode loop coordinator
   - Lightweight Adapter for data transformations
   - Observation → Action pipeline

5. **MuJoCo Viewer**
   - WebGL-based 3D renderer
   - Camera controls (orbit, zoom, pan, reset)
   - Real-time state visualization
   - Action overlay display

---

## Design Decisions

All major architectural decisions are documented in **[ADR-001: Plugin Architecture](../ARCHITECTURE/ADR_001-Plugin_Architecture.md)**. This section highlights key decisions specific to Phase 1 implementation.

### 1. Plugin-based Architecture (ADR-001)

**Decision:** Use plugin interfaces for Physics Engine and VLA Policy

**PhysicsEnginePlugin Interface:**
```typescript
interface PhysicsEnginePlugin {
  readonly type: 'local' | 'remote';
  readonly id: string;

  initialize(sceneConfig: SceneConfig): Promise<void>;
  step(action: Float32Array): Promise<void>;
  getObservation(): Promise<{
    images: Map<string, ImageData>;
    proprioception: Float32Array;
  }>;
  getActionSpec(): ActionSpec;
  getObservationSpec(): ObservationSpec;
}
```

**VLAPolicyPlugin Interface:**
```typescript
interface VLAPolicyPlugin {
  readonly type: 'local' | 'remote';
  readonly id: string;

  initialize(): Promise<void>;
  predict(input: {
    image: ImageData;
    proprioception: Float32Array;
    instruction: string;
  }): Promise<{
    action: Float32Array;
    metadata?: any;
  }>;
  getInputSpec(): PolicyInputSpec;
  getOutputSpec(): PolicyOutputSpec;
}
```

**Phase 1 Implementations:**
- `LocalMuJoCoPlugin`: MuJoCo WASM-based physics
- `LocalVLAPlugin`: transformers.js-based inference (NOT ONNX Runtime Web)

**Rationale:**
- Interface-based design (no inheritance complexity)
- Easy to swap implementations (local ↔ remote)
- Testable with mocks
- Phase 2-ready (add Remote plugins without changing interfaces)

### 2. Robot-First Selection Flow (ADR-001)

**Decision:** Users select components in order: Robot → Scene → VLA Model → Physics

```
1. Select Robot (e.g., Franka Panda)
   ↓ (no filtering)
2. Select Scene (filtered by robot tags)
   ✅ Kitchen (compatible: [7dof-arm, 6dof-arm])
   ❌ Locomotion (compatible: [humanoid])
   ↓
3. Select VLA Model (filtered by robot tags)
   ✅ RT-1 (compatible: [7dof-arm, 6dof-arm])
   ❌ HumanoidVLA (compatible: [humanoid])
   ↓
4. Select Physics Engine
   ● Local (WASM)  ○ Remote (Phase 2)
```

**Rationale:**
- Robot is the compatibility anchor point
- Prevents invalid combinations at selection time
- Natural mental model: "What can this robot do?"
- Supports Phase 2: same robot+scene, random VLA selection

### 3. Tag-based Compatibility (ADR-001)

**Decision:** Use string tags for compatibility matching

**Robot Configuration:**
```typescript
{
  id: 'franka-panda',
  name: 'Franka Panda',
  action_dim: 7,
  tags: ['7dof-arm', 'gripper', 'manipulation']
}
```

**Scene Configuration:**
```typescript
{
  id: 'kitchen',
  name: 'Kitchen Environment',
  path: '/mujoco/scenes/franka_kitchen.xml',
  compatible_robots: ['7dof-arm', '6dof-arm'],  // Tag filtering
  tags: ['manipulation', 'indoor']
}
```

**VLA Model Configuration:**
```typescript
{
  id: 'rt-1',
  name: 'RT-1 (Robotics Transformer)',
  modelPath: '/models/rt1.transformers',
  compatible_with: ['7dof-arm', '6dof-arm'],  // Tag filtering
  input_spec: { image_size: [256, 256], action_dim: 7 }
}
```

**Filtering Logic:**
```typescript
// User selects Franka (tags: [7dof-arm, gripper, manipulation])
const compatibleScenes = scenes.filter(scene =>
  scene.compatible_robots.some(tag => robot.tags.includes(tag))
);
const compatibleVLAs = vlas.filter(vla =>
  vla.compatible_with.some(tag => robot.tags.includes(tag))
);
```

**Standard Tag Vocabulary:**
- Robot type: `7dof-arm`, `6dof-arm`, `humanoid`, `quadruped`
- Capabilities: `gripper`, `manipulation`, `locomotion`
- Scene type: `manipulation`, `locomotion`, `indoor`, `outdoor`

**Rationale:**
- Simple string intersection (no complex logic)
- Explicit compatibility (no runtime guessing)
- Prevents invalid combinations
- Easy to extend (just add tags)

### 4. VLA Model Format & Inference

**Decision:** transformers.js for browser-based VLA inference (Phase 1)

**Technology:**
- **NOT** ONNX Runtime Web (changed from initial plan)
- **USE** transformers.js for in-browser inference
- Models in HuggingFace format
- Lightweight models (<100 MB for Phase 1)

**Rationale:**
- transformers.js has better VLA model support
- Easier integration with vision-language models
- Native support for instruction encoding
- Performance sufficient for Phase 1 demonstrations

**Future (Phase 2):**
- Remote VLA API for larger models
- GPU acceleration on server

### 5. Instruction Input

**Decision:** Natural language text input

**UI Component:**
- Text input field for instructions
- Example instructions dropdown
- Instruction history
- Clear/Reset button

**Example Instructions:**
- "Pick up the red cube and place it on the blue plate"
- "Open the drawer and take out the object inside"
- "Walk forward and turn left at the corner"

**Encoding:**
- Handled by VLA model's built-in encoder
- No separate language model needed (Phase 1)

### 6. Visualization & Playback Controls

**Decision:** Essential controls for policy observation

**Playback Controls:**
- Play/Pause/Reset buttons
- Speed adjustment (0.25x, 0.5x, 1x, 2x)
- Step-by-step execution (single timestep forward/back)
- Episode timer and step counter

**Camera Controls:**
- Mouse: Orbit (drag), Zoom (scroll), Pan (right-drag)
- Keyboard shortcuts: R (reset), Space (play/pause)
- Preset camera angles (front, side, top, wrist)

**Action Visualization:**
- Current action vector display (numerical)
- Active joints highlighted in 3D view
- Action history graph (last 100 steps)
- Trajectory predictions (if available from VLA)

**Observation Views:**
- Main 3D viewer (user-controlled camera)
- Observation camera feeds (robot POV)
- Side-by-side comparison option

**UI Layout:**
```
┌─────────────────────────────────────────────────────┐
│  Header: [Robot] [Scene] [VLA] [Physics] [Go!]     │
├─────────────────────────────────────────────────────┤
│  Instruction: [Text input...] [Examples ▼]         │
├─────────────────────────────────────────────────────┤
│                                                       │
│              MuJoCo 3D Viewer (Main)                 │
│                                                       │
│              [Action Vector Overlay]                 │
│                                                       │
├─────────────────────────────────────────────────────┤
│  Playback: [◀ Step] [▶ Play] [⏸ Pause] [↻ Reset]  │
│  Speed: [0.5x] [1x] [2x]   Step: 0/1000            │
├─────────────────────────────────────────────────────┤
│  Observation Cameras:  [Wrist] [Third Person]       │
│  ┌──────────┐  ┌──────────┐  [Action History]      │
│  │ Camera 1 │  │ Camera 2 │  └──────────────┘      │
│  └──────────┘  └──────────┘                         │
└─────────────────────────────────────────────────────┘
```

---

## Phase Breakdown

Based on **Plugin Architecture (ADR-001)**, Phase 1 is reorganized to build the system layer-by-layer from configurations to UI.

### Phase 1.1: Configuration System & Tag Matching (Week 1)

**Goal:** Establish configuration system with tag-based compatibility filtering

**Tasks:**
- [ ] Define configuration schemas (Robot, Scene, VLA, Physics)
- [ ] Create configuration files with standard tags
  - [ ] `config/robots.ts` - At least 3 robots (Franka, UR5, Humanoid)
  - [ ] `config/scenes.ts` - At least 3 scenes (Kitchen, Table, Locomotion)
  - [ ] `config/vla-models.ts` - At least 2 VLA models
  - [ ] `config/physics.ts` - Local/Remote options
- [ ] Implement tag constants to prevent typos
- [ ] Build compatibility matcher (tag-based filtering)
- [ ] Add config validation (validate tags, paths, specs)

**Acceptance Criteria:**
- ✅ All configs follow standard schema
- ✅ Tag vocabulary defined and enforced
- ✅ Filtering logic works correctly
- ✅ Invalid configs rejected with clear errors

**Files to Create:**
- `frontend/config/robots.ts`
- `frontend/config/scenes.ts`
- `frontend/config/vla-models.ts`
- `frontend/config/physics.ts`
- `frontend/config/tags.ts` (tag constants)
- `frontend/lib/compatibility/matcher.ts`
- `frontend/lib/compatibility/validator.ts`
- `frontend/lib/types/config.ts`

---

### Phase 1.2: Plugin System Foundation (Week 1-2)

**Goal:** Define plugin interfaces and factory system

**Tasks:**
- [ ] Define `PhysicsEnginePlugin` interface
- [ ] Define `VLAPolicyPlugin` interface
- [ ] Define spec types (ActionSpec, ObservationSpec, etc.)
- [ ] Create plugin factory functions
- [ ] Implement plugin registry system
- [ ] Add helper functions (resize, normalize, clip)
- [ ] Write unit tests for interfaces

**Acceptance Criteria:**
- ✅ Plugin interfaces well-defined
- ✅ Factory can create plugins from configs
- ✅ Registry manages plugin lifecycle
- ✅ Helper functions tested

**Files to Create:**
- `frontend/lib/plugins/physics-engine.ts` (interface)
- `frontend/lib/plugins/vla-policy.ts` (interface)
- `frontend/lib/plugins/types.ts` (specs)
- `frontend/lib/factory/plugin-factory.ts`
- `frontend/lib/factory/plugin-registry.ts`
- `frontend/lib/helpers/image-utils.ts`
- `frontend/lib/helpers/action-utils.ts`
- `frontend/__tests__/plugins/*.test.ts`

---

### Phase 1.3: Physics Integration (Week 2)

**Goal:** Implement LocalMuJoCoPlugin and MuJoCo Viewer

**Tasks:**
- [ ] Implement `LocalMuJoCoPlugin`
  - [ ] initialize(sceneConfig)
  - [ ] step(action)
  - [ ] getObservation()
  - [ ] getActionSpec() / getObservationSpec()
- [ ] Enhance MuJoCo viewer component
  - [ ] Camera controls (orbit, zoom, pan, reset)
  - [ ] Keyboard shortcuts
  - [ ] FPS counter
- [ ] Scene loading from public/mujoco/scenes/
- [ ] Add at least 3 MuJoCo scene files
- [ ] Test physics simulation loop

**Acceptance Criteria:**
- ✅ LocalMuJoCoPlugin implements interface correctly
- ✅ MuJoCo WASM loads and runs scenes
- ✅ Camera controls work smoothly
- ✅ Performance >= 30 FPS
- ✅ All 3 scenes load successfully

**Files to Create/Modify:**
- `frontend/lib/plugins/local-mujoco.ts` (new)
- `frontend/components/mujoco-viewer.tsx` (enhance)
- `frontend/components/camera-controls.tsx` (new)
- `frontend/lib/mujoco/scene-loader.ts` (new)
- `public/mujoco/scenes/franka_kitchen.xml` (new)
- `public/mujoco/scenes/ur5_table.xml` (new)
- `public/mujoco/scenes/humanoid_flat.xml` (new)

---

### Phase 1.4: VLA Integration (Week 3)

**Goal:** Implement LocalVLAPlugin, VLA Executor, and Adapter

**Tasks:**
- [ ] Implement `LocalVLAPlugin` (transformers.js)
  - [ ] initialize() - Load VLA model
  - [ ] predict(observation, instruction) - Generate action
  - [ ] getInputSpec() / getOutputSpec()
- [ ] Build VLA Executor (episode loop)
  - [ ] Coordinate Physics ↔ VLA interaction
  - [ ] Integrate Adapter for data transformations
  - [ ] Manage episode state (running, paused, reset)
- [ ] Implement Adapter layer
  - [ ] Image resize and normalization
  - [ ] Action scaling and clipping
- [ ] Add at least 1 sample VLA model (transformers.js format)
- [ ] Test full execution loop

**Acceptance Criteria:**
- ✅ LocalVLAPlugin loads VLA model successfully
- ✅ VLA Executor coordinates Physics and VLA
- ✅ Adapter transforms data correctly
- ✅ Full loop runs at >10 Hz
- ✅ Actions control robot in simulation

**Files to Create:**
- `frontend/lib/plugins/local-vla.ts` (new)
- `frontend/lib/executor/vla-executor.ts` (new)
- `frontend/lib/executor/adapter.ts` (new)
- `frontend/lib/executor/episode-state.ts` (new)
- `public/models/sample-vla/` (VLA model files)
- `frontend/__tests__/executor/*.test.ts`

---

### Phase 1.5: UI Components & Selection Flow (Week 3-4)

**Goal:** Build user-facing UI with Robot-First selection flow

**Tasks:**
- [ ] Create component selectors
  - [ ] Robot Selector (no filtering)
  - [ ] Scene Selector (filtered by robot tags)
  - [ ] VLA Model Selector (filtered by robot tags)
  - [ ] Physics Engine Selector (Local/Remote toggle)
- [ ] Build instruction input UI
- [ ] Implement playback controls
  - [ ] Play/Pause/Reset buttons
  - [ ] Speed adjustment (0.25x, 0.5x, 1x, 2x)
  - [ ] Step-by-step execution
- [ ] Add action visualization
  - [ ] Current action vector display
  - [ ] Action history graph
- [ ] Create main page layout with selection flow
- [ ] Implement React Context for global state

**Acceptance Criteria:**
- ✅ Selection flow works correctly (Robot → Scene → VLA → Physics)
- ✅ Only compatible options shown in filtered selectors
- ✅ Playback controls functional
- ✅ Actions visualized in real-time
- ✅ UI is responsive and intuitive

**Files to Create:**
- `frontend/components/robot-selector.tsx`
- `frontend/components/scene-selector.tsx`
- `frontend/components/vla-selector.tsx`
- `frontend/components/physics-selector.tsx`
- `frontend/components/instruction-input.tsx`
- `frontend/components/playback-controls.tsx`
- `frontend/components/action-display.tsx`
- `frontend/contexts/vla-visualization-context.tsx`
- `frontend/app/visualization/page.tsx` (main page)

---

### Phase 1.6: Testing & Polish (Week 4)

**Goal:** Test, optimize, and polish the VLA visualization system

**Tasks:**
- [ ] Integration testing
  - [ ] Test all Robot × Scene × VLA combinations
  - [ ] Verify tag filtering prevents invalid combos
  - [ ] Test episode playback and reset
- [ ] Performance profiling and optimization
  - [ ] WASM execution speed
  - [ ] Rendering FPS (target: 30+)
  - [ ] Memory usage
  - [ ] Bundle size optimization
- [ ] UI/UX improvements
  - [ ] Loading states for all async operations
  - [ ] Error messages with actionable suggestions
  - [ ] Responsive design (desktop-first for Phase 1)
  - [ ] Accessibility (keyboard navigation)
- [ ] Documentation
  - [ ] User guide with screenshots
  - [ ] Developer documentation for adding new configs
  - [ ] Example instructions for each scene
- [ ] Create demo video

**Acceptance Criteria:**
- ✅ All valid combinations work correctly
- ✅ Invalid combinations prevented by UI
- ✅ Performance targets met (30+ FPS, <5s load)
- ✅ User documentation complete
- ✅ Demo video recorded
- ✅ No critical bugs

**Files to Create/Modify:**
- `frontend/__tests__/integration/*.test.ts`
- `docs/USER_GUIDE.md` (new)
- `docs/DEVELOPER_GUIDE.md` (new)
- `frontend/README.md` (update)
- `WORKSPACE/FEATURES/001_VLA_VISUALIZATION.md` (update)
- Performance optimization across all files

---

## Technical Specifications

### Frontend Stack
- **Framework**: Next.js 15 with App Router
- **UI Library**: React 19 with Server Components
- **3D Rendering**: Three.js (WebGL)
- **Physics**: MuJoCo WebAssembly
- **ML Inference**: transformers.js (NOT ONNX Runtime Web)
- **Styling**: Tailwind CSS + shadcn/ui components

### Plugin Interface Specifications

**PhysicsEnginePlugin:**
```typescript
interface PhysicsEnginePlugin {
  readonly type: 'local' | 'remote';
  readonly id: string;

  // Lifecycle
  initialize(config: SceneConfig): Promise<void>;
  reset(): Promise<void>;
  cleanup(): Promise<void>;

  // Simulation
  step(action: Float32Array): Promise<void>;

  // Observation
  getObservation(): Promise<{
    images: Map<string, ImageData>;      // Camera name → image
    proprioception: Float32Array;        // Joint positions, velocities
    timestamp: number;
  }>;

  // Specs
  getActionSpec(): { dim: number; min: number[]; max: number[] };
  getObservationSpec(): {
    imageSizes: Map<string, [number, number]>;
    proprioceptionDim: number;
  };
}
```

**VLAPolicyPlugin:**
```typescript
interface VLAPolicyPlugin {
  readonly type: 'local' | 'remote';
  readonly id: string;

  // Lifecycle
  initialize(): Promise<void>;
  cleanup(): Promise<void>;

  // Inference
  predict(input: {
    image: ImageData;
    proprioception: Float32Array;
    instruction: string;
  }): Promise<{
    action: Float32Array;
    metadata?: {
      confidence?: number;
      attention?: Float32Array;
      reasoning?: string;
    };
  }>;

  // Specs
  getInputSpec(): {
    imageSize: [number, number];
    proprioceptionDim: number;
  };
  getOutputSpec(): { actionDim: number };
}
```

### VLA Model Requirements (transformers.js)

**Format:**
- HuggingFace format (model files + config.json)
- Compatible with transformers.js
- Vision-language-action architecture

**Size & Performance:**
- **Size**: < 100 MB (for browser loading in Phase 1)
- **Inference Speed**: > 10 Hz (< 100ms per action)
- **Input**: RGB image (224x224 or 256x256), robot state, text instruction
- **Output**: Action vector (7-21 dims, continuous)

**Phase 1 Sample Models:**
- Simple pick-and-place policy (demonstration)
- Pre-trained on manipulation tasks
- Quantized for browser performance

**Future (Phase 2):**
- Larger models via Remote VLA API
- Full RT-1, RT-2, OpenVLA support

### Scene Requirements (MuJoCo XML)

**Format:**
- MuJoCo XML (MJCF) format
- Compatible with MuJoCo 3.0+

**Size & Complexity:**
- **File Size**: < 50 MB per scene
- **Mesh Vertices**: < 10,000 per model
- **Bodies**: < 100 (for good performance)
- **Cameras**: 1-3 observation cameras

**Configuration:**
- Define `<camera>` tags for observation views
- Include robot model (URDF or built-in)
- Specify environment props and obstacles

**Phase 1 Scenes:**
- Franka Kitchen (manipulation)
- UR5 Table (pick-and-place)
- Humanoid Flat (locomotion)

### Browser Compatibility

**Minimum Requirements:**
- **Chrome**: 90+
- **Firefox**: 88+
- **Safari**: 14+
- **Edge**: 90+
- **WebGL 2.0**: Required (for Three.js rendering)
- **WebAssembly**: Required (for MuJoCo WASM)
- **SIMD Support**: Recommended (for transformers.js performance)

**Recommended:**
- Modern GPU (for smooth 3D rendering)
- 8 GB+ RAM (for VLA model loading)
- Desktop browser (mobile not optimized in Phase 1)

---

## Testing Strategy

### Unit Tests
- Component rendering tests (React Testing Library)
- VLA policy loading and execution tests
- Environment configuration parsing tests

### Integration Tests
- End-to-end VLA execution flow
- Environment switching
- Playback controls

### Performance Tests
- WASM execution benchmark
- Rendering FPS measurement
- Memory usage profiling

### User Testing
- Test with sample VLA policies
- Gather feedback on UI/UX
- Identify usability issues

---

## Success Metrics

**Phase 1 MVP Success:**
1. ✅ VLA policy can execute in 3+ environments
2. ✅ Real-time visualization at 30+ FPS
3. ✅ Instruction-based control works
4. ✅ User can observe policy behavior clearly
5. ✅ No critical browser compatibility issues

**User Experience Goals:**
- Load time < 5 seconds
- Interaction latency < 100ms
- Intuitive UI (first-time users can use without docs)

---

## Future Enhancements (Post-Phase 1)

### Phase 1.5 Ideas
- **Multi-camera views**: Split-screen observations
- **Episode recording**: Save and replay episodes
- **Policy comparison**: Side-by-side execution (preview of Phase 2)
- **Advanced visualization**: Attention maps, trajectory predictions
- **Mobile support**: Touch controls for mobile browsers

### Phase 2 Integration
- Server-side VLA execution (for larger models)
- Battle mode preparation (compare two policies)
- Episode storage in database

---

## References

### MuJoCo Resources
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo WASM](https://github.com/google-deepmind/mujoco/tree/main/javascript)
- [muwanx Viewer](../../../muwanx) - Reference implementation

### VLA Research
- [RT-1: Robotics Transformer](https://arxiv.org/abs/2212.06817)
- [RT-2: Vision-Language-Action Models](https://arxiv.org/abs/2307.15818)
- [OpenVLA](https://openvla.github.io/)

### Similar Projects
- [RoboTurk](https://roboturk.stanford.edu/)
- [RoboSuite](https://robosuite.ai/)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)

---

## Notes

### Open Questions
1. **VLA Model Access**: Where to source pre-trained VLA models?
   - Option A: Use public models (OpenVLA, RT-1 reproductions)
   - Option B: Create simple demo policies
   - **Decision TBD**

2. **Instruction Encoding**: How to encode instructions?
   - Option A: Use VLA model's built-in encoder
   - Option B: Use separate language model (CLIP, BERT)
   - **Decision**: Use VLA model's encoder (Phase 1)

3. **Observation Format**: What observation format?
   - RGB images (256x256 or 512x512)
   - Depth images (optional)
   - Robot proprioception (joint positions, velocities)
   - **Decision**: RGB + proprioception (Phase 1)

### Risk Mitigation
- **Performance Risk**: Use lightweight models, optimize WASM
- **Model Availability Risk**: Create simple demo policies if needed
- **Browser Compatibility Risk**: Test early on multiple browsers

---

## Related Documentation

For detailed architectural decisions and design rationale, see:

**[ADR-001: Plugin-based Architecture](../ARCHITECTURE/ADR_001-Plugin_Architecture.md)**

Key topics covered in ADR-001:
- Component Separation (Robot, Scene, VLA, Physics)
- Robot-First Selection Flow
- Tag-based Compatibility System
- Plugin Interface Pattern
- Adapter Layer Design
- Local vs Remote execution strategies

All architecture decisions in this feature document are based on ADR-001.

---

**Created:** 2025-11-01
**Last Updated:** 2025-11-01
**Status:** Updated with Plugin Architecture, ready for implementation
