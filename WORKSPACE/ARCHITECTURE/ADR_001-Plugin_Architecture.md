# ADR-001: Plugin-based Architecture with Robot-First Selection

**Status:** Accepted
**Date:** 2025-11-01
**Decision Makers:** Development Team
**Related:** Phase 1 (VLA Visualization), Phase 2 (Model Comparison)

---

## Context

### Problem Statement

The mujoco-web project aims to visualize and compare VLA (Vision-Language-Action) models across different robotic environments. We face several key challenges:

1. **Model Diversity**: Different VLA models have varying input/output specifications
   - RT-1: 256x256 RGB, 7-dim action
   - OpenVLA: 224x224 RGB, variable action dim
   - Custom models: Completely different specs

2. **Robot Diversity**: Different robots have different action spaces
   - Franka Panda: 7-DoF arm + gripper (8-dim)
   - UR5: 6-DoF arm + gripper (7-dim)
   - Humanoid: 21-DoF full body

3. **Scene Diversity**: Various scenes and tasks
   - Kitchen manipulation
   - Table pick-and-place
   - Locomotion scenes

4. **Execution Modes**: Need to support multiple execution environments
   - Local MuJoCo (WASM in browser)
   - Remote MuJoCo (API server)
   - Local VLA (transformers.js)
   - Remote VLA (API server - future)

5. **Phase 2 Requirements**: Blind A/B testing for model comparison
   - Users select robot and scene
   - System randomly selects two VLA models
   - Users vote on performance
   - ELO-based leaderboard

### Key Questions

- How to handle incompatible model-robot combinations?
- How to make the system extensible (easy to add new models/robots)?
- How to reuse architecture between Phase 1 and Phase 2?
- How to manage compatibility without complex runtime adaptation?

---

## Decision

We adopt a **Plugin-based Architecture with Robot-First Selection** and **Tag-based Compatibility Filtering**.

### Core Design Principles

#### 1. Component Separation

Separate Robot, Scene, VLA Model, and Physics Engine into **independent, composable components**:

```
Robot (독립)
  - ID, name, action_dim
  - Compatibility tags
  - Action spec

Scene (독립)
  - ID, name, scene file path/ID
  - Compatible robot tags
  - Camera configs

VLA Model (독립)
  - ID, name, model file path
  - Compatible robot tags
  - Input/output specs

Physics Engine (독립)
  - Type: local (WASM) / remote (API)
  - Plugin implementation
```

**Benefits:**
- No duplication (define once, use in many combinations)
- Easy to extend (add robot without touching scenes)
- Clear separation of concerns

#### 2. Robot-First Selection Flow

Users select components in this order:

```
1. Select Robot (no filtering)
   ↓
2. Select Scene (filtered by robot compatibility)
   ↓
3. Select VLA Model (filtered by robot compatibility)
   ↓
4. Select Physics Engine (local/remote)
```

**Rationale:**
- Robot is the compatibility anchor point
- Fewer robots (3-5) than models (potentially dozens)
- Natural user mental model: "What can this robot do?"
- Enables Phase 2: same robot+scene, different random VLA models

#### 3. Tag-based Compatibility

Use string tags for compatibility matching instead of runtime adaptation:

**Robot tags:**
```
Franka Panda: [7dof-arm, gripper, manipulation]
UR5: [6dof-arm, gripper, manipulation]
Humanoid: [humanoid, locomotion, 21dof]
```

**Scene compatibility:**
```
Kitchen: compatible_robots = [7dof-arm, 6dof-arm]
Table: compatible_robots = [7dof-arm, 6dof-arm]
Flat-ground: compatible_robots = [humanoid, quadruped]
```

**VLA model compatibility:**
```
RT-1: compatible_with = [7dof-arm, 6dof-arm]
OpenVLA-Franka: compatible_with = [7dof-arm]
HumanoidVLA: compatible_with = [humanoid]
```

**Benefits:**
- Simple matching (string intersection)
- Explicit compatibility (no guessing)
- Prevents invalid combinations at selection time
- No complex runtime adaptation needed

**Standard Tag Vocabulary:**

To ensure consistency and prevent typos, we define standard tags:

**Robot Type Tags:**
- `7dof-arm` - 7 degrees of freedom arm
- `6dof-arm` - 6 degrees of freedom arm
- `21dof` - 21 degrees of freedom (full body)
- `humanoid` - Humanoid robot
- `quadruped` - Four-legged robot

**Robot Capability Tags:**
- `gripper` - Has gripper end-effector
- `manipulation` - Capable of manipulation tasks
- `locomotion` - Capable of locomotion tasks
- `bimanual` - Two-arm system

**Scene Type Tags:**
- `manipulation` - Manipulation task environment
- `locomotion` - Locomotion task environment
- `indoor` - Indoor environment
- `outdoor` - Outdoor environment
- `simple` - Simple task/environment
- `complex` - Complex task/environment

**Usage:**
- Robot tags describe capabilities: `[7dof-arm, gripper, manipulation]`
- Scene `compatible_robots` uses robot type tags: `[7dof-arm, 6dof-arm]`
- VLA `compatible_with` uses robot type tags: `[7dof-arm, 6dof-arm]`

**Tag Validation:**
Define tag constants in code to prevent typos:
```typescript
export const ROBOT_TAGS = {
  DOF_7: '7dof-arm',
  DOF_6: '6dof-arm',
  GRIPPER: 'gripper',
  MANIPULATION: 'manipulation',
  // ...
} as const;
```

#### 4. Plugin Interface Pattern

Define interfaces (not abstract classes) for each component type:

```
PhysicsEnginePlugin:
  - initialize(config)
  - step(action) → state
  - getObservation() → {images, proprioception}
  - getActionSpec() → spec
  - getObservationSpec() → spec

VLAPolicyPlugin:
  - initialize(config)
  - predict(observation, instruction) → action
  - getInputSpec() → spec
  - getOutputSpec() → spec
```

**Implementation strategies:**
- LocalMuJoCoPlugin (WASM)
- RemoteMuJoCoPlugin (API)
- LocalVLAPlugin (transformers.js)
- RemoteVLAPlugin (API - future)

**Benefits:**
- Interface-based (no inheritance complexity)
- Easy to mock for testing
- Clear contracts between components
- Helper functions for common operations

#### 5. Adapter Layer

A lightweight **Adapter** handles data transformations between plugins:

**Responsibilities:**
- Basic image transformations (resize, normalize)
- Data format conversion (ImageData, Float32Array, etc.)
- Action scaling and safety clipping

**Design principle:**
Tag-based filtering ensures structural compatibility (dimensions, action spaces). The Adapter only handles format and scale transformations, not structural changes.

---

## Consequences

### Positive

✅ **Extensibility**
- Add new robot: define once, works with all compatible scenes/models
- Add new VLA model: define compatibility tags, automatically available
- Add new scene: define compatible robots, filtered automatically

✅ **Simplicity**
- No complex runtime adaptation logic
- Compatibility errors caught at selection time (not runtime)
- Clear error messages: "UR5 not compatible with Humanoid scenes"

✅ **Phase 1 → Phase 2 Reuse**
- Same Robot + Scene configs
- Same compatibility matching
- Only difference: VLA selection (explicit vs random)
- Architecture supports blind A/B testing naturally

✅ **Developer Experience**
- Clear separation of concerns
- Easy to understand component responsibilities
- Simple to add new implementations

✅ **User Experience**
- Only see valid combinations (no confusion)
- Clear selection flow
- Phase 2: select robot+scene, system picks models randomly

### Negative

⚠️ **Limited Flexibility**
- Cannot mix incompatible components (by design)
- Requires explicit compatibility definitions
- "Almost compatible" combinations not allowed (strict mode)

⚠️ **Tag Management**
- Need to maintain consistent tag vocabulary
- Risk of typos in tags (e.g., "7dof-arm" vs "7dof_arm")
- Documentation needed for tag meanings

⚠️ **Selection Steps**
- 3-4 selection steps (robot, scene, physics, VLA)
- More steps than monolithic "environment" approach
- Mitigated by: smart defaults, filtering, saved presets

### Mitigation Strategies

**For tag management:**
- Define tag constants in code (prevent typos)
- Document tag meanings in CONVENTIONS
- Validation on config load

**For selection complexity:**
- Remember last selections
- Provide "Quick Start" presets
- Default physics to "Local" (most common)

---

## Alternatives Considered

### Alternative 1: Runtime Adaptation with Complex Adapter

**Approach:**
- Allow any robot-model combination
- Runtime adapter converts between specs (resize images, pad dimensions, etc.)
- Flexible but complex

**Rejected because:**
- High complexity (adapter logic for every combination)
- Runtime errors hard to debug
- Uncertain behavior (auto-padding may not work correctly)
- Over-engineering for Phase 1

**Note:** May revisit in Phase 2+ for "soft compatibility" mode

### Alternative 2: Monolithic Environment Configs

**Approach:**
- Pre-define all combinations as "Environments"
- User selects one environment (e.g., "Franka Kitchen Local")
- Simple selection but rigid

**Rejected because:**
- Combinatorial explosion (Robot × Scene × Physics = many configs)
- Duplication of robot/scene definitions
- Hard to extend (add 1 scene = add N robot combinations)
- Doesn't support Phase 2 random selection well

### Alternative 3: VLA-First Selection

**Approach:**
- User selects VLA model first
- System filters compatible robots

**Rejected because:**
- Less intuitive ("which robots work with this model?")
- VLA models may be numerous (harder to browse)
- Robot is more stable anchor point
- Doesn't match typical use case

---

## Implementation Notes

### Phase 1 Scope

**Implement:**
- Robot, Scene, VLA configs with tags
- LocalMuJoCoPlugin (WASM)
- LocalVLAPlugin (transformers.js)
- Tag-based filtering UI
- Simple adapter (image resize, normalization only)
- Physics selection UI (local default, remote option for Phase 2)

**Defer to Phase 2:**
- RemoteMuJoCoPlugin (structure only)
- RemoteVLAPlugin
- Complex adaptation logic
- Soft compatibility mode

### Local vs Remote Physics Comparison

| Aspect | Local (WASM) | Remote (API) |
|--------|-------------|--------------|
| **Execution** | Browser (client-side) | Server |
| **Scene Files** | `public/mujoco/scenes/*.xml` | Server-side only |
| **Latency** | <1ms | 10-50ms (network) |
| **GPU Support** | No | Yes |
| **Offline Mode** | Yes | No |
| **Max Scene Size** | ~50MB (browser limit) | Unlimited |
| **Concurrent Users** | Unlimited (client-side) | Limited (server capacity) |
| **Phase** | Phase 1 (implemented) | Phase 2 (planned) |

**Phase 1:** Local WASM only (fast iteration, no infrastructure)
**Phase 2:** Add Remote option (larger models, GPU acceleration)

### Configuration Structure

**File organization:**
```
config/
  robots.ts          # Robot definitions
  scenes.ts          # Scene definitions
  vla-models.ts      # VLA model definitions
  physics.ts         # Physics engine configs
```

**Registry pattern:**
```
PluginRegistry:
  - registerRobot(config)
  - registerScene(config)
  - registerVLA(config)
  - filterScenes(robotTags)
  - filterVLAs(robotTags)
```

### Testing Strategy

**Unit tests:**
- Tag matching logic
- Plugin interface compliance
- Config validation

**Integration tests:**
- Robot + Scene + VLA combinations
- Plugin initialization and execution
- Compatibility filtering

---

## Practical Examples

### Example 1: Adding a New Robot

```typescript
// config/robots.ts
{
  id: 'ur10',
  name: 'UR10',
  action_dim: 6,
  tags: ['6dof-arm', 'gripper', 'manipulation']
}
```

**Result:** Automatically available in all scenes with `6dof-arm` compatibility.
No code changes needed.

### Example 2: Adding a New Scene

```typescript
// config/scenes.ts
{
  id: 'warehouse',
  name: 'Warehouse Pick-Place',
  path: '/mujoco/scenes/warehouse.xml',
  compatible_robots: ['7dof-arm', '6dof-arm'],
  tags: ['manipulation', 'complex']
}
```

**Result:** Selectable for Franka and UR5 robots. VLA models filtered by robot selection.

---

## Related Decisions

- **ADR-002** (future): Remote Physics Engine Architecture
- **ADR-003** (future): VLA Model Format Standards
- **ADR-004** (future): Soft Compatibility Mode

---

## References

- [FEATURES/001_VLA_VISUALIZATION.md](../FEATURES/001_VLA_VISUALIZATION.md)
- [FEATURES/002_MODEL_COMPARISON.md](../FEATURES/002_MODEL_COMPARISON.md)
- [lmarena-clone ADR-001](../../lmarena-clone/WORKSPACE/ARCHITECTURE/ADR_001-No_Foreign_Keys.md) (reference project)

---

**Last Updated:** 2025-11-01
**Status:** Accepted and ready for implementation
