# Feature: MuJoCo WASM Viewer (Frontend)

**Status:** Planning - Backend API Complete
**Priority:** HIGH
**Timeline:** 2-3 days
**Dependencies:** Backend XML API (✅ Complete)

---

## 📋 Overview

Frontend integration of MuJoCo WASM for interactive episode replay in the browser. Users will see side-by-side 3D visualization of robot movements, with timeline controls for frame-by-frame playback.

**Key Features:**
- Browser-based 3D visualization (Three.js + MuJoCo WASM)
- State-based replay (qpos/qvel from episode data)
- Timeline controls (play/pause/seek/speed)
- Side-by-side comparison (Model A vs Model B)

---

## 🎯 Goals

### Primary Goals
1. ✅ Render MuJoCo simulation states in browser
2. ✅ Frame-by-frame replay with timeline control
3. ✅ Smooth 30 FPS visualization
4. ✅ Interactive camera controls (orbit, zoom, pan)

### Deferred (Post-MVP)
- ❌ Video recording/export
- ❌ Trajectory overlay visualization
- ❌ Slow-motion/frame-step debugging
- ❌ Multiple camera angles

---

## 🏗️ Architecture

### High-Level Flow

```
Frontend Components:
┌──────────────────────────────────────┐
│  Battle Page (battle-arena-client)   │
│  ┌────────────────────────────────┐  │
│  │  MujocoViewer Component        │  │
│  │  ├─ Canvas (Three.js)          │  │
│  │  ├─ Timeline Controls           │  │
│  │  └─ Episode Info               │  │
│  └────────────────────────────────┘  │
└──────────────────────────────────────┘
            │
            ├─ GET /api/models/xml?robot_id=franka&scene_id=table
            │  → Returns: Complete MuJoCo XML
            │
            └─ GET /api/episodes/{episode_id}
               → Returns: {actions[], states[{qpos, qvel, time}]}
```

### Data Flow

```typescript
// 1. Initialize MuJoCo
const xml = await fetch('/api/models/xml?robot_id=franka&scene_id=table');
const mujoco = await loadMujoco();
const model = mujoco.load_model_from_xml(xml);
const data = new mujoco.State(model);

// 2. Load episode
const episode = await fetch(`/api/episodes/${episodeId}`);
// episode.states = [{qpos: [...], qvel: [...], time: 0.0}, ...]

// 3. Render frame
function renderFrame(frameIndex) {
  const state = episode.states[frameIndex];

  // Apply state
  data.qpos.set(state.qpos);
  data.qvel.set(state.qvel);

  // Forward kinematics
  mujoco.mj_forward(model, data);

  // Update Three.js scene
  updateBodiesFromMujoco(scene, model, data);
  renderer.render(scene, camera);
}
```

---

## 📦 Implementation Plan

### Phase 1: Core Infrastructure (Day 1)

**Files to Create:**
```
frontend/
├── lib/
│   └── mujoco/
│       ├── loader.ts           # MuJoCo WASM initialization
│       ├── xml-fetcher.ts      # Fetch XML from backend API
│       └── state-player.ts     # State replay logic
├── hooks/
│   └── useMujocoViewer.ts     # React hook for MuJoCo viewer
└── components/
    └── battle/
        ├── mujoco-viewer.tsx       # Main viewer (updated)
        └── mujoco-timeline.tsx     # Timeline controls (new)
```

**Key Tasks:**
- [ ] Install dependencies (already done, just need to commit)
  ```bash
  npm install three@^0.150.0 @types/three mujoco-js@^0.0.7
  ```
- [ ] Create `lib/mujoco/loader.ts` - MuJoCo WASM initialization
- [ ] Create `lib/mujoco/xml-fetcher.ts` - API client for XML
- [ ] Create `lib/mujoco/state-player.ts` - Frame management

### Phase 2: MuJoCo Viewer Component (Day 2)

**Update `components/battle/mujoco-viewer.tsx`:**
- [ ] Replace placeholder with actual canvas
- [ ] Initialize Three.js scene/camera/renderer
- [ ] Integrate MuJoCo WASM
- [ ] Implement frame rendering loop
- [ ] Add camera controls (OrbitControls)

**Create `components/battle/mujoco-timeline.tsx`:**
- [ ] Play/Pause button
- [ ] Seek slider (0 to total frames)
- [ ] Speed control (0.5x, 1x, 2x)
- [ ] Frame counter display

### Phase 3: Integration & Testing (Day 3)

**Integration:**
- [ ] Connect `useMujocoViewer` hook to components
- [ ] Handle episode loading state
- [ ] Synchronize left/right viewers
- [ ] Error handling (WASM load failure, XML error)

**Testing:**
- [ ] Manual testing with real episodes
- [ ] Performance testing (30 FPS target)
- [ ] Chrome DevTools MCP verification (MANDATORY)
- [ ] Different episode lengths (10-50 steps)

---

## 🔧 Technical Details

### Dependencies

**Already Installed (in working directory):**
```json
{
  "three": "^0.150.0",
  "@types/three": "^0.150.0",
  "mujoco-js": "^0.0.7"
}
```

### File Structure (Detailed)

#### `lib/mujoco/loader.ts`
```typescript
// Initialize MuJoCo WASM
export async function loadMujoco(): Promise<MuJoCo> {
  const mujoco = await import('mujoco-js');
  return mujoco.default || mujoco;
}
```

#### `lib/mujoco/xml-fetcher.ts`
```typescript
// Fetch composed XML from backend
export async function fetchModelXML(
  robotId: string,
  sceneId: string
): Promise<string> {
  const response = await fetch(
    `/api/models/xml?robot_id=${robotId}&scene_id=${sceneId}`
  );
  if (!response.ok) {
    throw new Error(`Failed to fetch XML: ${response.statusText}`);
  }
  return response.text();
}
```

#### `lib/mujoco/state-player.ts`
```typescript
// Manage frame playback
export class StatePlayer {
  private states: EpisodeState[];
  private currentFrame: number = 0;
  private isPlaying: boolean = false;
  private playbackSpeed: number = 1.0; // 1x speed

  constructor(states: EpisodeState[]) {
    this.states = states;
  }

  play() { ... }
  pause() { ... }
  seek(frame: number) { ... }
  setSpeed(speed: number) { ... }
  getCurrentFrame(): number { ... }
}
```

#### `hooks/useMujocoViewer.ts`
```typescript
export function useMujocoViewer(
  containerRef: RefObject<HTMLCanvasElement>,
  episode: EpisodeResponse | null
) {
  const [currentFrame, setCurrentFrame] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    if (!containerRef.current || !episode) return;

    // 1. Initialize MuJoCo WASM
    // 2. Fetch XML
    // 3. Create Three.js scene
    // 4. Setup state player

    return () => {
      // Cleanup
    };
  }, [containerRef, episode]);

  return {
    currentFrame,
    isPlaying,
    isLoading,
    play: () => { ... },
    pause: () => { ... },
    seek: (frame: number) => { ... },
    setSpeed: (speed: number) => { ... },
  };
}
```

---

## 🎨 UI/UX Design

### MujocoViewer Component Layout

```
┌─────────────────────────────────────┐
│  Model A                        [A] │ ← Badge
├─────────────────────────────────────┤
│                                     │
│       [3D Canvas - MuJoCo]          │ ← Three.js rendering
│                                     │
│                                     │
├─────────────────────────────────────┤
│  [▶] ━━━━━●━━━━━━ [1.0x]           │ ← Timeline controls
│       0:02 / 0:05                   │
├─────────────────────────────────────┤
│  States: 35 | Actions: 35           │ ← Episode info
│  Duration: 3.50s                    │
└─────────────────────────────────────┘
```

### Timeline Controls Features

- **Play/Pause Button**: Toggle playback
- **Seek Slider**: Drag to any frame
- **Speed Control**: Dropdown (0.5x, 1x, 2x)
- **Frame Counter**: Current / Total frames
- **Time Display**: Current time / Total duration

---

## 🧪 Testing Strategy

### Manual Testing Checklist

- [ ] **MuJoCo WASM loads** - No console errors
- [ ] **XML fetched successfully** - Correct robot + scene
- [ ] **3D scene renders** - Robot visible, table visible, objects visible
- [ ] **Frame playback works** - Robot moves smoothly
- [ ] **Timeline controls work**:
  - [ ] Play/Pause toggles correctly
  - [ ] Seek updates frame immediately
  - [ ] Speed control changes playback rate
- [ ] **Camera controls work**:
  - [ ] Orbit (drag)
  - [ ] Zoom (scroll)
  - [ ] Pan (right-click drag)
- [ ] **Performance**:
  - [ ] 30 FPS during playback
  - [ ] No memory leaks
  - [ ] Smooth on both viewers simultaneously

### Chrome DevTools MCP Verification (MANDATORY)

**Before committing UI changes:**
```bash
# Use Chrome DevTools MCP to verify:
# 1. Visual correctness
# 2. No console errors
# 3. Smooth animation
# 4. Responsive layout
```

---

## 📊 Success Criteria

### MVP Complete When:

1. ✅ Users can see 3D robot visualization
2. ✅ Episode states replay smoothly (30 FPS)
3. ✅ Timeline controls work (play/pause/seek)
4. ✅ Camera is interactive (orbit/zoom/pan)
5. ✅ Side-by-side comparison works (2 viewers)
6. ✅ No performance issues (<60s load time)
7. ✅ Chrome DevTools MCP verification passed

---

## 🚀 Next Steps (After This Feature)

1. **Add Trajectory Overlay** - Visualize end-effector path
2. **Multiple Camera Angles** - Top-down, side view, first-person
3. **Frame-by-Frame Debugging** - Step forward/backward by 1 frame
4. **Slow Motion** - 0.1x, 0.25x playback speeds
5. **Video Export** - Record and download episode as MP4

---

## 📚 References

### Backend API (Already Implemented)

**GET /api/models/xml**
- Query: `?robot_id=franka&scene_id=table`
- Returns: Complete MuJoCo XML string (text/plain)
- Status: ✅ Implemented, tested, committed

**GET /api/episodes/{episode_id}**
- Returns: Episode data with states array
- Status: ✅ Already implemented (earlier)

### External Libraries

- **MuJoCo WASM**: [mujoco-js npm package](https://www.npmjs.com/package/mujoco-js)
- **Three.js**: [threejs.org](https://threejs.org/)
- **Reference Implementation**: `/Users/dapi/work/mujoco-web/muwanx/`
  - See: `src/core/mujoco/runtime/MujocoRuntime.ts`
  - See: `src/viewer/MuwanxViewer.vue` (Vue, but logic is portable)

### Project Patterns

- **Next.js RSC**: `nextjs-rsc-patterns` skill
- **Frontend UI Testing**: `frontend-ui-testing` skill (Chrome DevTools MCP)

---

**Created:** 2025-11-11
**Last Updated:** 2025-11-11
**Status:** Planning - Ready to Implement
**Estimated Effort:** 2-3 days (frontend only)
