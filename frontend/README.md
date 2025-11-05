# VLA Arena Frontend

Next.js 15 application for VLA model comparison platform.

## Tech Stack

- **Framework:** Next.js 15 with App Router
- **UI Library:** React 18, shadcn/ui
- **Styling:** Tailwind CSS
- **3D Rendering:** Three.js, MuJoCo WASM
- **API Client:** Custom apiClient (centralized)

## Quick Start

```bash
# Install dependencies
npm install

# Start development server
npm run dev  # http://localhost:3000

# Build for production
npm run build

# Start production server
npm start
```

## Code Quality

```bash
# Linting
npm run lint

# If UI changed: Chrome DevTools MCP verification (MANDATORY)
# See: .claude/skills/frontend-ui-testing/SKILL.md
```

## Project Structure

```
frontend/
├── app/                    # Next.js App Router
│   ├── battle/             # Battle page (VLA comparison)
│   │   ├── page.tsx        # Server Component
│   │   ├── battle-client.tsx  # Client Component
│   │   ├── service.ts      # API calls
│   │   └── _types.ts       # Type definitions
│   ├── leaderboard/        # Leaderboard page
│   │   ├── page.tsx        # Server Component
│   │   ├── leaderboard-client.tsx  # Client Component
│   │   ├── service.ts      # API calls
│   │   ├── use-leaderboard.ts  # Custom hook
│   │   └── _types.ts       # Type definitions
│   ├── api/                # API routes
│   ├── layout.tsx          # Root layout
│   └── page.tsx            # Home page
├── components/             # Shared components
│   └── ui/                 # shadcn/ui components (DO NOT EDIT)
├── lib/                    # Utilities
│   ├── api-client.ts       # API client
│   └── mujoco/             # MuJoCo WASM integration
└── public/                 # Static assets
```

## Conventions

### File Naming

- `page.tsx` - Server Component (async, data fetching)
- `*-client.tsx` - Client Component (interactive, hooks)
- `service.ts` - API calls (using apiClient)
- `_types.ts` - Type definitions (private, not routes)
- `use-*.ts` - Custom React hooks

### Architecture Pattern (RSC)

**Server Components (page.tsx):**
```typescript
// app/battle/page.tsx
export default async function BattlePage() {
  // Server-side data fetching
  const initialData = await fetchInitialData();

  return <BattleClient initialData={initialData} />;
}
```

**Client Components (*-client.tsx):**
```typescript
"use client";

// app/battle/battle-client.tsx
export function BattleClient({ initialData }) {
  const [state, setState] = useState(initialData);
  // Client-side interactivity

  return <div>...</div>;
}
```

### Styling

- Use **Tailwind CSS** for styling
- Use **shadcn/ui** components (NOT raw HTML)
- Install new components: `npx shadcn@latest add <component>`
- **NEVER** edit `components/ui/` files directly

### API Calls

Always use the centralized `apiClient`:

```typescript
// app/battle/service.ts
import { apiClient } from "@/lib/api-client";

export async function createSession(data: CreateSessionRequest) {
  return apiClient.post<SessionResponse>("/sessions/init", data);
}
```

## UI Verification (MANDATORY)

**All UI changes MUST be verified with Chrome DevTools MCP before committing.**

```bash
# 1. Start dev server
npm run dev

# 2. Use Chrome DevTools MCP tools:
mcp__chrome-devtools__navigate_page({ url: "http://localhost:3000/battle" })
mcp__chrome-devtools__take_snapshot()
mcp__chrome-devtools__list_console_messages()
```

**See:** `.claude/skills/frontend-ui-testing/SKILL.md` for complete verification guide.

## Common Tasks

### Adding a New Page

```bash
# 1. Create directory structure
mkdir -p app/my-feature

# 2. Create files
touch app/my-feature/page.tsx          # Server Component
touch app/my-feature/my-feature-client.tsx  # Client Component
touch app/my-feature/service.ts        # API calls
touch app/my-feature/_types.ts         # Types

# 3. Implement using pattern from battle/ or leaderboard/

# 4. Verify with Chrome DevTools MCP (MANDATORY)
```

### Adding shadcn/ui Component

```bash
# Install component
npx shadcn@latest add button

# Use in code
import { Button } from "@/components/ui/button";
```

## Environment Variables

Create `.env.local`:

```bash
NEXT_PUBLIC_API_URL=http://localhost:8000
```

## Related Documentation

- [Next.js RSC Patterns](./.claude/skills/nextjs-rsc-patterns/SKILL.md)
- [Using shadcn Components](./.claude/skills/using-shadcn-components/SKILL.md)
- [Frontend UI Testing](./.claude/skills/frontend-ui-testing/SKILL.md)

---

**Last Updated:** 2025-11-05
