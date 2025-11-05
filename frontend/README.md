# llmbattler-frontend

Next.js 15 frontend for LLM Battle Arena with App Router, React Server Components, and shadcn/ui.

## Quick Start

### Prerequisites
- Node.js 20+
- npm

### Development Setup

```bash
cd frontend

# Install dependencies
npm install

# Start development server with Turbopack
npm run dev
```

**Visit:** http://localhost:3000

### Environment Variables

Copy `.env.example` to `.env.local` and configure:

```bash
cp .env.example .env.local
```

Key settings:
- `NEXT_PUBLIC_API_URL`: Backend API URL (default: http://localhost:8000)

### Building for Production

```bash
npm run build
npm start
```

### Linting

```bash
npm run lint
```

## Project Structure

```
frontend/
├── app/                      # Next.js App Router (no src/)
│   ├── layout.tsx            # Root layout with ThemeProvider
│   ├── page.tsx              # Home page
│   ├── globals.css           # Global styles with CSS variables
│   ├── battle/               # Battle mode
│   │   └── page.tsx
│   └── leaderboard/          # Leaderboard
│       └── page.tsx
├── components/               # React components
│   └── ui/                   # shadcn/ui components
├── hooks/                    # Custom React hooks
├── lib/                      # Utilities
│   └── utils.ts              # cn() helper
├── services/                 # API clients
├── public/                   # Static assets
├── package.json
├── components.json           # shadcn/ui config
├── tsconfig.json
├── tailwind.config.ts
├── next.config.ts
├── eslint.config.mjs
├── Dockerfile
└── README.md
```

## Tech Stack

- **Next.js 15** - React framework with App Router + Turbopack
- **React 19** - UI library
- **TypeScript 5** - Type safety
- **Tailwind CSS v4** - Utility-first CSS with CSS variables
- **shadcn/ui** - Re-usable components (Radix UI + Tailwind)
- **next-themes** - Dark mode support
- **lucide-react** - Icon library
- **ESLint** - Code linting (flat config)

## Features

### Design System
- shadcn/ui components (New York style)
- CSS variables for theming
- Dark mode with next-themes
- Responsive design
- Accessible components (Radix UI)

### Pages

**Home (`/`)**
- Landing page with navigation
- Links to Battle and Leaderboard
- Responsive layout

**Battle Mode (`/battle`)**
- Placeholder for battle UI
- TODO: Full implementation

**Leaderboard (`/leaderboard`)**
- Placeholder for leaderboard UI
- TODO: Full implementation

## shadcn/ui

Add components using the CLI:

```bash
npx shadcn@latest add button
npx shadcn@latest add card
npx shadcn@latest add textarea
# etc.
```

**Configuration:** `components.json`

## TODO

### Battle UI (Phase 1.4)
- [ ] Add shadcn/ui components (Button, Card, Textarea, ScrollArea)
- [ ] Prompt input field
- [ ] Side-by-side response display
- [ ] Conversation history
- [ ] Follow-up messages support (max 6 total)
- [ ] Message counter
- [ ] Voting buttons (Left/Right/Tie/Both Bad)
- [ ] Model reveal after voting
- [ ] Loading states (Spinner, Skeleton)
- [ ] Error handling (Alert component)

### Leaderboard UI (Phase 2.4)
- [ ] Add shadcn/ui Table component
- [ ] Ranking table (Rank, Model, ELO, CI, Votes, Org, License)
- [ ] Search/filter with Input component
- [ ] Sortable columns
- [ ] Metadata display (Badge, Card)
- [ ] Loading states
- [ ] Error handling

### API Integration
- [ ] Create API clients (`services/`)
  - [ ] Battle API (create, followup, vote)
  - [ ] Leaderboard API (get rankings)
  - [ ] Models API (list models)
- [ ] Add custom hooks (`hooks/`)
  - [ ] `useBattle()` - Battle state management
  - [ ] `useLeaderboard()` - Leaderboard data fetching
  - [ ] `useModels()` - Available models list
- [ ] Error handling and loading states
- [ ] TypeScript types for API responses

### UI Components
- [ ] Battle components
  - [ ] BattleCard
  - [ ] ResponseDisplay
  - [ ] VoteButtons
  - [ ] MessageCounter
- [ ] Leaderboard components
  - [ ] LeaderboardTable
  - [ ] ModelRow
  - [ ] RankBadge
- [ ] Shared components (from shadcn/ui)
  - [ ] Button
  - [ ] Card
  - [ ] Input
  - [ ] Textarea
  - [ ] Table
  - [ ] Badge
  - [ ] Spinner
  - [ ] Alert

### Testing & Polish
- [ ] Chrome DevTools MCP verification
- [ ] Responsive design testing
- [ ] Dark mode testing
- [ ] Accessibility testing
- [ ] Error boundaries

## Related Documentation

- [WORKSPACE/CONVENTIONS/frontend/](../WORKSPACE/CONVENTIONS/frontend/) - Frontend conventions
- [WORKSPACE/FEATURES/001_BATTLE_MVP.md](../WORKSPACE/FEATURES/001_BATTLE_MVP.md) - Battle mode spec
- [WORKSPACE/FEATURES/002_LEADERBOARD_MVP.md](../WORKSPACE/FEATURES/002_LEADERBOARD_MVP.md) - Leaderboard spec
- [shadcn/ui Documentation](https://ui.shadcn.com) - Component library
