import Link from "next/link";

export default function Home() {
  return (
    <div className="min-h-screen flex flex-col items-center justify-center p-8">
      <main className="flex flex-col gap-8 items-center">
        <h1 className="text-4xl font-bold text-center">
          VLA Arena
        </h1>
        <p className="text-xl text-center max-w-2xl text-muted-foreground">
          Compare Vision-Language-Action models through blind A/B testing.
          Watch robots execute tasks, vote on performance, and see model identities revealed.
        </p>
        <div className="flex gap-4">
          <Link
            href="/battle"
            className="px-8 py-4 bg-primary text-primary-foreground rounded-lg hover:bg-primary/90 transition font-semibold text-lg"
          >
            Start Battle →
          </Link>
          <Link
            href="/leaderboard"
            className="px-8 py-4 border-2 border-primary text-primary rounded-lg hover:bg-primary/10 transition font-semibold text-lg"
          >
            View Leaderboard
          </Link>
        </div>
      </main>
    </div>
  );
}
