import Link from "next/link";

export default function Home() {
  return (
    <div className="min-h-screen flex flex-col items-center justify-center p-8">
      <main className="flex flex-col gap-8 items-center">
        <h1 className="text-4xl font-bold text-center">
          vlaarena - AI Battle Arena
        </h1>
        <p className="text-xl text-center max-w-2xl text-muted-foreground">
          Compare and evaluate LLM responses through blind side-by-side testing.
          Vote on the better response and see model identities revealed.
        </p>
        <div className="flex gap-4">
          <Link
            href="/battle"
            className="px-6 py-3 bg-primary text-primary-foreground rounded-lg hover:bg-primary/90 transition"
          >
            Start Battle
          </Link>
          <Link
            href="/leaderboard"
            className="px-6 py-3 bg-secondary text-secondary-foreground rounded-lg hover:bg-secondary/80 transition"
          >
            View Leaderboard
          </Link>
        </div>
      </main>
    </div>
  );
}
