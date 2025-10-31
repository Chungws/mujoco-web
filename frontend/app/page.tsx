import { MujocoViewer } from "@/components/mujoco-viewer";

export default function Home() {
  return (
    <div className="flex flex-col h-screen w-full">
      <header className="bg-slate-900 text-white px-6 py-4 shadow-lg">
        <div className="max-w-7xl mx-auto">
          <h1 className="text-2xl font-bold">MuJoCo Web Viewer</h1>
          <p className="text-sm text-slate-300">
            MuJoCo 3.3.7 running in your browser with WebAssembly
          </p>
        </div>
      </header>

      <main className="flex-1 relative">
        <MujocoViewer className="w-full h-full" />
      </main>

      <footer className="bg-slate-100 px-6 py-3 text-center text-sm text-slate-600">
        <p>
          Powered by{" "}
          <a
            href="https://mujoco.org"
            className="text-blue-600 hover:underline"
            target="_blank"
            rel="noopener noreferrer"
          >
            MuJoCo
          </a>
          {" "}and{" "}
          <a
            href="https://nextjs.org"
            className="text-blue-600 hover:underline"
            target="_blank"
            rel="noopener noreferrer"
          >
            Next.js
          </a>
        </p>
      </footer>
    </div>
  );
}
