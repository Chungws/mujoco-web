import type { Metadata } from "next";
import { ThemeProvider } from "@/components/theme-provider";
import { SessionProvider } from "@/lib/contexts/session-context";
import { Sidebar } from "@/components/sidebar";
import { MobileSidebar } from "@/components/mobile-sidebar";
import "./globals.css";

export const metadata: Metadata = {
  title: "vlaarena - AI Battle Arena",
  description: "Compare and evaluate LLM responses through blind side-by-side testing",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className="antialiased">
        <ThemeProvider
          attribute="class"
          defaultTheme="system"
          enableSystem
          disableTransitionOnChange
        >
          <SessionProvider>
            <div className="flex h-screen overflow-hidden">
              {/* Desktop Sidebar */}
              <aside className="hidden md:block">
                <Sidebar />
              </aside>

              {/* Main Content */}
              <div className="flex flex-1 flex-col overflow-hidden">
                {/* Mobile Header */}
                <header className="flex items-center gap-4 border-b bg-card p-4 md:hidden">
                  <MobileSidebar />
                  <h1 className="text-lg font-semibold text-card-foreground">LLM Battler</h1>
                </header>

                {/* Page Content */}
                <main className="flex-1 overflow-auto bg-background">
                  {children}
                </main>
              </div>
            </div>
          </SessionProvider>
        </ThemeProvider>
      </body>
    </html>
  );
}
