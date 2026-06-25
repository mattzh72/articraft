import { Suspense, type JSX } from "react";

import { lazyWithChunkReload } from "@/lib/lazy";
import { ViewerProvider } from "@/lib/viewer-context";
import { TooltipProvider } from "@/components/ui/tooltip";
import { AppHeader } from "@/components/layout/AppHeader";

const ViewerShell = lazyWithChunkReload(() => import("@/ViewerShell"));

function AppLoadingFallback(): JSX.Element {
  return (
    <div className="flex min-h-0 flex-1 items-center justify-center">
      <p className="text-[12px] text-[var(--text-quaternary)]">Loading...</p>
    </div>
  );
}

export default function App(): JSX.Element {
  return (
    <TooltipProvider>
      <ViewerProvider>
        <div className="flex h-screen flex-col bg-[var(--surface-2)]">
          <AppHeader />
          <Suspense fallback={<AppLoadingFallback />}>
            <ViewerShell />
          </Suspense>
        </div>
      </ViewerProvider>
    </TooltipProvider>
  );
}
