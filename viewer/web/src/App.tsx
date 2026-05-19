import { lazy, Suspense, type JSX } from "react";

import { useRoute } from "@/lib/useRoute";
import { ViewerProvider } from "@/lib/viewer-context";
import { TooltipProvider } from "@/components/ui/tooltip";
import { AppHeader } from "@/components/layout/AppHeader";

const ViewerShell = lazy(() => import("@/ViewerShell"));
const DashboardPage = lazy(() =>
  import("@/components/dashboard/DashboardPage").then((module) => ({
    default: module.DashboardPage,
  })),
);

function AppLoadingFallback(): JSX.Element {
  return (
    <div className="flex min-h-0 flex-1 items-center justify-center">
      <p className="text-[12px] text-[var(--text-quaternary)]">Loading...</p>
    </div>
  );
}

function DashboardApp(): JSX.Element {
  return (
    <div className="flex h-screen flex-col bg-[var(--surface-2)]">
      <AppHeader />
      <div className="min-h-0 flex-1">
        <Suspense fallback={<AppLoadingFallback />}>
          <DashboardPage />
        </Suspense>
      </div>
    </div>
  );
}

function ViewerApp(): JSX.Element {
  return (
    <ViewerProvider>
      <div className="flex h-screen flex-col bg-[var(--surface-2)]">
        <AppHeader />
        <Suspense
          fallback={<AppLoadingFallback />}
        >
          <ViewerShell />
        </Suspense>
      </div>
    </ViewerProvider>
  );
}

export default function App(): JSX.Element {
  const route = useRoute();

  return (
    <TooltipProvider>
      {route.page === "dashboard" ? <DashboardApp /> : <ViewerApp />}
    </TooltipProvider>
  );
}
