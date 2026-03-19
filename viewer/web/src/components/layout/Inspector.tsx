import { type JSX, type ReactNode } from "react";

import { useViewer } from "@/lib/viewer-context";
import { cn } from "@/lib/utils";

interface InspectorProps {
  children: ReactNode;
  disabledOverlay?: ReactNode;
}

export function Inspector({ children, disabledOverlay }: InspectorProps): JSX.Element {
  const { inspectorOpen } = useViewer();

  if (!inspectorOpen) {
    return <aside className="h-full min-w-0 bg-[var(--surface-0)]" />;
  }

  return (
    <aside className="relative flex h-full min-w-0 flex-col overflow-hidden bg-[var(--surface-0)]">
      <div
        className={cn(
          "flex h-full min-w-0 flex-col overflow-hidden transition duration-200",
          disabledOverlay ? "pointer-events-none select-none blur-[7px] saturate-50 opacity-35" : "",
        )}
      >
        {children}
      </div>
      {disabledOverlay ? <div className="absolute inset-0 z-10">{disabledOverlay}</div> : null}
    </aside>
  );
}
