import { type JSX, type ReactNode } from "react";

import { useViewer } from "@/lib/viewer-context";

interface InspectorProps {
  children: ReactNode;
}

export function Inspector({ children }: InspectorProps): JSX.Element {
  const { inspectorOpen } = useViewer();

  if (!inspectorOpen) {
    return <aside className="h-full min-w-0 bg-[var(--surface-0)]" />;
  }

  return (
    <aside className="flex h-full min-w-0 flex-col overflow-hidden bg-[var(--surface-0)]">
      {children}
    </aside>
  );
}
