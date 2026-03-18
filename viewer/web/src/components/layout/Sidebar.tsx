import { type JSX, type ReactNode } from "react";

interface SidebarProps {
  children: ReactNode;
}

export function Sidebar({ children }: SidebarProps): JSX.Element {
  return (
    <aside className="flex h-full min-w-0 flex-col overflow-hidden bg-[var(--surface-0)]">
      {children}
    </aside>
  );
}
