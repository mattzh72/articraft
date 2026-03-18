import { type JSX } from "react";
import { Search } from "lucide-react";

import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { Input } from "@/components/ui/input";

export function RecordSearch(): JSX.Element {
  const { searchQuery } = useViewer();
  const dispatch = useViewerDispatch();

  return (
    <div className="relative">
      <Search className="absolute left-2.5 top-1/2 size-3.5 -translate-y-1/2 text-[var(--text-quaternary)]" />
      <Input
        type="text"
        placeholder="Search records…"
        value={searchQuery}
        onChange={(e) => dispatch({ type: "SET_SEARCH", payload: e.target.value })}
        className="h-8 pl-8 text-[12px]"
      />
    </div>
  );
}
