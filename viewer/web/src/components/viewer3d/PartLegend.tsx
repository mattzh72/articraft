import { ChevronRight } from "lucide-react";
import { type JSX } from "react";

import { ScrollArea } from "@/components/ui/scroll-area";
import { cn } from "@/lib/utils";

export type PartLegendSelection = {
  partName: string;
  subpartKey: string | null;
};

export type PartLegendSubpartItem = {
  key: string;
  name: string;
  color: string;
};

export type PartLegendItem = {
  name: string;
  color: string;
  subparts: PartLegendSubpartItem[];
};

type PartLegendProps = {
  items: PartLegendItem[];
  selection: PartLegendSelection | null;
  onSelectPart: (selection: PartLegendSelection | null) => void;
};

export function PartLegend({
  items,
  selection,
  onSelectPart,
}: PartLegendProps): JSX.Element | null {
  if (items.length === 0) {
    return null;
  }

  return (
    <div className="pointer-events-auto absolute right-3 top-3 z-20 w-[min(14rem,calc(100%-1.5rem))] overflow-hidden rounded-[18px] bg-[rgba(245,245,245,0.94)] p-1.5 shadow-[0_10px_28px_rgba(17,17,17,0.06)] backdrop-blur-xl">
      <ScrollArea className="max-h-[min(46vh,15rem)]">
        <div className="space-y-0.5">
          {items.map((item) => {
            const isSelectedPart = selection?.partName === item.name;
            const isExpanded = isSelectedPart && item.subparts.length > 0;

            return (
              <div key={item.name} className="space-y-0.5">
                <button
                  type="button"
                  aria-pressed={isSelectedPart && selection?.subpartKey == null}
                  onClick={() => onSelectPart(
                    isSelectedPart && selection?.subpartKey == null
                      ? null
                      : { partName: item.name, subpartKey: null },
                  )}
                  className={cn(
                    "group flex w-full items-center gap-2 rounded-lg px-2 py-1.5 text-left transition-colors duration-150",
                    isSelectedPart
                      ? "bg-white/[0.96] shadow-[inset_0_0_0_1px_rgba(12,140,233,0.14)]"
                      : "bg-transparent hover:bg-white/[0.7]",
                  )}
                >
                  <span
                    aria-hidden="true"
                    className="h-2 w-2 shrink-0 rounded-full"
                    style={{ backgroundColor: item.color }}
                  />
                  <span
                    className="min-w-0 flex-1 truncate font-mono text-[10px] text-[var(--text-primary)]"
                    title={item.name}
                  >
                    {item.name}
                  </span>
                  {item.subparts.length > 0 ? (
                    <span className="inline-flex items-center gap-1 rounded-full bg-[var(--surface-2)] px-1.5 py-0.5 font-mono text-[9px] text-[var(--text-tertiary)]">
                      <ChevronRight
                        aria-hidden="true"
                        className={cn(
                          "h-2.5 w-2.5 transition-transform duration-150",
                          isExpanded ? "rotate-90" : "rotate-0",
                        )}
                      />
                      {item.subparts.length}
                    </span>
                  ) : null}
                  <span
                    aria-hidden="true"
                    className={cn(
                      "h-1 w-1 shrink-0 rounded-full transition-opacity duration-150",
                      isSelectedPart
                        ? "bg-[var(--accent)] opacity-100"
                        : "bg-[var(--border-strong)] opacity-0 group-hover:opacity-80",
                    )}
                  />
                </button>

                {isExpanded ? (
                  <div className="ml-3 space-y-0.5 border-l border-[var(--border-subtle)] pl-2">
                    {item.subparts.map((subpart) => {
                      const isSelectedSubpart =
                        selection?.partName === item.name && selection.subpartKey === subpart.key;

                      return (
                        <button
                          key={subpart.key}
                          type="button"
                          aria-pressed={isSelectedSubpart}
                          onClick={() => onSelectPart(
                            isSelectedSubpart
                              ? { partName: item.name, subpartKey: null }
                              : { partName: item.name, subpartKey: subpart.key },
                          )}
                          className={cn(
                            "group flex w-full items-center gap-2 rounded-lg px-2 py-1.5 text-left transition-colors duration-150",
                            isSelectedSubpart
                              ? "bg-white/[0.94] shadow-[inset_0_0_0_1px_rgba(12,140,233,0.12)]"
                              : "bg-transparent hover:bg-white/[0.65]",
                          )}
                        >
                          <span
                            aria-hidden="true"
                            className="h-1.5 w-1.5 shrink-0 rounded-full opacity-80"
                            style={{ backgroundColor: subpart.color }}
                          />
                          <span
                            className="min-w-0 flex-1 truncate font-mono text-[9.5px] text-[var(--text-secondary)]"
                            title={subpart.name}
                          >
                            {subpart.name}
                          </span>
                          <span
                            aria-hidden="true"
                            className={cn(
                              "h-1 w-1 shrink-0 rounded-full transition-opacity duration-150",
                              isSelectedSubpart
                                ? "bg-[var(--accent)] opacity-100"
                                : "bg-[var(--border-strong)] opacity-0 group-hover:opacity-75",
                            )}
                          />
                        </button>
                      );
                    })}
                  </div>
                ) : null}
              </div>
            );
          })}
        </div>
      </ScrollArea>
    </div>
  );
}
