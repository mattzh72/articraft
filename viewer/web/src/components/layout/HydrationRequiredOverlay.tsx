import { Download, Loader2 } from "lucide-react";
import { type JSX } from "react";

import type { RecordSummary } from "@/lib/types";

type HydrationRequiredOverlayProps = {
  record: RecordSummary;
  error?: string | null;
  hydrating: boolean;
  onHydrate: () => void;
  compact?: boolean;
};

export function HydrationRequiredOverlay({
  record,
  error = null,
  hydrating,
  onHydrate,
  compact = false,
}: HydrationRequiredOverlayProps): JSX.Element {
  const isMissing = record.payload_status === "missing";
  const title = isMissing ? "Record payload is missing" : "Record payload is not hydrated";
  const description = isMissing
    ? "This dataset entry is indexed, but its record files are not currently present on disk."
    : "This dataset entry is backed by Git LFS pointer files. Hydrate it before loading the model.";

  return (
    <div className="flex h-full w-full items-center justify-center p-4">
      <div
        className={[
          "w-full rounded-xl border border-[var(--border-default)] bg-[var(--surface-0)] shadow-[0_16px_40px_rgba(15,23,42,0.14)]",
          compact ? "max-w-sm p-4" : "max-w-xl p-5",
        ].join(" ")}
      >
        <div className="flex items-center gap-2">
          <span className="inline-flex h-2.5 w-2.5 rounded-full bg-[var(--accent)]" />
          <p className="text-[11px] font-semibold uppercase tracking-[0.08em] text-[var(--text-tertiary)]">
            Hydration Required
          </p>
        </div>

        <h3
          className={
            compact
              ? "mt-3 text-[15px] font-semibold text-[var(--text-primary)]"
              : "mt-3 text-[18px] font-semibold text-[var(--text-primary)]"
          }
        >
          {title}
        </h3>
        <p className="mt-2 text-[12px] leading-5 text-[var(--text-secondary)]">{description}</p>
        {error ? <p className="mt-2 text-[11px] leading-5 text-[var(--destructive)]">{error}</p> : null}

        <button
          type="button"
          onClick={onHydrate}
          disabled={hydrating}
          className={[
            "mt-4 inline-flex h-8 items-center justify-center gap-2 rounded-md bg-[var(--text-primary)] px-3 text-[12px] font-medium text-white shadow-[0_1px_2px_rgba(15,15,15,0.22),inset_0_0.5px_0_rgba(255,255,255,0.12)] transition duration-150 hover:bg-[#1f1f1f] disabled:cursor-not-allowed disabled:opacity-60",
            compact ? "w-full" : "",
          ].join(" ")}
        >
          {hydrating ? <Loader2 className="size-3.5 animate-spin" /> : <Download className="size-3.5" />}
          <span className="whitespace-nowrap">{hydrating ? "Hydrating..." : compact ? "Hydrate" : "Hydrate Record"}</span>
        </button>

        <div className="mt-4">
          <p className="text-[10px] font-medium uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
            Command
          </p>
          <code className="mt-1 block overflow-x-auto whitespace-pre rounded-md border border-[var(--border-subtle)] bg-[var(--surface-1)] px-3 py-2 font-mono text-[11px] text-[var(--text-primary)]">
            {`uv run articraft data hydrate --record ${record.record_id}`}
          </code>
        </div>
      </div>
    </div>
  );
}
