import { type JSX } from "react";

type MissingArtifactsOverlayProps = {
  recordId: string;
  hasCompileReport: boolean;
  detail?: string | null;
  compact?: boolean;
};

export function MissingArtifactsOverlay({
  recordId,
  hasCompileReport,
  detail = null,
  compact = false,
}: MissingArtifactsOverlayProps): JSX.Element {
  const title = hasCompileReport ? "Viewer artifacts are missing" : "Viewer artifacts are unavailable";
  const description = hasCompileReport
    ? "A compile report exists for this record, but the saved URDF or mesh files the viewer needs are missing or stale on disk."
    : "This record does not currently have the saved URDF and mesh files the viewer needs.";

  return (
    <div className="flex h-full w-full items-center justify-center p-4">
      <div
        className={[
          "w-full rounded-xl border border-[var(--border-default)] bg-[var(--surface-0)] shadow-[0_16px_40px_rgba(15,23,42,0.14)]",
          compact ? "max-w-sm p-4" : "max-w-xl p-5",
        ].join(" ")}
      >
        <div className="flex items-center gap-2">
          <span className="inline-flex h-2.5 w-2.5 rounded-full bg-[#d97706]" />
          <p className="text-[11px] font-semibold uppercase tracking-[0.08em] text-[var(--text-tertiary)]">
            Materialization Required
          </p>
        </div>

        <h3 className={compact ? "mt-3 text-[15px] font-semibold text-[var(--text-primary)]" : "mt-3 text-[18px] font-semibold text-[var(--text-primary)]"}>
          {title}
        </h3>
        <p className="mt-2 text-[12px] leading-5 text-[var(--text-secondary)]">{description}</p>
        {detail ? <p className="mt-2 text-[11px] leading-5 text-[var(--text-tertiary)]">{detail}</p> : null}

        <div className="mt-4 space-y-3">
          <div>
            <p className="text-[10px] font-medium uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
              Compile This Record
            </p>
            <code className="mt-1 block overflow-x-auto rounded-md border border-[var(--border-subtle)] bg-[var(--surface-1)] px-3 py-2 font-mono text-[11px] text-[var(--text-primary)]">
              {`just compile data/records/${recordId}`}
            </code>
          </div>

          <div>
            <p className="text-[10px] font-medium uppercase tracking-[0.06em] text-[var(--text-quaternary)]">
              Precompile Saved Records
            </p>
            <code className="mt-1 block overflow-x-auto rounded-md border border-[var(--border-subtle)] bg-[var(--surface-1)] px-3 py-2 font-mono text-[11px] text-[var(--text-primary)]">
              just compile-all
            </code>
          </div>
        </div>

        <p className="mt-4 text-[11px] leading-5 text-[var(--text-tertiary)]">
          The viewer attempted to materialize the record automatically before showing this message. If it still failed, the record likely needs manual recompilation.
        </p>
      </div>
    </div>
  );
}
