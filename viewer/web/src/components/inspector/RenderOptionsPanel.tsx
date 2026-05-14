import { useState, type JSX } from "react";
import { AlertTriangle, Camera, Check, Copy, Loader2, Sparkles, X, Zap } from "lucide-react";

import { Label } from "@/components/ui/label";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Switch } from "@/components/ui/switch";
import type { SnapshotExporter } from "@/components/viewer3d/SceneCanvas";
import type { RenderOptions } from "@/components/viewer3d/useRenderOptions";
import { copyTextToClipboard } from "@/lib/record-path";

type RenderOptionsPanelProps = {
  options: RenderOptions;
  onOptionChange: <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => void;
  onSnapshot?: SnapshotExporter | null;
  collisionSupport?: {
    available: boolean;
    summary: string;
    detail: string;
    compileCommand: string | null;
  } | null;
};

type SnapshotState = "idle" | "exporting" | "saved" | "error";

type OptionRow = {
  key: Exclude<keyof RenderOptions, "fancyGraphics">;
  label: string;
  description: string;
  section: "display" | "motion";
};

const optionRows: OptionRow[] = [
  {
    key: "showEdges",
    label: "Edges",
    description: "Wireframe overlay on meshes",
    section: "display",
  },
  {
    key: "showGrid",
    label: "Grid",
    description: "Ground-plane reference grid",
    section: "display",
  },
  {
    key: "showCollisions",
    label: "Collisions",
    description: "Semi-transparent collision geometry",
    section: "display",
  },
  {
    key: "showSegmentColors",
    label: "Part colors",
    description: "High-contrast colors for each segmented part",
    section: "display",
  },
  {
    key: "showSurfaceSamples",
    label: "Surface samples",
    description: "Sample point clouds across visible mesh surfaces",
    section: "display",
  },
  {
    key: "doubleSided",
    label: "Double-sided",
    description: "Render front & back faces",
    section: "display",
  },
  {
    key: "autoAnimate",
    label: "Preview motion",
    description: "Animate all movable joints together and sweep through the kinematic tree",
    section: "motion",
  },
  {
    key: "showJointOverlay",
    label: "Joint overlay",
    description: "Show joint axis markers with type and axis labels",
    section: "motion",
  },
];

export function RenderOptionsPanel({
  options,
  onOptionChange,
  onSnapshot = null,
  collisionSupport = null,
}: RenderOptionsPanelProps): JSX.Element {
  const displayRows = optionRows.filter((row) => row.section === "display");
  const motionRows = optionRows.filter((row) => row.section === "motion");
  const [copiedCommand, setCopiedCommand] = useState(false);
  const [snapshotState, setSnapshotState] = useState<SnapshotState>("idle");
  const collisionsUnavailable = collisionSupport != null && !collisionSupport.available;
  const snapshotDisabled = !onSnapshot || snapshotState === "exporting";

  const handleCopyCommand = async (): Promise<void> => {
    if (!collisionSupport?.compileCommand) {
      return;
    }
    await copyTextToClipboard(collisionSupport.compileCommand);
    setCopiedCommand(true);
    window.setTimeout(() => {
      setCopiedCommand(false);
    }, 1200);
  };

  const handleSnapshot = async (): Promise<void> => {
    if (!onSnapshot || snapshotState === "exporting") {
      return;
    }

    setSnapshotState("exporting");
    try {
      await onSnapshot();
      setSnapshotState("saved");
    } catch {
      setSnapshotState("error");
    }
    window.setTimeout(() => {
      setSnapshotState("idle");
    }, 1600);
  };

  const snapshotLabel =
    snapshotState === "exporting"
      ? "Saving…"
      : snapshotState === "saved"
        ? "Saved"
        : snapshotState === "error"
          ? "Retry"
          : "Save PNG";
  const snapshotStateClass =
    snapshotState === "saved"
      ? "bg-[var(--success)] text-white shadow-[0_1px_2px_rgba(26,138,74,0.30),inset_0_0.5px_0_rgba(255,255,255,0.18)]"
      : snapshotState === "error"
        ? "bg-[var(--destructive)] text-white shadow-[0_1px_2px_rgba(209,52,21,0.30),inset_0_0.5px_0_rgba(255,255,255,0.18)] hover:bg-[#e04125] hover:shadow-[0_2px_5px_rgba(209,52,21,0.36),inset_0_0.5px_0_rgba(255,255,255,0.20)]"
        : "bg-[var(--text-primary)] text-white shadow-[0_1px_2px_rgba(15,15,15,0.22),inset_0_0.5px_0_rgba(255,255,255,0.12)] hover:bg-[#1f1f1f] hover:shadow-[0_2px_5px_rgba(15,15,15,0.28),inset_0_0.5px_0_rgba(255,255,255,0.16)] active:translate-y-[0.5px] active:shadow-[0_0_0_rgba(0,0,0,0),inset_0_0.5px_0_rgba(255,255,255,0.10)]";

  return (
    <ScrollArea className="h-full">
      <div className="space-y-4 pb-1">
        <div className="flex items-center gap-2 pb-2">
          <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">Display</span>
          <div className="h-px flex-1 bg-[var(--border-subtle)]" />
        </div>
        <div className="space-y-2">
          <div className="flex items-center justify-between gap-3 py-1.5">
            <div className="min-w-0">
              <Label className="text-[12px] text-[var(--text-primary)]">Graphics</Label>
              <p className="text-[10px] leading-[1.3] text-[var(--text-tertiary)]">
                {options.fancyGraphics ? "Reflections, shadows, clean surfaces" : "Performance-oriented renderer"}
              </p>
            </div>
            <div
              role="radiogroup"
              aria-label="Graphics quality"
              className="relative grid h-7 w-[60px] shrink-0 grid-cols-2 rounded-md border border-[var(--border-subtle)] bg-[var(--surface-2)] p-[2px]"
            >
              <span
                aria-hidden
                className="pointer-events-none absolute top-[2px] bottom-[2px] left-[2px] w-[calc(50%-2px)] rounded-[4px] bg-[var(--surface-0)] shadow-[0_1px_2px_rgba(15,15,15,0.08),0_0_0_0.5px_rgba(15,15,15,0.06)]"
                style={{
                  transition: "transform 260ms cubic-bezier(0.32, 0.72, 0, 1)",
                  transform: options.fancyGraphics ? "translateX(100%)" : "translateX(0)",
                }}
              />
              <button
                type="button"
                role="radio"
                aria-checked={!options.fancyGraphics}
                aria-label="Fast graphics"
                title="Fast"
                onClick={() => onOptionChange("fancyGraphics", false)}
                className={`relative z-10 inline-flex items-center justify-center rounded-[4px] transition-colors duration-150 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent)] focus-visible:ring-offset-1 ${
                  !options.fancyGraphics
                    ? "text-[var(--text-primary)]"
                    : "text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
                }`}
              >
                <Zap className="size-[12px]" strokeWidth={2.25} />
              </button>
              <button
                type="button"
                role="radio"
                aria-checked={options.fancyGraphics}
                aria-label="Fancy graphics"
                title="Fancy"
                onClick={() => onOptionChange("fancyGraphics", true)}
                className={`relative z-10 inline-flex items-center justify-center rounded-[4px] transition-colors duration-150 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent)] focus-visible:ring-offset-1 ${
                  options.fancyGraphics
                    ? "text-[var(--text-primary)]"
                    : "text-[var(--text-tertiary)] hover:text-[var(--text-secondary)]"
                }`}
              >
                <Sparkles className="size-[12px]" strokeWidth={2.25} />
              </button>
            </div>
          </div>
          {displayRows.map((row) => (
            <div key={row.key}>
              <div
                className={`flex items-center justify-between gap-3 py-1.5 ${
                  row.key === "showCollisions" && collisionsUnavailable ? "opacity-60" : ""
                }`}
              >
                <div className="min-w-0">
                  <Label htmlFor={`render-${row.key}`} className="text-[12px] text-[var(--text-primary)]">
                    {row.label}
                  </Label>
                  <p className="text-[10px] leading-[1.3] text-[var(--text-tertiary)]">
                    {row.key === "showCollisions" && collisionsUnavailable ? collisionSupport.summary : row.description}
                  </p>
                </div>
                <Switch
                  id={`render-${row.key}`}
                  checked={row.key === "showCollisions" && collisionsUnavailable ? false : options[row.key]}
                  disabled={row.key === "showCollisions" && collisionsUnavailable}
                  onCheckedChange={(checked: boolean) => onOptionChange(row.key, checked)}
                />
              </div>
              {row.key === "showCollisions" && collisionsUnavailable ? (
                <div className="mt-2.5 mb-1 rounded-lg bg-amber-500/[0.08] px-3 py-2.5">
                  <div className="flex items-start gap-2">
                    <AlertTriangle className="mt-px size-3.5 shrink-0 text-amber-600" />
                    <div className="min-w-0">
                      <p className="text-[11px] font-semibold text-amber-900">Missing collisions</p>
                      <p className="mt-1 text-[11px] leading-[1.5] text-amber-800/70">
                        This model has visual meshes only. Run the compile command to generate collision geometry.
                      </p>
                    </div>
                  </div>
                  {collisionSupport.compileCommand ? (
                    <button
                      type="button"
                      onClick={() => void handleCopyCommand()}
                      className="group/cmd mt-2.5 flex w-full cursor-pointer items-start gap-2 rounded-md bg-amber-900/[0.06] px-2.5 py-2 text-left transition-colors hover:bg-amber-900/[0.10]"
                    >
                      <code className="min-w-0 flex-1 break-all font-mono text-[11px] leading-[1.5] text-amber-900/80">
                        {collisionSupport.compileCommand}
                      </code>
                      <span className="mt-0.5 shrink-0 text-amber-700/50 transition-colors group-hover/cmd:text-amber-800">
                        {copiedCommand ? <Check className="size-3.5" /> : <Copy className="size-3.5" />}
                      </span>
                    </button>
                  ) : (
                    <p className="mt-2 text-[11px] leading-[1.5] text-amber-800/70">
                      Save the record first, then compile to generate collisions.
                    </p>
                  )}
                </div>
              ) : null}
            </div>
          ))}
        </div>
        <div className="flex items-center gap-2 pb-2 pt-1">
          <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">Motion</span>
          <div className="h-px flex-1 bg-[var(--border-subtle)]" />
        </div>
        <div className="space-y-2">
          {motionRows.map((row) => (
            <div key={row.key} className="flex items-center justify-between gap-3 py-1.5">
              <div className="min-w-0">
                <Label htmlFor={`render-${row.key}`} className="text-[12px] text-[var(--text-primary)]">
                  {row.label}
                </Label>
                <p className="text-[10px] leading-[1.3] text-[var(--text-tertiary)]">{row.description}</p>
              </div>
              <Switch
                id={`render-${row.key}`}
                checked={options[row.key]}
                onCheckedChange={(checked: boolean) => onOptionChange(row.key, checked)}
              />
            </div>
          ))}
        </div>
        <div className="flex items-center gap-2 pb-2 pt-1">
          <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">Export</span>
          <div className="h-px flex-1 bg-[var(--border-subtle)]" />
        </div>
        <div className="flex items-center justify-between gap-3 py-1.5">
          <div className="min-w-0">
            <Label className="text-[12px] text-[var(--text-primary)]">Snapshot</Label>
            <p className="text-[10px] leading-[1.3] text-[var(--text-tertiary)]">
              Current pose on white background
            </p>
          </div>
          <button
            type="button"
            onClick={() => void handleSnapshot()}
            disabled={snapshotDisabled}
            aria-label={snapshotLabel}
            title={snapshotLabel}
            className={`inline-flex h-7 shrink-0 items-center gap-1.5 rounded-md px-2.5 text-[11px] font-medium duration-150 ease-out [transition-property:background-color,box-shadow,transform] focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent)] focus-visible:ring-offset-1 disabled:opacity-50 ${snapshotStateClass}`}
            style={{ touchAction: "manipulation" }}
          >
            {snapshotState === "exporting" ? (
              <Loader2 className="size-3 animate-spin" strokeWidth={2.5} />
            ) : snapshotState === "saved" ? (
              <Check className="size-3" strokeWidth={2.75} />
            ) : snapshotState === "error" ? (
              <X className="size-3" strokeWidth={2.75} />
            ) : (
              <Camera className="size-3" strokeWidth={2.25} />
            )}
            <span>{snapshotLabel}</span>
          </button>
        </div>
      </div>
    </ScrollArea>
  );
}
