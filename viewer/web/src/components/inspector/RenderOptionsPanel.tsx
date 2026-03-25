import { useState, type JSX } from "react";
import { AlertTriangle, Check, Copy } from "lucide-react";

import { Label } from "@/components/ui/label";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Switch } from "@/components/ui/switch";
import { copyTextToClipboard } from "@/lib/record-path";

export type RenderOptions = {
  showEdges: boolean;
  showGrid: boolean;
  showCollisions: boolean;
  showSegmentColors: boolean;
  showSurfaceSamples: boolean;
  doubleSided: boolean;
  autoAnimate: boolean;
  showJointOverlay: boolean;
};

export const defaultRenderOptions: RenderOptions = {
  showEdges: true,
  showGrid: true,
  showCollisions: false,
  showSegmentColors: false,
  showSurfaceSamples: false,
  doubleSided: true,
  autoAnimate: false,
  showJointOverlay: false,
};

type RenderOptionsPanelProps = {
  options: RenderOptions;
  onOptionChange: <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => void;
  collisionSupport?: {
    available: boolean;
    summary: string;
    detail: string;
    compileCommand: string | null;
  } | null;
};

type OptionRow = {
  key: keyof RenderOptions;
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
  collisionSupport = null,
}: RenderOptionsPanelProps): JSX.Element {
  const displayRows = optionRows.filter((row) => row.section === "display");
  const motionRows = optionRows.filter((row) => row.section === "motion");
  const [copiedCommand, setCopiedCommand] = useState(false);
  const collisionsUnavailable = collisionSupport != null && !collisionSupport.available;

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

  return (
    <ScrollArea className="h-full">
      <div className="space-y-4 pb-1">
        <div className="flex items-center gap-2 pb-2">
          <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">Display</span>
          <div className="h-px flex-1 bg-[var(--border-subtle)]" />
        </div>
        <div className="space-y-2">
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
      </div>
    </ScrollArea>
  );
}
