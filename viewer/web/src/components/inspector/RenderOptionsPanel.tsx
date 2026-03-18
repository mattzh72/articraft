import type { JSX } from "react";

import { Label } from "@/components/ui/label";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Switch } from "@/components/ui/switch";

export type RenderOptions = {
  showEdges: boolean;
  showGrid: boolean;
  showCollisions: boolean;
  doubleSided: boolean;
  environmentLighting: boolean;
  autoAnimate: boolean;
  showJointOverlay: boolean;
};

export const defaultRenderOptions: RenderOptions = {
  showEdges: false,
  showGrid: true,
  showCollisions: false,
  doubleSided: true,
  environmentLighting: true,
  autoAnimate: false,
  showJointOverlay: false,
};

type RenderOptionsPanelProps = {
  options: RenderOptions;
  onOptionChange: <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => void;
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
    key: "doubleSided",
    label: "Double-sided",
    description: "Render front & back faces",
    section: "display",
  },
  {
    key: "environmentLighting",
    label: "Environment",
    description: "HDR environment lighting",
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

export function RenderOptionsPanel({ options, onOptionChange }: RenderOptionsPanelProps): JSX.Element {
  const displayRows = optionRows.filter((row) => row.section === "display");
  const motionRows = optionRows.filter((row) => row.section === "motion");

  return (
    <ScrollArea className="h-full">
      <div className="space-y-4 pb-1">
        <div className="flex items-center gap-2 pb-2">
          <span className="text-[10px] font-medium uppercase tracking-[0.05em] text-[var(--text-tertiary)]">Display</span>
          <div className="h-px flex-1 bg-[var(--border-subtle)]" />
        </div>
        <div className="space-y-2">
          {displayRows.map((row) => (
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
