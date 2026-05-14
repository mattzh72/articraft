import { lazy, Suspense, useState, type JSX } from "react";

import type { InspectorTab } from "@/lib/types";
import { useViewer, useViewerDispatch } from "@/lib/viewer-context";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { InspectPanel } from "@/components/inspector/InspectPanel";
import { RenderOptionsPanel } from "@/components/inspector/RenderOptionsPanel";
import { defaultRenderOptions, type RenderOptions } from "@/components/viewer3d/useRenderOptions";
import type { SnapshotExporter } from "@/components/viewer3d/SceneCanvas";
import type { UrdfJoint } from "@/components/inspector/JointSlider";

const CodePanel = lazy(() =>
  import("@/components/inspector/CodePanel").then((module) => ({
    default: module.CodePanel,
  })),
);

const MetadataPanel = lazy(() =>
  import("@/components/inspector/MetadataPanel").then((module) => ({
    default: module.MetadataPanel,
  })),
);

type InspectorTabsProps = {
  urdfSpec?: { joints: UrdfJoint[] } | null;
  jointValues?: Map<string, number>;
  onJointChange?: (name: string, value: number) => void;
  onResetAll?: () => void;
  renderOptions?: RenderOptions;
  onRenderOptionChange?: <K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => void;
  onSnapshot?: SnapshotExporter | null;
  collisionSupport?: {
    available: boolean;
    summary: string;
    detail: string;
    compileCommand: string | null;
  } | null;
};

function InspectorTabFallback(): JSX.Element {
  return (
    <div className="flex h-32 items-center justify-center">
      <p className="text-[10px] text-[var(--text-quaternary)]">Loading...</p>
    </div>
  );
}

export function InspectorTabs({
  urdfSpec = null,
  jointValues = new Map(),
  onJointChange = () => {},
  onResetAll = () => {},
  renderOptions,
  onRenderOptionChange,
  onSnapshot = null,
  collisionSupport = null,
}: InspectorTabsProps): JSX.Element {
  const { selectedInspectorTab, selection } = useViewer();
  const dispatch = useViewerDispatch();

  const [localOptions, setLocalOptions] = useState<RenderOptions>(defaultRenderOptions);
  const activeOptions = renderOptions ?? localOptions;
  const handleOptionChange = onRenderOptionChange ?? (<K extends keyof RenderOptions>(key: K, value: RenderOptions[K]) => {
    setLocalOptions((prev) => ({ ...prev, [key]: value }));
  });

  const tabLabels = {
    inspect: "Inspect",
    render: "Render",
    code: "Code",
    metadata: "Metadata",
  } as const;
  const tabs = Object.keys(tabLabels) as Array<keyof typeof tabLabels>;

  return (
    <Tabs
      value={selectedInspectorTab}
      onValueChange={(value) => dispatch({ type: "SET_INSPECTOR_TAB", payload: value as InspectorTab })}
      className="relative flex h-full flex-col gap-0"
    >
      {/* Tab bar */}
      <div className="flex border-b border-[var(--border-default)] px-3">
        <TabsList className="flex h-auto gap-0 rounded-none border-none bg-transparent p-0">
          {tabs.map((tab) => (
            <TabsTrigger
              key={tab}
              value={tab}
              className="relative rounded-none px-3 py-2.5 text-[11px] font-medium text-[var(--text-tertiary)] transition-colors duration-150 hover:text-[var(--text-secondary)] data-[state=active]:text-[var(--text-primary)] data-[state=active]:bg-transparent data-[state=active]:shadow-none data-[state=active]:after:absolute data-[state=active]:after:bottom-0 data-[state=active]:after:left-3 data-[state=active]:after:right-3 data-[state=active]:after:h-[1.5px] data-[state=active]:after:rounded-full data-[state=active]:after:bg-[var(--text-primary)]"
            >{tabLabels[tab]}</TabsTrigger>
          ))}
        </TabsList>
      </div>

      <TabsContent value="inspect" className="min-h-0 flex-1 overflow-hidden px-3 pb-3 pt-3">
        <InspectPanel
          urdfSpec={urdfSpec}
          jointValues={jointValues}
          onJointChange={onJointChange}
          onResetAll={onResetAll}
        />
      </TabsContent>

      <TabsContent value="render" className="min-h-0 flex-1 overflow-hidden px-3 pb-3 pt-3">
        <RenderOptionsPanel
          options={activeOptions}
          onOptionChange={handleOptionChange}
          onSnapshot={onSnapshot}
          collisionSupport={collisionSupport}
        />
      </TabsContent>

      <TabsContent value="code" className="min-h-0 flex-1 overflow-hidden px-3 pb-3 pt-3">
        {selectedInspectorTab === "code" ? (
          <Suspense fallback={<InspectorTabFallback />}>
            <CodePanel />
          </Suspense>
        ) : null}
      </TabsContent>

      <TabsContent value="metadata" className="min-h-0 flex-1 overflow-hidden px-3 pb-3 pt-3">
        {selectedInspectorTab === "metadata" ? (
          <Suspense fallback={<InspectorTabFallback />}>
            <MetadataPanel />
          </Suspense>
        ) : null}
      </TabsContent>

      {!selection && (
        <div className="absolute inset-0 flex items-center justify-center bg-[var(--surface-0)]/90">
          <div className="px-4 py-2.5 text-center">
            <p className="text-[12px] text-[var(--text-quaternary)]">No record selected</p>
          </div>
        </div>
      )}
    </Tabs>
  );
}
