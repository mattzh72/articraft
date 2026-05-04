import { createContext, useContext, useState, type JSX, type ReactNode } from "react";

import type { RecordAnimationFrame } from "@/lib/types";

type AnimationContextValue = {
  activeFrame: RecordAnimationFrame | null;
  setActiveFrame: (frame: RecordAnimationFrame | null) => void;
};

const AnimationContext = createContext<AnimationContextValue>({
  activeFrame: null,
  setActiveFrame: () => {},
});

export function AnimationProvider({ children }: { children: ReactNode }): JSX.Element {
  const [activeFrame, setActiveFrame] = useState<RecordAnimationFrame | null>(null);
  return (
    <AnimationContext.Provider value={{ activeFrame, setActiveFrame }}>
      {children}
    </AnimationContext.Provider>
  );
}

export function useAnimation(): AnimationContextValue {
  return useContext(AnimationContext);
}
