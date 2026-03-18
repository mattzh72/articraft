import * as React from "react";
import { Slider as SliderPrimitive } from "radix-ui";

import { cn } from "@/lib/utils";

export function Slider({
  className,
  defaultValue,
  value,
  min = 0,
  max = 100,
  ...props
}: React.ComponentProps<typeof SliderPrimitive.Root>): React.JSX.Element {
  return (
    <SliderPrimitive.Root
      data-slot="slider"
      defaultValue={defaultValue}
      value={value}
      min={min}
      max={max}
      className={cn("relative flex w-full touch-none select-none items-center", className)}
      {...props}
    >
      <SliderPrimitive.Track className="relative h-[3px] w-full grow overflow-hidden rounded-full bg-[var(--surface-3)]">
        <SliderPrimitive.Range className="absolute h-full bg-[var(--accent)]" />
      </SliderPrimitive.Track>
      <SliderPrimitive.Thumb className="block size-3 rounded-full border-[1.5px] border-[var(--accent)] bg-white shadow-[0_1px_3px_rgba(0,0,0,0.08)] transition-all duration-150 hover:shadow-[0_1px_4px_rgba(12,140,233,0.25)] focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]" />
    </SliderPrimitive.Root>
  );
}
