import * as React from "react";
import { Switch as SwitchPrimitive } from "radix-ui";

import { cn } from "@/lib/utils";

export function Switch({
  className,
  ...props
}: React.ComponentProps<typeof SwitchPrimitive.Root>): React.JSX.Element {
  return (
    <SwitchPrimitive.Root
      data-slot="switch"
      className={cn(
        "peer inline-flex h-[18px] w-[32px] shrink-0 cursor-pointer items-center rounded-full transition-colors duration-150 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)] disabled:cursor-not-allowed disabled:opacity-40 data-[state=checked]:bg-[var(--accent)] data-[state=unchecked]:bg-[var(--border-strong)]",
        className,
      )}
      {...props}
    >
      <SwitchPrimitive.Thumb
        className={cn(
          "pointer-events-none block size-[14px] rounded-full bg-white shadow-[0_1px_2px_rgba(0,0,0,0.1)] ring-0 transition-transform duration-150 data-[state=checked]:translate-x-[15px] data-[state=unchecked]:translate-x-[1px]",
        )}
      />
    </SwitchPrimitive.Root>
  );
}
