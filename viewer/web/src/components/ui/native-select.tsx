import * as React from "react";
import { ChevronDownIcon } from "lucide-react";

import { cn } from "@/lib/utils";

type NativeSelectProps = React.SelectHTMLAttributes<HTMLSelectElement> & {
  selectSize?: "sm" | "default";
};

export const NativeSelect = React.forwardRef<HTMLSelectElement, NativeSelectProps>(
  ({ className, selectSize = "default", children, ...props }, ref) => (
    <div className="relative w-full">
      <select
        ref={ref}
        data-size={selectSize}
        className={cn(
          "w-full appearance-none rounded-md border border-[var(--border-default)] bg-[var(--surface-0)] py-1.5 pl-2.5 pr-8 text-[12px] text-[var(--text-primary)] shadow-none outline-none transition-all duration-150 focus-visible:border-[var(--accent)] focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)] disabled:cursor-not-allowed disabled:opacity-40 data-[size=default]:h-8 data-[size=sm]:h-7",
          className,
        )}
        {...props}
      >
        {children}
      </select>
      <ChevronDownIcon className="pointer-events-none absolute right-2.5 top-1/2 size-3.5 -translate-y-1/2 text-[var(--text-tertiary)]" />
    </div>
  ),
);

NativeSelect.displayName = "NativeSelect";
