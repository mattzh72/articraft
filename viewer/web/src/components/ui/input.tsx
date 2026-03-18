import * as React from "react"

import { cn } from "@/lib/utils"

function Input({ className, type, ...props }: React.ComponentProps<"input">) {
  return (
    <input
      type={type}
      data-slot="input"
      className={cn(
        "h-8 w-full min-w-0 rounded-md border border-[var(--border-default)] bg-[var(--surface-0)] px-2.5 py-1.5 text-[12px] text-[var(--text-primary)] shadow-none transition-all duration-150 outline-none placeholder:text-[var(--text-quaternary)] disabled:pointer-events-none disabled:opacity-40",
        "focus-visible:border-[var(--accent)] focus-visible:ring-2 focus-visible:ring-[var(--accent-soft)]",
        "aria-invalid:border-[var(--destructive)]",
        className
      )}
      {...props}
    />
  )
}

export { Input }
