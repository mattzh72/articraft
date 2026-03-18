import * as React from "react";
import { cva, type VariantProps } from "class-variance-authority";

import { cn } from "@/lib/utils";

const badgeVariants = cva("inline-flex max-w-full items-center rounded-full px-2 py-[1px] text-[10px] font-medium tracking-[0.01em]", {
  variants: {
    variant: {
      default: "bg-[var(--accent-soft)] text-[var(--accent)]",
      success: "bg-[rgba(26,138,74,0.08)] text-[var(--success)]",
      warning: "bg-[rgba(194,120,3,0.08)] text-[var(--warning)]",
      destructive: "bg-[rgba(209,52,21,0.08)] text-[var(--destructive)]",
      secondary: "bg-[var(--surface-2)] text-[var(--text-tertiary)]",
    },
  },
  defaultVariants: {
    variant: "default",
  },
});

export type BadgeProps = React.HTMLAttributes<HTMLSpanElement> & VariantProps<typeof badgeVariants>;

export function Badge({ className, variant, ...props }: BadgeProps): React.JSX.Element {
  return <span className={cn(badgeVariants({ variant, className }))} {...props} />;
}
