import * as React from "react"

import { cn } from "@/lib/utils"

const ScrollArea = React.forwardRef<
  HTMLDivElement,
  React.HTMLAttributes<HTMLDivElement>
>(({ className, children, ...props }, ref) => {
  return (
    <div
      ref={ref}
      data-slot="scroll-area"
      className={cn("min-w-0 overflow-y-auto overflow-x-hidden", className)}
      {...props}
    >
      {children}
    </div>
  )
})
ScrollArea.displayName = "ScrollArea"

function ScrollBar(): null {
  return null
}

const ScrollArea = React.forwardRef<HTMLDivElement, ScrollAreaProps>(
  ({ className, children, ...props }, ref) => {
    const { type: ignoredType, scrollHideDelay: ignoredScrollHideDelay, ...divProps } = props
    void ignoredType
    void ignoredScrollHideDelay

    return (
      <div
        ref={ref}
        data-slot="scroll-area"
        className={cn("min-w-0 overflow-auto overscroll-contain", className)}
        {...divProps}
      >
        {children}
      </div>
    )
  },
)
ScrollArea.displayName = "ScrollArea"

type ScrollBarProps = React.ComponentPropsWithoutRef<"div"> & {
  orientation?: "vertical" | "horizontal"
}

const ScrollBar = React.forwardRef<HTMLDivElement, ScrollBarProps>(
  ({ className, orientation = "vertical", ...props }, ref) => (
    <div
      ref={ref}
      data-slot="scroll-area-scrollbar"
      data-orientation={orientation}
      className={cn("hidden", className)}
      {...props}
    />
  ),
)
ScrollBar.displayName = "ScrollBar"

export { ScrollArea, ScrollBar }
