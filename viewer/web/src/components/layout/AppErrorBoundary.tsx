import { Component, type ErrorInfo, type ReactNode } from "react";

type AppErrorBoundaryProps = {
  children: ReactNode;
};

type AppErrorBoundaryState = {
  error: Error | null;
};

export class AppErrorBoundary extends Component<AppErrorBoundaryProps, AppErrorBoundaryState> {
  state: AppErrorBoundaryState = {
    error: null,
  };

  static getDerivedStateFromError(error: unknown): AppErrorBoundaryState {
    return {
      error: error instanceof Error ? error : new Error("The viewer hit an unexpected error."),
    };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo): void {
    console.error("Viewer render error", error, errorInfo);
  }

  render(): ReactNode {
    if (!this.state.error) {
      return this.props.children;
    }

    return (
      <div className="flex h-screen items-center justify-center bg-[var(--surface-2)] px-4">
        <div className="w-full max-w-md rounded-lg border border-[var(--border-default)] bg-[var(--surface-0)] p-5 shadow-[0_16px_40px_rgba(15,23,42,0.14)]">
          <p className="text-[11px] font-semibold uppercase tracking-[0.08em] text-[var(--text-tertiary)]">
            Viewer Error
          </p>
          <h1 className="mt-3 text-[18px] font-semibold text-[var(--text-primary)]">
            The viewer needs a refresh
          </h1>
          <p className="mt-2 text-[12px] leading-5 text-[var(--text-secondary)]">
            A frontend module failed to load. This usually happens when the local viewer was rebuilt while this tab was already open.
          </p>
          <p className="mt-2 break-words font-mono text-[10px] leading-5 text-[var(--text-tertiary)]">
            {this.state.error.message}
          </p>
          <button
            type="button"
            onClick={() => window.location.reload()}
            className="mt-4 inline-flex h-8 items-center justify-center rounded-md bg-[var(--text-primary)] px-3 text-[12px] font-medium text-white shadow-[0_1px_2px_rgba(15,15,15,0.22),inset_0_0.5px_0_rgba(255,255,255,0.12)] transition duration-150 hover:bg-[#1f1f1f]"
          >
            Refresh Viewer
          </button>
        </div>
      </div>
    );
  }
}
