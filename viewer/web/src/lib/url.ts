export function updateUrlSearchParams(
  updater: (params: URLSearchParams) => void,
): void {
  if (typeof window === "undefined") {
    return;
  }

  const url = new URL(window.location.href);
  updater(url.searchParams);
  window.history.replaceState(window.history.state, "", url);
}
