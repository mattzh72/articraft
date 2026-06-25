export type AppRoute = { page: "viewer" };

export function parseRoute(): AppRoute {
  return { page: "viewer" };
}

export function navigateTo(): void {
  if (window.location.pathname === "/viewer") {
    return;
  }
  window.history.pushState(null, "", "/viewer");
  window.dispatchEvent(new PopStateEvent("popstate"));
}
