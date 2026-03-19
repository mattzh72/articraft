export type AppRoute = { page: "dashboard" } | { page: "viewer" };

export function parseRoute(): AppRoute {
  const pathname = window.location.pathname;
  if (pathname === "/viewer" || pathname.startsWith("/viewer/")) {
    return { page: "viewer" };
  }
  return { page: "dashboard" };
}

export function navigateTo(route: AppRoute): void {
  const path = route.page === "viewer" ? "/viewer" : "/";
  if (window.location.pathname === path) {
    return;
  }
  window.history.pushState(null, "", path + window.location.search);
  window.dispatchEvent(new PopStateEvent("popstate"));
}
