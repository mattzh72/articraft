import { useSyncExternalStore } from "react";
import { parseRoute, type AppRoute } from "@/lib/router";

function subscribe(callback: () => void): () => void {
  window.addEventListener("popstate", callback);
  return () => window.removeEventListener("popstate", callback);
}

type PageName = AppRoute["page"];

function getSnapshot(): PageName {
  return parseRoute().page;
}

export function useRoute(): AppRoute {
  const page = useSyncExternalStore(subscribe, getSnapshot);
  return { page };
}
