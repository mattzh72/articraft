import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { QueryClientProvider } from "@tanstack/react-query";

import AblationComparePage from "@/components/compare/AblationComparePage";
import { viewerQueryClient } from "@/lib/query-client";
import "./styles.css";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <QueryClientProvider client={viewerQueryClient}>
      <AblationComparePage />
    </QueryClientProvider>
  </StrictMode>,
);
