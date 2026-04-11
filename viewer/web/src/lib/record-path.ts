function pathSeparatorForRoot(repoRoot: string): string {
  return repoRoot.includes("\\") ? "\\" : "/";
}

function isAbsolutePath(value: string): boolean {
  return value.startsWith("/") || value.startsWith("\\\\") || /^[A-Za-z]:[\\/]/.test(value);
}

export function buildRepoPath(repoRoot: string, path: string): string {
  const normalizedPath = path.trim();
  if (!normalizedPath) {
    return normalizedPath;
  }
  if (isAbsolutePath(normalizedPath)) {
    return normalizedPath;
  }

  const separator = pathSeparatorForRoot(repoRoot);
  const normalizedRoot = repoRoot.replace(/[\\/]+$/, "");
  const normalizedRelativePath = normalizedPath.replace(/^[\\/]+/, "").replace(/[\\/]+/g, separator);
  return [normalizedRoot, normalizedRelativePath].join(separator);
}

export function buildRecordPath(repoRoot: string, recordId: string): string {
  const separator = repoRoot.includes("\\") ? "\\" : "/";
  return buildRepoPath(repoRoot, ["data", "records", recordId].join(separator));
}

export async function copyTextToClipboard(value: string): Promise<void> {
  if (typeof navigator !== "undefined" && navigator.clipboard?.writeText) {
    await navigator.clipboard.writeText(value);
    return;
  }

  if (typeof document === "undefined") {
    throw new Error("Clipboard is not available.");
  }

  const textarea = document.createElement("textarea");
  textarea.value = value;
  textarea.setAttribute("readonly", "");
  textarea.style.position = "absolute";
  textarea.style.left = "-9999px";
  document.body.appendChild(textarea);
  textarea.select();

  const copied = document.execCommand("copy");
  document.body.removeChild(textarea);

  if (!copied) {
    throw new Error("Clipboard copy failed.");
  }
}
