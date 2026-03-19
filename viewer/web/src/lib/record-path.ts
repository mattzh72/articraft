export function buildRecordPath(repoRoot: string, recordId: string): string {
  const separator = repoRoot.includes("\\") ? "\\" : "/";
  const normalizedRoot = repoRoot.replace(/[\\/]+$/, "");
  return [normalizedRoot, "data", "records", recordId].join(separator);
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
