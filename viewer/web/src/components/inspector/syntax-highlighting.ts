import { Light as SyntaxHighlighter } from "react-syntax-highlighter";
import json from "react-syntax-highlighter/dist/esm/languages/hljs/json";
import python from "react-syntax-highlighter/dist/esm/languages/hljs/python";
import xml from "react-syntax-highlighter/dist/esm/languages/hljs/xml";
import { atomOneLight } from "react-syntax-highlighter/dist/esm/styles/hljs";

export type HighlightLanguage = "json" | "python" | "xml";

SyntaxHighlighter.registerLanguage("json", json);
SyntaxHighlighter.registerLanguage("python", python);
SyntaxHighlighter.registerLanguage("xml", xml);

export const codeTheme = {
  ...atomOneLight,
  hljs: {
    ...atomOneLight.hljs,
    background: "transparent",
    color: "#1f2937",
  },
};

export function normalizeHighlightLanguage(language: string | undefined): HighlightLanguage | null {
  if (language === "json" || language === "python" || language === "xml") {
    return language;
  }
  if (language === "py") {
    return "python";
  }
  if (language === "html" || language === "urdf") {
    return "xml";
  }
  return null;
}

export { SyntaxHighlighter };
