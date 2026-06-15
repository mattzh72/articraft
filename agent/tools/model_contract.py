from __future__ import annotations

import ast


def missing_required_model_contract(model_code: str) -> list[str]:
    try:
        tree = ast.parse(model_code)
    except SyntaxError:
        # Let syntax validation surface this with richer line details.
        return []

    defined = {
        node.name for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    missing = [name for name in ("build_object_model", "run_tests") if name not in defined]
    if not _has_object_model_assignment(tree):
        missing.append("object_model = build_object_model()")
    return missing


def _has_object_model_assignment(tree: ast.Module) -> bool:
    for node in tree.body:
        value: ast.AST | None = None
        targets: list[ast.expr] = []
        if isinstance(node, ast.Assign):
            value = node.value
            targets = list(node.targets)
        elif isinstance(node, ast.AnnAssign):
            value = node.value
            targets = [node.target]

        if not isinstance(value, ast.Call):
            continue
        if not isinstance(value.func, ast.Name) or value.func.id != "build_object_model":
            continue
        if any(isinstance(target, ast.Name) and target.id == "object_model" for target in targets):
            return True
    return False
