from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from sdk_hybrid import ArticulatedObject, AssetContext, TestContext, TestReport

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="draft_model", assets=ASSETS)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual connectivity gate for floating/disconnected subassemblies inside one part.
    ctx.check_part_geometry_connected()

    # Encode the actual visual/mechanical claims with prompt-specific exact checks.
    # If you add a warning-tier heuristic and it fires, investigate it with
    # `probe_model` before editing geometry or relaxing thresholds.
    # Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is
    # genuinely uncertain or mechanically important.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # If the object has a mounted subassembly, prefer exact `expect_contact(...)`,
    # `expect_gap(...)`, `expect_overlap(...)`, and `expect_within(...)` checks on
    # named local features over broad warning-tier heuristics.
    # Add prompt-specific exact visual checks below; optional warning heuristics are not enough.
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
