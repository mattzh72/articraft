from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_air_purifier", assets=ASSETS)

    shell_white = model.material("shell_white", rgba=(0.95, 0.96, 0.94, 1.0))
    base_gray = model.material("base_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.20, 0.22, 1.0))

    shell = model.part("shell")
    shell.visual(
        Box((0.11, 0.18, 0.80)),
        origin=Origin(xyz=(-0.03, 0.0, 0.40)),
        material=shell_white,
        name="rear_body",
    )
    shell.visual(
        Cylinder(radius=0.09, length=0.80),
        origin=Origin(xyz=(0.035, 0.0, 0.40)),
        material=shell_white,
        name="rounded_front",
    )

    base = model.part("base")
    base.visual(
        Box((0.14, 0.20, 0.035)),
        origin=Origin(xyz=(-0.03, 0.0, -0.0175)),
        material=base_gray,
        name="base_rear",
    )
    base.visual(
        Cylinder(radius=0.10, length=0.035),
        origin=Origin(xyz=(0.04, 0.0, -0.0175)),
        material=base_gray,
        name="base_front",
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.12, 0.19, 0.115)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0575)),
        material=shell_white,
        name="housing_rear",
    )
    housing.visual(
        Cylinder(radius=0.095, length=0.115),
        origin=Origin(xyz=(0.04, 0.0, 0.0575)),
        material=shell_white,
        name="housing_front",
    )
    housing.visual(
        Box((0.075, 0.11, 0.006)),
        origin=Origin(xyz=(0.01, 0.0, 0.112)),
        material=charcoal,
        name="top_outlet",
    )
    housing.visual(
        Box((0.01, 0.07, 0.028)),
        origin=Origin(xyz=(0.108, 0.0, 0.072)),
        material=charcoal,
        name="control_strip",
    )

    intake = model.part("intake")
    intake.visual(
        Box((0.010, 0.008, 0.56)),
        origin=Origin(xyz=(0.0, -0.046, 0.0)),
        material=charcoal,
        name="left_rail",
    )
    intake.visual(
        Box((0.010, 0.008, 0.56)),
        origin=Origin(xyz=(0.0, 0.046, 0.0)),
        material=charcoal,
        name="right_rail",
    )
    intake.visual(
        Box((0.010, 0.10, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=charcoal,
        name="top_frame",
    )
    intake.visual(
        Box((0.010, 0.10, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material=charcoal,
        name="bottom_frame",
    )
    for index in range(7):
        intake.visual(
            Box((0.006, 0.086, 0.013)),
            origin=Origin(xyz=(0.001, 0.0, -0.21 + 0.07 * index)),
            material=charcoal,
            name=f"slat_{index + 1}",
        )

    model.articulation(
        "shell_to_base",
        ArticulationType.FIXED,
        parent=shell,
        child=base,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "shell_to_housing",
        ArticulationType.FIXED,
        parent=shell,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
    )
    model.articulation(
        "shell_to_intake",
        ArticulationType.FIXED,
        parent=shell,
        child=intake,
        origin=Origin(xyz=(0.112, 0.0, 0.40)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    shell = object_model.get_part("shell")
    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    intake = object_model.get_part("intake")
    top_outlet = housing.get_visual("top_outlet")
    rounded_front = shell.get_visual("rounded_front")
    rear_body = shell.get_visual("rear_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
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
    ctx.expect_gap(shell, base, axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_overlap(shell, base, axes="xy", min_overlap=0.02)
    ctx.expect_within(shell, base, axes="xy")

    ctx.expect_gap(housing, shell, axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_overlap(housing, shell, axes="xy", min_overlap=0.02)
    ctx.expect_origin_distance(housing, shell, axes="xy", max_dist=0.02)
    ctx.expect_within(shell, housing, axes="xy")
    ctx.expect_within(housing, housing, axes="xy", inner_elem=top_outlet)

    ctx.expect_within(intake, shell, axes="yz")
    ctx.expect_overlap(intake, shell, axes="yz", min_overlap=0.04)
    ctx.expect_gap(intake, shell, axis="x", max_gap=0.002, max_penetration=0.02)
    ctx.expect_origin_distance(intake, shell, axes="y", max_dist=0.005)
    ctx.expect_overlap(shell, shell, axes="yz", elem_a=rounded_front, elem_b=rear_body, min_overlap=0.15)

    ctx.expect_gap(housing, base, axis="z", min_gap=0.75)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
