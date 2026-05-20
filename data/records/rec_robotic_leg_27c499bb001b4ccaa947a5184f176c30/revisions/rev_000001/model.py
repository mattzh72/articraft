from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    housing_grey = model.material("housing_grey", rgba=(0.28, 0.30, 0.33, 1.0))
    link_grey = model.material("link_grey", rgba=(0.42, 0.45, 0.48, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.15, 1.0))

    upper_leg_housing = model.part("upper_leg_housing")
    upper_leg_housing.visual(
        Box((0.24, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=housing_grey,
        name="main_casing",
    )
    upper_leg_housing.visual(
        Box((0.10, 0.16, 0.09)),
        origin=Origin(xyz=(-0.055, 0.0, 0.23)),
        material=dark_trim,
        name="rear_powerpack",
    )
    upper_leg_housing.visual(
        Box((0.18, 0.12, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=link_grey,
        name="mount_plate",
    )
    for side, y_pos in (("left", 0.0625), ("right", -0.0625)):
        upper_leg_housing.visual(
            Box((0.11, 0.025, 0.10)),
            origin=Origin(xyz=(0.0, y_pos, -0.005)),
            material=housing_grey,
            name=f"hip_{side}_cheek",
        )
    upper_leg_housing.visual(
        Box((0.11, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_trim,
        name="hip_bridge",
    )
    upper_leg_housing.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.28)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    thigh_link = model.part("thigh_link")
    thigh_link.visual(
        Box((0.11, 0.07, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=link_grey,
        name="hip_hub",
    )
    for side, y_pos in (("left", 0.035), ("right", -0.035)):
        thigh_link.visual(
            Box((0.06, 0.03, 0.06)),
            origin=Origin(xyz=(0.0, y_pos, -0.025)),
            material=dark_trim,
            name=f"hip_{side}_trunnion",
        )
    thigh_link.visual(
        Box((0.10, 0.11, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
        material=link_grey,
        name="thigh_beam",
    )
    thigh_link.visual(
        Box((0.07, 0.12, 0.16)),
        origin=Origin(xyz=(0.02, 0.0, -0.24)),
        material=dark_trim,
        name="thigh_cover",
    )
    for side, y_pos in (("left", 0.0575), ("right", -0.0575)):
        thigh_link.visual(
            Box((0.10, 0.025, 0.10)),
            origin=Origin(xyz=(0.0, y_pos, -0.39)),
            material=link_grey,
            name=f"knee_{side}_cheek",
        )
    thigh_link.visual(
        Box((0.10, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
        material=dark_trim,
        name="knee_bridge",
    )
    thigh_link.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.44)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
    )

    shank_link = model.part("shank_link")
    shank_link.visual(
        Box((0.10, 0.068, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=link_grey,
        name="knee_hub",
    )
    for side, y_pos in (("left", 0.03), ("right", -0.03)):
        shank_link.visual(
            Box((0.055, 0.03, 0.06)),
            origin=Origin(xyz=(0.0, y_pos, -0.025)),
            material=dark_trim,
            name=f"knee_{side}_trunnion",
        )
    shank_link.visual(
        Box((0.09, 0.10, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
        material=link_grey,
        name="shank_beam",
    )
    shank_link.visual(
        Box((0.065, 0.11, 0.16)),
        origin=Origin(xyz=(-0.015, 0.0, -0.24)),
        material=dark_trim,
        name="shank_cover",
    )
    for side, y_pos in (("left", 0.0525), ("right", -0.0525)):
        shank_link.visual(
            Box((0.095, 0.023, 0.095)),
            origin=Origin(xyz=(0.0, y_pos, -0.35)),
            material=link_grey,
            name=f"ankle_{side}_cheek",
        )
    shank_link.visual(
        Box((0.095, 0.072, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.325)),
        material=dark_trim,
        name="ankle_bridge",
    )
    shank_link.inertial = Inertial.from_geometry(
        Box((0.11, 0.11, 0.40)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, -0.20)),
    )

    foot_module = model.part("foot_module")
    foot_module.visual(
        Box((0.09, 0.06, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material=link_grey,
        name="ankle_hub",
    )
    for side, y_pos in (("left", 0.029), ("right", -0.029)):
        foot_module.visual(
            Box((0.05, 0.024, 0.05)),
            origin=Origin(xyz=(0.0, y_pos, -0.0125)),
            material=dark_trim,
            name=f"ankle_{side}_trunnion",
        )
    foot_module.visual(
        Box((0.08, 0.095, 0.05)),
        origin=Origin(xyz=(-0.02, 0.0, -0.08)),
        material=dark_trim,
        name="heel_block",
    )
    foot_module.visual(
        Box((0.14, 0.11, 0.035)),
        origin=Origin(xyz=(0.045, 0.0, -0.095)),
        material=link_grey,
        name="mid_sole",
    )
    foot_module.visual(
        Box((0.08, 0.10, 0.024)),
        origin=Origin(xyz=(0.145, 0.0, -0.0865)),
        material=link_grey,
        name="toe_block",
    )
    foot_module.inertial = Inertial.from_geometry(
        Box((0.23, 0.11, 0.13)),
        mass=4.0,
        origin=Origin(xyz=(0.06, 0.0, -0.08)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_leg_housing,
        child=thigh_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.5,
            lower=math.radians(-35.0),
            upper=math.radians(75.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.0, 0.0, -0.39)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(130.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=foot_module,
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=3.5,
            lower=math.radians(-35.0),
            upper=math.radians(45.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    upper_leg_housing = object_model.get_part("upper_leg_housing")
    thigh_link = object_model.get_part("thigh_link")
    shank_link = object_model.get_part("shank_link")
    foot_module = object_model.get_part("foot_module")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    ctx.expect_gap(
        upper_leg_housing,
        thigh_link,
        axis="z",
        positive_elem="hip_bridge",
        negative_elem="hip_hub",
        max_gap=0.03,
        max_penetration=0.0,
        name="hip hub seats just below housing bridge",
    )
    ctx.expect_overlap(
        upper_leg_housing,
        thigh_link,
        axes="xy",
        elem_a="hip_bridge",
        elem_b="hip_hub",
        min_overlap=0.06,
        name="hip hub stays captured inside housing footprint",
    )
    ctx.expect_gap(
        thigh_link,
        shank_link,
        axis="z",
        positive_elem="knee_bridge",
        negative_elem="knee_hub",
        max_gap=0.03,
        max_penetration=0.0,
        name="knee hub seats just below thigh bridge",
    )
    ctx.expect_overlap(
        thigh_link,
        shank_link,
        axes="xy",
        elem_a="knee_bridge",
        elem_b="knee_hub",
        min_overlap=0.055,
        name="knee hub stays captured inside thigh footprint",
    )
    ctx.expect_gap(
        shank_link,
        foot_module,
        axis="z",
        positive_elem="ankle_bridge",
        negative_elem="ankle_hub",
        max_gap=0.03,
        max_penetration=0.0,
        name="ankle hub seats just below shank bridge",
    )
    ctx.expect_overlap(
        shank_link,
        foot_module,
        axes="xy",
        elem_a="ankle_bridge",
        elem_b="ankle_hub",
        min_overlap=0.05,
        name="ankle hub stays captured inside shank footprint",
    )

    rest_aabb = ctx.part_element_world_aabb(thigh_link, elem="thigh_beam")
    with ctx.pose({hip_pitch: math.radians(35.0)}):
        flexed_aabb = ctx.part_element_world_aabb(thigh_link, elem="thigh_beam")
    if rest_aabb is not None and flexed_aabb is not None:
        rest_center_x = 0.5 * (rest_aabb[0][0] + rest_aabb[1][0])
        flexed_center_x = 0.5 * (flexed_aabb[0][0] + flexed_aabb[1][0])
        ctx.check(
            "positive hip pitch swings thigh forward",
            flexed_center_x > rest_center_x + 0.04,
            details=f"rest_center_x={rest_center_x}, flexed_center_x={flexed_center_x}",
        )
    else:
        ctx.fail("positive hip pitch swings thigh forward", "Could not measure thigh beam AABB.")

    knee_rest_aabb = ctx.part_element_world_aabb(shank_link, elem="shank_beam")
    with ctx.pose({knee_pitch: math.radians(65.0)}):
        knee_flexed_aabb = ctx.part_element_world_aabb(shank_link, elem="shank_beam")
    if knee_rest_aabb is not None and knee_flexed_aabb is not None:
        knee_rest_center_x = 0.5 * (knee_rest_aabb[0][0] + knee_rest_aabb[1][0])
        knee_flexed_center_x = 0.5 * (knee_flexed_aabb[0][0] + knee_flexed_aabb[1][0])
        ctx.check(
            "positive knee pitch swings shank forward",
            knee_flexed_center_x > knee_rest_center_x + 0.08,
            details=f"rest_center_x={knee_rest_center_x}, flexed_center_x={knee_flexed_center_x}",
        )
    else:
        ctx.fail("positive knee pitch swings shank forward", "Could not measure shank beam AABB.")

    toe_rest_aabb = ctx.part_element_world_aabb(foot_module, elem="toe_block")
    with ctx.pose({ankle_pitch: math.radians(25.0)}):
        toe_lifted_aabb = ctx.part_element_world_aabb(foot_module, elem="toe_block")
    if toe_rest_aabb is not None and toe_lifted_aabb is not None:
        toe_rest_center_z = 0.5 * (toe_rest_aabb[0][2] + toe_rest_aabb[1][2])
        toe_lifted_center_z = 0.5 * (toe_lifted_aabb[0][2] + toe_lifted_aabb[1][2])
        ctx.check(
            "positive ankle pitch lifts the toe",
            toe_lifted_center_z > toe_rest_center_z + 0.03,
            details=f"rest_center_z={toe_rest_center_z}, lifted_center_z={toe_lifted_center_z}",
        )
    else:
        ctx.fail("positive ankle pitch lifts the toe", "Could not measure toe block AABB.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
