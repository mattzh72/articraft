from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_gimbal_joystick")

    housing = model.material("powder_coated_housing", rgba=(0.10, 0.11, 0.12, 1.0))
    top_panel = model.material("satin_black_panel", rgba=(0.015, 0.017, 0.020, 1.0))
    dark_metal = model.material("dark_burnished_metal", rgba=(0.06, 0.065, 0.070, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    lever_mat = model.material("black_anodized_lever", rgba=(0.02, 0.023, 0.026, 1.0))
    cap_mat = model.material("matte_block_cap", rgba=(0.18, 0.19, 0.20, 1.0))
    screw_mat = model.material("black_oxide_screws", rgba=(0.01, 0.01, 0.012, 1.0))

    pitch_bearing_ring = mesh_from_geometry(
        TorusGeometry(radius=0.014, tube=0.003, radial_segments=14, tubular_segments=36),
        "pitch_bearing_ring",
    )
    roll_bearing_ring = mesh_from_geometry(
        TorusGeometry(radius=0.012, tube=0.0025, radial_segments=14, tubular_segments=36),
        "roll_bearing_ring",
    )

    base = model.part("base_housing")
    base.visual(
        Box((0.240, 0.240, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=housing,
        name="boxed_body",
    )
    base.visual(
        Box((0.205, 0.205, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=top_panel,
        name="top_plate",
    )
    base.visual(
        Box((0.152, 0.152, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_metal,
        name="recessed_gimbal_well",
    )
    for index, (x, y) in enumerate(
        ((0.083, 0.083), (-0.083, 0.083), (-0.083, -0.083), (0.083, -0.083))
    ):
        base.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, y, 0.0735)),
            material=screw_mat,
            name=f"screw_{index}",
        )

    pitch_support_names = ("pitch_support_0", "pitch_support_1")
    for index, x in enumerate((-0.095, 0.095)):
        base.visual(
            Box((0.026, 0.076, 0.112)),
            origin=Origin(xyz=(x, 0.0, 0.134)),
            material=housing,
            name=pitch_support_names[index],
        )
        base.visual(
            Box((0.050, 0.092, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.084)),
            material=dark_metal,
            name=f"support_foot_{index}",
        )
        base.visual(
            pitch_bearing_ring,
            origin=Origin(xyz=(x * 0.8632, 0.0, 0.150), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"pitch_bearing_{index}",
        )

    outer_yoke = model.part("outer_yoke")
    pitch_trunnion_names = ("pitch_trunnion_0", "pitch_trunnion_1")
    for index, x in enumerate((-0.065, 0.065)):
        outer_yoke.visual(
            Cylinder(radius=0.0065, length=0.034),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=pitch_trunnion_names[index],
        )
    for index, x in enumerate((-0.041, 0.041)):
        outer_yoke.visual(
            Box((0.014, 0.112, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=dark_metal,
            name=f"side_rail_{index}",
        )
    roll_bearing_block_names = ("roll_bearing_block_0", "roll_bearing_block_1")
    for index, y in enumerate((-0.056, 0.056)):
        outer_yoke.visual(
            Box((0.096, 0.018, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_metal,
            name=roll_bearing_block_names[index],
        )
        outer_yoke.visual(
            roll_bearing_ring,
            origin=Origin(xyz=(0.0, y * 0.8393, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"roll_bearing_{index}",
        )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Cylinder(radius=0.005, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="roll_shaft",
    )
    inner_cradle.visual(
        Box((0.038, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="central_hub",
    )
    for index, x in enumerate((-0.020, 0.020)):
        inner_cradle.visual(
            Box((0.008, 0.030, 0.034)),
            origin=Origin(xyz=(x, 0.0, 0.016)),
            material=dark_metal,
            name=f"cradle_cheek_{index}",
        )
    inner_cradle.visual(
        Cylinder(radius=0.0065, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=lever_mat,
        name="short_lever",
    )
    inner_cradle.visual(
        Box((0.034, 0.034, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=cap_mat,
        name="block_cap",
    )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.48, upper=0.48),
    )
    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-0.48, upper=0.48),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_cradle = object_model.get_part("inner_cradle")
    pitch = object_model.get_articulation("pitch_axis")
    roll = object_model.get_articulation("roll_axis")

    ctx.check(
        "orthogonal two-axis gimbal",
        pitch.axis == (1.0, 0.0, 0.0)
        and roll.axis == (0.0, 1.0, 0.0)
        and pitch.motion_limits.lower < 0.0
        and pitch.motion_limits.upper > 0.0
        and roll.motion_limits.lower < 0.0
        and roll.motion_limits.upper > 0.0,
        details=f"pitch_axis={pitch.axis}, roll_axis={roll.axis}",
    )
    ctx.expect_contact(
        outer_yoke,
        base,
        elem_a="pitch_trunnion_0",
        elem_b="pitch_support_0",
        contact_tol=0.001,
        name="outer yoke trunnion seats in pitch support",
    )
    ctx.expect_contact(
        inner_cradle,
        outer_yoke,
        elem_a="roll_shaft",
        elem_b="roll_bearing_block_0",
        contact_tol=0.001,
        name="inner cradle shaft seats in roll bearing",
    )
    ctx.expect_gap(
        inner_cradle,
        outer_yoke,
        axis="z",
        positive_elem="block_cap",
        negative_elem="roll_bearing_block_0",
        min_gap=0.075,
        name="block cap stands above the gimbal cradle",
    )

    def element_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_cap = element_center(inner_cradle, "block_cap")
    with ctx.pose({pitch: 0.36}):
        pitched_cap = element_center(inner_cradle, "block_cap")
    with ctx.pose({roll: 0.36}):
        rolled_cap = element_center(inner_cradle, "block_cap")
    ctx.check(
        "pitch joint tilts the capped lever fore-aft",
        rest_cap is not None and pitched_cap is not None and abs(pitched_cap[1] - rest_cap[1]) > 0.025,
        details=f"rest={rest_cap}, pitched={pitched_cap}",
    )
    ctx.check(
        "roll joint tilts the capped lever sideways",
        rest_cap is not None and rolled_cap is not None and abs(rolled_cap[0] - rest_cap[0]) > 0.025,
        details=f"rest={rest_cap}, rolled={rolled_cap}",
    )
    with ctx.pose({pitch: 0.42, roll: 0.42}):
        ctx.expect_gap(
            inner_cradle,
            base,
            axis="z",
            positive_elem="block_cap",
            negative_elem="top_plate",
            min_gap=0.12,
            name="tilted cap clears the boxed base",
        )

    return ctx.report()


object_model = build_object_model()
