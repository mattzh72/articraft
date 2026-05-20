from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_bevel_miter_saw")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    tool_teal = model.material("tool_teal", rgba=(0.10, 0.45, 0.44, 1.0))
    guard_gray = model.material("guard_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    blade_silver = model.material("blade_silver", rgba=(0.90, 0.90, 0.88, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.48, 0.06)),
        origin=Origin(xyz=(0.03, 0.0, 0.03)),
        material=cast_aluminum,
        name="base_deck",
    )
    base.visual(
        Box((0.02, 0.10, 0.04)),
        origin=Origin(xyz=(-0.27, 0.0, 0.08)),
        material=cast_aluminum,
        name="rear_plinth",
    )
    base.visual(
        Box((0.02, 0.14, 0.07)),
        origin=Origin(xyz=(-0.265, -0.125, 0.095)),
        material=machined_steel,
        name="left_fence",
    )
    base.visual(
        Box((0.02, 0.14, 0.07)),
        origin=Origin(xyz=(-0.265, 0.125, 0.095)),
        material=machined_steel,
        name="right_fence",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.62, 0.48, 0.16)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.16, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=machined_steel,
        name="table_disc",
    )
    turntable.visual(
        Box((0.18, 0.12, 0.04)),
        origin=Origin(xyz=(-0.14, 0.0, 0.045)),
        material=cast_aluminum,
        name="trunnion_pedestal",
    )
    turntable.visual(
        Box((0.10, 0.03, 0.09)),
        origin=Origin(xyz=(-0.20, -0.07, 0.095)),
        material=cast_aluminum,
        name="left_trunnion_cheek",
    )
    turntable.visual(
        Box((0.10, 0.03, 0.09)),
        origin=Origin(xyz=(-0.20, 0.07, 0.095)),
        material=cast_aluminum,
        name="right_trunnion_cheek",
    )
    turntable.inertial = Inertial.from_geometry(
        Box((0.32, 0.32, 0.12)),
        mass=5.0,
        origin=Origin(xyz=(-0.03, 0.0, 0.06)),
    )

    arm = model.part("arm")
    arm.visual(
        Box((0.12, 0.10, 0.08)),
        origin=Origin(xyz=(0.02, 0.0, 0.04)),
        material=guard_gray,
        name="arm_yoke_base",
    )
    arm.visual(
        Box((0.09, 0.08, 0.06)),
        origin=Origin(xyz=(0.04, 0.0, 0.10)),
        material=guard_gray,
        name="bevel_carrier",
    )
    arm.visual(
        Cylinder(radius=0.03, length=0.11),
        origin=Origin(xyz=(-0.02, 0.0, 0.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_gray,
        name="chop_hinge_barrel",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.16, 0.18, 0.16)),
        mass=3.0,
        origin=Origin(xyz=(0.03, 0.0, 0.08)),
    )

    head = model.part("head")
    head.visual(
        Box((0.26, 0.08, 0.06)),
        origin=Origin(xyz=(0.175, 0.0, -0.07)),
        material=tool_teal,
        name="upper_arm_shell",
    )
    head.visual(
        Box((0.12, 0.18, 0.16)),
        origin=Origin(xyz=(0.28, 0.0, -0.09)),
        material=guard_gray,
        name="gear_case",
    )
    head.visual(
        Cylinder(radius=0.07, length=0.14),
        origin=Origin(xyz=(0.22, 0.10, -0.08), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tool_teal,
        name="motor_can",
    )
    head.visual(
        Cylinder(radius=0.15, length=0.07),
        origin=Origin(xyz=(0.35, 0.0, -0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_gray,
        name="blade_guard_hood",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.14),
        origin=Origin(xyz=(0.14, 0.0, -0.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="carry_handle",
    )
    head.visual(
        Cylinder(radius=0.127, length=0.004),
        origin=Origin(xyz=(0.36, 0.0, -0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blade_silver,
        name="blade_disc",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.34, 0.24, 0.24)),
        mass=6.5,
        origin=Origin(xyz=(0.17, 0.02, -0.08)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-math.radians(50.0),
            upper=math.radians(50.0),
        ),
    )

    model.articulation(
        "turntable_to_arm",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=arm,
        origin=Origin(xyz=(-0.20, 0.0, 0.105), rpy=(0.0, -0.22, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.1,
            lower=0.0,
            upper=1.15,
        ),
    )

    model.articulation(
        "arm_to_head",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        origin=Origin(xyz=(0.04, 0.0, 0.11)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-math.radians(48.0),
            upper=math.radians(48.0),
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

    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")

    table_joint = object_model.get_articulation("base_to_turntable")
    chop_joint = object_model.get_articulation("turntable_to_arm")
    bevel_joint = object_model.get_articulation("arm_to_head")

    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="table_disc",
        negative_elem="base_deck",
        name="turntable sits on the base deck",
    )
    ctx.expect_overlap(
        turntable,
        base,
        axes="xy",
        min_overlap=0.20,
        elem_a="table_disc",
        elem_b="base_deck",
        name="turntable remains centered over the base footprint",
    )

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        part = object_model.get_part(part_name)
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_blade = elem_center("head", "blade_disc")
    with ctx.pose({table_joint: math.radians(40.0)}):
        mitered_blade = elem_center("head", "blade_disc")
    ctx.check(
        "miter table yaws the blade around the vertical axis",
        rest_blade is not None
        and mitered_blade is not None
        and abs(mitered_blade[1] - rest_blade[1]) > 0.12,
        details=f"rest={rest_blade}, mitered={mitered_blade}",
    )

    with ctx.pose({chop_joint: 0.0}):
        raised_blade = elem_center("head", "blade_disc")
    with ctx.pose({chop_joint: 0.95}):
        lowered_blade = elem_center("head", "blade_disc")
    ctx.check(
        "chop hinge lowers the blade toward the table",
        raised_blade is not None
        and lowered_blade is not None
        and lowered_blade[2] < raised_blade[2] - 0.10,
        details=f"raised={raised_blade}, lowered={lowered_blade}",
    )

    with ctx.pose({bevel_joint: math.radians(-35.0)}):
        left_bevel = elem_center("head", "blade_disc")
    with ctx.pose({bevel_joint: math.radians(35.0)}):
        right_bevel = elem_center("head", "blade_disc")
    ctx.check(
        "bevel hinge tilts the blade to both sides",
        left_bevel is not None
        and right_bevel is not None
        and left_bevel[1] < -0.03
        and right_bevel[1] > 0.03,
        details=f"left={left_bevel}, right={right_bevel}",
    )

    ctx.expect_origin_distance(
        head,
        arm,
        axes="xz",
        min_dist=0.08,
        max_dist=0.35,
        name="head sits forward of the bevel carrier",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
