from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_rotary_tilt_fixture")

    painted_cast = model.material("painted_cast_iron", color=(0.20, 0.22, 0.23, 1.0))
    dark_steel = model.material("dark_blued_steel", color=(0.05, 0.055, 0.06, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.62, 0.64, 0.62, 1.0))
    table_blue = model.material("machined_blue_table", color=(0.10, 0.23, 0.40, 1.0))
    rubber_black = model.material("black_recesses", color=(0.005, 0.005, 0.004, 1.0))

    support = model.part("side_support")
    # Fixed side-wall casting: a wall plate, projecting shelf, gusset webs, and
    # the stationary lower bearing pad.  +X is forward from the wall, +Z is up.
    support.visual(
        Box((0.050, 0.380, 0.500)),
        origin=Origin(xyz=(-0.025, 0.0, 0.250)),
        material=painted_cast,
        name="wall_plate",
    )
    support.visual(
        Box((0.200, 0.430, 0.035)),
        origin=Origin(xyz=(0.060, 0.0, 0.0175)),
        material=painted_cast,
        name="floor_foot",
    )
    support.visual(
        Box((0.310, 0.185, 0.040)),
        origin=Origin(xyz=(0.145, 0.0, 0.205)),
        material=painted_cast,
        name="projecting_shelf",
    )
    for idx, y in enumerate((-0.070, 0.070)):
        support.visual(
            Box((0.260, 0.022, 0.120)),
            origin=Origin(xyz=(0.115, y, 0.125)),
            material=painted_cast,
            name=f"shelf_web_{idx}",
        )
    support.visual(
        Cylinder(radius=0.076, length=0.025),
        origin=Origin(xyz=(0.250, 0.0, 0.2375)),
        material=dark_steel,
        name="fixed_bearing_pad",
    )

    rotary = model.part("rotary_stage")
    rotary.visual(
        Cylinder(radius=0.090, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=brushed_steel,
        name="rotary_disk",
    )
    rotary.visual(
        Cylinder(radius=0.062, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=dark_steel,
        name="spindle_collar",
    )
    rotary.visual(
        Cylinder(radius=0.036, length=0.124),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=brushed_steel,
        name="vertical_spindle",
    )
    rotary.visual(
        Box((0.300, 0.090, 0.042)),
        origin=Origin(xyz=(0.140, 0.0, 0.130)),
        material=painted_cast,
        name="forward_arm",
    )
    rotary.visual(
        Box((0.135, 0.320, 0.036)),
        origin=Origin(xyz=(0.285, 0.0, 0.098)),
        material=painted_cast,
        name="yoke_base",
    )
    rotary.visual(
        Box((0.115, 0.030, 0.182)),
        origin=Origin(xyz=(0.290, -0.145, 0.181)),
        material=painted_cast,
        name="yoke_cheek_0",
    )
    rotary.visual(
        Box((0.115, 0.030, 0.182)),
        origin=Origin(xyz=(0.290, 0.145, 0.181)),
        material=painted_cast,
        name="yoke_cheek_1",
    )
    rotary.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.290, -0.166, 0.230), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_boss_0",
    )
    rotary.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.290, 0.166, 0.230), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_boss_1",
    )

    table = model.part("tilt_table")
    table.visual(
        Box((0.365, 0.240, 0.035)),
        origin=Origin(xyz=(0.160, 0.0, 0.0225)),
        material=table_blue,
        name="table_slab",
    )
    table.visual(
        Cylinder(radius=0.018, length=0.305),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="trunnion_pin",
    )
    for idx, y in enumerate((-0.116, 0.116)):
        table.visual(
            Cylinder(radius=0.030, length=0.026),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"table_hub_{idx}",
        )
    for idx, y in enumerate((-0.060, 0.060)):
        table.visual(
            Box((0.300, 0.014, 0.004)),
            origin=Origin(xyz=(0.180, y, 0.042)),
            material=rubber_black,
            name=f"tee_slot_{idx}",
        )
    table.visual(
        Box((0.030, 0.190, 0.030)),
        origin=Origin(xyz=(0.010, 0.0, -0.025)),
        material=table_blue,
        name="underside_trunnion_rib",
    )

    model.articulation(
        "support_to_rotary",
        ArticulationType.REVOLUTE,
        parent=support,
        child=rotary,
        origin=Origin(xyz=(0.250, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.2, lower=-pi, upper=pi),
    )
    model.articulation(
        "rotary_to_table",
        ArticulationType.REVOLUTE,
        parent=rotary,
        child=table,
        origin=Origin(xyz=(0.290, 0.0, 0.230)),
        # The table extends along local +X; using -Y makes positive command tilt
        # the nose of the table upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.8, lower=-0.70, upper=0.90),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("side_support")
    rotary = object_model.get_part("rotary_stage")
    table = object_model.get_part("tilt_table")
    lower_joint = object_model.get_articulation("support_to_rotary")
    tilt_joint = object_model.get_articulation("rotary_to_table")

    for cheek in ("yoke_cheek_0", "yoke_cheek_1"):
        ctx.allow_overlap(
            rotary,
            table,
            elem_a=cheek,
            elem_b="trunnion_pin",
            reason=(
                "The table trunnion pin is intentionally captured inside the "
                "solid-proxy yoke cheek bore."
            ),
        )
    ctx.expect_contact(
        support,
        rotary,
        elem_a="fixed_bearing_pad",
        elem_b="rotary_disk",
        name="rotary disk seats on fixed bearing pad",
    )
    ctx.expect_overlap(
        rotary,
        table,
        axes="y",
        elem_a="yoke_base",
        elem_b="table_slab",
        min_overlap=0.20,
        name="tilt table sits between the yoke cheeks",
    )
    ctx.expect_within(
        table,
        rotary,
        axes="y",
        inner_elem="table_slab",
        outer_elem="yoke_base",
        margin=0.0,
        name="table width fits inside yoke span",
    )
    ctx.expect_overlap(
        table,
        rotary,
        axes="y",
        elem_a="trunnion_pin",
        elem_b="yoke_cheek_0",
        min_overlap=0.015,
        name="trunnion pin remains captured in first cheek",
    )
    ctx.expect_overlap(
        table,
        rotary,
        axes="y",
        elem_a="trunnion_pin",
        elem_b="yoke_cheek_1",
        min_overlap=0.015,
        name="trunnion pin remains captured in second cheek",
    )

    rest_aabb = ctx.part_world_aabb(table)
    with ctx.pose({tilt_joint: 0.55}):
        raised_aabb = ctx.part_world_aabb(table)
    ctx.check(
        "positive table tilt raises forward edge",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.08,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    rest_pos = ctx.part_world_position(table)
    with ctx.pose({lower_joint: pi / 2.0}):
        turned_pos = ctx.part_world_position(table)
    ctx.check(
        "lower stage rotates table around vertical axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(turned_pos[1] - rest_pos[1]) > 0.20,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
