from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OPEN_ANGLE = math.radians(120.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_leaf_garden_gate")

    black_metal = Material("black_powder_coated_metal", rgba=(0.02, 0.025, 0.022, 1.0))
    hinge_metal = Material("slightly_worn_hinge_metal", rgba=(0.35, 0.36, 0.33, 1.0))
    concrete = Material("weathered_concrete", rgba=(0.45, 0.43, 0.39, 1.0))

    post_base = model.part("post_base")
    post_x = 1.58
    post_size = 0.12
    post_height = 1.90
    post_z = post_height / 2.0

    # A shallow concrete strip ties the two post footings together so the fixed
    # support is one grounded assembly rather than two visually floating posts.
    post_base.visual(
        Box((3.50, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=concrete,
        name="concrete_strip",
    )
    for side, x in (("post_0", -post_x), ("post_1", post_x)):
        post_base.visual(
            Box((0.34, 0.30, 0.12)),
            origin=Origin(xyz=(x, 0.0, 0.06)),
            material=concrete,
            name=f"{side}_footing",
        )
        post_base.visual(
            Box((post_size, post_size, post_height)),
            origin=Origin(xyz=(x, 0.0, post_z)),
            material=black_metal,
            name=f"{side}_upright",
        )
        post_base.visual(
            Box((0.16, 0.16, 0.06)),
            origin=Origin(xyz=(x, 0.0, post_height + 0.025)),
            material=black_metal,
            name=f"{side}_cap",
        )

    # Flat hinge leaves/plates are bolted to the inner faces of the posts.
    # The rotating hinge barrels live on each gate leaf on the same vertical axis.
    hinge_axis_x = 1.47
    hinge_plate_size_x = 0.029
    for x_sign, plate_names in (
        (-1.0, ("post_0_hinge_plate_0", "post_0_hinge_plate_1", "post_0_hinge_plate_2")),
        (1.0, ("post_1_hinge_plate_0", "post_1_hinge_plate_1", "post_1_hinge_plate_2")),
    ):
        # Each plate overlaps the square post by 4 mm and reaches to the tangent
        # of the rotating barrel, giving a real hinge contact path at q=0.
        plate_x = x_sign * (hinge_axis_x + 0.025 + hinge_plate_size_x / 2.0)
        for plate_name, z in zip(plate_names, (0.42, 0.88, 1.34)):
            post_base.visual(
                Box((hinge_plate_size_x, 0.15, 0.16)),
                origin=Origin(xyz=(plate_x, 0.0, z)),
                material=hinge_metal,
                name=plate_name,
            )

    tube_x = 0.06
    tube_y = 0.045
    leaf_width = 1.44
    leaf_bottom = 0.15
    leaf_height = 1.40
    leaf_center_z = leaf_bottom + leaf_height / 2.0
    leaf_top = leaf_bottom + leaf_height

    def add_leaf(part_name: str, direction: float):
        leaf = model.part(part_name)
        # direction = +1 extends the leaf along local +X from its hinge line;
        # direction = -1 mirrors it toward local -X.
        hinge_stile_x = direction * (tube_x * 0.75)
        meeting_stile_x = direction * (leaf_width - tube_x / 2.0)
        rail_center_x = direction * (leaf_width / 2.0)

        leaf.visual(
            Box((tube_x, tube_y, leaf_height)),
            origin=Origin(xyz=(hinge_stile_x, 0.0, leaf_center_z)),
            material=black_metal,
            name="hinge_stile",
        )
        leaf.visual(
            Box((tube_x, tube_y, leaf_height)),
            origin=Origin(xyz=(meeting_stile_x, 0.0, leaf_center_z)),
            material=black_metal,
            name="meeting_stile",
        )
        for name, z, rail_z in (
            ("bottom_rail", leaf_bottom + tube_x / 2.0, tube_x),
            ("middle_rail_0", 0.62, 0.04),
            ("middle_rail_1", 1.08, 0.04),
            ("top_rail", leaf_top - tube_x / 2.0, tube_x),
        ):
            leaf.visual(
                Box((leaf_width, tube_y, rail_z)),
                origin=Origin(xyz=(rail_center_x, 0.0, z)),
                material=black_metal,
                name=name,
            )

        # Three short barrels make the visible hinge set and are connected to the
        # hinge stile by small straps welded/bolted to the leaf frame.
        for barrel_name, strap_name, z in (
            ("hinge_barrel_0", "hinge_strap_0", 0.42),
            ("hinge_barrel_1", "hinge_strap_1", 0.88),
            ("hinge_barrel_2", "hinge_strap_2", 1.34),
        ):
            leaf.visual(
                Cylinder(radius=0.025, length=0.18),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=hinge_metal,
                name=barrel_name,
            )
            leaf.visual(
                Box((0.10, 0.055, 0.055)),
                origin=Origin(xyz=(direction * 0.045, 0.0, z)),
                material=hinge_metal,
                name=strap_name,
            )

        return leaf

    leaf_0 = add_leaf("leaf_0", 1.0)
    leaf_1 = add_leaf("leaf_1", -1.0)

    model.articulation(
        "post_to_leaf_0",
        ArticulationType.REVOLUTE,
        parent=post_base,
        child=leaf_0,
        origin=Origin(xyz=(-hinge_axis_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=OPEN_ANGLE),
    )
    model.articulation(
        "post_to_leaf_1",
        ArticulationType.REVOLUTE,
        parent=post_base,
        child=leaf_1,
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=OPEN_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    leaf_0 = object_model.get_part("leaf_0")
    leaf_1 = object_model.get_part("leaf_1")
    post_base = object_model.get_part("post_base")
    hinge_0 = object_model.get_articulation("post_to_leaf_0")
    hinge_1 = object_model.get_articulation("post_to_leaf_1")

    for hinge, expected_sign in ((hinge_0, 1.0), (hinge_1, -1.0)):
        limits = hinge.motion_limits
        ctx.check(
            f"{hinge.name} is a vertical 120 degree revolute hinge",
            hinge.articulation_type == ArticulationType.REVOLUTE
            and abs(hinge.axis[0]) < 1e-6
            and abs(hinge.axis[1]) < 1e-6
            and abs(hinge.axis[2] - expected_sign) < 1e-6
            and limits is not None
            and limits.lower == 0.0
            and abs(limits.upper - OPEN_ANGLE) < 1e-6,
            details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={limits}",
        )

    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        ctx.expect_gap(
            leaf_1,
            leaf_0,
            axis="x",
            min_gap=0.04,
            max_gap=0.12,
            name="closed leaves meet with a narrow center gap",
        )
        ctx.expect_contact(
            post_base,
            leaf_0,
            elem_a="post_0_hinge_plate_1",
            elem_b="hinge_barrel_1",
            contact_tol=0.002,
            name="leaf_0 hinge barrel is mounted to its post plate",
        )
        ctx.expect_contact(
            post_base,
            leaf_1,
            elem_a="post_1_hinge_plate_1",
            elem_b="hinge_barrel_1",
            contact_tol=0.002,
            name="leaf_1 hinge barrel is mounted to its post plate",
        )
        closed_0 = ctx.part_element_world_aabb(leaf_0, elem="meeting_stile")
        closed_1 = ctx.part_element_world_aabb(leaf_1, elem="meeting_stile")

    with ctx.pose({hinge_0: OPEN_ANGLE, hinge_1: OPEN_ANGLE}):
        open_0 = ctx.part_element_world_aabb(leaf_0, elem="meeting_stile")
        open_1 = ctx.part_element_world_aabb(leaf_1, elem="meeting_stile")

    def center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) / 2.0

    closed_0_y = center_y(closed_0)
    closed_1_y = center_y(closed_1)
    open_0_y = center_y(open_0)
    open_1_y = center_y(open_1)
    ctx.check(
        "both leaves swing outward together",
        closed_0_y is not None
        and closed_1_y is not None
        and open_0_y is not None
        and open_1_y is not None
        and open_0_y > closed_0_y + 0.75
        and open_1_y > closed_1_y + 0.75,
        details=(
            f"closed_y=({closed_0_y}, {closed_1_y}), "
            f"open_y=({open_0_y}, {open_1_y})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
