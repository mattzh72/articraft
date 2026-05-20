from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_sliding_security_gate", assets=ASSETS)

    support_paint = model.material("support_paint", rgba=(0.22, 0.23, 0.24, 1.0))
    gate_paint = model.material("gate_paint", rgba=(0.24, 0.31, 0.25, 1.0))
    galvanized = model.material("galvanized", rgba=(0.66, 0.68, 0.70, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.74, 0.76, 0.79, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.80, 0.82, 0.84, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.88, 0.74, 0.12, 1.0))

    leaf_width = 1.44
    leaf_height = 1.86
    leaf_depth = 0.05
    slide_travel = 1.36
    gate_origin = (0.16, 0.0, 0.18)
    wheel_radius = 0.055
    wheel_width = 0.035

    def add_bolt_head(
        part,
        *,
        name: str,
        xyz: tuple[float, float, float],
        axis: str = "x",
        radius: float = 0.008,
        length: float = 0.012,
    ) -> None:
        if axis == "x":
            rpy = (0.0, math.pi / 2.0, 0.0)
        elif axis == "y":
            rpy = (math.pi / 2.0, 0.0, 0.0)
        else:
            rpy = (0.0, 0.0, 0.0)
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=fastener_steel,
            name=name,
        )

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((3.38, 0.50, 0.04)),
        origin=Origin(xyz=(1.69, 0.0, 0.02)),
        material=galvanized,
        name="foundation_block",
    )
    support_frame.visual(
        Box((3.14, 0.10, 0.02)),
        origin=Origin(xyz=(1.63, 0.0, 0.05)),
        material=support_paint,
        name="floor_channel_base",
    )
    support_frame.visual(
        Box((3.14, 0.02, 0.06)),
        origin=Origin(xyz=(1.63, -0.04, 0.09)),
        material=support_paint,
        name="floor_channel_back_wall",
    )
    support_frame.visual(
        Box((3.14, 0.02, 0.06)),
        origin=Origin(xyz=(1.63, 0.04, 0.09)),
        material=support_paint,
        name="floor_channel_front_wall",
    )
    support_frame.visual(
        Box((0.14, 0.16, 2.24)),
        origin=Origin(xyz=(0.07, 0.0, 1.12)),
        material=support_paint,
        name="latch_post",
    )
    support_frame.visual(
        Box((0.18, 0.20, 2.24)),
        origin=Origin(xyz=(3.29, 0.0, 1.12)),
        material=support_paint,
        name="storage_post",
    )
    support_frame.visual(
        Box((3.38, 0.20, 0.12)),
        origin=Origin(xyz=(1.69, 0.0, 2.26)),
        material=support_paint,
        name="top_support_beam",
    )
    support_frame.visual(
        Box((3.10, 0.14, 0.02)),
        origin=Origin(xyz=(1.61, 0.0, 2.19)),
        material=galvanized,
        name="rail_top_plate",
    )
    support_frame.visual(
        Box((3.10, 0.02, 0.10)),
        origin=Origin(xyz=(1.61, -0.06, 2.13)),
        material=galvanized,
        name="rail_back_wall",
    )
    support_frame.visual(
        Box((3.10, 0.02, 0.10)),
        origin=Origin(xyz=(1.61, 0.06, 2.13)),
        material=galvanized,
        name="rail_front_wall",
    )
    support_frame.visual(
        Box((0.20, 0.05, 0.20)),
        origin=Origin(xyz=(0.18, 0.0, 2.15)),
        material=support_paint,
        name="left_beam_brace",
    )
    support_frame.visual(
        Box((0.20, 0.05, 0.20)),
        origin=Origin(xyz=(3.10, 0.0, 2.15)),
        material=support_paint,
        name="right_beam_brace",
    )
    support_frame.visual(
        Box((0.02, 0.05, 0.12)),
        origin=Origin(xyz=(0.14, 0.065, 1.23)),
        material=galvanized,
        name="keeper_back",
    )
    support_frame.visual(
        Box((0.08, 0.05, 0.02)),
        origin=Origin(xyz=(0.11, 0.065, 1.28)),
        material=galvanized,
        name="keeper_top",
    )
    support_frame.visual(
        Box((0.08, 0.05, 0.02)),
        origin=Origin(xyz=(0.11, 0.065, 1.18)),
        material=galvanized,
        name="keeper_bottom",
    )
    support_frame.visual(
        Box((0.02, 0.05, 0.12)),
        origin=Origin(xyz=(0.08, 0.065, 1.23)),
        material=galvanized,
        name="keeper_front_lip",
    )
    support_frame.visual(
        Box((0.09, 0.10, 0.16)),
        origin=Origin(xyz=(3.07, 0.0, 2.08)),
        material=galvanized,
        name="rail_end_stop",
    )
    support_frame.visual(
        Box((0.10, 0.14, 0.20)),
        origin=Origin(xyz=(3.12, 0.0, 0.14)),
        material=support_paint,
        name="ground_end_stop",
    )

    for idx, z in enumerate((1.86, 2.10), start=1):
        add_bolt_head(
            support_frame,
            name=f"left_post_bolt_{idx}",
            xyz=(0.145, 0.07, z),
            axis="x",
            radius=0.009,
            length=0.014,
        )
        add_bolt_head(
            support_frame,
            name=f"right_post_bolt_{idx}",
            xyz=(3.193, 0.08, z),
            axis="x",
            radius=0.009,
            length=0.014,
        )
    for idx, (x, z) in enumerate(((0.11, 1.28), (0.11, 1.18)), start=1):
        add_bolt_head(
            support_frame,
            name=f"keeper_bolt_{idx}",
            xyz=(x, 0.097, z),
            axis="y",
            radius=0.007,
            length=0.014,
        )

    support_frame.inertial = Inertial.from_geometry(
        Box((3.38, 0.50, 2.34)),
        mass=420.0,
        origin=Origin(xyz=(1.69, 0.0, 1.17)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((leaf_width, leaf_depth, 0.08)),
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, 0.04)),
        material=gate_paint,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((leaf_width, leaf_depth, 0.08)),
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, leaf_height - 0.04)),
        material=gate_paint,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((0.08, leaf_depth, leaf_height - 0.16)),
        origin=Origin(xyz=(0.04, 0.0, leaf_height / 2.0)),
        material=gate_paint,
        name="leading_stile",
    )
    gate_leaf.visual(
        Box((0.08, leaf_depth, leaf_height - 0.16)),
        origin=Origin(xyz=(leaf_width - 0.04, 0.0, leaf_height / 2.0)),
        material=gate_paint,
        name="trailing_stile",
    )
    gate_leaf.visual(
        Box((0.06, leaf_depth, leaf_height - 0.16)),
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, leaf_height / 2.0)),
        material=gate_paint,
        name="center_stile",
    )
    gate_leaf.visual(
        Box((1.30, 0.04, 0.06)),
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, leaf_height / 2.0)),
        material=gate_paint,
        name="mid_rail",
    )
    gate_leaf.visual(
        Box((2.10, 0.03, 0.04)),
        origin=Origin(
            xyz=(leaf_width / 2.0, 0.0, leaf_height / 2.0),
            rpy=(0.0, -math.atan2(1.62, 1.28), 0.0),
        ),
        material=gate_paint,
        name="diagonal_brace",
    )

    for idx, x in enumerate((0.20, 0.34, 0.48, 0.96, 1.10, 1.24), start=1):
        gate_leaf.visual(
            Box((0.024, 0.018, 1.62)),
            origin=Origin(xyz=(x, 0.0, leaf_height / 2.0)),
            material=galvanized,
            name=f"infill_bar_{idx}",
        )

    gate_leaf.visual(
        Box((0.12, 0.01, 0.11)),
        origin=Origin(xyz=(0.72, 0.0, -0.055)),
        material=galvanized,
        name="bottom_guide_blade",
    )

    for prefix, x in (("front", 0.30), ("rear", 1.14)):
        gate_leaf.visual(
            Box((0.10, 0.012, 0.12)),
            origin=Origin(xyz=(x, -0.024, 1.92)),
            material=galvanized,
            name=f"{prefix}_hanger_back_plate",
        )
        gate_leaf.visual(
            Box((0.10, 0.012, 0.12)),
            origin=Origin(xyz=(x, 0.024, 1.92)),
            material=galvanized,
            name=f"{prefix}_hanger_front_plate",
        )
        gate_leaf.visual(
            Box((0.12, 0.05, 0.03)),
            origin=Origin(xyz=(x, 0.0, 1.875)),
            material=galvanized,
            name=f"{prefix}_hanger_cross_block",
        )

    gate_leaf.visual(
        Box((0.22, 0.01, 0.11)),
        origin=Origin(xyz=(0.11, 0.03, 1.05)),
        material=galvanized,
        name="latch_housing_back",
    )
    gate_leaf.visual(
        Box((0.22, 0.06, 0.01)),
        origin=Origin(xyz=(0.11, 0.06, 1.10)),
        material=galvanized,
        name="latch_housing_top",
    )
    gate_leaf.visual(
        Box((0.22, 0.06, 0.01)),
        origin=Origin(xyz=(0.11, 0.06, 1.00)),
        material=galvanized,
        name="latch_housing_bottom",
    )
    gate_leaf.visual(
        Box((0.03, 0.03, 0.32)),
        origin=Origin(xyz=(0.22, 0.070, 1.05)),
        material=galvanized,
        name="pull_handle",
    )

    for idx, x in enumerate((0.12, leaf_width - 0.12), start=1):
        gate_leaf.visual(
            Box((0.16, 0.012, 0.16)),
            origin=Origin(xyz=(x, 0.031, 0.14)),
            material=gate_paint,
            name=f"lower_corner_gusset_{idx}",
        )
        gate_leaf.visual(
            Box((0.16, 0.012, 0.16)),
            origin=Origin(xyz=(x, 0.031, leaf_height - 0.14)),
            material=gate_paint,
            name=f"upper_corner_gusset_{idx}",
        )

    for idx, (x, z) in enumerate(((0.08, 1.10), (0.14, 1.00)), start=1):
        add_bolt_head(
            gate_leaf,
            name=f"housing_bolt_{idx}",
            xyz=(x, 0.096, z),
            axis="y",
            radius=0.006,
            length=0.014,
        )
    for idx, x in enumerate((0.30, 1.14), start=1):
        add_bolt_head(
            gate_leaf,
            name=f"hanger_bolt_back_{idx}",
            xyz=(x, -0.031, 1.94),
            axis="y",
            radius=0.006,
            length=0.014,
        )
        add_bolt_head(
            gate_leaf,
            name=f"hanger_bolt_front_{idx}",
            xyz=(x, 0.031, 1.94),
            axis="y",
            radius=0.006,
            length=0.014,
        )

    gate_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, 0.10, 1.98)),
        mass=135.0,
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, 0.99)),
    )

    front_trolley_wheel = model.part("front_trolley_wheel")
    front_trolley_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="front_wheel_tire",
    )
    front_trolley_wheel.visual(
        Cylinder(radius=0.022, length=wheel_width + 0.008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="front_wheel_hub",
    )
    front_trolley_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=4.0,
    )

    rear_trolley_wheel = model.part("rear_trolley_wheel")
    rear_trolley_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="rear_wheel_tire",
    )
    rear_trolley_wheel.visual(
        Cylinder(radius=0.022, length=wheel_width + 0.008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="rear_wheel_hub",
    )
    rear_trolley_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=4.0,
    )

    latch_bolt = model.part("latch_bolt")
    latch_bolt.visual(
        Box((0.08, 0.03, 0.05)),
        origin=Origin(xyz=(-0.04, 0.0, 0.0)),
        material=galvanized,
        name="bolt_bar",
    )
    latch_bolt.visual(
        Box((0.018, 0.018, 0.07)),
        origin=Origin(xyz=(0.005, 0.024, 0.0)),
        material=safety_yellow,
        name="bolt_pull",
    )
    latch_bolt.inertial = Inertial.from_geometry(
        Box((0.09, 0.04, 0.06)),
        mass=0.8,
        origin=Origin(xyz=(-0.03, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=gate_leaf,
        origin=Origin(xyz=gate_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.35,
            lower=0.0,
            upper=slide_travel,
        ),
    )
    model.articulation(
        "gate_to_front_trolley_wheel",
        ArticulationType.CONTINUOUS,
        parent=gate_leaf,
        child=front_trolley_wheel,
        origin=Origin(xyz=(0.30, 0.0, 1.945)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )
    model.articulation(
        "gate_to_rear_trolley_wheel",
        ArticulationType.CONTINUOUS,
        parent=gate_leaf,
        child=rear_trolley_wheel,
        origin=Origin(xyz=(1.14, 0.0, 1.945)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )
    model.articulation(
        "gate_to_latch_bolt",
        ArticulationType.PRISMATIC,
        parent=gate_leaf,
        child=latch_bolt,
        origin=Origin(xyz=(0.07, 0.05, 1.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.15,
            lower=0.0,
            upper=0.07,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support_frame = object_model.get_part("support_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    front_trolley_wheel = object_model.get_part("front_trolley_wheel")
    rear_trolley_wheel = object_model.get_part("rear_trolley_wheel")
    latch_bolt = object_model.get_part("latch_bolt")

    slide_joint = object_model.get_articulation("support_to_gate_leaf")
    front_wheel_joint = object_model.get_articulation("gate_to_front_trolley_wheel")
    rear_wheel_joint = object_model.get_articulation("gate_to_rear_trolley_wheel")
    latch_joint = object_model.get_articulation("gate_to_latch_bolt")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "slide_axis_is_x",
        slide_joint.axis == (1.0, 0.0, 0.0),
        f"expected slide axis (1,0,0), got {slide_joint.axis}",
    )
    ctx.check(
        "front_trolley_axis_is_y",
        front_wheel_joint.axis == (0.0, 1.0, 0.0),
        f"expected wheel axis (0,1,0), got {front_wheel_joint.axis}",
    )
    ctx.check(
        "rear_trolley_axis_is_y",
        rear_wheel_joint.axis == (0.0, 1.0, 0.0),
        f"expected wheel axis (0,1,0), got {rear_wheel_joint.axis}",
    )
    ctx.check(
        "latch_axis_is_x",
        latch_joint.axis == (1.0, 0.0, 0.0),
        f"expected latch axis (1,0,0), got {latch_joint.axis}",
    )
    slide_limits = slide_joint.motion_limits
    ctx.check(
        "slide_travel_is_realistic",
        slide_limits is not None and slide_limits.upper == 1.36 and slide_limits.lower == 0.0,
        f"unexpected slide limits: {slide_limits}",
    )

    ctx.expect_contact(
        front_trolley_wheel,
        support_frame,
        elem_b="rail_top_plate",
        name="front_trolley_wheel_contacts_rail",
    )
    ctx.expect_contact(
        rear_trolley_wheel,
        support_frame,
        elem_b="rail_top_plate",
        name="rear_trolley_wheel_contacts_rail",
    )
    ctx.expect_within(
        gate_leaf,
        support_frame,
        axes="x",
        inner_elem="bottom_guide_blade",
        outer_elem="floor_channel_base",
        name="guide_blade_within_floor_channel_x",
    )
    ctx.expect_within(
        gate_leaf,
        support_frame,
        axes="y",
        inner_elem="bottom_guide_blade",
        outer_elem="floor_channel_base",
        name="guide_blade_within_floor_channel_y",
    )
    ctx.expect_gap(
        gate_leaf,
        support_frame,
        axis="z",
        positive_elem="bottom_guide_blade",
        negative_elem="floor_channel_base",
        min_gap=0.009,
        max_gap=0.011,
        name="guide_blade_clears_channel_base",
    )

    with ctx.pose({slide_joint: 0.0, latch_joint: 0.0}):
        ctx.expect_contact(
            latch_bolt,
            support_frame,
            elem_b="keeper_back",
            name="latch_bolt_seats_on_keeper_back",
        )
        ctx.expect_overlap(
            latch_bolt,
            support_frame,
            axes="yz",
            min_overlap=0.03,
            elem_b="keeper_back",
            name="latch_bolt_aligned_with_keeper",
        )

    latch_rest = ctx.part_world_position(latch_bolt)
    assert latch_rest is not None
    with ctx.pose({latch_joint: 0.07}):
        latch_retracted = ctx.part_world_position(latch_bolt)
        assert latch_retracted is not None
        ctx.check(
            "latch_retracts_rightward",
            latch_retracted[0] > latch_rest[0] + 0.06,
            f"rest={latch_rest}, retracted={latch_retracted}",
        )
        ctx.expect_gap(
            latch_bolt,
            support_frame,
            axis="x",
            min_gap=0.069,
            max_gap=0.071,
            negative_elem="keeper_back",
            name="latch_bolt_clears_keeper_when_retracted",
        )
        ctx.expect_within(
            latch_bolt,
            gate_leaf,
            axes="x",
            name="retracted_latch_stays_within_gate_envelope",
        )

    gate_closed = ctx.part_world_position(gate_leaf)
    assert gate_closed is not None
    with ctx.pose({slide_joint: 1.36}):
        gate_open = ctx.part_world_position(gate_leaf)
        assert gate_open is not None
        ctx.check(
            "gate_leaf_translates_full_travel",
            gate_open[0] > gate_closed[0] + 1.35,
            f"closed={gate_closed}, open={gate_open}",
        )
        ctx.expect_contact(
            front_trolley_wheel,
            support_frame,
            elem_b="rail_top_plate",
            name="front_trolley_wheel_contacts_rail_open",
        )
        ctx.expect_contact(
            rear_trolley_wheel,
            support_frame,
            elem_b="rail_top_plate",
            name="rear_trolley_wheel_contacts_rail_open",
        )
        ctx.expect_within(
            gate_leaf,
            support_frame,
            axes="x",
            inner_elem="bottom_guide_blade",
            outer_elem="floor_channel_base",
            name="guide_blade_within_floor_channel_x_open",
        )
        ctx.expect_within(
            gate_leaf,
            support_frame,
            axes="y",
            inner_elem="bottom_guide_blade",
            outer_elem="floor_channel_base",
            name="guide_blade_within_floor_channel_y_open",
        )

    with ctx.pose({front_wheel_joint: math.pi / 2.0, rear_wheel_joint: -math.pi / 2.0}):
        ctx.expect_contact(
            front_trolley_wheel,
            support_frame,
            elem_b="rail_top_plate",
            name="front_trolley_rotation_keeps_contact",
        )
        ctx.expect_contact(
            rear_trolley_wheel,
            support_frame,
            elem_b="rail_top_plate",
            name="rear_trolley_rotation_keeps_contact",
        )

    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({slide_joint: slide_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="slide_lower_no_floating")
        with ctx.pose({slide_joint: slide_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="slide_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
