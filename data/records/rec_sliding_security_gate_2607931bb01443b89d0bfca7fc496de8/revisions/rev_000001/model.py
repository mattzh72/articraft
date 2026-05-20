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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_visual(
    part,
    geometry,
    *,
    xyz=(0.0, 0.0, 0.0),
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
):
    return part.visual(
        geometry,
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_diagonal_box(
    part,
    start,
    end,
    *,
    thickness,
    depth,
    material=None,
    name=None,
    embed=0.05,
):
    x0, y0, z0 = start
    x1, y1, z1 = end
    dx = x1 - x0
    dy = y1 - y0
    dz = z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz) + embed
    yaw = math.atan2(dy, dx)
    planar = math.sqrt(dx * dx + dy * dy)
    pitch = -math.atan2(dz, planar)
    mid = ((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5)
    _add_visual(
        part,
        Box((length, depth, thickness)),
        xyz=mid,
        rpy=(0.0, pitch, yaw),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    steel_dark = model.material("steel_dark", rgba=(0.22, 0.24, 0.26, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.38, 0.40, 0.42, 1.0))
    galvanised = model.material("galvanised", rgba=(0.62, 0.65, 0.67, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    service_orange = model.material("service_orange", rgba=(0.82, 0.42, 0.10, 1.0))

    base = model.part("base_frame")

    # Reinforced ground beam with set-back posts so the leaf can slide clear.
    _add_visual(
        base,
        Box((10.4, 0.34, 0.22)),
        xyz=(-0.75, 0.0, 0.11),
        material="steel_mid",
        name="beam",
    )
    _add_visual(
        base,
        Box((0.20, 0.20, 2.38)),
        xyz=(0.06, -0.20, 1.41),
        material="steel_dark",
        name="guide_post",
    )
    _add_visual(
        base,
        Box((0.20, 0.20, 2.38)),
        xyz=(4.10, -0.20, 1.41),
        material="steel_dark",
        name="receiver_post",
    )
    _add_visual(
        base,
        Box((0.52, 0.12, 0.12)),
        xyz=(0.06, -0.10, 2.30),
        material="steel_dark",
        name="guide_post_cap",
    )
    _add_visual(
        base,
        Box((0.38, 0.12, 0.12)),
        xyz=(4.10, -0.10, 2.30),
        material="steel_dark",
        name="receiver_post_cap",
    )

    truck_front = model.part("truck_front")
    _add_visual(
        truck_front,
        Box((0.34, 0.26, 0.02)),
        xyz=(0.0, 0.0, 0.01),
        material="galvanised",
        name="mount_plate",
    )
    _add_visual(
        truck_front,
        Box((0.24, 0.02, 0.18)),
        xyz=(0.0, 0.11, 0.11),
        material="galvanised",
        name="outer_cheek",
    )
    _add_visual(
        truck_front,
        Box((0.24, 0.02, 0.18)),
        xyz=(0.0, -0.11, 0.11),
        material="galvanised",
        name="inner_cheek",
    )
    _add_visual(
        truck_front,
        Cylinder(radius=0.015, length=0.08),
        xyz=(0.0, 0.0, 0.14),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material="steel_dark",
        name="axle",
    )
    _add_visual(
        truck_front,
        Box((0.08, 0.04, 0.14)),
        xyz=(0.0, 0.0, 0.08),
        material="galvanised",
        name="hanger_block",
    )
    _add_visual(
        truck_front,
        Cylinder(radius=0.06, length=0.10),
        xyz=(0.0, 0.0, 0.14),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material="service_orange",
        name="roller",
    )

    truck_rear = model.part("truck_rear")
    _add_visual(
        truck_rear,
        Box((0.34, 0.26, 0.02)),
        xyz=(0.0, 0.0, 0.01),
        material="galvanised",
        name="mount_plate",
    )
    _add_visual(
        truck_rear,
        Box((0.24, 0.02, 0.18)),
        xyz=(0.0, 0.11, 0.11),
        material="galvanised",
        name="outer_cheek",
    )
    _add_visual(
        truck_rear,
        Box((0.24, 0.02, 0.18)),
        xyz=(0.0, -0.11, 0.11),
        material="galvanised",
        name="inner_cheek",
    )
    _add_visual(
        truck_rear,
        Cylinder(radius=0.015, length=0.08),
        xyz=(0.0, 0.0, 0.14),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material="steel_dark",
        name="axle",
    )
    _add_visual(
        truck_rear,
        Box((0.08, 0.04, 0.14)),
        xyz=(0.0, 0.0, 0.08),
        material="galvanised",
        name="hanger_block",
    )
    _add_visual(
        truck_rear,
        Cylinder(radius=0.06, length=0.10),
        xyz=(0.0, 0.0, 0.14),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material="service_orange",
        name="roller",
    )

    top_guide = model.part("top_guide")
    _add_visual(
        top_guide,
        Box((0.16, 0.02, 0.30)),
        xyz=(0.0, 0.01, 0.0),
        material="galvanised",
        name="back_plate",
    )
    _add_visual(
        top_guide,
        Box((0.12, 0.18, 0.05)),
        xyz=(0.0, 0.11, 0.09),
        material="galvanised",
        name="crosshead",
    )
    _add_visual(
        top_guide,
        Cylinder(radius=0.04, length=0.08),
        xyz=(0.0, 0.17, 0.0),
        material="service_orange",
        name="outer_roller",
    )
    _add_visual(
        top_guide,
        Box((0.06, 0.04, 0.03)),
        xyz=(0.0, 0.17, 0.055),
        material="galvanised",
        name="outer_roller_carrier",
    )
    _add_visual(
        top_guide,
        Cylinder(radius=0.04, length=0.08),
        xyz=(0.0, 0.03, 0.0),
        material="service_orange",
        name="inner_roller",
    )

    receiver_catch = model.part("receiver_catch")
    _add_visual(
        receiver_catch,
        Box((0.02, 0.18, 0.44)),
        xyz=(-0.01, 0.0, 0.0),
        material="galvanised",
        name="back_plate",
    )
    _add_visual(
        receiver_catch,
        Box((0.10, 0.03, 0.04)),
        xyz=(-0.05, 0.015, 0.13),
        material="galvanised",
        name="upper_jaw",
    )
    _add_visual(
        receiver_catch,
        Box((0.10, 0.03, 0.04)),
        xyz=(-0.05, 0.015, -0.13),
        material="galvanised",
        name="lower_jaw",
    )
    _add_visual(
        receiver_catch,
        Box((0.06, 0.03, 0.18)),
        xyz=(-0.03, 0.015, 0.0),
        material="rubber",
        name="keeper_bumper",
    )

    open_stop = model.part("open_stop")
    _add_visual(
        open_stop,
        Box((0.22, 0.18, 0.02)),
        xyz=(0.0, 0.0, 0.01),
        material="galvanised",
        name="mount_plate",
    )
    _add_visual(
        open_stop,
        Box((0.08, 0.18, 0.20)),
        xyz=(-0.05, 0.0, 0.11),
        material="galvanised",
        name="stop_block",
    )
    _add_visual(
        open_stop,
        Box((0.04, 0.12, 0.18)),
        xyz=(0.01, 0.0, 0.10),
        material="rubber",
        name="bumper",
    )

    leaf = model.part("gate_leaf")
    leaf_length = 5.53
    opening_start = 1.60
    channel_center_x = leaf_length * 0.5
    _add_visual(
        leaf,
        Box((leaf_length, 0.16, 0.02)),
        xyz=(channel_center_x, 0.0, 0.43),
        material="steel_dark",
        name="channel_web",
    )
    _add_visual(
        leaf,
        Box((leaf_length, 0.02, 0.16)),
        xyz=(channel_center_x, 0.07, 0.34),
        material="steel_dark",
        name="channel_outer_wall",
    )
    _add_visual(
        leaf,
        Box((leaf_length, 0.02, 0.16)),
        xyz=(channel_center_x, -0.07, 0.34),
        material="steel_dark",
        name="channel_inner_wall",
    )
    _add_visual(
        leaf,
        Box((leaf_length, 0.06, 0.08)),
        xyz=(channel_center_x, 0.0, 2.08),
        material="steel_dark",
        name="top_rail",
    )
    _add_visual(
        leaf,
        Box((0.08, 0.06, 1.60)),
        xyz=(0.04, 0.0, 1.24),
        material="steel_dark",
        name="tail_stile",
    )
    _add_visual(
        leaf,
        Box((0.08, 0.06, 1.60)),
        xyz=(opening_start, 0.0, 1.24),
        material="steel_dark",
        name="hinge_side_stile",
    )
    _add_visual(
        leaf,
        Box((0.08, 0.06, 1.60)),
        xyz=(leaf_length - 0.04, 0.0, 1.24),
        material="steel_dark",
        name="leading_stile",
    )

    # Counterbalance bracing and service-friendly extra upright.
    _add_visual(
        leaf,
        Box((0.06, 0.06, 1.60)),
        xyz=(0.78, 0.0, 1.24),
        material="steel_dark",
        name="tail_mid_stile",
    )
    _add_diagonal_box(
        leaf,
        (0.08, 0.0, 0.48),
        (opening_start - 0.04, 0.0, 2.04),
        thickness=0.04,
        depth=0.04,
        material="steel_mid",
        name="tail_brace_rise",
    )
    _add_diagonal_box(
        leaf,
        (0.08, 0.0, 2.04),
        (opening_start - 0.04, 0.0, 0.48),
        thickness=0.04,
        depth=0.04,
        material="steel_mid",
        name="tail_brace_fall",
    )

    # Security infill bars across the drive opening.
    picket_count = 15
    first_picket_x = opening_start + 0.23
    last_picket_x = leaf_length - 0.23
    spacing = (last_picket_x - first_picket_x) / (picket_count - 1)
    for i in range(picket_count):
        x = first_picket_x + i * spacing
        _add_visual(
            leaf,
            Box((0.03, 0.02, 1.60)),
            xyz=(x, 0.0, 1.24),
            material="steel_mid",
            name=f"picket_{i + 1:02d}",
        )

    _add_visual(
        leaf,
        Box((0.12, 0.08, 0.18)),
        xyz=(leaf_length - 0.05, -0.07, 1.16),
        material="service_orange",
        name="latch_nose",
    )
    _add_visual(
        leaf,
        Box((0.04, 0.10, 0.18)),
        xyz=(0.02, 0.0, 0.35),
        material="rubber",
        name="tail_stop_pad",
    )

    model.articulation(
        "base_to_truck_front",
        ArticulationType.FIXED,
        parent=base,
        child=truck_front,
        origin=Origin(xyz=(-1.05, 0.0, 0.22)),
    )
    model.articulation(
        "base_to_truck_rear",
        ArticulationType.FIXED,
        parent=base,
        child=truck_rear,
        origin=Origin(xyz=(-0.35, 0.0, 0.22)),
    )
    model.articulation(
        "base_to_top_guide",
        ArticulationType.FIXED,
        parent=base,
        child=top_guide,
        origin=Origin(xyz=(-0.12, -0.10, 2.08)),
    )
    model.articulation(
        "base_to_receiver_catch",
        ArticulationType.FIXED,
        parent=base,
        child=receiver_catch,
        origin=Origin(xyz=(4.00, -0.10, 1.16)),
    )
    model.articulation(
        "base_to_open_stop",
        ArticulationType.FIXED,
        parent=base,
        child=open_stop,
        origin=Origin(xyz=(-5.48, 0.0, 0.22)),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=leaf,
        origin=Origin(xyz=(-1.60, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.40,
            lower=0.0,
            upper=3.93,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    truck_front = object_model.get_part("truck_front")
    truck_rear = object_model.get_part("truck_rear")
    top_guide = object_model.get_part("top_guide")
    receiver_catch = object_model.get_part("receiver_catch")
    open_stop = object_model.get_part("open_stop")
    leaf = object_model.get_part("gate_leaf")
    gate_slide = object_model.get_articulation("gate_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(truck_front, base, elem_a="mount_plate", elem_b="beam")
    ctx.expect_contact(truck_rear, base, elem_a="mount_plate", elem_b="beam")
    ctx.expect_contact(top_guide, base, elem_a="back_plate", elem_b="guide_post")
    ctx.expect_contact(receiver_catch, base, elem_a="back_plate", elem_b="receiver_post")
    ctx.expect_contact(open_stop, base, elem_a="mount_plate", elem_b="beam")

    with ctx.pose({gate_slide: 0.0}):
        ctx.expect_contact(leaf, truck_front, elem_a="channel_web", elem_b="roller")
        ctx.expect_contact(leaf, truck_rear, elem_a="channel_web", elem_b="roller")
        ctx.expect_contact(leaf, top_guide, elem_a="top_rail", elem_b="inner_roller")
        ctx.expect_contact(leaf, top_guide, elem_a="top_rail", elem_b="outer_roller")
        ctx.expect_contact(leaf, receiver_catch, elem_a="latch_nose", elem_b="keeper_bumper")
        ctx.expect_gap(
            leaf,
            base,
            axis="z",
            min_gap=0.18,
            max_gap=0.22,
            positive_elem="channel_web",
            negative_elem="beam",
        )

    with ctx.pose({gate_slide: gate_slide.motion_limits.upper}):
        ctx.expect_contact(leaf, truck_front, elem_a="channel_web", elem_b="roller")
        ctx.expect_contact(leaf, truck_rear, elem_a="channel_web", elem_b="roller")
        ctx.expect_contact(leaf, top_guide, elem_a="top_rail", elem_b="inner_roller")
        ctx.expect_contact(leaf, top_guide, elem_a="top_rail", elem_b="outer_roller")
        ctx.expect_contact(leaf, open_stop, elem_a="tail_stop_pad", elem_b="bumper")
        ctx.expect_gap(
            receiver_catch,
            leaf,
            axis="x",
            min_gap=3.80,
            positive_elem="back_plate",
            negative_elem="leading_stile",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
