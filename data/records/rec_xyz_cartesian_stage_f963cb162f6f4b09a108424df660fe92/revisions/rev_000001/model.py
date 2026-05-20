from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box_wp(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz)


def _build_base_body() -> cq.Workplane:
    plate = _box_wp((1.02, 0.42, 0.028), (0.0, 0.0, 0.014))
    left_beam = _box_wp((0.90, 0.06, 0.09), (0.0, -0.15, 0.073))
    right_beam = _box_wp((0.90, 0.06, 0.09), (0.0, 0.15, 0.073))
    left_end = _box_wp((0.07, 0.32, 0.10), (-0.415, 0.0, 0.078))
    right_end = _box_wp((0.07, 0.32, 0.10), (0.415, 0.0, 0.078))
    center_drive = _box_wp((0.82, 0.05, 0.045), (0.0, 0.0, 0.1005))
    left_drive_cover = _box_wp((0.09, 0.12, 0.07), (-0.47, 0.0, 0.085))
    right_drive_cover = _box_wp((0.08, 0.10, 0.06), (0.47, 0.0, 0.08))
    return (
        plate.union(left_beam)
        .union(right_beam)
        .union(left_end)
        .union(right_end)
        .union(center_drive)
        .union(left_drive_cover)
        .union(right_drive_cover)
    )


def _build_x_body() -> cq.Workplane:
    deck = _box_wp((0.44, 0.26, 0.014), (0.0, 0.0, -0.007))
    saddle = _box_wp((0.28, 0.18, 0.06), (0.0, 0.0, 0.03))
    left_web = _box_wp((0.32, 0.04, 0.03), (0.0, -0.09, 0.015))
    right_web = _box_wp((0.32, 0.04, 0.03), (0.0, 0.09, 0.015))
    return deck.union(saddle).union(left_web).union(right_web)


def _build_y_body() -> cq.Workplane:
    plate = _box_wp((0.22, 0.16, 0.014), (0.0, 0.0, -0.007))
    rear_column = _box_wp((0.12, 0.03, 0.30), (0.0, -0.05, 0.15))
    left_gusset = _box_wp((0.05, 0.06, 0.11), (-0.055, -0.035, 0.055))
    right_gusset = _box_wp((0.05, 0.06, 0.11), (0.055, -0.035, 0.055))
    upper_bridge = _box_wp((0.18, 0.04, 0.03), (0.0, -0.03, 0.285))
    return (
        plate.union(rear_column)
        .union(left_gusset)
        .union(right_gusset)
        .union(upper_bridge)
    )


def _build_z_body() -> cq.Workplane:
    left_runner_column = _box_wp((0.04, 0.02, 0.26), (-0.055, 0.019, 0.13))
    right_runner_column = _box_wp((0.04, 0.02, 0.26), (0.055, 0.019, 0.13))
    lower_tie = _box_wp((0.10, 0.014, 0.045), (0.0, -0.012, 0.055))
    center_post = _box_wp((0.08, 0.018, 0.09), (0.0, -0.01, 0.21))
    top_head = _box_wp((0.17, 0.05, 0.04), (0.0, 0.004, 0.282))
    return (
        left_runner_column.union(right_runner_column)
        .union(lower_tie)
        .union(center_post)
        .union(top_head)
    )


def _build_output_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.26, 0.18, 0.018)
        .translate((0.0, 0.02, 0.309))
        .faces(">Z")
        .workplane()
        .pushPoints(((-0.08, -0.05), (0.08, -0.05), (-0.08, 0.05), (0.08, 0.05)))
        .hole(0.012)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_plate_xyz_transfer_unit")

    model.material("frame_dark", color=(0.22, 0.24, 0.27, 1.0))
    model.material("machined_alloy", color=(0.73, 0.75, 0.78, 1.0))
    model.material("rail_steel", color=(0.38, 0.40, 0.43, 1.0))
    model.material("plate_alloy", color=(0.84, 0.85, 0.87, 1.0))

    base = model.part("base")
    base.visual(
        _mesh(_build_base_body(), "base_body"),
        material="frame_dark",
        name="base_body",
    )
    base.visual(
        Box((0.84, 0.028, 0.020)),
        origin=_origin((0.0, -0.125, 0.128)),
        material="rail_steel",
        name="x_rail_left",
    )
    base.visual(
        Box((0.84, 0.028, 0.020)),
        origin=_origin((0.0, 0.125, 0.128)),
        material="rail_steel",
        name="x_rail_right",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        _mesh(_build_x_body(), "x_carriage_body"),
        material="machined_alloy",
        name="x_body",
    )
    for name, center in {
        "x_block_left_front": (-0.14, -0.125, -0.017),
        "x_block_left_rear": (0.14, -0.125, -0.017),
        "x_block_right_front": (-0.14, 0.125, -0.017),
        "x_block_right_rear": (0.14, 0.125, -0.017),
    }.items():
        x_carriage.visual(
            Box((0.10, 0.055, 0.034)),
            origin=_origin(center),
            material="machined_alloy",
            name=name,
        )
    x_carriage.visual(
        Box((0.026, 0.30, 0.018)),
        origin=_origin((-0.065, 0.0, 0.069)),
        material="rail_steel",
        name="y_rail_left",
    )
    x_carriage.visual(
        Box((0.026, 0.30, 0.018)),
        origin=_origin((0.065, 0.0, 0.069)),
        material="rail_steel",
        name="y_rail_right",
    )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        Box((0.16, 0.05, 0.30)),
        origin=_origin((0.0, -0.055, 0.15)),
        material="machined_alloy",
        name="y_body",
    )
    y_carriage.visual(
        Box((0.20, 0.16, 0.014)),
        origin=_origin((0.0, 0.0, -0.007)),
        material="machined_alloy",
        name="y_deck",
    )
    y_carriage.visual(
        Box((0.18, 0.04, 0.026)),
        origin=_origin((0.0, -0.040, 0.287)),
        material="machined_alloy",
        name="y_top_cap",
    )
    for name, center in {
        "y_block_left_front": (-0.065, -0.055, -0.014),
        "y_block_left_rear": (-0.065, 0.055, -0.014),
        "y_block_right_front": (0.065, -0.055, -0.014),
        "y_block_right_rear": (0.065, 0.055, -0.014),
    }.items():
        y_carriage.visual(
            Box((0.06, 0.09, 0.028)),
            origin=_origin(center),
            material="machined_alloy",
            name=name,
        )
    y_carriage.visual(
        Box((0.025, 0.018, 0.24)),
        origin=_origin((-0.055, 0.010, 0.12)),
        material="rail_steel",
        name="z_rail_left",
    )
    y_carriage.visual(
        Box((0.025, 0.018, 0.24)),
        origin=_origin((0.055, 0.010, 0.12)),
        material="rail_steel",
        name="z_rail_right",
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.14, 0.014, 0.30)),
        origin=_origin((0.0, -0.015, 0.15)),
        material="machined_alloy",
        name="z_body",
    )
    z_carriage.visual(
        Box((0.14, 0.014, 0.03)),
        origin=_origin((0.0, -0.008, 0.045)),
        material="machined_alloy",
        name="z_lower_tie",
    )
    z_carriage.visual(
        Box((0.14, 0.014, 0.03)),
        origin=_origin((0.0, -0.008, 0.22)),
        material="machined_alloy",
        name="z_upper_tie",
    )
    z_carriage.visual(
        Box((0.12, 0.03, 0.025)),
        origin=_origin((0.0, 0.0, 0.2905)),
        material="machined_alloy",
        name="z_top_mount",
    )
    z_carriage.visual(
        Box((0.035, 0.014, 0.20)),
        origin=_origin((-0.055, -0.004, 0.11)),
        material="machined_alloy",
        name="z_runner_left",
    )
    z_carriage.visual(
        Box((0.035, 0.014, 0.20)),
        origin=_origin((0.055, -0.004, 0.11)),
        material="machined_alloy",
        name="z_runner_right",
    )

    output_plate = model.part("output_plate")
    output_plate.visual(
        Box((0.26, 0.18, 0.018)),
        origin=_origin((0.0, 0.0, 0.009)),
        material="plate_alloy",
        name="plate",
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=_origin((0.0, 0.0, 0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.8, lower=-0.22, upper=0.22),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=_origin((0.0, 0.0, 0.106)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.6, lower=-0.10, upper=0.10),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=_origin((0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.4, lower=0.0, upper=0.12),
    )
    model.articulation(
        "z_to_output_plate",
        ArticulationType.FIXED,
        parent=z_carriage,
        child=output_plate,
        origin=_origin((0.0, 0.02, 0.303)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")
    output_plate_part = object_model.get_part("output_plate")

    x_joint = object_model.get_articulation("base_to_x")
    y_joint = object_model.get_articulation("x_to_y")
    z_joint = object_model.get_articulation("y_to_z")

    base_body = base.get_visual("base_body")
    x_rail_left = base.get_visual("x_rail_left")
    x_rail_right = base.get_visual("x_rail_right")
    x_block_left_front = x_carriage.get_visual("x_block_left_front")
    x_block_right_rear = x_carriage.get_visual("x_block_right_rear")
    y_rail_left = x_carriage.get_visual("y_rail_left")
    y_rail_right = x_carriage.get_visual("y_rail_right")
    y_block_left_front = y_carriage.get_visual("y_block_left_front")
    y_block_right_rear = y_carriage.get_visual("y_block_right_rear")
    z_rail_left = y_carriage.get_visual("z_rail_left")
    z_rail_right = y_carriage.get_visual("z_rail_right")
    z_runner_left = z_carriage.get_visual("z_runner_left")
    z_runner_right = z_carriage.get_visual("z_runner_right")
    z_top_mount = z_carriage.get_visual("z_top_mount")
    output_plate = output_plate_part.get_visual("plate")

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

    ctx.expect_contact(
        x_carriage,
        base,
        elem_a=x_block_left_front,
        elem_b=x_rail_left,
        name="x_left_front_block_contacts_left_base_rail",
    )
    ctx.expect_contact(
        x_carriage,
        base,
        elem_a=x_block_right_rear,
        elem_b=x_rail_right,
        name="x_right_rear_block_contacts_right_base_rail",
    )
    ctx.expect_contact(
        y_carriage,
        x_carriage,
        elem_a=y_block_left_front,
        elem_b=y_rail_left,
        name="y_left_front_block_contacts_left_cross_rail",
    )
    ctx.expect_contact(
        y_carriage,
        x_carriage,
        elem_a=y_block_right_rear,
        elem_b=y_rail_right,
        name="y_right_rear_block_contacts_right_cross_rail",
    )
    ctx.expect_contact(
        z_carriage,
        y_carriage,
        elem_a=z_runner_left,
        elem_b=z_rail_left,
        name="z_left_runner_contacts_left_vertical_rail",
    )
    ctx.expect_contact(
        z_carriage,
        y_carriage,
        elem_a=z_runner_right,
        elem_b=z_rail_right,
        name="z_right_runner_contacts_right_vertical_rail",
    )
    ctx.expect_contact(
        output_plate_part,
        z_carriage,
        elem_a=output_plate,
        elem_b=z_top_mount,
        name="output_plate_is_mounted_to_vertical_slide",
    )
    ctx.expect_gap(
        output_plate_part,
        base,
        axis="z",
        min_gap=0.16,
        positive_elem=output_plate,
        negative_elem=base_body,
        name="output_plate_sits_well_above_base_frame",
    )

    def _check_positive_motion(part_obj, joint_obj, axis_index: int, min_delta: float, name: str) -> None:
        rest = ctx.part_world_position(part_obj)
        upper = None if joint_obj.motion_limits is None else joint_obj.motion_limits.upper
        moved = None
        if upper is not None:
            with ctx.pose({joint_obj: upper}):
                moved = ctx.part_world_position(part_obj)
        ok = (
            rest is not None
            and moved is not None
            and upper is not None
            and moved[axis_index] > rest[axis_index] + min_delta
        )
        ctx.check(name, ok, details=f"rest={rest}, moved={moved}, expected axis {axis_index} delta > {min_delta}")

    _check_positive_motion(
        x_carriage,
        x_joint,
        axis_index=0,
        min_delta=0.12,
        name="x_joint_positive_travel_moves_lower_carriage_along_x",
    )
    _check_positive_motion(
        y_carriage,
        y_joint,
        axis_index=1,
        min_delta=0.05,
        name="y_joint_positive_travel_moves_cross_slide_along_y",
    )
    _check_positive_motion(
        output_plate_part,
        z_joint,
        axis_index=2,
        min_delta=0.08,
        name="z_joint_positive_travel_lifts_output_plate_along_z",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
