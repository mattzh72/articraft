from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulationType,
    ArticulatedObject,
    AssetContext,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (-pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_bolt(
    part,
    *,
    name: str,
    center: tuple[float, float, float],
    axis: str = "z",
    radius: float = 0.003,
    length: float = 0.004,
    material: str = "fastener_steel",
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def _hinge_sleeve_mesh(filename: str, *, outer_radius: float, inner_radius: float, length: float):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(-0.0 + outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
            [(-0.0 + inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
            segments=40,
            start_cap="flat",
            end_cap="flat",
        ),
        ASSETS.mesh_path(filename),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_watch_winder_box", assets=ASSETS)

    model.material("utility_body", rgba=(0.24, 0.27, 0.23, 1.0))
    model.material("liner_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("rubber_foot", rgba=(0.08, 0.08, 0.08, 1.0))
    model.material("hinge_steel", rgba=(0.50, 0.53, 0.56, 1.0))
    model.material("fastener_steel", rgba=(0.36, 0.38, 0.40, 1.0))
    model.material("bearing_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("clear_window", rgba=(0.70, 0.78, 0.82, 0.32))
    model.material("cradle_pad", rgba=(0.28, 0.22, 0.16, 1.0))
    model.material("foam_pad", rgba=(0.13, 0.13, 0.14, 1.0))

    width = 0.240
    depth = 0.180
    base_height = 0.120
    wall_t = 0.012
    floor_t = 0.014
    rim_t = 0.008

    lid_outer_w = 0.244
    lid_frame_t = 0.014
    lid_frame_h = 0.046
    lid_bottom_z = -0.004
    lid_frame_z = lid_bottom_z + lid_frame_h / 2.0
    lid_back_rail_y = 0.022
    lid_front_rail_y = 0.177
    lid_side_rail_y = (lid_back_rail_y + lid_front_rail_y) / 2.0
    lid_side_rail_d = lid_front_rail_y - lid_back_rail_y
    hinge_axis_y = -depth / 2.0 - 0.005
    hinge_axis_z = base_height + 0.004

    cradle_axis_z = 0.064
    hinge_pin_radius = 0.004
    hinge_knuckle_outer_r = 0.007
    hinge_knuckle_len = 0.040
    hinge_knuckle_mesh = _hinge_sleeve_mesh(
        "watch_winder_hinge_knuckle.obj",
        outer_radius=hinge_knuckle_outer_r,
        inner_radius=hinge_pin_radius,
        length=hinge_knuckle_len,
    )

    base = model.part("base")
    base.visual(
        Box((width, depth, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material="utility_body",
        name="floor_plate",
    )
    base.visual(
        Box((wall_t, depth, base_height - floor_t)),
        origin=Origin(xyz=(-(width - wall_t) / 2.0, 0.0, floor_t + (base_height - floor_t) / 2.0)),
        material="utility_body",
        name="left_wall",
    )
    base.visual(
        Box((wall_t, depth, base_height - floor_t)),
        origin=Origin(xyz=((width - wall_t) / 2.0, 0.0, floor_t + (base_height - floor_t) / 2.0)),
        material="utility_body",
        name="right_wall",
    )
    base.visual(
        Box((width - 2.0 * wall_t, wall_t, base_height - floor_t)),
        origin=Origin(xyz=(0.0, (depth - wall_t) / 2.0, floor_t + (base_height - floor_t) / 2.0)),
        material="utility_body",
        name="front_wall",
    )
    base.visual(
        Box((width - 2.0 * wall_t, wall_t, base_height - floor_t)),
        origin=Origin(xyz=(0.0, -(depth - wall_t) / 2.0, floor_t + (base_height - floor_t) / 2.0)),
        material="utility_body",
        name="back_wall",
    )
    base.visual(
        Box((width - 2.0 * wall_t, rim_t, rim_t)),
        origin=Origin(xyz=(0.0, (depth - wall_t) / 2.0, base_height - rim_t / 2.0)),
        material="utility_body",
        name="front_rim",
    )
    base.visual(
        Box((width - 2.0 * wall_t, rim_t, rim_t)),
        origin=Origin(xyz=(0.0, -(depth - wall_t) / 2.0, base_height - rim_t / 2.0)),
        material="utility_body",
        name="back_rim",
    )
    base.visual(
        Box((rim_t, depth - 2.0 * wall_t - 0.004, rim_t)),
        origin=Origin(xyz=(-(width / 2.0 - wall_t - rim_t / 2.0), 0.0, base_height - rim_t / 2.0)),
        material="utility_body",
        name="left_rim",
    )
    base.visual(
        Box((rim_t, depth - 2.0 * wall_t - 0.004, rim_t)),
        origin=Origin(xyz=((width / 2.0 - wall_t - rim_t / 2.0), 0.0, base_height - rim_t / 2.0)),
        material="utility_body",
        name="right_rim",
    )
    base.visual(
        Box((0.140, 0.090, 0.006)),
        origin=Origin(xyz=(0.0, 0.020, 0.017)),
        material="foam_pad",
        name="interior_pad",
    )
    base.visual(
        Box((0.136, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, -0.060, 0.034)),
        material="bearing_dark",
        name="rear_motor_pod",
    )
    base.visual(
        Box((0.130, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="bearing_dark",
        name="cradle_crossmember",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        base.visual(
            Box((0.014, 0.060, 0.056)),
            origin=Origin(xyz=(sign * 0.070, 0.0, 0.042)),
            material="utility_body",
            name=f"{side}_support_tower",
        )
        base.visual(
            Box((0.004, 0.024, 0.024)),
            origin=Origin(xyz=(sign * 0.061, 0.0, cradle_axis_z)),
            material="bearing_dark",
            name=f"{side}_bearing_block",
        )
        base.visual(
            Box((0.020, 0.020, 0.032)),
            origin=Origin(xyz=(sign * 0.086, hinge_axis_y + 0.010, 0.108)),
            material="hinge_steel",
            name=f"{side}_hinge_support",
        )
        base.visual(
            Box((0.038, 0.014, 0.018)),
            origin=Origin(xyz=(sign * 0.083, hinge_axis_y + 0.009, 0.095)),
            material="hinge_steel",
            name=f"{side}_hinge_gusset",
        )
    for idx, (x, y) in enumerate(((-0.110, -0.080), (-0.110, 0.080), (0.110, -0.080), (0.110, 0.080)), start=1):
        base.visual(
            Box((0.020, 0.020, 0.030)),
            origin=Origin(xyz=(x, y, 0.105)),
            material="hinge_steel",
            name=f"corner_guard_{idx}",
        )
    for idx, (x, y) in enumerate(((-0.082, -0.056), (-0.082, 0.056), (0.082, -0.056), (0.082, 0.056)), start=1):
        base.visual(
            Box((0.032, 0.032, 0.010)),
            origin=Origin(xyz=(x, y, -0.005)),
            material="rubber_foot",
            name=f"foot_{idx}",
        )

    for idx, x in enumerate((-0.070, 0.070), start=1):
        _add_bolt(base, name=f"front_bolt_{idx}", center=(x, depth / 2.0, 0.094), axis="y")
        _add_bolt(base, name=f"rear_bolt_{idx}", center=(x, -depth / 2.0, 0.094), axis="y")
        _add_bolt(base, name=f"hinge_bolt_{idx}", center=(x, hinge_axis_y + 0.013, 0.106), axis="y", length=0.010)
    for idx, y in enumerate((-0.045, 0.045), start=1):
        _add_bolt(base, name=f"left_side_bolt_{idx}", center=(-width / 2.0, y, 0.078), axis="x")
        _add_bolt(base, name=f"right_side_bolt_{idx}", center=(width / 2.0, y, 0.078), axis="x")
    base.visual(
        Cylinder(radius=hinge_pin_radius, length=0.176),
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z), rpy=_axis_rpy("x")),
        material="hinge_steel",
        name="hinge_pin",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_outer_w, lid_frame_t, lid_frame_h)),
        origin=Origin(xyz=(0.0, lid_front_rail_y, lid_frame_z)),
        material="utility_body",
        name="lid_front_rail",
    )
    lid.visual(
        Box((lid_outer_w, lid_frame_t, lid_frame_h)),
        origin=Origin(xyz=(0.0, lid_back_rail_y, lid_frame_z)),
        material="utility_body",
        name="lid_back_rail",
    )
    lid.visual(
        Box((lid_frame_t, lid_side_rail_d, lid_frame_h)),
        origin=Origin(xyz=(-(lid_outer_w - lid_frame_t) / 2.0, lid_side_rail_y, lid_frame_z)),
        material="utility_body",
        name="lid_left_rail",
    )
    lid.visual(
        Box((lid_frame_t, lid_side_rail_d, lid_frame_h)),
        origin=Origin(xyz=((lid_outer_w - lid_frame_t) / 2.0, lid_side_rail_y, lid_frame_z)),
        material="utility_body",
        name="lid_right_rail",
    )
    lid.visual(
        Box((0.218, 0.143, 0.004)),
        origin=Origin(xyz=(0.0, lid_side_rail_y, 0.030)),
        material="clear_window",
        name="lid_window",
    )
    lid.visual(
        Box((0.008, 0.143, 0.008)),
        origin=Origin(xyz=(-0.055, lid_side_rail_y, 0.032)),
        material="hinge_steel",
        name="window_guard_left",
    )
    lid.visual(
        Box((0.008, 0.143, 0.008)),
        origin=Origin(xyz=(0.055, lid_side_rail_y, 0.032)),
        material="hinge_steel",
        name="window_guard_right",
    )
    lid.visual(
        Box((0.218, 0.143, 0.004)),
        origin=Origin(xyz=(0.0, lid_side_rail_y, -0.002)),
        material="liner_black",
        name="lid_inner_panel",
    )
    lid.visual(
        hinge_knuckle_mesh,
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=_axis_rpy("x")),
        material="hinge_steel",
        name="left_hinge_knuckle",
    )
    lid.visual(
        hinge_knuckle_mesh,
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=_axis_rpy("x")),
        material="hinge_steel",
        name="right_hinge_knuckle",
    )
    lid.visual(
        Box((0.020, 0.006, 0.006)),
        origin=Origin(xyz=(-0.055, 0.006, 0.006)),
        material="hinge_steel",
        name="left_hinge_strap",
    )
    lid.visual(
        Box((0.020, 0.006, 0.006)),
        origin=Origin(xyz=(0.055, 0.006, 0.006)),
        material="hinge_steel",
        name="right_hinge_strap",
    )
    lid.visual(
        Box((0.022, 0.014, 0.014)),
        origin=Origin(xyz=(-0.055, 0.015, 0.015)),
        material="hinge_steel",
        name="left_hinge_web",
    )
    lid.visual(
        Box((0.022, 0.014, 0.014)),
        origin=Origin(xyz=(0.055, 0.015, 0.015)),
        material="hinge_steel",
        name="right_hinge_web",
    )
    lid.visual(
        Box((0.066, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.016, 0.010)),
        material="hinge_steel",
        name="center_hinge_bridge",
    )
    for idx, x in enumerate((-0.090, -0.030, 0.030, 0.090), start=1):
        _add_bolt(
            lid,
            name=f"lid_front_bolt_{idx}",
            center=(x, lid_front_rail_y, 0.040),
            length=0.008,
            material="fastener_steel",
        )
        _add_bolt(
            lid,
            name=f"lid_back_bolt_{idx}",
            center=(x, lid_back_rail_y, 0.040),
            length=0.008,
            material="fastener_steel",
        )
    for idx, y in enumerate((0.060, 0.132), start=1):
        _add_bolt(
            lid,
            name=f"lid_left_bolt_{idx}",
            center=(-(lid_outer_w / 2.0 - 0.002), y, 0.020),
            axis="x",
            length=0.008,
            material="fastener_steel",
        )
        _add_bolt(
            lid,
            name=f"lid_right_bolt_{idx}",
            center=((lid_outer_w / 2.0 - 0.002), y, 0.020),
            axis="x",
            length=0.008,
            material="fastener_steel",
        )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.036, length=0.070),
        origin=Origin(rpy=_axis_rpy("x")),
        material="cradle_pad",
        name="cradle_drum",
    )
    cradle.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(-0.038, 0.0, 0.0), rpy=_axis_rpy("x")),
        material="hinge_steel",
        name="cradle_left_endcap",
    )
    cradle.visual(
        Cylinder(radius=0.041, length=0.006),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=_axis_rpy("x")),
        material="hinge_steel",
        name="cradle_right_endcap",
    )
    cradle.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(-0.044, 0.0, 0.0), rpy=_axis_rpy("x")),
        material="bearing_dark",
        name="cradle_left_hub",
    )
    cradle.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=_axis_rpy("x")),
        material="bearing_dark",
        name="cradle_right_hub",
    )
    cradle.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(-0.053, 0.0, 0.0), rpy=_axis_rpy("x")),
        material="hinge_steel",
        name="cradle_left_trunnion",
    )
    cradle.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=_axis_rpy("x")),
        material="hinge_steel",
        name="cradle_right_trunnion",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "cradle_drive",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, cradle_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_drive = object_model.get_articulation("cradle_drive")

    floor_plate = base.get_visual("floor_plate")
    front_rim = base.get_visual("front_rim")
    left_bearing = base.get_visual("left_bearing_block")
    right_bearing = base.get_visual("right_bearing_block")
    hinge_pin = base.get_visual("hinge_pin")

    lid_front_rail = lid.get_visual("lid_front_rail")
    lid_window = lid.get_visual("lid_window")
    left_knuckle = lid.get_visual("left_hinge_knuckle")
    right_knuckle = lid.get_visual("right_hinge_knuckle")
    left_trunnion = cradle.get_visual("cradle_left_trunnion")
    right_trunnion = cradle.get_visual("cradle_right_trunnion")
    cradle_drum = cradle.get_visual("cradle_drum")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=4)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        base,
        lid,
        elem_a=hinge_pin,
        elem_b=left_knuckle,
        reason="Left hinge knuckle wraps around the fixed steel pin as an intentional pivot sleeve.",
    )
    ctx.allow_overlap(
        base,
        lid,
        elem_a=hinge_pin,
        elem_b=right_knuckle,
        reason="Right hinge knuckle wraps around the fixed steel pin as an intentional pivot sleeve.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check("base_present", base.name == "base", f"unexpected part: {base.name}")
    ctx.check("lid_present", lid.name == "lid", f"unexpected part: {lid.name}")
    ctx.check("cradle_present", cradle.name == "cradle", f"unexpected part: {cradle.name}")
    ctx.check(
        "lid_is_revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
        f"lid articulation type was {lid_hinge.articulation_type}",
    )
    ctx.check(
        "lid_hinge_axis",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"lid hinge axis was {lid_hinge.axis}",
    )
    lid_limits = lid_hinge.motion_limits
    ctx.check(
        "lid_hinge_limits",
        lid_limits is not None
        and lid_limits.lower is not None
        and lid_limits.upper is not None
        and lid_limits.lower >= 0.0
        and lid_limits.upper >= 1.8,
        f"lid limits were {lid_limits}",
    )
    ctx.check(
        "cradle_is_continuous",
        cradle_drive.articulation_type == ArticulationType.CONTINUOUS,
        f"cradle articulation type was {cradle_drive.articulation_type}",
    )
    ctx.check(
        "cradle_drive_axis",
        tuple(cradle_drive.axis) == (1.0, 0.0, 0.0),
        f"cradle drive axis was {cradle_drive.axis}",
    )
    drive_limits = cradle_drive.motion_limits
    ctx.check(
        "cradle_drive_limits",
        drive_limits is not None
        and drive_limits.lower is None
        and drive_limits.upper is None
        and drive_limits.velocity > 0.0,
        f"cradle drive limits were {drive_limits}",
    )

    with ctx.pose({lid_hinge: 0.0, cradle_drive: 0.0}):
        ctx.expect_contact(
            lid,
            base,
            elem_a=lid_front_rail,
            elem_b=front_rim,
            name="lid_front_rail_seats_on_front_rim",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.17,
            name="lid_covers_main_opening",
        )
        ctx.expect_within(
            cradle,
            base,
            axes="xy",
            margin=0.0,
            name="cradle_stays_within_box_plan",
        )
        ctx.expect_gap(
            cradle,
            base,
            axis="z",
            positive_elem=cradle_drum,
            negative_elem=floor_plate,
            min_gap=0.008,
            max_gap=0.020,
            name="cradle_clears_floor",
        )
        ctx.expect_contact(
            lid,
            base,
            elem_a=left_knuckle,
            elem_b=hinge_pin,
            name="left_hinge_knuckle_runs_on_pin",
        )
        ctx.expect_contact(
            lid,
            base,
            elem_a=right_knuckle,
            elem_b=hinge_pin,
            name="right_hinge_knuckle_runs_on_pin",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a=left_trunnion,
            elem_b=left_bearing,
            name="left_trunnion_supported",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a=right_trunnion,
            elem_b=right_bearing,
            name="right_trunnion_supported",
        )
        ctx.expect_within(
            lid,
            lid,
            axes="xy",
            inner_elem=lid_window,
            margin=0.0,
            name="window_within_lid_footprint",
        )

    if lid_limits is not None and lid_limits.lower is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.lower, cradle_drive: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(max_pose_samples=1, name="lid_hinge_lower_no_floating")
        with ctx.pose({lid_hinge: lid_limits.upper, cradle_drive: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(max_pose_samples=1, name="lid_hinge_upper_no_floating")
            ctx.expect_contact(
                lid,
                base,
                elem_a=left_knuckle,
                elem_b=hinge_pin,
                name="left_hinge_knuckle_supported_when_open",
            )
            ctx.expect_contact(
                lid,
                base,
                elem_a=right_knuckle,
                elem_b=hinge_pin,
                name="right_hinge_knuckle_supported_when_open",
            )
            ctx.expect_gap(
                lid,
                base,
                axis="z",
                positive_elem=lid_front_rail,
                negative_elem=front_rim,
                min_gap=0.100,
                name="lid_front_rail_lifts_clear_when_open",
            )

    with ctx.pose({lid_hinge: 0.0, cradle_drive: pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="cradle_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(max_pose_samples=1, name="cradle_quarter_turn_no_floating")
        ctx.expect_contact(
            cradle,
            base,
            elem_a=left_trunnion,
            elem_b=left_bearing,
            name="left_trunnion_supported_at_quarter_turn",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a=right_trunnion,
            elem_b=right_bearing,
            name="right_trunnion_supported_at_quarter_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
