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
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 64,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _front_panel_mesh(
    *,
    name: str,
    width: float,
    height: float,
    thickness: float,
    opening_radius: float,
    opening_center_z: float,
):
    panel = ExtrudeWithHolesGeometry(
        [
            (-0.5 * width, 0.0),
            (0.5 * width, 0.0),
            (0.5 * width, height),
            (-0.5 * width, height),
        ],
        [_circle_profile(opening_radius, center=(0.0, opening_center_z), segments=72)],
        height=thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(name, panel)


def _ring_mesh(
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    outer_taper: float,
    inner_taper: float,
):
    ring = LatheGeometry.from_shell_profiles(
        [
            (outer_radius - outer_taper, -0.5 * thickness),
            (outer_radius, -0.18 * thickness),
            (outer_radius, 0.18 * thickness),
            (outer_radius - outer_taper, 0.5 * thickness),
        ],
        [
            (inner_radius + inner_taper, -0.42 * thickness),
            (inner_radius, -0.10 * thickness),
            (inner_radius, 0.16 * thickness),
            (inner_radius + inner_taper, 0.42 * thickness),
        ],
        segments=72,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(name, ring)


def _drum_shell_mesh(
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    length: float,
):
    drum = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -0.5 * length),
            (outer_radius * 1.01, -0.30 * length),
            (outer_radius * 1.01, 0.30 * length),
            (outer_radius * 0.95, 0.5 * length),
        ],
        [
            (inner_radius, -0.5 * length),
            (inner_radius, -0.32 * length),
            (inner_radius * 0.92, 0.28 * length),
            (0.032, 0.5 * length),
        ],
        segments=72,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(name, drum)


def _vec_close(
    actual: tuple[float, float, float] | None,
    expected: tuple[float, float, float],
    tol: float = 1e-6,
) -> bool:
    return actual is not None and all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vented_tumble_dryer")

    body_white = model.material("body_white", rgba=(0.95, 0.96, 0.97, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    stainless = model.material("stainless", rgba=(0.66, 0.69, 0.73, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.18, 0.23, 0.28, 0.52))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    panel_black = model.material("panel_black", rgba=(0.10, 0.11, 0.13, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.82, 0.83, 0.85, 1.0))

    body_width = 0.598
    body_depth = 0.625
    cabinet_height = 0.832
    cover_thickness = 0.018
    wall_thickness = 0.012
    rear_panel_thickness = 0.010
    front_panel_height = 0.742
    door_center_z = 0.402
    drum_center_y = 0.310
    door_outer_radius = 0.245
    door_opening_radius = 0.225
    door_axis_x = 0.248
    door_thickness = 0.054
    fascia_depth = 0.050
    fascia_height = 0.108
    upper_hinge_z = 0.530
    lower_hinge_z = 0.274

    front_fascia_mesh = _front_panel_mesh(
        name="dryer_front_fascia",
        width=body_width,
        height=front_panel_height,
        thickness=wall_thickness,
        opening_radius=door_opening_radius,
        opening_center_z=door_center_z,
    )
    cabinet_trim_mesh = _ring_mesh(
        name="dryer_porthole_trim",
        outer_radius=0.237,
        inner_radius=0.214,
        thickness=0.018,
        outer_taper=0.006,
        inner_taper=0.004,
    )
    door_outer_ring_mesh = _ring_mesh(
        name="dryer_door_outer_frame",
        outer_radius=door_outer_radius,
        inner_radius=0.184,
        thickness=door_thickness,
        outer_taper=0.010,
        inner_taper=0.010,
    )
    door_inner_bezel_mesh = _ring_mesh(
        name="dryer_door_inner_bezel",
        outer_radius=0.188,
        inner_radius=0.170,
        thickness=0.026,
        outer_taper=0.006,
        inner_taper=0.004,
    )
    door_seal_mesh = _ring_mesh(
        name="dryer_door_seal",
        outer_radius=0.182,
        inner_radius=0.168,
        thickness=0.010,
        outer_taper=0.002,
        inner_taper=0.002,
    )
    drum_shell_mesh = _drum_shell_mesh(
        name="dryer_drum_shell",
        outer_radius=0.222,
        inner_radius=0.208,
        length=0.470,
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall_thickness, body_depth, cabinet_height)),
        origin=Origin(xyz=(-0.5 * body_width + 0.5 * wall_thickness, 0.5 * body_depth, 0.5 * cabinet_height)),
        material=body_white,
        name="left_side",
    )
    cabinet.visual(
        Box((wall_thickness, body_depth, cabinet_height)),
        origin=Origin(xyz=(0.5 * body_width - 0.5 * wall_thickness, 0.5 * body_depth, 0.5 * cabinet_height)),
        material=body_white,
        name="right_side",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall_thickness, body_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.5 * body_depth, 0.5 * wall_thickness)),
        material=body_white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall_thickness, rear_panel_thickness, cabinet_height)),
        origin=Origin(
            xyz=(0.0, body_depth - 0.5 * rear_panel_thickness, 0.5 * cabinet_height),
        ),
        material=body_white,
        name="rear_panel",
    )
    cabinet.visual(
        front_fascia_mesh,
        origin=Origin(xyz=(0.0, 0.5 * wall_thickness, 0.0)),
        material=body_white,
        name="front_fascia",
    )
    cabinet.visual(
        cabinet_trim_mesh,
        origin=Origin(xyz=(0.0, 0.008, door_center_z)),
        material=trim_gray,
        name="porthole_trim",
    )
    cabinet.visual(
        Cylinder(radius=0.036, length=0.024),
        origin=Origin(xyz=(0.0, 0.603, door_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="rear_bearing",
    )
    cabinet.visual(
        Box((body_width - 2.0 * wall_thickness, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, body_depth - 0.015, cabinet_height - 0.014)),
        material=trim_gray,
        name="rear_hinge_rail",
    )
    cabinet.visual(
        Cylinder(radius=0.044, length=0.050),
        origin=Origin(xyz=(0.0, body_depth + 0.025, 0.640), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="vent_collar",
    )
    for foot_index, foot_x in enumerate((-0.5 * body_width + 0.065, 0.5 * body_width - 0.065)):
        for rear_index, foot_y in enumerate((0.070, body_depth - 0.070)):
            cabinet.visual(
                Cylinder(radius=0.017, length=0.020),
                origin=Origin(xyz=(foot_x, foot_y, -0.010)),
                material=rubber_black,
                name=f"foot_{foot_index}_{rear_index}",
            )

    upper_hinge = model.part("upper_hinge")
    upper_hinge.visual(
        Box((0.042, 0.022, 0.082)),
        origin=Origin(xyz=(0.021, -0.011, 0.0)),
        material=trim_gray,
        name="mount_plate",
    )
    upper_hinge.visual(
        Box((0.016, 0.034, 0.050)),
        origin=Origin(xyz=(0.008, -0.017, 0.0)),
        material=trim_gray,
        name="hinge_block",
    )

    lower_hinge = model.part("lower_hinge")
    lower_hinge.visual(
        Box((0.042, 0.022, 0.082)),
        origin=Origin(xyz=(0.021, -0.011, 0.0)),
        material=trim_gray,
        name="mount_plate",
    )
    lower_hinge.visual(
        Box((0.016, 0.034, 0.050)),
        origin=Origin(xyz=(0.008, -0.017, 0.0)),
        material=trim_gray,
        name="hinge_block",
    )

    drum = model.part("drum")
    drum.visual(
        drum_shell_mesh,
        material=stainless,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.032, length=0.046),
        origin=Origin(xyz=(0.0, 0.258, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="rear_shaft",
    )
    drum.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.225, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="rear_hub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.210, 0.030, 0.026)),
            origin=Origin(
                xyz=(0.120, 0.205, 0.0),
                rpy=(0.0, angle, 0.0),
            ),
            material=stainless,
            name=f"rear_spider_{index}",
        )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.030, 0.180, 0.022)),
            origin=Origin(
                xyz=(0.0, -0.010, 0.195),
                rpy=(0.0, angle, 0.0),
            ),
            material=stainless,
            name=f"baffle_{index}",
        )

    door = model.part("door")
    door.visual(
        door_outer_ring_mesh,
        origin=Origin(xyz=(-door_axis_x, 0.0, 0.0)),
        material=body_white,
        name="outer_frame",
    )
    door.visual(
        door_inner_bezel_mesh,
        origin=Origin(xyz=(-door_axis_x, 0.008, 0.0)),
        material=trim_gray,
        name="inner_bezel",
    )
    door.visual(
        Cylinder(radius=0.176, length=0.006),
        origin=Origin(xyz=(-door_axis_x, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_glass,
        name="glass_window",
    )
    door.visual(
        door_seal_mesh,
        origin=Origin(xyz=(-door_axis_x, 0.015, 0.0)),
        material=rubber_black,
        name="seal_ring",
    )
    door.visual(
        Box((0.090, 0.022, 0.030)),
        origin=Origin(xyz=(-door_axis_x - 0.170, -0.010, -0.010)),
        material=trim_gray,
        name="handle_grip",
    )
    door.visual(
        Box((0.024, 0.018, 0.068)),
        origin=Origin(xyz=(-0.012, -0.015, upper_hinge_z - door_center_z)),
        material=trim_gray,
        name="upper_leaf",
    )
    door.visual(
        Box((0.024, 0.018, 0.068)),
        origin=Origin(xyz=(-0.012, -0.015, lower_hinge_z - door_center_z)),
        material=trim_gray,
        name="lower_leaf",
    )

    top_cover = model.part("top_cover")
    top_cover.visual(
        Box((body_width, body_depth, cover_thickness)),
        origin=Origin(xyz=(0.0, -0.5 * body_depth, 0.5 * cover_thickness)),
        material=body_white,
        name="top_panel",
    )
    top_cover.visual(
        Box((body_width, fascia_depth, fascia_height)),
        origin=Origin(
            xyz=(0.0, -body_depth - 0.5 * fascia_depth, cover_thickness - 0.5 * fascia_height),
        ),
        material=body_white,
        name="control_fascia",
    )
    top_cover.visual(
        Box((0.145, 0.008, 0.040)),
        origin=Origin(
            xyz=(-0.105, -body_depth - fascia_depth - 0.004, cover_thickness - 0.038),
        ),
        material=panel_black,
        name="display_window",
    )
    top_cover.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(
            xyz=(0.195, -body_depth - fascia_depth - 0.009, cover_thickness - 0.050),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_silver,
        name="control_dial",
    )
    top_cover.visual(
        Box((0.090, 0.006, 0.018)),
        origin=Origin(
            xyz=(0.045, -body_depth - fascia_depth - 0.003, cover_thickness - 0.044),
        ),
        material=panel_black,
        name="button_strip",
    )
    top_cover.visual(
        Cylinder(radius=0.007, length=body_width * 0.82),
        origin=Origin(xyz=(0.0, 0.004, 0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="rear_hinge_barrel",
    )

    model.articulation(
        "upper_hinge_mount",
        ArticulationType.FIXED,
        parent=cabinet,
        child=upper_hinge,
        origin=Origin(xyz=(door_axis_x, 0.0, upper_hinge_z)),
    )
    model.articulation(
        "lower_hinge_mount",
        ArticulationType.FIXED,
        parent=cabinet,
        child=lower_hinge,
        origin=Origin(xyz=(door_axis_x, 0.0, lower_hinge_z)),
    )
    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, drum_center_y, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
    )
    model.articulation(
        "door_swing",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(door_axis_x, -0.028, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=0.0, upper=2.05),
    )
    model.articulation(
        "top_cover_service_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=top_cover,
        origin=Origin(xyz=(0.0, body_depth, cabinet_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.3, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    upper_hinge = object_model.get_part("upper_hinge")
    lower_hinge = object_model.get_part("lower_hinge")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    top_cover = object_model.get_part("top_cover")

    door_swing = object_model.get_articulation("door_swing")
    drum_spin = object_model.get_articulation("drum_spin")
    top_cover_service_hinge = object_model.get_articulation("top_cover_service_hinge")

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

    ctx.check(
        "door hinge axis is vertical",
        _vec_close(door_swing.axis, (0.0, 0.0, 1.0)),
        f"expected (0, 0, 1), got {door_swing.axis}",
    )
    ctx.check(
        "door hinge opens to a wide service angle",
        door_swing.motion_limits is not None
        and door_swing.motion_limits.lower == 0.0
        and door_swing.motion_limits.upper is not None
        and door_swing.motion_limits.upper >= 1.9,
        f"door motion limits are {door_swing.motion_limits}",
    )
    ctx.check(
        "drum axle spins about the depth axis",
        _vec_close(drum_spin.axis, (0.0, 1.0, 0.0)),
        f"expected (0, 1, 0), got {drum_spin.axis}",
    )
    ctx.check(
        "top cover uses a rear hinge line",
        _vec_close(top_cover_service_hinge.axis, (-1.0, 0.0, 0.0))
        and top_cover_service_hinge.origin.xyz[1] > 0.60,
        f"cover hinge axis/origin are {top_cover_service_hinge.axis} at {top_cover_service_hinge.origin.xyz}",
    )

    with ctx.pose({door_swing: 0.0, top_cover_service_hinge: 0.0}):
        ctx.expect_contact(cabinet, upper_hinge, elem_a="front_fascia", elem_b="mount_plate")
        ctx.expect_contact(cabinet, lower_hinge, elem_a="front_fascia", elem_b="mount_plate")
        ctx.expect_contact(door, upper_hinge, elem_a="upper_leaf", elem_b="hinge_block")
        ctx.expect_contact(door, lower_hinge, elem_a="lower_leaf", elem_b="hinge_block")
        ctx.expect_contact(drum, cabinet, elem_a="rear_shaft", elem_b="rear_bearing")
        ctx.expect_contact(top_cover, cabinet, elem_a="top_panel", elem_b="rear_panel")
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            positive_elem="porthole_trim",
            negative_elem="outer_frame",
            max_gap=0.002,
            max_penetration=0.0,
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="xz",
            elem_a="porthole_trim",
            elem_b="outer_frame",
            min_overlap=0.43,
        )
        ctx.expect_overlap(
            cabinet,
            drum,
            axes="xz",
            elem_a="porthole_trim",
            elem_b="drum_shell",
            min_overlap=0.40,
        )
        ctx.expect_within(drum, cabinet, axes="xz", margin=0.04)
        ctx.expect_origin_distance(
            upper_hinge,
            lower_hinge,
            axes="z",
            min_dist=0.22,
            max_dist=0.30,
            name="door uses two separated hinge bodies",
        )

    door_closed = ctx.part_world_aabb(door)
    assert door_closed is not None
    with ctx.pose({door_swing: 1.25}):
        door_open = ctx.part_world_aabb(door)
        assert door_open is not None
        ctx.check(
            "door swings forward from the cabinet",
            door_open[0][1] < door_closed[0][1] - 0.16,
            f"closed min y {door_closed[0][1]:.4f}, open min y {door_open[0][1]:.4f}",
        )

    fascia_closed = ctx.part_element_world_aabb(top_cover, elem="control_fascia")
    assert fascia_closed is not None
    with ctx.pose({top_cover_service_hinge: 0.95}):
        fascia_open = ctx.part_element_world_aabb(top_cover, elem="control_fascia")
        assert fascia_open is not None
        ctx.check(
            "top control cover lifts for service access",
            fascia_open[1][2] > fascia_closed[1][2] + 0.28,
            f"closed max z {fascia_closed[1][2]:.4f}, open max z {fascia_open[1][2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
