from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_mesh(
    name: str,
    *,
    outer_width: float,
    outer_height: float,
    opening_width: float,
    opening_height: float,
    thickness: float,
    outer_radius: float,
    opening_radius: float,
):
    ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_height, outer_radius, corner_segments=10),
        [rounded_rect_profile(opening_width, opening_height, opening_radius, corner_segments=10)],
        height=thickness,
        center=True,
    ).rotate_x(-pi / 2.0)
    return _mesh(name, ring)


def _sleeve_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    sleeve = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length * 0.5), (outer_radius, length * 0.5)],
        [(inner_radius, -length * 0.5), (inner_radius, length * 0.5)],
        segments=40,
    ).rotate_y(pi / 2.0)
    return _mesh(name, sleeve)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_pet_flap")

    frame_white = model.material("frame_white", rgba=(0.92, 0.93, 0.94, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hood_smoke = model.material("hood_smoke", rgba=(0.20, 0.23, 0.25, 0.96))
    clear_panel = model.material("clear_panel", rgba=(0.72, 0.86, 0.95, 0.28))
    gasket_black = model.material("gasket_black", rgba=(0.05, 0.05, 0.06, 1.0))
    hardware_gray = model.material("hardware_gray", rgba=(0.70, 0.72, 0.75, 1.0))

    frame_width = 0.30
    frame_height = 0.40
    opening_width = 0.205
    opening_height = 0.29
    door_thickness = 0.045
    bezel_thickness = 0.008
    front_y = door_thickness * 0.5 + bezel_thickness * 0.5
    rear_y = -front_y
    side_center_x = (opening_width + frame_width) * 0.25
    top_center_z = (opening_height + frame_height) * 0.25
    tunnel_side_width = (frame_width - opening_width) * 0.5
    tunnel_cap_height = (frame_height - opening_height) * 0.5

    flap_axis_z = opening_height * 0.5 - 0.004
    hood_axis_y = front_y + bezel_thickness * 0.5
    hood_axis_z = frame_height * 0.5 + 0.011

    front_ring_mesh = _ring_mesh(
        "pet_flap_front_ring",
        outer_width=frame_width,
        outer_height=frame_height,
        opening_width=opening_width,
        opening_height=opening_height,
        thickness=bezel_thickness,
        outer_radius=0.024,
        opening_radius=0.014,
    )
    rear_ring_mesh = _ring_mesh(
        "pet_flap_rear_ring",
        outer_width=frame_width,
        outer_height=frame_height,
        opening_width=opening_width,
        opening_height=opening_height,
        thickness=bezel_thickness,
        outer_radius=0.018,
        opening_radius=0.012,
    )
    flap_sleeve_mesh = _sleeve_mesh(
        "flap_hinge_sleeve",
        outer_radius=0.0075,
        inner_radius=0.0055,
        length=0.040,
    )
    hood_sleeve_mesh = _sleeve_mesh(
        "hood_hinge_clip",
        outer_radius=0.0085,
        inner_radius=0.0060,
        length=0.032,
    )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((frame_width, door_thickness + 0.020, frame_height)),
        mass=2.4,
    )
    frame.visual(front_ring_mesh, origin=Origin(xyz=(0.0, front_y, 0.0)), material=frame_white, name="front_ring")
    frame.visual(rear_ring_mesh, origin=Origin(xyz=(0.0, rear_y, 0.0)), material=frame_white, name="rear_ring")
    frame.visual(
        Box((tunnel_side_width, door_thickness, opening_height)),
        origin=Origin(xyz=(side_center_x, 0.0, 0.0)),
        material=frame_white,
        name="right_tunnel_wall",
    )
    frame.visual(
        Box((tunnel_side_width, door_thickness, opening_height)),
        origin=Origin(xyz=(-side_center_x, 0.0, 0.0)),
        material=frame_white,
        name="left_tunnel_wall",
    )
    frame.visual(
        Box((opening_width, door_thickness, tunnel_cap_height)),
        origin=Origin(xyz=(0.0, 0.0, top_center_z)),
        material=frame_white,
        name="top_tunnel_wall",
    )
    frame.visual(
        Box((opening_width, door_thickness, tunnel_cap_height)),
        origin=Origin(xyz=(0.0, 0.0, -top_center_z)),
        material=frame_white,
        name="bottom_tunnel_wall",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=0.176),
        origin=Origin(xyz=(0.0, 0.0, flap_axis_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_black,
        name="flap_hinge_rod",
    )
    frame.visual(
        Cylinder(radius=0.0050, length=0.228),
        origin=Origin(xyz=(0.0, hood_axis_y, hood_axis_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_black,
        name="hood_hinge_rod",
    )
    frame.visual(
        Box((0.010, 0.004, 0.022)),
        origin=Origin(xyz=(-0.119, hood_axis_y - 0.005, hood_axis_z - 0.006)),
        material=trim_black,
        name="left_hood_hinge_ear",
    )
    frame.visual(
        Box((0.010, 0.004, 0.022)),
        origin=Origin(xyz=(0.119, hood_axis_y - 0.005, hood_axis_z - 0.006)),
        material=trim_black,
        name="right_hood_hinge_ear",
    )
    frame.visual(
        Box((0.060, 0.014, 0.028)),
        origin=Origin(xyz=(0.0, rear_y - 0.008, -top_center_z)),
        material=trim_black,
        name="lock_housing",
    )
    frame.visual(
        Box((0.024, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, rear_y - 0.017, -top_center_z + 0.006)),
        material=hardware_gray,
        name="lock_slider",
    )
    for name, x_pos, z_pos in (
        ("screw_cap_tl", -0.108, 0.150),
        ("screw_cap_tr", 0.108, 0.150),
        ("screw_cap_bl", -0.108, -0.150),
        ("screw_cap_br", 0.108, -0.150),
    ):
        frame.visual(
            Cylinder(radius=0.009, length=0.003),
            origin=Origin(xyz=(x_pos, front_y + 0.0055, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hardware_gray,
            name=name,
        )

    flap = model.part("flap")
    flap.inertial = Inertial.from_geometry(
        Box((0.194, 0.020, 0.272)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.136)),
    )
    flap.visual(
        Box((0.194, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.0155)),
        material=trim_black,
        name="top_rail",
    )
    flap.visual(
        Box((0.186, 0.004, 0.257)),
        origin=Origin(xyz=(0.0, 0.0, -0.1525)),
        material=clear_panel,
        name="panel_clear",
    )
    flap.visual(
        Box((0.008, 0.010, 0.236)),
        origin=Origin(xyz=(-0.089, 0.0, -0.145)),
        material=gasket_black,
        name="left_edge_seal",
    )
    flap.visual(
        Box((0.008, 0.010, 0.236)),
        origin=Origin(xyz=(0.089, 0.0, -0.145)),
        material=gasket_black,
        name="right_edge_seal",
    )
    flap.visual(
        Box((0.186, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.260)),
        material=gasket_black,
        name="bottom_sweep",
    )
    flap.visual(
        Box((0.050, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.002, -0.254)),
        material=trim_black,
        name="latch_plate",
    )
    flap.visual(
        flap_sleeve_mesh,
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=trim_black,
        name="left_hinge_sleeve",
    )
    flap.visual(
        flap_sleeve_mesh,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=trim_black,
        name="right_hinge_sleeve",
    )

    hood = model.part("weather_hood")
    hood.inertial = Inertial.from_geometry(
        Box((0.240, 0.090, 0.050)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.040, -0.020)),
    )
    hood.visual(
        Box((0.216, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.006, -0.0155)),
        material=hood_smoke,
        name="back_strap",
    )
    hood.visual(
        Box((0.240, 0.074, 0.004)),
        origin=Origin(xyz=(0.0, 0.040, -0.017), rpy=(-0.20, 0.0, 0.0)),
        material=hood_smoke,
        name="top_shell",
    )
    hood.visual(
        Box((0.232, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, 0.074, -0.024), rpy=(-0.20, 0.0, 0.0)),
        material=hood_smoke,
        name="front_lip",
    )
    hood.visual(
        Box((0.004, 0.072, 0.028)),
        origin=Origin(xyz=(-0.118, 0.039, -0.020), rpy=(-0.20, 0.0, 0.0)),
        material=hood_smoke,
        name="left_cheek",
    )
    hood.visual(
        Box((0.004, 0.072, 0.028)),
        origin=Origin(xyz=(0.118, 0.039, -0.020), rpy=(-0.20, 0.0, 0.0)),
        material=hood_smoke,
        name="right_cheek",
    )
    hood.visual(
        hood_sleeve_mesh,
        origin=Origin(xyz=(-0.078, 0.0, 0.0)),
        material=hood_smoke,
        name="left_hinge_clip",
    )
    hood.visual(
        hood_sleeve_mesh,
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=hood_smoke,
        name="right_hinge_clip",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, flap_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "frame_to_weather_hood",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hood,
        origin=Origin(xyz=(0.0, hood_axis_y, hood_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    hood = object_model.get_part("weather_hood")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    hood_hinge = object_model.get_articulation("frame_to_weather_hood")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "flap_hinge_axis_is_horizontal",
        tuple(round(value, 6) for value in flap_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected horizontal flap hinge axis, got {flap_hinge.axis!r}",
    )
    ctx.check(
        "hood_hinge_axis_is_horizontal",
        tuple(round(value, 6) for value in hood_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected horizontal hood hinge axis, got {hood_hinge.axis!r}",
    )
    ctx.check(
        "flap_swings_both_directions",
        flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.lower is not None
        and flap_hinge.motion_limits.upper is not None
        and flap_hinge.motion_limits.lower < 0.0 < flap_hinge.motion_limits.upper,
        "Flap should rotate both inward and outward around the top hinge.",
    )
    ctx.check(
        "hood_lifts_upward_from_closed_rest",
        hood_hinge.motion_limits is not None
        and hood_hinge.motion_limits.lower == 0.0
        and hood_hinge.motion_limits.upper is not None
        and hood_hinge.motion_limits.upper >= 1.0,
        "Weather hood should start closed and rotate upward from the top hinge.",
    )

    ctx.expect_within(flap, frame, axes="x", margin=0.0)
    ctx.expect_overlap(hood, frame, axes="x", min_overlap=0.18)
    ctx.expect_gap(hood, frame, axis="y", positive_elem="front_lip", min_gap=0.025, max_gap=0.080)
    ctx.expect_contact(
        flap,
        frame,
        elem_a="left_hinge_sleeve",
        elem_b="flap_hinge_rod",
        contact_tol=0.0012,
        name="left_flap_sleeve_tracks_hinge_rod",
    )
    ctx.expect_contact(
        flap,
        frame,
        elem_a="right_hinge_sleeve",
        elem_b="flap_hinge_rod",
        contact_tol=0.0012,
        name="right_flap_sleeve_tracks_hinge_rod",
    )
    ctx.expect_contact(
        hood,
        frame,
        elem_a="left_hinge_clip",
        elem_b="hood_hinge_rod",
        contact_tol=0.0015,
        name="left_hood_clip_stays_on_hinge_rod",
    )
    ctx.expect_contact(
        hood,
        frame,
        elem_a="right_hinge_clip",
        elem_b="hood_hinge_rod",
        contact_tol=0.0015,
        name="right_hood_clip_stays_on_hinge_rod",
    )

    flap_closed = ctx.part_element_world_aabb(flap, elem="panel_clear")
    hood_closed = ctx.part_element_world_aabb(hood, elem="front_lip")
    assert flap_closed is not None
    assert hood_closed is not None

    with ctx.pose({flap_hinge: 0.85}):
        flap_open = ctx.part_element_world_aabb(flap, elem="panel_clear")
        assert flap_open is not None
        assert flap_open[1][1] > flap_closed[1][1] + 0.09
        ctx.expect_contact(
            flap,
            frame,
            elem_a="left_hinge_sleeve",
            elem_b="flap_hinge_rod",
            contact_tol=0.0012,
            name="left_flap_sleeve_remains_captured_when_open",
        )
        ctx.expect_contact(
            flap,
            frame,
            elem_a="right_hinge_sleeve",
            elem_b="flap_hinge_rod",
            contact_tol=0.0012,
            name="right_flap_sleeve_remains_captured_when_open",
        )

    with ctx.pose({hood_hinge: 1.0}):
        hood_open = ctx.part_element_world_aabb(hood, elem="front_lip")
        assert hood_open is not None
        assert hood_open[1][2] > hood_closed[1][2] + 0.05
        ctx.expect_contact(
            hood,
            frame,
            elem_a="left_hinge_clip",
            elem_b="hood_hinge_rod",
            contact_tol=0.0015,
            name="left_hood_clip_stays_attached_when_raised",
        )
        ctx.expect_contact(
            hood,
            frame,
            elem_a="right_hinge_clip",
            elem_b="hood_hinge_rod",
            contact_tol=0.0015,
            name="right_hood_clip_stays_attached_when_raised",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
