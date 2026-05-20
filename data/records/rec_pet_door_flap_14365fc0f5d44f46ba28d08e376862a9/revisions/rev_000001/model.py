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


def _ring_plate_mesh(
    name: str,
    *,
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    thickness: float,
    outer_radius: float,
    inner_radius: float,
):
    outer_profile = rounded_rect_profile(
        outer_width,
        outer_height,
        outer_radius,
        corner_segments=8,
    )
    inner_profile = rounded_rect_profile(
        inner_width,
        inner_height,
        inner_radius,
        corner_segments=8,
    )
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        thickness,
        center=True,
    ).rotate_x(-pi / 2.0)
    return mesh_from_geometry(geom, name)


def _hinge_sleeve_mesh(
    name: str,
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
):
    half_length = length * 0.5
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_length), (outer_radius, half_length)],
        [(inner_radius, -half_length), (inner_radius, half_length)],
        segments=36,
    ).rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _add_flap_visuals(
    flap_part,
    *,
    prefix: str,
    sleeve_mesh,
    flap_width: float,
    flap_height: float,
    flap_thickness: float,
    rail_height: float,
    rail_thickness: float,
    clip_x_positions: tuple[float, ...],
    flap_material,
    edge_material,
):
    flap_part.visual(
        Box((flap_width, flap_thickness, flap_height)),
        origin=Origin(xyz=(0.0, 0.0, -0.128)),
        material=flap_material,
        name=f"{prefix}_panel",
    )
    flap_part.visual(
        Box((flap_width, rail_thickness, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=edge_material,
        name=f"{prefix}_top_rail",
    )
    flap_part.visual(
        Box((flap_width, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=edge_material,
        name=f"{prefix}_weight_bar",
    )
    for clip_name, x_pos in zip(("left", "center", "right"), clip_x_positions):
        flap_part.visual(
            sleeve_mesh,
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=edge_material,
            name=f"{prefix}_clip_{clip_name}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_tunnel_pet_door")

    frame_white = model.material("frame_white", rgba=(0.93, 0.93, 0.90, 1.0))
    tunnel_liner = model.material("tunnel_liner", rgba=(0.78, 0.80, 0.82, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    flap_smoke = model.material("flap_smoke", rgba=(0.26, 0.33, 0.39, 0.52))
    flap_edge = model.material("flap_edge", rgba=(0.07, 0.08, 0.09, 1.0))
    dial_black = model.material("dial_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.86, 0.22, 0.14, 1.0))

    opening_width = 0.190
    opening_height = 0.250
    tunnel_depth = 0.200
    wall_thickness = 0.018
    trim_thickness = 0.018

    sleeve_outer_width = opening_width + 2.0 * wall_thickness
    sleeve_outer_height = opening_height + 2.0 * wall_thickness

    outer_trim_width = 0.330
    outer_trim_height = 0.400
    inner_trim_width = 0.310
    inner_trim_height = 0.380

    front_face_y = -tunnel_depth * 0.5
    rear_face_y = tunnel_depth * 0.5
    outer_trim_y = front_face_y - trim_thickness * 0.5
    inner_trim_y = rear_face_y + trim_thickness * 0.5

    hinge_y_offset = tunnel_depth * 0.5 - 0.016
    hinge_z = opening_height * 0.5 - 0.007
    hinge_pin_radius = 0.004
    hinge_pin_length = 0.162

    flap_width = opening_width - 0.014
    flap_height = 0.222
    flap_thickness = 0.006
    flap_rail_height = 0.014
    flap_rail_thickness = 0.010

    outer_trim_mesh = _ring_plate_mesh(
        "outer_trim_frame",
        outer_width=outer_trim_width,
        outer_height=outer_trim_height,
        inner_width=sleeve_outer_width,
        inner_height=sleeve_outer_height,
        thickness=trim_thickness,
        outer_radius=0.020,
        inner_radius=0.006,
    )
    inner_trim_mesh = _ring_plate_mesh(
        "inner_trim_frame",
        outer_width=inner_trim_width,
        outer_height=inner_trim_height,
        inner_width=sleeve_outer_width,
        inner_height=sleeve_outer_height,
        thickness=trim_thickness,
        outer_radius=0.018,
        inner_radius=0.006,
    )
    hinge_sleeve_mesh = _hinge_sleeve_mesh(
        "flap_hinge_sleeve",
        inner_radius=hinge_pin_radius + 0.0006,
        outer_radius=hinge_pin_radius + 0.0032,
        length=0.030,
    )
    tunnel_frame = model.part("tunnel_frame")
    tunnel_frame.inertial = Inertial.from_geometry(
        Box((outer_trim_width, tunnel_depth + 2.0 * trim_thickness, outer_trim_height)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    tunnel_frame.visual(
        outer_trim_mesh,
        origin=Origin(xyz=(0.0, outer_trim_y, 0.0)),
        material=frame_white,
        name="outer_trim_frame",
    )
    tunnel_frame.visual(
        inner_trim_mesh,
        origin=Origin(xyz=(0.0, inner_trim_y, 0.0)),
        material=frame_white,
        name="inner_trim_frame",
    )
    tunnel_frame.visual(
        Box((sleeve_outer_width, tunnel_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, opening_height * 0.5 + wall_thickness * 0.5)),
        material=tunnel_liner,
        name="tunnel_top_wall",
    )
    tunnel_frame.visual(
        Box((sleeve_outer_width, tunnel_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -opening_height * 0.5 - wall_thickness * 0.5)),
        material=tunnel_liner,
        name="tunnel_bottom_wall",
    )
    tunnel_frame.visual(
        Box((wall_thickness, tunnel_depth, opening_height)),
        origin=Origin(xyz=(opening_width * 0.5 + wall_thickness * 0.5, 0.0, 0.0)),
        material=tunnel_liner,
        name="tunnel_right_wall",
    )
    tunnel_frame.visual(
        Box((wall_thickness, tunnel_depth, opening_height)),
        origin=Origin(xyz=(-opening_width * 0.5 - wall_thickness * 0.5, 0.0, 0.0)),
        material=tunnel_liner,
        name="tunnel_left_wall",
    )

    for prefix, hinge_y in (("outer", -hinge_y_offset), ("inner", hinge_y_offset)):
        tunnel_frame.visual(
            Cylinder(radius=hinge_pin_radius, length=hinge_pin_length),
            origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_dark,
            name=f"{prefix}_hinge_pin",
        )
        tunnel_frame.visual(
            Box((0.014, 0.010, 0.016)),
            origin=Origin(
                xyz=(opening_width * 0.5 + wall_thickness * 0.5, hinge_y, hinge_z),
            ),
            material=hinge_dark,
            name=f"{prefix}_hinge_bracket_right",
        )
        tunnel_frame.visual(
            Box((0.014, 0.010, 0.016)),
            origin=Origin(
                xyz=(-opening_width * 0.5 - wall_thickness * 0.5, hinge_y, hinge_z),
            ),
            material=hinge_dark,
            name=f"{prefix}_hinge_bracket_left",
        )
        tunnel_frame.visual(
            Box((opening_width - 0.014, 0.004, 0.012)),
            origin=Origin(xyz=(0.0, hinge_y + (0.004 if prefix == "outer" else -0.004), hinge_z + 0.003)),
            material=hinge_dark,
            name=f"{prefix}_hinge_header",
        )

    outer_flap = model.part("outer_flap")
    outer_flap.inertial = Inertial.from_geometry(
        Box((flap_width, flap_rail_thickness, flap_height + 0.022)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -0.118)),
    )
    _add_flap_visuals(
        outer_flap,
        prefix="outer",
        sleeve_mesh=hinge_sleeve_mesh,
        flap_width=flap_width,
        flap_height=flap_height,
        flap_thickness=flap_thickness,
        rail_height=flap_rail_height,
        rail_thickness=flap_rail_thickness,
        clip_x_positions=(-0.054, 0.0, 0.054),
        flap_material=flap_smoke,
        edge_material=flap_edge,
    )

    inner_flap = model.part("inner_flap")
    inner_flap.inertial = Inertial.from_geometry(
        Box((flap_width, flap_rail_thickness, flap_height + 0.022)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -0.118)),
    )
    _add_flap_visuals(
        inner_flap,
        prefix="inner",
        sleeve_mesh=hinge_sleeve_mesh,
        flap_width=flap_width,
        flap_height=flap_height,
        flap_thickness=flap_thickness,
        rail_height=flap_rail_height,
        rail_thickness=flap_rail_thickness,
        clip_x_positions=(-0.054, 0.0, 0.054),
        flap_material=flap_smoke,
        edge_material=flap_edge,
    )

    selector_dial = model.part("selector_dial")
    selector_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.012),
        mass=0.12,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    selector_dial.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_body",
    )
    selector_dial.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="dial_spindle",
    )
    selector_dial.visual(
        Box((0.026, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, 0.010)),
        material=dial_mark,
        name="dial_indicator",
    )

    model.articulation(
        "outer_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=tunnel_frame,
        child=outer_flap,
        origin=Origin(xyz=(0.0, -hinge_y_offset, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "inner_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=tunnel_frame,
        child=inner_flap,
        origin=Origin(xyz=(0.0, hinge_y_offset, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "selector_dial_joint",
        ArticulationType.REVOLUTE,
        parent=tunnel_frame,
        child=selector_dial,
        origin=Origin(xyz=(0.133, inner_trim_y + trim_thickness * 0.5 + 0.006, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tunnel_frame = object_model.get_part("tunnel_frame")
    outer_flap = object_model.get_part("outer_flap")
    inner_flap = object_model.get_part("inner_flap")
    selector_dial = object_model.get_part("selector_dial")

    outer_hinge = object_model.get_articulation("outer_flap_hinge")
    inner_hinge = object_model.get_articulation("inner_flap_hinge")
    dial_joint = object_model.get_articulation("selector_dial_joint")

    def _hinge_anchor_holds(flap, hinge, *, name: str) -> bool:
        flap_pos = ctx.part_world_position(flap)
        hinge_xyz = tuple(hinge.origin.xyz)
        if flap_pos is None:
            return ctx.fail(name, "missing world position for flap part")
        deltas = tuple(abs(flap_pos[i] - hinge_xyz[i]) for i in range(3))
        return ctx.check(
            name,
            all(delta <= 1e-6 for delta in deltas),
            details=f"flap origin {flap_pos} drifted from hinge origin {hinge_xyz} by {deltas}",
        )

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
        "outer flap hinge axis is horizontal",
        tuple(outer_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1,0,0), got {outer_hinge.axis}",
    )
    ctx.check(
        "inner flap hinge axis is horizontal",
        tuple(inner_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1,0,0), got {inner_hinge.axis}",
    )
    ctx.check(
        "selector dial rotates on frame-normal axis",
        tuple(dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0,1,0), got {dial_joint.axis}",
    )
    ctx.check(
        "flap hinges have bidirectional swing",
        (
            outer_hinge.motion_limits is not None
            and inner_hinge.motion_limits is not None
            and outer_hinge.motion_limits.lower is not None
            and outer_hinge.motion_limits.upper is not None
            and inner_hinge.motion_limits.lower is not None
            and inner_hinge.motion_limits.upper is not None
            and outer_hinge.motion_limits.lower < 0.0 < outer_hinge.motion_limits.upper
            and inner_hinge.motion_limits.lower < 0.0 < inner_hinge.motion_limits.upper
        ),
        details="both flaps should swing through the closed pose on their top hinge lines",
    )

    ctx.expect_overlap(
        outer_flap,
        tunnel_frame,
        axes="xyz",
        min_overlap=0.008,
        elem_a="outer_clip_center",
        elem_b="outer_hinge_pin",
        name="outer flap center sleeve stays around outer hinge pin",
    )
    ctx.expect_overlap(
        inner_flap,
        tunnel_frame,
        axes="xyz",
        min_overlap=0.008,
        elem_a="inner_clip_center",
        elem_b="inner_hinge_pin",
        name="inner flap center sleeve stays around inner hinge pin",
    )
    ctx.expect_contact(
        selector_dial,
        tunnel_frame,
        elem_a="dial_body",
        elem_b="inner_trim_frame",
        contact_tol=0.0005,
        name="selector dial seats on inner trim face",
    )
    ctx.expect_overlap(
        selector_dial,
        tunnel_frame,
        axes="xz",
        min_overlap=0.016,
        elem_a="dial_body",
        elem_b="inner_trim_frame",
        name="selector dial footprint stays on inner frame face",
    )

    with ctx.pose({outer_hinge: 0.55}):
        _hinge_anchor_holds(
            outer_flap,
            outer_hinge,
            name="outer flap hinge origin remains the attachment point when opened",
        )
        ctx.expect_overlap(
            outer_flap,
            tunnel_frame,
            axes="yz",
            min_overlap=0.008,
            elem_a="outer_clip_center",
            elem_b="outer_hinge_pin",
            name="outer flap stays clipped while opened",
        )
    with ctx.pose({inner_hinge: -0.55}):
        _hinge_anchor_holds(
            inner_flap,
            inner_hinge,
            name="inner flap hinge origin remains the attachment point when opened",
        )
        ctx.expect_overlap(
            inner_flap,
            tunnel_frame,
            axes="yz",
            min_overlap=0.008,
            elem_a="inner_clip_center",
            elem_b="inner_hinge_pin",
            name="inner flap stays clipped while opened",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
