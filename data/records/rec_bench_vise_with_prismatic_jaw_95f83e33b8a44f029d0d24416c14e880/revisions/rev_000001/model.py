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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    half_height = height * 0.5
    return [
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height),
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _jaw_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    bar_hole_size: float,
    bar_center_x: float,
    screw_hole_diameter: float,
    corner_radius: float,
):
    outer = rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    holes = [
        _translate_profile(_rect_profile(bar_hole_size, bar_hole_size), dx=-bar_center_x),
        _translate_profile(_rect_profile(bar_hole_size, bar_hole_size), dx=bar_center_x),
        superellipse_profile(
            screw_hole_diameter,
            screw_hole_diameter,
            exponent=2.0,
            segments=28,
        ),
    ]
    return ExtrudeWithHolesGeometry(outer, holes, height=thickness, center=True).rotate_x(pi / 2.0)


def _threaded_screw_mesh(
    *,
    length: float,
    root_radius: float,
    crest_radius: float,
    pitch: float,
):
    start = -length * 0.5
    end = length * 0.5
    profile: list[tuple[float, float]] = [
        (0.0, start),
        (root_radius * 0.98, start),
    ]
    z = start
    while z + pitch < end:
        profile.extend(
            [
                (root_radius, z + pitch * 0.16),
                (crest_radius, z + pitch * 0.34),
                (crest_radius, z + pitch * 0.66),
                (root_radius, z + pitch * 0.84),
            ]
        )
        z += pitch
    profile.extend(
        [
            (root_radius, end),
            (0.0, end),
        ]
    )
    return LatheGeometry(profile, segments=56).rotate_x(-pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="woodworking_face_vise")

    frame_width = 0.56
    frame_height = 0.24
    fixed_jaw_width = 0.50
    fixed_jaw_height = 0.17
    fixed_jaw_thickness = 0.038
    moving_jaw_width = 0.48
    moving_jaw_height = 0.165
    moving_jaw_thickness = 0.052
    closed_gap = 0.006
    jaw_travel = 0.22

    support_depth = 0.115
    side_block_width = 0.075
    rail_height = 0.052
    rear_beam_depth = 0.04

    guide_bar_size = 0.028
    guide_bar_center_x = 0.145
    guide_bar_rear_y = -0.153
    guide_bar_front_y = 0.325
    guide_bar_length = guide_bar_front_y - guide_bar_rear_y

    screw_root_radius = 0.013
    screw_crest_radius = 0.0165
    screw_pitch = 0.018
    screw_rear_y = -0.028
    screw_front_y = 0.314
    screw_length = screw_front_y - screw_rear_y
    screw_hole_diameter = 0.042

    wheel_center_y = 0.372
    wheel_radius = 0.078
    wheel_tube = 0.012
    jaw_center_closed_y = closed_gap + moving_jaw_thickness * 0.5

    ash_wood = model.material("ash_wood", rgba=(0.77, 0.63, 0.39, 1.0))
    walnut_wood = model.material("walnut_wood", rgba=(0.43, 0.30, 0.18, 1.0))
    handle_wood = model.material("handle_wood", rgba=(0.71, 0.52, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.60, 0.30, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((frame_width, fixed_jaw_thickness + support_depth + rear_beam_depth, frame_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.096, 0.0)),
    )

    frame.visual(
        _mesh(
            "fixed_jaw_face",
            _jaw_mesh(
                width=fixed_jaw_width,
                height=fixed_jaw_height,
                thickness=fixed_jaw_thickness,
                bar_hole_size=guide_bar_size + 0.004,
                bar_center_x=guide_bar_center_x,
                screw_hole_diameter=screw_hole_diameter + 0.010,
                corner_radius=0.013,
            ),
        ),
        origin=Origin(xyz=(0.0, -fixed_jaw_thickness * 0.5, 0.0)),
        material=ash_wood,
        name="fixed_jaw_face",
    )

    support_center_y = -fixed_jaw_thickness - support_depth * 0.5
    rear_beam_center_y = -fixed_jaw_thickness - support_depth - rear_beam_depth * 0.5
    side_x = frame_width * 0.5 - side_block_width * 0.5
    rail_z = frame_height * 0.5 - rail_height * 0.5

    frame.visual(
        Box((side_block_width, support_depth, frame_height)),
        origin=Origin(xyz=(-side_x, support_center_y, 0.0)),
        material=walnut_wood,
        name="left_cheek",
    )
    frame.visual(
        Box((side_block_width, support_depth, frame_height)),
        origin=Origin(xyz=(side_x, support_center_y, 0.0)),
        material=walnut_wood,
        name="right_cheek",
    )
    frame.visual(
        Box((frame_width, support_depth, rail_height)),
        origin=Origin(xyz=(0.0, support_center_y, rail_z)),
        material=walnut_wood,
        name="top_rail",
    )
    frame.visual(
        Box((frame_width, support_depth, rail_height)),
        origin=Origin(xyz=(0.0, support_center_y, -rail_z)),
        material=walnut_wood,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.18, rear_beam_depth, fixed_jaw_height)),
        origin=Origin(xyz=(-0.19, rear_beam_center_y, 0.0)),
        material=walnut_wood,
        name="rear_left_block",
    )
    frame.visual(
        Box((0.18, rear_beam_depth, fixed_jaw_height)),
        origin=Origin(xyz=(0.19, rear_beam_center_y, 0.0)),
        material=walnut_wood,
        name="rear_right_block",
    )

    guide_bar_origin_y = guide_bar_rear_y + guide_bar_length * 0.5
    frame.visual(
        Box((guide_bar_size, guide_bar_length, guide_bar_size)),
        origin=Origin(xyz=(-guide_bar_center_x, guide_bar_origin_y, 0.0)),
        material=dark_steel,
        name="guide_bar_left",
    )
    frame.visual(
        Box((guide_bar_size, guide_bar_length, guide_bar_size)),
        origin=Origin(xyz=(guide_bar_center_x, guide_bar_origin_y, 0.0)),
        material=dark_steel,
        name="guide_bar_right",
    )
    frame.visual(
        _mesh(
            "front_bearing_trim",
            TorusGeometry(
                radius=screw_hole_diameter * 0.5 + 0.006,
                tube=0.0035,
                radial_segments=18,
                tubular_segments=48,
            ),
        ),
        origin=Origin(xyz=(0.0, -0.0015, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="front_bearing_trim",
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.inertial = Inertial.from_geometry(
        Box((moving_jaw_width, moving_jaw_thickness, moving_jaw_height)),
        mass=5.5,
        origin=Origin(),
    )
    jaw_face_thickness = 0.024
    jaw_face_center_y = -moving_jaw_thickness * 0.5 + jaw_face_thickness * 0.5
    socket_outer = 0.052
    socket_wall = (socket_outer - guide_bar_size) * 0.5
    socket_depth = 0.034
    socket_center_y = jaw_face_center_y + jaw_face_thickness * 0.5 + socket_depth * 0.5
    side_strip_z = guide_bar_size * 0.5
    top_strip_z = guide_bar_size * 0.5 + socket_wall * 0.5

    moving_jaw.visual(
        _mesh(
            "moving_jaw_body",
            _jaw_mesh(
                width=moving_jaw_width,
                height=moving_jaw_height,
                thickness=moving_jaw_thickness,
                bar_hole_size=guide_bar_size + 0.004,
                bar_center_x=guide_bar_center_x,
                screw_hole_diameter=screw_hole_diameter + 0.010,
                corner_radius=0.014,
            ),
        ),
        material=ash_wood,
        name="jaw_body",
    )
    for side_name, bar_x in (("left", -guide_bar_center_x), ("right", guide_bar_center_x)):
        moving_jaw.visual(
            Box((socket_wall, socket_depth, guide_bar_size)),
            origin=Origin(xyz=(bar_x - guide_bar_size * 0.5 - socket_wall * 0.5, socket_center_y, 0.0)),
            material=walnut_wood,
            name=f"{side_name}_socket_left",
        )
        moving_jaw.visual(
            Box((socket_wall, socket_depth, guide_bar_size)),
            origin=Origin(xyz=(bar_x + guide_bar_size * 0.5 + socket_wall * 0.5, socket_center_y, 0.0)),
            material=walnut_wood,
            name=f"{side_name}_socket_right",
        )
        moving_jaw.visual(
            Box((socket_outer, socket_depth, socket_wall)),
            origin=Origin(xyz=(bar_x, socket_center_y, top_strip_z)),
            material=walnut_wood,
            name=f"{side_name}_socket_top",
        )
        moving_jaw.visual(
            Box((socket_outer, socket_depth, socket_wall)),
            origin=Origin(xyz=(bar_x, socket_center_y, -top_strip_z)),
            material=walnut_wood,
            name=f"{side_name}_socket_bottom",
        )
    moving_jaw.visual(
        Box((0.12, socket_depth, 0.034)),
        origin=Origin(xyz=(-0.085, socket_center_y, 0.055)),
        material=walnut_wood,
        name="upper_left_rib",
    )
    moving_jaw.visual(
        Box((0.12, socket_depth, 0.034)),
        origin=Origin(xyz=(0.085, socket_center_y, 0.055)),
        material=walnut_wood,
        name="upper_right_rib",
    )
    moving_jaw.visual(
        Box((0.12, socket_depth, 0.034)),
        origin=Origin(xyz=(-0.085, socket_center_y, -0.055)),
        material=walnut_wood,
        name="lower_left_rib",
    )
    moving_jaw.visual(
        Box((0.12, socket_depth, 0.034)),
        origin=Origin(xyz=(0.085, socket_center_y, -0.055)),
        material=walnut_wood,
        name="lower_right_rib",
    )
    screw_assembly = model.part("screw_assembly")
    screw_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=0.46),
        mass=3.2,
        origin=Origin(
            xyz=(0.0, -0.06, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
    )
    threaded_rear_y = -0.255
    threaded_front_y = 0.026
    threaded_center_y = (threaded_rear_y + threaded_front_y) * 0.5
    threaded_length = threaded_front_y - threaded_rear_y
    front_hub_length = 0.024
    front_hub_center_y = moving_jaw_thickness * 0.5 + front_hub_length * 0.5
    wheel_center_local_y = moving_jaw_thickness * 0.5 + 0.099
    spindle_length = wheel_center_local_y - (front_hub_center_y + front_hub_length * 0.5) + 0.032
    spindle_start_y = front_hub_center_y + front_hub_length * 0.5 - 0.002
    spindle_center_y = spindle_start_y + spindle_length * 0.5
    screw_assembly.visual(
        _mesh(
            "threaded_shaft",
            _threaded_screw_mesh(
                length=threaded_length,
                root_radius=screw_root_radius,
                crest_radius=screw_crest_radius,
                pitch=screw_pitch,
            ),
        ),
        origin=Origin(xyz=(0.0, threaded_center_y, 0.0)),
        material=blackened_steel,
        name="threaded_shaft",
    )
    screw_assembly.visual(
        Cylinder(radius=0.034, length=front_hub_length),
        origin=Origin(xyz=(0.0, front_hub_center_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="front_hub",
    )
    screw_assembly.visual(
        Cylinder(radius=0.018, length=spindle_length),
        origin=Origin(
            xyz=(0.0, spindle_center_y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=blackened_steel,
        name="wheel_spindle",
    )
    screw_assembly.visual(
        _mesh(
            "handwheel_rim",
            TorusGeometry(radius=wheel_radius, tube=wheel_tube, radial_segments=18, tubular_segments=56),
        ),
        origin=Origin(xyz=(0.0, wheel_center_local_y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=handle_wood,
        name="handwheel_rim",
    )
    screw_assembly.visual(
        Cylinder(radius=0.037, length=0.038),
        origin=Origin(xyz=(0.0, wheel_center_local_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_wood,
        name="handwheel_hub",
    )
    for index, angle in enumerate((0.0, pi / 4.0, pi / 2.0, -pi / 4.0)):
        screw_assembly.visual(
            Box((0.138, 0.014, 0.020)),
            origin=Origin(xyz=(0.0, wheel_center_local_y, 0.0), rpy=(0.0, angle, 0.0)),
            material=handle_wood,
            name=f"handwheel_spoke_{index}",
        )
    screw_assembly.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(0.055, wheel_center_local_y, -0.040), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_wood,
        name="handwheel_grip",
    )
    screw_assembly.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.055, wheel_center_local_y + 0.026, -0.040)),
        material=handle_wood,
        name="handwheel_grip_knob_front",
    )
    screw_assembly.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.055, wheel_center_local_y - 0.026, -0.040)),
        material=handle_wood,
        name="handwheel_grip_knob_back",
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=moving_jaw,
        origin=Origin(xyz=(0.0, jaw_center_closed_y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.18,
            lower=0.0,
            upper=jaw_travel,
        ),
    )
    model.articulation(
        "screw_spin",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=screw_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    moving_jaw = object_model.get_part("moving_jaw")
    screw_assembly = object_model.get_part("screw_assembly")
    jaw_slide = object_model.get_articulation("jaw_slide")
    screw_spin = object_model.get_articulation("screw_spin")

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
        "jaw_slide_axis_is_depth",
        tuple(jaw_slide.axis) == (0.0, 1.0, 0.0),
        details=f"jaw_slide axis was {jaw_slide.axis}",
    )
    ctx.check(
        "screw_spin_axis_is_depth",
        tuple(screw_spin.axis) == (0.0, 1.0, 0.0),
        details=f"screw_spin axis was {screw_spin.axis}",
    )
    jaw_limits = jaw_slide.motion_limits
    ctx.check(
        "jaw_slide_has_realistic_travel",
        jaw_limits is not None and jaw_limits.lower == 0.0 and jaw_limits.upper is not None and jaw_limits.upper >= 0.20,
        details="jaw prismatic articulation should open at least 0.20 m from a closed stop",
    )

    ctx.expect_contact(
        moving_jaw,
        frame,
        elem_a="left_socket_top",
        elem_b="guide_bar_left",
        name="jaw_contacts_left_guide_bar",
    )
    ctx.expect_contact(
        moving_jaw,
        frame,
        elem_a="right_socket_top",
        elem_b="guide_bar_right",
        name="jaw_contacts_right_guide_bar",
    )
    ctx.expect_contact(
        screw_assembly,
        moving_jaw,
        elem_a="front_hub",
        elem_b="jaw_body",
        name="screw_is_captured_by_moving_jaw",
    )

    with ctx.pose({jaw_slide: 0.0}):
        ctx.expect_gap(
            moving_jaw,
            frame,
            axis="y",
            positive_elem="jaw_body",
            negative_elem="fixed_jaw_face",
            min_gap=0.005,
            max_gap=0.008,
            name="closed_jaw_face_gap",
        )
        ctx.expect_overlap(
            moving_jaw,
            frame,
            axes="xz",
            elem_a="jaw_body",
            elem_b="fixed_jaw_face",
            min_overlap=0.14,
            name="closed_jaws_align_in_width_and_height",
        )

    with ctx.pose({jaw_slide: 0.20}):
        ctx.expect_gap(
            moving_jaw,
            frame,
            axis="y",
            positive_elem="jaw_body",
            negative_elem="fixed_jaw_face",
            min_gap=0.205,
            max_gap=0.208,
            name="open_jaw_face_gap",
        )
        ctx.expect_overlap(
            moving_jaw,
            frame,
            axes="xz",
            elem_a="jaw_body",
            elem_b="fixed_jaw_face",
            min_overlap=0.14,
            name="open_jaws_stay_parallel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
