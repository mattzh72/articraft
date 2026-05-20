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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _add_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    radius: float,
    width: float,
    hub_width: float,
    hub_radius: float,
    tire_material,
    rim_material,
    hub_material,
) -> None:
    half_width = width * 0.5
    tire_profile = [
        (radius * 0.48, -half_width * 0.98),
        (radius * 0.70, -half_width),
        (radius * 0.89, -half_width * 0.80),
        (radius * 0.97, -half_width * 0.38),
        (radius, -half_width * 0.10),
        (radius, half_width * 0.10),
        (radius * 0.97, half_width * 0.38),
        (radius * 0.89, half_width * 0.80),
        (radius * 0.70, half_width),
        (radius * 0.48, half_width * 0.98),
        (radius * 0.38, half_width * 0.25),
        (radius * 0.34, 0.0),
        (radius * 0.38, -half_width * 0.25),
        (radius * 0.48, -half_width * 0.98),
    ]
    tire_mesh = _mesh(
        f"{mesh_prefix}_tire",
        LatheGeometry(tire_profile, segments=56).rotate_y(math.pi / 2.0),
    )
    part.visual(tire_mesh, material=tire_material, name="tire")

    rim_radius = radius * 0.78
    rim_outer = superellipse_profile(
        rim_radius * 2.0,
        rim_radius * 2.0,
        exponent=2.0,
        segments=36,
    )
    spoke_window = rounded_rect_profile(
        rim_radius * 0.22,
        rim_radius * 0.38,
        rim_radius * 0.05,
        corner_segments=5,
    )
    hole_profiles = [
        superellipse_profile(rim_radius * 0.56, rim_radius * 0.56, exponent=2.0, segments=24)
    ]
    for spoke_index in range(5):
        angle = 2.0 * math.pi * spoke_index / 5.0
        hole_profiles.append(
            _transform_profile(
                spoke_window,
                dx=math.cos(angle) * rim_radius * 0.28,
                dy=math.sin(angle) * rim_radius * 0.28,
                angle=angle,
            )
        )
    rim_face_mesh = _mesh(
        f"{mesh_prefix}_rim_face",
        ExtrudeWithHolesGeometry(
            rim_outer,
            hole_profiles,
            height=width * 0.18,
            center=True,
        ).rotate_y(math.pi / 2.0),
    )
    part.visual(
        rim_face_mesh,
        origin=Origin(xyz=(width * 0.18, 0.0, 0.0)),
        material=rim_material,
        name="rim_outer_face",
    )
    part.visual(
        rim_face_mesh,
        origin=Origin(xyz=(-width * 0.18, 0.0, 0.0)),
        material=rim_material,
        name="rim_inner_face",
    )
    part.visual(
        Cylinder(radius=radius * 0.40, length=width * 0.44),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.55, length=hub_width * 0.65),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rim_material,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rehab_knee_scooter")

    frame_blue = model.material("frame_blue", rgba=(0.14, 0.29, 0.56, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.14, 0.15, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.79, 0.80, 0.82, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.33, 0.35, 0.38, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    pad_base = model.material("pad_base", rgba=(0.30, 0.31, 0.33, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.56, 0.70, 0.98)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.04, 0.49)),
    )

    rear_cradle = frame.visual(
        Box((0.32, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, -0.21, 0.165)),
        material=frame_blue,
        name="rear_axle_cradle",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.042),
        origin=Origin(xyz=(0.181, -0.21, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="rear_left_axle_stub",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.042),
        origin=Origin(xyz=(-0.181, -0.21, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="rear_right_axle_stub",
    )

    left_rail_points = [
        (0.10, -0.205, 0.185),
        (0.11, -0.12, 0.27),
        (0.10, 0.00, 0.285),
        (0.075, 0.16, 0.24),
        (0.050, 0.295, 0.195),
    ]
    frame.visual(
        _mesh(
            "knee_scooter_left_rail",
            tube_from_spline_points(
                left_rail_points,
                radius=0.021,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_blue,
        name="left_lower_rail",
    )
    frame.visual(
        _mesh(
            "knee_scooter_right_rail",
            tube_from_spline_points(
                _mirror_x(left_rail_points),
                radius=0.021,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_blue,
        name="right_lower_rail",
    )
    frame.visual(
        _mesh(
            "knee_scooter_center_spine",
            tube_from_spline_points(
                [
                    (0.0, -0.205, 0.185),
                    (0.0, -0.08, 0.27),
                    (0.0, 0.05, 0.29),
                    (0.0, 0.19, 0.245),
                    (0.0, 0.295, 0.195),
                ],
                radius=0.020,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_blue,
        name="center_spine",
    )

    nose_plate_profile = rounded_rect_profile(0.30, 0.085, 0.020, corner_segments=8)
    caster_hole = superellipse_profile(0.040, 0.040, exponent=2.0, segments=24)
    nose_plate_mesh = _mesh(
        "knee_scooter_nose_plate",
        ExtrudeWithHolesGeometry(
            nose_plate_profile,
            [
                _transform_profile(caster_hole, dx=-0.115, dy=0.0),
                _transform_profile(caster_hole, dx=0.115, dy=0.0),
            ],
            height=0.016,
            center=True,
        ),
    )
    frame.visual(
        nose_plate_mesh,
        origin=Origin(xyz=(0.0, 0.295, 0.195)),
        material=frame_blue,
        name="nose_plate",
    )
    frame.visual(
        Box((0.08, 0.055, 0.065)),
        origin=Origin(xyz=(0.0, 0.265, 0.228)),
        material=frame_blue,
        name="column_gusset",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.73),
        origin=Origin(xyz=(0.0, 0.275, 0.565)),
        material=frame_blue,
        name="handlebar_column",
    )
    frame.visual(
        _mesh(
            "knee_scooter_column_brace",
            tube_from_spline_points(
                [
                    (0.0, 0.10, 0.275),
                    (0.0, 0.18, 0.32),
                    (0.0, 0.255, 0.29),
                ],
                radius=0.016,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_blue,
        name="column_brace",
    )
    frame.visual(
        Box((0.08, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.272, 0.925)),
        material=frame_blue,
        name="handlebar_clamp",
    )
    frame.visual(
        _mesh(
            "knee_scooter_handlebar",
            tube_from_spline_points(
                [
                    (-0.25, 0.246, 0.910),
                    (-0.16, 0.262, 0.930),
                    (-0.05, 0.272, 0.938),
                    (0.05, 0.272, 0.938),
                    (0.16, 0.262, 0.930),
                    (0.25, 0.246, 0.910),
                ],
                radius=0.014,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=frame_blue,
        name="handlebar_bar",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(0.275, 0.244, 0.907), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="right_grip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(-0.275, 0.244, 0.907), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="left_grip",
    )

    left_pad_strut_points = [
        (0.085, -0.02, 0.275),
        (0.078, -0.02, 0.40),
        (0.070, -0.02, 0.507),
    ]
    frame.visual(
        _mesh(
            "knee_scooter_left_pad_strut",
            tube_from_spline_points(
                left_pad_strut_points,
                radius=0.015,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_blue,
        name="left_pad_strut",
    )
    frame.visual(
        _mesh(
            "knee_scooter_right_pad_strut",
            tube_from_spline_points(
                _mirror_x(left_pad_strut_points),
                radius=0.015,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_blue,
        name="right_pad_strut",
    )
    frame.visual(
        Box((0.18, 0.10, 0.012)),
        origin=Origin(xyz=(0.0, -0.03, 0.509)),
        material=frame_blue,
        name="pad_support_plate",
    )

    knee_pad = model.part("knee_pad")
    knee_pad.visual(
        Box((0.29, 0.15, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=pad_base,
        name="pad_board",
    )
    knee_pad.visual(
        _mesh(
            "knee_scooter_pad_cushion",
            ExtrudeGeometry(
                rounded_rect_profile(0.30, 0.16, 0.028, corner_segments=10),
                0.045,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=pad_vinyl,
        name="pad_cushion",
    )
    knee_pad.inertial = Inertial.from_geometry(
        Box((0.30, 0.16, 0.055)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(
        rear_left_wheel,
        mesh_prefix="knee_scooter_rear_left_wheel",
        radius=0.125,
        width=0.055,
        hub_width=0.026,
        hub_radius=0.030,
        tire_material=dark_rubber,
        rim_material=wheel_silver,
        hub_material=steel_dark,
    )
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.055),
        mass=1.7,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(
        rear_right_wheel,
        mesh_prefix="knee_scooter_rear_right_wheel",
        radius=0.125,
        width=0.055,
        hub_width=0.026,
        hub_radius=0.030,
        tire_material=dark_rubber,
        rim_material=wheel_silver,
        hub_material=steel_dark,
    )
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.055),
        mass=1.7,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    front_left_caster = model.part("front_left_caster")
    front_left_caster.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=steel_dark,
        name="swivel_flange",
    )
    front_left_caster.visual(
        Cylinder(radius=0.013, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        material=steel_dark,
        name="stem",
    )
    front_left_caster.visual(
        Box((0.040, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=satin_black,
        name="fork_crown",
    )
    front_left_caster.visual(
        Box((0.018, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=satin_black,
        name="offset_neck",
    )
    front_left_caster.visual(
        Box((0.010, 0.120, 0.092)),
        origin=Origin(xyz=(0.020, -0.060, -0.126)),
        material=satin_black,
        name="left_fork_arm",
    )
    front_left_caster.visual(
        Box((0.010, 0.120, 0.092)),
        origin=Origin(xyz=(-0.020, -0.060, -0.126)),
        material=satin_black,
        name="right_fork_arm",
    )
    front_left_caster.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.014, -0.100, -0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="left_axle_boss",
    )
    front_left_caster.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(-0.014, -0.100, -0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="right_axle_boss",
    )
    front_left_caster.inertial = Inertial.from_geometry(
        Box((0.050, 0.060, 0.180)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.015, -0.090)),
    )

    front_right_caster = model.part("front_right_caster")
    front_right_caster.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=steel_dark,
        name="swivel_flange",
    )
    front_right_caster.visual(
        Cylinder(radius=0.013, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        material=steel_dark,
        name="stem",
    )
    front_right_caster.visual(
        Box((0.040, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=satin_black,
        name="fork_crown",
    )
    front_right_caster.visual(
        Box((0.018, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=satin_black,
        name="offset_neck",
    )
    front_right_caster.visual(
        Box((0.010, 0.120, 0.092)),
        origin=Origin(xyz=(0.020, -0.060, -0.126)),
        material=satin_black,
        name="left_fork_arm",
    )
    front_right_caster.visual(
        Box((0.010, 0.120, 0.092)),
        origin=Origin(xyz=(-0.020, -0.060, -0.126)),
        material=satin_black,
        name="right_fork_arm",
    )
    front_right_caster.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.014, -0.100, -0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="left_axle_boss",
    )
    front_right_caster.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(-0.014, -0.100, -0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="right_axle_boss",
    )
    front_right_caster.inertial = Inertial.from_geometry(
        Box((0.050, 0.060, 0.180)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.015, -0.090)),
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(
        front_left_wheel,
        mesh_prefix="knee_scooter_front_left_wheel",
        radius=0.085,
        width=0.032,
        hub_width=0.020,
        hub_radius=0.020,
        tire_material=dark_rubber,
        rim_material=wheel_silver,
        hub_material=steel_dark,
    )
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.032),
        mass=0.75,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(
        front_right_wheel,
        mesh_prefix="knee_scooter_front_right_wheel",
        radius=0.085,
        width=0.032,
        hub_width=0.020,
        hub_radius=0.020,
        tire_material=dark_rubber,
        rim_material=wheel_silver,
        hub_material=steel_dark,
    )
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.032),
        mass=0.75,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_knee_pad",
        ArticulationType.FIXED,
        parent=frame,
        child=knee_pad,
        origin=Origin(xyz=(0.0, -0.03, 0.515)),
    )
    model.articulation(
        "frame_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.215, -0.21, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.215, -0.21, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_front_left_caster",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_left_caster,
        origin=Origin(xyz=(0.115, 0.295, 0.187)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "frame_to_front_right_caster",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_right_caster,
        origin=Origin(xyz=(-0.115, 0.295, 0.187)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "front_left_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_left_caster,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, -0.100, -0.103)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "front_right_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_right_caster,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, -0.100, -0.103)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    knee_pad = object_model.get_part("knee_pad")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_caster = object_model.get_part("front_left_caster")
    front_right_caster = object_model.get_part("front_right_caster")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")

    rear_left_spin = object_model.get_articulation("frame_to_rear_left_wheel")
    rear_right_spin = object_model.get_articulation("frame_to_rear_right_wheel")
    front_left_swivel = object_model.get_articulation("frame_to_front_left_caster")
    front_right_swivel = object_model.get_articulation("frame_to_front_right_caster")
    front_left_spin = object_model.get_articulation("front_left_caster_to_wheel")
    front_right_spin = object_model.get_articulation("front_right_caster_to_wheel")

    ctx.expect_contact(
        knee_pad,
        frame,
        elem_a="pad_board",
        elem_b="pad_support_plate",
        name="knee pad sits on the support plate",
    )
    ctx.expect_contact(
        rear_left_wheel,
        frame,
        elem_a="hub",
        elem_b="rear_left_axle_stub",
        name="left rear wheel rides on a supported axle stub",
    )
    ctx.expect_contact(
        rear_right_wheel,
        frame,
        elem_a="hub",
        elem_b="rear_right_axle_stub",
        name="right rear wheel rides on a supported axle stub",
    )
    ctx.expect_contact(
        front_left_wheel,
        front_left_caster,
        elem_a="hub",
        elem_b="left_axle_boss",
        name="left front wheel is carried by the caster fork",
    )
    ctx.expect_contact(
        front_right_wheel,
        front_right_caster,
        elem_a="hub",
        elem_b="right_axle_boss",
        name="right front wheel is carried by the caster fork",
    )
    ctx.expect_origin_distance(
        rear_left_wheel,
        rear_right_wheel,
        axes="x",
        min_dist=0.40,
        max_dist=0.46,
        name="rear wheel track matches a compact scooter stance",
    )
    ctx.expect_origin_gap(
        front_left_caster,
        knee_pad,
        axis="y",
        min_gap=0.25,
        name="front caster pivots sit ahead of the knee pad",
    )
    ctx.expect_origin_gap(
        knee_pad,
        frame,
        axis="z",
        min_gap=0.50,
        max_gap=0.54,
        name="knee pad rides above the frame at a usable height",
    )

    ctx.check(
        "rear wheels spin on lateral axle axes",
        rear_left_spin.axis == (1.0, 0.0, 0.0) and rear_right_spin.axis == (1.0, 0.0, 0.0),
        details=f"left={rear_left_spin.axis}, right={rear_right_spin.axis}",
    )
    ctx.check(
        "front caster stems swivel on vertical pivots",
        front_left_swivel.axis == (0.0, 0.0, 1.0) and front_right_swivel.axis == (0.0, 0.0, 1.0),
        details=f"left={front_left_swivel.axis}, right={front_right_swivel.axis}",
    )
    ctx.check(
        "front caster wheels spin on cross axles",
        front_left_spin.axis == (1.0, 0.0, 0.0) and front_right_spin.axis == (1.0, 0.0, 0.0),
        details=f"left={front_left_spin.axis}, right={front_right_spin.axis}",
    )

    left_rest = ctx.part_world_position(front_left_wheel)
    right_rest = ctx.part_world_position(front_right_wheel)
    with ctx.pose(
        {
            front_left_swivel: math.radians(35.0),
            front_right_swivel: math.radians(-35.0),
        }
    ):
        left_turned = ctx.part_world_position(front_left_wheel)
        right_turned = ctx.part_world_position(front_right_wheel)
    ctx.check(
        "front casters swivel independently about their own stems",
        left_rest is not None
        and right_rest is not None
        and left_turned is not None
        and right_turned is not None
        and left_turned[0] > left_rest[0] + 0.01
        and right_turned[0] < right_rest[0] - 0.01,
        details=(
            f"left_rest={left_rest}, left_turned={left_turned}, "
            f"right_rest={right_rest}, right_turned={right_turned}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
