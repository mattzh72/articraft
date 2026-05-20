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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stockroom_hand_truck")

    frame_red = model.material("frame_red", rgba=(0.76, 0.11, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _build_wheel(part, *, tire_radius: float, tire_width: float) -> None:
        half_width = tire_width * 0.5
        tire_profile = [
            (tire_radius * 0.42, -half_width * 0.98),
            (tire_radius * 0.64, -half_width),
            (tire_radius * 0.88, -half_width * 0.80),
            (tire_radius * 0.98, -half_width * 0.28),
            (tire_radius, 0.0),
            (tire_radius * 0.98, half_width * 0.28),
            (tire_radius * 0.88, half_width * 0.80),
            (tire_radius * 0.64, half_width),
            (tire_radius * 0.42, half_width * 0.98),
            (tire_radius * 0.38, 0.0),
            (tire_radius * 0.42, -half_width * 0.98),
        ]
        tire_mesh = _mesh(
            f"{part.name}_tire_mesh",
            LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0),
        )
        spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))

        part.visual(tire_mesh, material=tire_rubber, name="tire")
        part.visual(
            Cylinder(radius=tire_radius * 0.70, length=0.008),
            origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="inner_rim",
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.70, length=0.008),
            origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="outer_rim",
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.31, length=0.054),
            origin=spin_origin,
            material=dark_steel,
            name="hub",
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.13, length=0.062),
            origin=spin_origin,
            material=steel,
            name="hub_cap",
        )
        part.visual(
            Box((0.010, 0.005, 0.014)),
            origin=Origin(xyz=(0.018, 0.0, tire_radius - 0.004)),
            material=steel,
            name="valve",
        )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.52, 0.28, 1.20)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )

    frame_outline = [
        (-0.185, 0.040, 0.135),
        (-0.185, 0.015, 0.360),
        (-0.182, 0.000, 0.720),
        (-0.140, -0.005, 1.030),
        (-0.060, 0.005, 1.155),
        (0.060, 0.005, 1.155),
        (0.140, -0.005, 1.030),
        (0.182, 0.000, 0.720),
        (0.185, 0.015, 0.360),
        (0.185, 0.040, 0.135),
    ]
    frame.visual(
        _mesh(
            "hand_truck_main_frame",
            tube_from_spline_points(
                frame_outline,
                radius=0.022,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_red,
        name="main_tube",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.370),
        origin=Origin(xyz=(0.0, 0.040, 0.135), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="axle_cross",
    )
    frame.visual(
        Box((0.362, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, -0.012, 0.480)),
        material=frame_red,
        name="lower_backplate",
    )
    frame.visual(
        Box((0.342, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, -0.012, 0.770)),
        material=frame_red,
        name="upper_backplate",
    )
    frame.visual(
        Box((0.070, 0.018, 0.300)),
        origin=Origin(xyz=(0.0, -0.012, 0.625)),
        material=frame_red,
        name="center_backplate",
    )
    frame.visual(
        Box((0.380, 0.180, 0.008)),
        origin=Origin(xyz=(0.0, -0.095, 0.034)),
        material=frame_red,
        name="toe_plate",
    )
    frame.visual(
        Box((0.380, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, -0.182, 0.048)),
        material=frame_red,
        name="toe_lip",
    )
    frame.visual(
        Box((0.018, 0.180, 0.026)),
        origin=Origin(xyz=(-0.181, -0.095, 0.047)),
        material=frame_red,
        name="left_toe_flange",
    )
    frame.visual(
        Box((0.018, 0.180, 0.026)),
        origin=Origin(xyz=(0.181, -0.095, 0.047)),
        material=frame_red,
        name="right_toe_flange",
    )
    frame.visual(
        _mesh(
            "hand_truck_left_toe_brace",
            tube_from_spline_points(
                [
                    (-0.185, 0.040, 0.135),
                    (-0.172, 0.006, 0.082),
                    (-0.160, -0.005, 0.040),
                ],
                radius=0.016,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_red,
        name="left_toe_brace",
    )
    frame.visual(
        _mesh(
            "hand_truck_right_toe_brace",
            tube_from_spline_points(
                [
                    (0.185, 0.040, 0.135),
                    (0.172, 0.006, 0.082),
                    (0.160, -0.005, 0.040),
                ],
                radius=0.016,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=frame_red,
        name="right_toe_brace",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.320),
        origin=Origin(xyz=(0.0, -0.005, 0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="toe_rear_tube",
    )
    frame.visual(
        Box((0.040, 0.060, 0.085)),
        origin=Origin(xyz=(-0.190, 0.055, 0.135)),
        material=dark_steel,
        name="left_axle_bracket",
    )
    frame.visual(
        Box((0.040, 0.060, 0.085)),
        origin=Origin(xyz=(0.190, 0.055, 0.135)),
        material=dark_steel,
        name="right_axle_bracket",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(-0.215, 0.055, 0.135), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_axle_pin",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.215, 0.055, 0.135), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_axle_pin",
    )
    frame.visual(
        Box((0.090, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.048, 0.110)),
        material=dark_steel,
        name="kickstand_bracket",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.055),
        mass=2.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_wheel(left_wheel, tire_radius=0.135, tire_width=0.055)

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.055),
        mass=2.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_wheel(right_wheel, tire_radius=0.135, tire_width=0.055)

    kickstand = model.part("kickstand")
    kickstand.inertial = Inertial.from_geometry(
        Box((0.08, 0.04, 0.20)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
    )
    kickstand.visual(
        Cylinder(radius=0.012, length=0.056),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    kickstand.visual(
        Cylinder(radius=0.010, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=dark_steel,
        name="leg_tube",
    )
    kickstand.visual(
        Box((0.070, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=foot_rubber,
        name="foot_pad",
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(-0.247, 0.055, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(0.247, 0.055, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=kickstand,
        origin=Origin(xyz=(0.0, 0.048, 0.083), rpy=(0.55, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=0.85),
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
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    kickstand = object_model.get_part("kickstand")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    kickstand_hinge = object_model.get_articulation("kickstand_hinge")

    def _center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.check(
        "wheel joints are continuous axle spins",
        left_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_wheel_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=(
            f"left={left_wheel_spin.articulation_type}/{left_wheel_spin.axis}, "
            f"right={right_wheel_spin.articulation_type}/{right_wheel_spin.axis}"
        ),
    )
    ctx.expect_contact(
        left_wheel,
        frame,
        elem_a="hub",
        elem_b="left_axle_pin",
        contact_tol=0.001,
        name="left wheel hub meets the left axle pin",
    )
    ctx.expect_contact(
        right_wheel,
        frame,
        elem_a="hub",
        elem_b="right_axle_pin",
        contact_tol=0.001,
        name="right wheel hub meets the right axle pin",
    )
    ctx.expect_contact(
        kickstand,
        frame,
        elem_a="hinge_barrel",
        elem_b="kickstand_bracket",
        contact_tol=0.001,
        name="kickstand barrel mounts beneath the frame bracket",
    )

    left_valve_rest = _center(ctx.part_element_world_aabb(left_wheel, elem="valve"))
    with ctx.pose({left_wheel_spin: pi / 2.0}):
        left_valve_quarter_turn = _center(ctx.part_element_world_aabb(left_wheel, elem="valve"))
    ctx.check(
        "left wheel visibly spins around the axle",
        left_valve_rest is not None
        and left_valve_quarter_turn is not None
        and abs(left_valve_quarter_turn[0] - left_valve_rest[0]) < 0.01
        and abs(left_valve_quarter_turn[1] - left_valve_rest[1]) > 0.06
        and abs(left_valve_quarter_turn[2] - left_valve_rest[2]) > 0.06,
        details=f"rest={left_valve_rest}, quarter_turn={left_valve_quarter_turn}",
    )

    kickstand_foot_rest = _center(ctx.part_element_world_aabb(kickstand, elem="foot_pad"))
    with ctx.pose({kickstand_hinge: 0.75}):
        kickstand_foot_folded = _center(ctx.part_element_world_aabb(kickstand, elem="foot_pad"))
    ctx.check(
        "kickstand leg rotates rearward and upward",
        kickstand_foot_rest is not None
        and kickstand_foot_folded is not None
        and abs(kickstand_foot_folded[0] - kickstand_foot_rest[0]) < 0.02
        and kickstand_foot_folded[1] > kickstand_foot_rest[1] + 0.03
        and kickstand_foot_folded[2] > kickstand_foot_rest[2] + 0.03,
        details=f"rest={kickstand_foot_rest}, folded={kickstand_foot_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
