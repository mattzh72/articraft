from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


FRAME_HALF_WIDTH = 0.24
WHEEL_CENTER_X = 0.36
WHEEL_RADIUS = 0.18
WHEEL_WIDTH = 0.08
AXLE_Y = -0.05
AXLE_Z = WHEEL_RADIUS
PIVOT_Y = 0.03
PIVOT_Z = 0.93


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _wheel_tire_geometry(radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.38, -half_width * 0.94),
        (radius * 0.74, -half_width),
        (radius * 0.91, -half_width * 0.72),
        (radius * 1.00, -half_width * 0.20),
        (radius * 1.00, half_width * 0.20),
        (radius * 0.91, half_width * 0.72),
        (radius * 0.74, half_width),
        (radius * 0.38, half_width * 0.94),
        (radius * 0.31, half_width * 0.28),
        (radius * 0.29, 0.0),
        (radius * 0.31, -half_width * 0.28),
    ]
    return LatheGeometry(profile, segments=60).rotate_y(math.pi / 2.0)


def _wheel_rim_geometry(radius: float, width: float):
    geom = CylinderGeometry(radius * 0.60, width * 0.66).rotate_y(math.pi / 2.0)
    geom.merge(
        CylinderGeometry(radius * 0.66, 0.010)
        .rotate_y(math.pi / 2.0)
        .translate(width * 0.20, 0.0, 0.0)
    )
    geom.merge(
        CylinderGeometry(radius * 0.66, 0.010)
        .rotate_y(math.pi / 2.0)
        .translate(-width * 0.20, 0.0, 0.0)
    )
    geom.merge(CylinderGeometry(radius * 0.28, width).rotate_y(math.pi / 2.0))
    geom.merge(CylinderGeometry(radius * 0.12, width * 0.46).rotate_y(math.pi / 2.0))
    return geom


def _add_wheel_visuals(part, mesh_prefix: str, *, tire_radius: float, tire_width: float, tire_material, rim_material) -> None:
    part.visual(
        _save_mesh(f"{mesh_prefix}_tire", _wheel_tire_geometry(tire_radius, tire_width)),
        material=tire_material,
        name="tire",
    )
    part.visual(
        _save_mesh(f"{mesh_prefix}_rim", _wheel_rim_geometry(tire_radius, tire_width)),
        material=rim_material,
        name="rim",
    )
    part.visual(
        Box((0.012, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, tire_radius * 0.56, 0.0)),
        material=rim_material,
        name="valve_stem",
    )


def _frame_body_geometry():
    geom = BoxGeometry((0.50, 0.034, 0.060)).translate(0.0, -0.032, 0.078)
    geom.merge(BoxGeometry((0.12, 0.040, 0.200)).translate(0.19, -0.050, 0.170))
    geom.merge(BoxGeometry((0.12, 0.040, 0.200)).translate(-0.19, -0.050, 0.170))
    geom.merge(
        tube_from_spline_points(
            [
                (FRAME_HALF_WIDTH - 0.005, -0.033, 0.080),
                (FRAME_HALF_WIDTH - 0.002, -0.035, 0.380),
                (FRAME_HALF_WIDTH + 0.007, -0.045, 0.820),
                (FRAME_HALF_WIDTH + 0.015, -0.095, 1.280),
                (0.205, -0.130, 1.460),
            ],
            radius=0.020,
            samples_per_segment=14,
            radial_segments=18,
        )
    )
    geom.merge(
        tube_from_spline_points(
            _mirror_x(
                [
                    (FRAME_HALF_WIDTH - 0.005, -0.033, 0.080),
                    (FRAME_HALF_WIDTH - 0.002, -0.035, 0.380),
                    (FRAME_HALF_WIDTH + 0.007, -0.045, 0.820),
                    (FRAME_HALF_WIDTH + 0.015, -0.095, 1.280),
                    (0.205, -0.130, 1.460),
                ]
            ),
            radius=0.020,
            samples_per_segment=14,
            radial_segments=18,
        )
    )
    geom.merge(
        tube_from_spline_points(
            [
                (FRAME_HALF_WIDTH - 0.010, -0.012, 0.082),
                (FRAME_HALF_WIDTH + 0.015, -0.032, 0.120),
                (FRAME_HALF_WIDTH + 0.030, -0.048, AXLE_Z),
            ],
            radius=0.016,
            samples_per_segment=8,
            radial_segments=16,
        )
    )
    geom.merge(
        tube_from_spline_points(
            _mirror_x(
                [
                    (FRAME_HALF_WIDTH - 0.010, -0.012, 0.082),
                    (FRAME_HALF_WIDTH + 0.015, -0.032, 0.120),
                    (FRAME_HALF_WIDTH + 0.030, -0.048, AXLE_Z),
                ]
            ),
            radius=0.016,
            samples_per_segment=8,
            radial_segments=16,
        )
    )
    geom.merge(CylinderGeometry(0.016, 0.44).rotate_y(math.pi / 2.0).translate(0.0, -0.038, 0.46))
    geom.merge(CylinderGeometry(0.016, 0.46).rotate_y(math.pi / 2.0).translate(0.0, -0.050, 0.80))
    geom.merge(CylinderGeometry(0.015, 0.36).rotate_y(math.pi / 2.0).translate(0.0, -0.115, 1.330))
    return geom


def _pivot_hardware_geometry(side: float):
    x_pos = side * FRAME_HALF_WIDTH
    geom = BoxGeometry((0.022, 0.038, 0.160)).translate(x_pos, -0.004, PIVOT_Z)
    geom.merge(BoxGeometry((0.030, 0.060, 0.050)).translate(x_pos, -0.018, PIVOT_Z - 0.048))
    return geom


def _clamp_arm_geometry(side: float):
    return tube_from_spline_points(
        [
            (side * FRAME_HALF_WIDTH, 0.042, -0.010),
            (side * (FRAME_HALF_WIDTH - 0.006), 0.082, -0.040),
            (side * (FRAME_HALF_WIDTH - 0.018), 0.122, -0.105),
            (side * (FRAME_HALF_WIDTH - 0.026), 0.160, -0.180),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=16,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="appliance_hand_truck")

    frame_blue = model.material("frame_blue", rgba=(0.22, 0.30, 0.41, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    pad_gray = model.material("pad_gray", rgba=(0.20, 0.22, 0.23, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.56, 0.24, 0.012)),
        origin=Origin(xyz=(0.0, 0.120, 0.006)),
        material=dark_steel,
        name="toe_plate",
    )
    frame.visual(
        Box((0.56, 0.022, 0.075)),
        origin=Origin(xyz=(0.0, -0.011, 0.0375)),
        material=frame_blue,
        name="toe_lip",
    )
    frame.visual(
        _save_mesh("frame_body", _frame_body_geometry()),
        material=frame_blue,
        name="frame_body",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.64),
        origin=Origin(xyz=(0.0, AXLE_Y, AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_beam",
    )
    frame.visual(
        _save_mesh("left_pivot_hardware", _pivot_hardware_geometry(1.0)),
        material=steel,
        name="left_pivot_hardware",
    )
    frame.visual(
        _save_mesh("right_pivot_hardware", _pivot_hardware_geometry(-1.0)),
        material=steel,
        name="right_pivot_hardware",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.38, 1.50)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.03, 0.75)),
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(
        left_wheel,
        "left_wheel",
        tire_radius=WHEEL_RADIUS,
        tire_width=WHEEL_WIDTH,
        tire_material=rubber,
        rim_material=steel,
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=3.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(
        right_wheel,
        "right_wheel",
        tire_radius=WHEEL_RADIUS,
        tire_width=WHEEL_WIDTH,
        tire_material=rubber,
        rim_material=steel,
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=3.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    clamp_bar = model.part("clamp_bar")
    clamp_bar.visual(
        Cylinder(radius=0.016, length=0.44),
        origin=Origin(xyz=(0.0, 0.160, -0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="clamp_frame",
    )
    clamp_bar.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(FRAME_HALF_WIDTH, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_pivot_eye",
    )
    clamp_bar.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(-FRAME_HALF_WIDTH, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_pivot_eye",
    )
    clamp_bar.visual(
        Box((0.020, 0.032, 0.018)),
        origin=Origin(xyz=(FRAME_HALF_WIDTH - 0.005, 0.010, -0.006), rpy=(-0.30, 0.0, 0.0)),
        material=steel,
        name="left_eye_link",
    )
    clamp_bar.visual(
        Box((0.020, 0.032, 0.018)),
        origin=Origin(xyz=(-FRAME_HALF_WIDTH + 0.005, 0.010, -0.006), rpy=(-0.30, 0.0, 0.0)),
        material=steel,
        name="right_eye_link",
    )
    clamp_bar.visual(
        Box((0.020, 0.245, 0.018)),
        origin=Origin(xyz=(0.230, 0.080, -0.090), rpy=(-0.844, 0.0, 0.0)),
        material=steel,
        name="left_arm",
    )
    clamp_bar.visual(
        Box((0.020, 0.245, 0.018)),
        origin=Origin(xyz=(-0.230, 0.080, -0.090), rpy=(-0.844, 0.0, 0.0)),
        material=steel,
        name="right_arm",
    )
    clamp_bar.visual(
        Cylinder(radius=0.034, length=0.40),
        origin=Origin(xyz=(0.0, 0.160, -0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pad_gray,
        name="clamp_pad",
    )
    clamp_bar.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.19, 0.160, -0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="left_pad_end",
    )
    clamp_bar.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(-0.19, 0.160, -0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="right_pad_end",
    )
    clamp_bar.inertial = Inertial.from_geometry(
        Box((0.52, 0.22, 0.18)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.090, -0.090)),
    )

    model.articulation(
        "frame_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, AXLE_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=30.0),
    )
    model.articulation(
        "frame_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(-WHEEL_CENTER_X, AXLE_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=30.0),
    )
    model.articulation(
        "frame_to_clamp_bar",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=clamp_bar,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.20, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    clamp_bar = object_model.get_part("clamp_bar")
    left_spin = object_model.get_articulation("frame_to_left_wheel")
    right_spin = object_model.get_articulation("frame_to_right_wheel")
    clamp_hinge = object_model.get_articulation("frame_to_clamp_bar")

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
        "wheel_axes_are_lateral",
        tuple(left_spin.axis) == (1.0, 0.0, 0.0) and tuple(right_spin.axis) == (1.0, 0.0, 0.0),
        details=f"Left axis={left_spin.axis}, right axis={right_spin.axis}",
    )
    ctx.check(
        "clamp_axis_is_lateral",
        tuple(clamp_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"Clamp axis={clamp_hinge.axis}",
    )

    ctx.expect_contact(left_wheel, frame, elem_b="axle_beam", name="left_wheel_seated_on_axle")
    ctx.expect_contact(right_wheel, frame, elem_b="axle_beam", name="right_wheel_seated_on_axle")
    ctx.expect_contact(
        clamp_bar,
        frame,
        elem_a="left_pivot_eye",
        elem_b="left_pivot_hardware",
        name="left_pivot_captures_clamp",
    )
    ctx.expect_contact(
        clamp_bar,
        frame,
        elem_a="right_pivot_eye",
        elem_b="right_pivot_hardware",
        name="right_pivot_captures_clamp",
    )
    ctx.expect_gap(
        clamp_bar,
        frame,
        axis="z",
        positive_elem="clamp_pad",
        negative_elem="toe_plate",
        min_gap=0.60,
        max_gap=0.90,
        name="clamp_pad_sits_above_toe_plate",
    )
    ctx.expect_overlap(
        clamp_bar,
        frame,
        axes="x",
        elem_a="clamp_pad",
        elem_b="toe_plate",
        min_overlap=0.36,
        name="clamp_pad_spans_loading_width",
    )
    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="x",
        min_dist=0.70,
        max_dist=0.74,
        name="rear_wheels_flank_frame_evenly",
    )

    left_valve_rest = ctx.part_element_world_aabb(left_wheel, elem="valve_stem")
    clamp_pad_rest = ctx.part_element_world_aabb(clamp_bar, elem="clamp_pad")
    assert left_valve_rest is not None
    assert clamp_pad_rest is not None

    with ctx.pose({left_spin: math.pi / 2.0}):
        left_valve_quarter_turn = ctx.part_element_world_aabb(left_wheel, elem="valve_stem")
        assert left_valve_quarter_turn is not None
        ctx.expect_contact(left_wheel, frame, elem_b="axle_beam", name="left_wheel_stays_on_axle_when_rotated")
        ctx.check(
            "left_wheel_rotation_moves_valve_stem",
            left_valve_quarter_turn[1][2] > left_valve_rest[1][2] + 0.08,
            details=f"Rest stem zmax={left_valve_rest[1][2]:.3f}, turned zmax={left_valve_quarter_turn[1][2]:.3f}",
        )

    with ctx.pose({clamp_hinge: 1.0}):
        clamp_pad_open = ctx.part_element_world_aabb(clamp_bar, elem="clamp_pad")
        assert clamp_pad_open is not None
        ctx.expect_contact(
            clamp_bar,
            frame,
            elem_a="left_pivot_eye",
            elem_b="left_pivot_hardware",
            name="left_pivot_retains_clamp_when_open",
        )
        ctx.expect_contact(
            clamp_bar,
            frame,
            elem_a="right_pivot_eye",
            elem_b="right_pivot_hardware",
            name="right_pivot_retains_clamp_when_open",
        )
        ctx.check(
            "clamp_bar_swings_upward",
            clamp_pad_open[1][2] > clamp_pad_rest[1][2] + 0.18,
            details=f"Rest pad zmax={clamp_pad_rest[1][2]:.3f}, open zmax={clamp_pad_open[1][2]:.3f}",
        )
        ctx.expect_gap(
            clamp_bar,
            frame,
            axis="z",
            positive_elem="clamp_pad",
            negative_elem="toe_plate",
            min_gap=0.78,
            name="open_clamp_clears_toe_plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
