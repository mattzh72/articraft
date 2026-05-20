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


DRUM_OUTER_RADIUS = 0.132
DRUM_INNER_RADIUS = 0.116
DRUM_REAR_Y = -0.110
DRUM_FRONT_Y = 0.080

FRAME_OUTER = 0.320
FRAME_RAIL = 0.028
FRAME_THICKNESS = 0.014
FRAME_CENTER_Y = 0.087

COVER_RADIUS = 0.120
COVER_RIM_RADIUS = 0.008
COVER_THICKNESS = 0.016
COVER_PANEL_Y = 0.008
COVER_HINGE_Y = 0.096
COVER_HINGE_Z = 0.136

KNOB_X = 0.146
KNOB_Z = -0.118


def _managed_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _build_housing_mesh():
    housing = LatheGeometry.from_shell_profiles(
        [
            (DRUM_OUTER_RADIUS, DRUM_REAR_Y),
            (DRUM_OUTER_RADIUS, 0.066),
            (0.138, 0.076),
            (0.142, DRUM_FRONT_Y),
        ],
        [
            (DRUM_INNER_RADIUS, DRUM_REAR_Y),
            (DRUM_INNER_RADIUS, 0.070),
            (0.126, 0.077),
            (0.130, DRUM_FRONT_Y),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(-math.pi / 2.0)

    housing.merge(
        CylinderGeometry(radius=0.034, height=0.018, radial_segments=32)
        .rotate_x(-math.pi / 2.0)
        .translate(0.0, -0.034, 0.0)
    )
    for angle in (0.0, math.tau / 3.0, 2.0 * math.tau / 3.0):
        housing.merge(
            BoxGeometry((0.246, 0.008, 0.012))
            .rotate_y(angle)
            .translate(0.0, -0.034, 0.0)
        )

    return housing


def _build_inner_frame_mesh():
    frame = BoxGeometry((FRAME_OUTER, FRAME_THICKNESS, FRAME_RAIL)).translate(
        0.0, FRAME_CENTER_Y, 0.5 * (FRAME_OUTER - FRAME_RAIL)
    )
    frame.merge(
        BoxGeometry((FRAME_OUTER, FRAME_THICKNESS, FRAME_RAIL)).translate(
            0.0, FRAME_CENTER_Y, -0.5 * (FRAME_OUTER - FRAME_RAIL)
        )
    )
    frame.merge(
        BoxGeometry((FRAME_RAIL, FRAME_THICKNESS, FRAME_OUTER - 2.0 * FRAME_RAIL)).translate(
            0.5 * (FRAME_OUTER - FRAME_RAIL), FRAME_CENTER_Y, 0.0
        )
    )
    frame.merge(
        BoxGeometry((FRAME_RAIL, FRAME_THICKNESS, FRAME_OUTER - 2.0 * FRAME_RAIL)).translate(
            -0.5 * (FRAME_OUTER - FRAME_RAIL), FRAME_CENTER_Y, 0.0
        )
    )
    frame.merge(
        CylinderGeometry(radius=0.022, height=0.018, radial_segments=28)
        .rotate_x(-math.pi / 2.0)
        .translate(KNOB_X, FRAME_CENTER_Y, KNOB_Z)
    )
    frame.merge(
        BoxGeometry((0.220, 0.012, 0.012)).translate(0.0, 0.090, 0.130)
    )
    frame.merge(
        BoxGeometry((0.036, 0.018, 0.032)).translate(-0.110, 0.087, 0.118)
    )
    frame.merge(
        BoxGeometry((0.036, 0.018, 0.032)).translate(0.110, 0.087, 0.118)
    )

    return frame


def _build_rotor_mesh():
    rotor = (
        CylinderGeometry(radius=0.033, height=0.030, radial_segments=36)
        .rotate_x(-math.pi / 2.0)
        .translate(0.0, -0.010, 0.0)
    )
    rotor.merge(
        CylinderGeometry(radius=0.010, height=0.050, radial_segments=20)
        .rotate_x(-math.pi / 2.0)
        .translate(0.0, -0.002, 0.0)
    )
    rotor.merge(
        CylinderGeometry(radius=0.050, height=0.008, radial_segments=28)
        .rotate_x(-math.pi / 2.0)
        .translate(0.0, 0.004, 0.0)
    )

    base_blade = (
        BoxGeometry((0.024, 0.010, 0.082))
        .rotate_x(0.42)
        .translate(0.0, 0.003, 0.055)
    )
    for index in range(5):
        rotor.merge(base_blade.copy().rotate_y(index * math.tau / 5.0))

    return rotor


def _build_cover_mesh():
    circle_points = []
    for index in range(20):
        angle = math.tau * index / 20.0
        circle_points.append(
            (
                COVER_RADIUS * math.cos(angle),
                0.010,
                -COVER_RADIUS + COVER_RADIUS * math.sin(angle),
            )
        )

    cover = tube_from_spline_points(
        circle_points,
        radius=COVER_RIM_RADIUS,
        samples_per_segment=12,
        radial_segments=18,
        closed_spline=True,
        cap_ends=False,
        up_hint=(0.0, 1.0, 0.0),
    )
    cover.merge(BoxGeometry((0.208, 0.012, 0.010)).translate(0.0, 0.006, -0.005))

    for z_pos, width in (
        (-0.056, 0.180),
        (-0.088, 0.208),
        (-0.122, 0.224),
        (-0.158, 0.214),
        (-0.192, 0.180),
    ):
        cover.merge(
            BoxGeometry((width, 0.013, 0.012))
            .rotate_x(0.58)
            .translate(0.0, 0.010, z_pos)
        )

    for x_pos in (-0.092, 0.092):
        cover.merge(
            CylinderGeometry(radius=0.007, height=0.070, radial_segments=24)
            .rotate_y(math.pi / 2.0)
            .translate(x_pos, 0.010, -0.006)
        )
        cover.merge(
            BoxGeometry((0.012, 0.012, 0.212)).translate(x_pos, 0.010, -0.116)
        )

    cover.merge(BoxGeometry((0.064, 0.010, 0.012)).translate(0.0, 0.012, -0.232))
    return cover


def _build_knob_mesh():
    knob = (
        CylinderGeometry(radius=0.006, height=0.008, radial_segments=20)
        .rotate_x(-math.pi / 2.0)
        .translate(0.0, 0.004, 0.0)
    )
    knob.merge(
        CylinderGeometry(radius=0.014, height=0.014, radial_segments=28)
        .rotate_x(-math.pi / 2.0)
        .translate(0.0, 0.015, 0.0)
    )
    knob.merge(BoxGeometry((0.006, 0.006, 0.010)).translate(0.0, 0.020, 0.010))
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="through_wall_ventilation_fan")

    housing_plastic = model.material("housing_plastic", rgba=(0.86, 0.86, 0.82, 1.0))
    frame_plastic = model.material("frame_plastic", rgba=(0.82, 0.82, 0.78, 1.0))
    rotor_metal = model.material("rotor_metal", rgba=(0.52, 0.55, 0.58, 1.0))
    dark_knob = model.material("dark_knob", rgba=(0.20, 0.22, 0.24, 1.0))

    housing = model.part("housing")
    housing.visual(
        _managed_mesh(_build_housing_mesh(), "housing_shell"),
        material=housing_plastic,
        name="housing_shell",
    )
    housing.inertial = Inertial.from_geometry(
        Cylinder(radius=DRUM_OUTER_RADIUS, length=0.190),
        mass=2.8,
        origin=Origin(),
    )

    inner_frame = model.part("inner_frame")
    inner_frame.visual(
        _managed_mesh(_build_inner_frame_mesh(), "inner_frame"),
        material=frame_plastic,
        name="inner_frame",
    )
    inner_frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER, 0.060, FRAME_OUTER)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        _managed_mesh(_build_rotor_mesh(), "rotor"),
        material=rotor_metal,
        name="rotor",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.030),
        mass=0.35,
        origin=Origin(),
    )

    cover = model.part("cover")
    cover.visual(
        _managed_mesh(_build_cover_mesh(), "cover"),
        material=housing_plastic,
        name="cover",
    )
    cover.inertial = Inertial.from_geometry(
        Box((2.0 * COVER_RADIUS, COVER_THICKNESS, 2.0 * COVER_RADIUS)),
        mass=0.7,
        origin=Origin(xyz=(0.0, COVER_PANEL_Y, -COVER_RADIUS)),
    )

    knob = model.part("speed_knob")
    knob.visual(
        _managed_mesh(_build_knob_mesh(), "speed_knob"),
        material=dark_knob,
        name="speed_knob",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.022),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
    )

    model.articulation(
        "housing_to_inner_frame",
        ArticulationType.FIXED,
        parent=housing,
        child=inner_frame,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=18.0),
    )
    model.articulation(
        "inner_frame_to_cover",
        ArticulationType.REVOLUTE,
        parent=inner_frame,
        child=cover,
        origin=Origin(xyz=(0.0, COVER_HINGE_Y, COVER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "inner_frame_to_knob",
        ArticulationType.CONTINUOUS,
        parent=inner_frame,
        child=knob,
        origin=Origin(xyz=(KNOB_X, 0.096, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    inner_frame = object_model.get_part("inner_frame")
    rotor = object_model.get_part("rotor")
    cover = object_model.get_part("cover")
    knob = object_model.get_part("speed_knob")

    cover_hinge = object_model.get_articulation("inner_frame_to_cover")
    fan_spin = object_model.get_articulation("housing_to_rotor")
    knob_spin = object_model.get_articulation("inner_frame_to_knob")

    ctx.check(
        "fan blade articulation is continuous",
        fan_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={fan_spin.articulation_type}",
    )
    ctx.check(
        "speed knob articulation is continuous",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )
    ctx.check(
        "cover hinge opens downward from top edge",
        cover_hinge.articulation_type == ArticulationType.REVOLUTE
        and cover_hinge.axis == (1.0, 0.0, 0.0)
        and cover_hinge.motion_limits is not None
        and cover_hinge.motion_limits.upper is not None
        and cover_hinge.motion_limits.upper > 1.2,
        details=f"axis={cover_hinge.axis}, limits={cover_hinge.motion_limits}",
    )

    ctx.expect_within(
        rotor,
        housing,
        axes="xz",
        margin=0.004,
        name="rotor stays within drum diameter",
    )
    ctx.expect_contact(
        rotor,
        housing,
        contact_tol=0.0015,
        name="rotor hub is mounted to the housing support",
    )
    ctx.expect_contact(
        cover,
        inner_frame,
        contact_tol=0.0015,
        name="closed cover rests on the top hinge support",
    )
    ctx.expect_gap(
        cover,
        housing,
        axis="y",
        min_gap=0.004,
        max_gap=0.024,
        name="closed cover sits ahead of the drum housing",
    )
    ctx.expect_overlap(
        cover,
        housing,
        axes="xz",
        min_overlap=0.220,
        name="closed cover spans the housing opening",
    )
    ctx.expect_contact(
        knob,
        inner_frame,
        contact_tol=0.002,
        name="speed knob mounts to the inner frame",
    )
    ctx.expect_gap(
        knob,
        cover,
        axis="x",
        min_gap=0.003,
        name="speed knob clears the louvered cover",
    )

    closed_cover_aabb = None
    open_cover_aabb = None
    closed_rotor_aabb = ctx.part_world_aabb(rotor)
    with ctx.pose({cover_hinge: 0.0}):
        closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.25}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "cover swings forward when opened",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.085,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    with ctx.pose({fan_spin: 1.8}):
        spun_rotor_aabb = ctx.part_world_aabb(rotor)
        ctx.check(
            "fan blade spin keeps blade envelope centered",
            closed_rotor_aabb is not None
            and spun_rotor_aabb is not None
            and abs(spun_rotor_aabb[0][0] - closed_rotor_aabb[0][0]) < 0.02
            and abs(spun_rotor_aabb[1][2] - closed_rotor_aabb[1][2]) < 0.02,
            details=f"rest={closed_rotor_aabb}, spun={spun_rotor_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
