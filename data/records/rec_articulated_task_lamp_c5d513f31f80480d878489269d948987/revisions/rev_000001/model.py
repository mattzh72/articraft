from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BASE_LENGTH = 0.34
BASE_WIDTH = 0.20
BASE_HEIGHT = 0.035
BASE_HINGE_X = -0.115
BASE_HINGE_Z = 0.095

LOWER_LENGTH = 0.42
UPPER_LENGTH = 0.36
LOWER_ELEVATION = math.radians(63.0)
UPPER_ELEVATION = math.radians(18.0)
SHADE_ELEVATION = math.radians(-25.0)


def _cylinder_x(length: float, *, radius: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _cylinder_x_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _cylinder_y_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _rounded_base_mesh() -> MeshGeometry:
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(BASE_LENGTH, BASE_WIDTH, 0.026, corner_segments=8),
        BASE_HEIGHT,
        cap=True,
        closed=True,
    )


def _shade_shell_mesh() -> MeshGeometry:
    # Thin-walled spun reflector shell, authored along local +X.
    rear_x = 0.045
    length = 0.180
    outer_profile = [
        (0.044, 0.000),
        (0.057, 0.030),
        (0.079, 0.105),
        (0.096, length),
    ]
    inner_profile = [
        (0.016, 0.000),
        (0.044, 0.034),
        (0.068, 0.108),
        (0.084, length - 0.010),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    shell.rotate_y(math.pi / 2.0).translate(rear_x, 0.0, 0.0)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_task_lamp")

    dark_base = model.material("dark_weighted_base", rgba=(0.06, 0.065, 0.07, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    shade_paint = model.material("green_enamel", rgba=(0.02, 0.20, 0.16, 1.0))
    warm_glass = model.material("warm_bulb_glass", rgba=(1.0, 0.84, 0.42, 0.72))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_rounded_base_mesh(), "weighted_base"),
        material=dark_base,
        name="weighted_base",
    )
    base.visual(
        Box((0.080, 0.085, 0.018)),
        origin=Origin(xyz=(BASE_HINGE_X, 0.0, BASE_HEIGHT + 0.009)),
        material=dark_base,
        name="pedestal_block",
    )
    base.visual(
        Box((0.055, 0.012, 0.095)),
        origin=Origin(xyz=(BASE_HINGE_X, -0.066, 0.0825)),
        material=dark_base,
        name="base_yoke_0",
    )
    base.visual(
        Box((0.055, 0.012, 0.095)),
        origin=Origin(xyz=(BASE_HINGE_X, 0.066, 0.0825)),
        material=dark_base,
        name="base_yoke_1",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.152),
        origin=_cylinder_y_origin(BASE_HINGE_X, 0.0, BASE_HINGE_Z),
        material=hinge_metal,
        name="base_hinge_pin",
    )
    for index, (x, y) in enumerate(
        (
            (-0.122, -0.066),
            (-0.122, 0.066),
            (0.122, -0.066),
            (0.122, 0.066),
        )
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, y, -0.0035)),
            material=black_rubber,
            name=f"rubber_foot_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.021, length=0.098),
        origin=_cylinder_y_origin(0.0, 0.0, 0.0),
        material=hinge_metal,
        name="lower_hinge_barrel",
    )
    lower_rod_start = 0.020
    lower_rod_length = LOWER_LENGTH - lower_rod_start
    for y in (-0.050, 0.050):
        lower_arm.visual(
            _cylinder_x(lower_rod_length, radius=0.009),
            origin=_cylinder_x_origin(lower_rod_start + lower_rod_length / 2.0, y, 0.0),
            material=satin_metal,
            name=f"lower_rod_{0 if y < 0 else 1}",
        )
    lower_arm.visual(
        Cylinder(radius=0.0085, length=0.132),
        origin=_cylinder_y_origin(LOWER_LENGTH, 0.0, 0.0),
        material=hinge_metal,
        name="lower_elbow_pin",
    )
    for y in (-0.055, 0.055):
        lower_arm.visual(
            Cylinder(radius=0.019, length=0.030),
            origin=_cylinder_y_origin(LOWER_LENGTH, y, 0.0),
            material=hinge_metal,
            name=f"lower_elbow_knuckle_{0 if y < 0 else 1}",
        )
    lower_arm.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=LOWER_LENGTH),
        mass=0.55,
        origin=_cylinder_x_origin(LOWER_LENGTH / 2.0, 0.0, 0.0),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.020, length=0.064),
        origin=_cylinder_y_origin(0.0, 0.0, 0.0),
        material=hinge_metal,
        name="upper_hinge_barrel",
    )
    upper_rod_start = 0.019
    upper_rod_length = UPPER_LENGTH - upper_rod_start
    for y in (-0.039, 0.039):
        upper_arm.visual(
            _cylinder_x(upper_rod_length, radius=0.008),
            origin=_cylinder_x_origin(upper_rod_start + upper_rod_length / 2.0, y, 0.0),
            material=satin_metal,
            name=f"upper_rod_{0 if y < 0 else 1}",
        )
    upper_arm.visual(
        Cylinder(radius=0.0075, length=0.112),
        origin=_cylinder_y_origin(UPPER_LENGTH, 0.0, 0.0),
        material=hinge_metal,
        name="upper_wrist_pin",
    )
    for y in (-0.047, 0.047):
        upper_arm.visual(
            Cylinder(radius=0.017, length=0.026),
            origin=_cylinder_y_origin(UPPER_LENGTH, y, 0.0),
            material=hinge_metal,
            name=f"upper_wrist_knuckle_{0 if y < 0 else 1}",
        )
    upper_arm.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=UPPER_LENGTH),
        mass=0.45,
        origin=_cylinder_x_origin(UPPER_LENGTH / 2.0, 0.0, 0.0),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.019, length=0.060),
        origin=_cylinder_y_origin(0.0, 0.0, 0.0),
        material=hinge_metal,
        name="shade_hinge_barrel",
    )
    shade.visual(
        _cylinder_x(0.060, radius=0.017),
        origin=_cylinder_x_origin(0.048, 0.0, 0.0),
        material=shade_paint,
        name="shade_neck",
    )
    shade.visual(
        mesh_from_geometry(_shade_shell_mesh(), "shade_shell"),
        material=shade_paint,
        name="shade_shell",
    )
    shade.visual(
        _cylinder_x(0.044, radius=0.026),
        origin=_cylinder_x_origin(0.075, 0.0, 0.0),
        material=hinge_metal,
        name="bulb_socket",
    )
    shade.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.114, 0.0, 0.0)),
        material=warm_glass,
        name="glowing_bulb",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.190),
        mass=0.42,
        origin=_cylinder_x_origin(0.130, 0.0, 0.0),
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(
            xyz=(BASE_HINGE_X, 0.0, BASE_HINGE_Z),
            rpy=(0.0, -LOWER_ELEVATION, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.55, upper=0.70),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(
            xyz=(LOWER_LENGTH, 0.0, 0.0),
            rpy=(0.0, LOWER_ELEVATION - UPPER_ELEVATION, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "upper_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(
            xyz=(UPPER_LENGTH, 0.0, 0.0),
            rpy=(0.0, -SHADE_ELEVATION + UPPER_ELEVATION, 0.0),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")
    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")
    upper_to_shade = object_model.get_articulation("upper_to_shade")

    ctx.allow_overlap(
        base,
        lower_arm,
        elem_a="base_hinge_pin",
        elem_b="lower_hinge_barrel",
        reason="The fixed yoke pin is intentionally captured inside the lower arm hinge barrel.",
    )
    ctx.expect_within(
        base,
        lower_arm,
        axes="xz",
        inner_elem="base_hinge_pin",
        outer_elem="lower_hinge_barrel",
        margin=0.002,
        name="base pin is centered in lower hinge barrel",
    )
    ctx.expect_overlap(
        base,
        lower_arm,
        axes="y",
        elem_a="base_hinge_pin",
        elem_b="lower_hinge_barrel",
        min_overlap=0.090,
        name="base hinge pin spans lower barrel",
    )

    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        elem_a="lower_elbow_pin",
        elem_b="upper_hinge_barrel",
        reason="The elbow pin is intentionally captured inside the upper arm center barrel.",
    )
    ctx.expect_within(
        lower_arm,
        upper_arm,
        axes="xz",
        inner_elem="lower_elbow_pin",
        outer_elem="upper_hinge_barrel",
        margin=0.002,
        name="elbow pin is centered in upper hinge barrel",
    )
    ctx.expect_overlap(
        lower_arm,
        upper_arm,
        axes="y",
        elem_a="lower_elbow_pin",
        elem_b="upper_hinge_barrel",
        min_overlap=0.055,
        name="elbow pin spans upper barrel",
    )

    ctx.allow_overlap(
        upper_arm,
        shade,
        elem_a="upper_wrist_pin",
        elem_b="shade_hinge_barrel",
        reason="The wrist pin is intentionally captured inside the reflector shade hinge barrel.",
    )
    ctx.expect_within(
        upper_arm,
        shade,
        axes="xz",
        inner_elem="upper_wrist_pin",
        outer_elem="shade_hinge_barrel",
        margin=0.002,
        name="wrist pin is centered in shade barrel",
    )
    ctx.expect_overlap(
        upper_arm,
        shade,
        axes="y",
        elem_a="upper_wrist_pin",
        elem_b="shade_hinge_barrel",
        min_overlap=0.052,
        name="wrist pin spans shade barrel",
    )

    ctx.check(
        "task lamp has three revolute mechanisms",
        len(object_model.articulations) == 3,
        details=f"articulations={object_model.articulations}",
    )

    rest_elbow = ctx.part_world_position(upper_arm)
    rest_wrist = ctx.part_world_position(shade)
    rest_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({base_to_lower: 0.35}):
        raised_elbow = ctx.part_world_position(upper_arm)
    with ctx.pose({lower_to_upper: 0.35}):
        raised_wrist = ctx.part_world_position(shade)
    with ctx.pose({upper_to_shade: 0.35}):
        tilted_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")

    ctx.check(
        "base hinge raises elbow at positive travel",
        rest_elbow is not None
        and raised_elbow is not None
        and raised_elbow[2] > rest_elbow[2] + 0.040,
        details=f"rest={rest_elbow}, raised={raised_elbow}",
    )
    ctx.check(
        "elbow hinge raises wrist at positive travel",
        rest_wrist is not None
        and raised_wrist is not None
        and raised_wrist[2] > rest_wrist[2] + 0.040,
        details=f"rest={rest_wrist}, raised={raised_wrist}",
    )
    ctx.check(
        "shade tilt changes reflector aim",
        rest_shade_aabb is not None
        and tilted_shade_aabb is not None
        and tilted_shade_aabb[0][2] > rest_shade_aabb[0][2] + 0.020,
        details=f"rest_aabb={rest_shade_aabb}, tilted_aabb={tilted_shade_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
