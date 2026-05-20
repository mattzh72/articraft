from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _add_segment_visual(part, start, end, radius, material, *, name=None) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-6:
        return
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(
                0.5 * (start[0] + end[0]),
                0.5 * (start[1] + end[1]),
                0.5 * (start[2] + end[2]),
            ),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.18, 0.20, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    warm_metal = model.material("warm_metal", rgba=(0.63, 0.66, 0.69, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.105, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=graphite,
        name="foot_disk",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=graphite,
        name="foot_plinth",
    )
    base.visual(
        Box((0.094, 0.094, 0.094)),
        origin=Origin(xyz=(-0.030, 0.0, 0.085)),
        material=warm_metal,
        name="pedestal_body",
    )
    base.visual(
        Box((0.124, 0.192, 0.036)),
        origin=Origin(xyz=(-0.014, 0.0, 0.130)),
        material=graphite,
        name="lower_crossbar",
    )
    for side in (-1.0, 1.0):
        base.visual(
            Box((0.102, 0.028, 0.202)),
            origin=Origin(xyz=(0.0, side * 0.104, 0.205)),
            material=graphite,
            name=f"side_bracket_{'left' if side > 0 else 'right'}",
        )
        base.visual(
            Cylinder(radius=0.034, length=0.036),
            origin=Origin(
                xyz=(0.032, side * 0.104, 0.244),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=dark_plastic,
            name=f"bearing_housing_{'left' if side > 0 else 'right'}",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.028),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    head = model.part("head")
    front_guard_rim = mesh_from_geometry(
        TorusGeometry(radius=0.110, tube=0.0045, radial_segments=18, tubular_segments=64),
        "front_guard_rim",
    )
    rear_guard_rim = mesh_from_geometry(
        TorusGeometry(radius=0.106, tube=0.0045, radial_segments=18, tubular_segments=64),
        "rear_guard_rim",
    )
    head.visual(
        Cylinder(radius=0.084, length=0.106),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="motor_barrel",
    )
    head.visual(
        Sphere(radius=0.058),
        origin=Origin(xyz=(-0.064, 0.0, 0.0)),
        material=dark_plastic,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.044),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="front_collar",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.031),
        origin=Origin(xyz=(0.0515, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_metal,
        name="spindle_nose",
    )
    head.visual(
        rear_guard_rim,
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="rear_guard_rim",
    )
    head.visual(
        front_guard_rim,
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="front_guard_rim",
    )
    for index, angle in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        y = cos(angle)
        z = sin(angle)
        _add_segment_visual(
            head,
            (0.046, 0.044 * y, 0.044 * z),
            (0.060, 0.106 * y, 0.106 * z),
            0.004,
            warm_metal,
            name=f"guard_mount_strut_{index}",
        )
    for index, angle in enumerate((0.0, pi / 3.0, 2.0 * pi / 3.0, pi, 4.0 * pi / 3.0, 5.0 * pi / 3.0)):
        y = cos(angle)
        z = sin(angle)
        _add_segment_visual(
            head,
            (0.060, 0.106 * y, 0.106 * z),
            (0.108, 0.110 * y, 0.110 * z),
            0.0032,
            warm_metal,
            name=f"guard_side_wire_{index}",
        )
    for index, angle in enumerate((0.0, pi / 4.0, pi / 2.0, 3.0 * pi / 4.0, pi, 5.0 * pi / 4.0, 3.0 * pi / 2.0, 7.0 * pi / 4.0)):
        y = cos(angle)
        z = sin(angle)
        _add_segment_visual(
            head,
            (0.108, 0.020 * y, 0.020 * z),
            (0.108, 0.106 * y, 0.106 * z),
            0.0025,
            warm_metal,
            name=f"front_grille_wire_{index}",
        )
    for side in (-1.0, 1.0):
        head.visual(
            Cylinder(radius=0.030, length=0.008),
            origin=Origin(
                xyz=(0.0, side * 0.062, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=warm_metal,
            name=f"trunnion_flange_{'left' if side > 0 else 'right'}",
        )
        head.visual(
            Cylinder(radius=0.022, length=0.020),
            origin=Origin(
                xyz=(0.0, side * 0.076, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=warm_metal,
            name=f"pivot_boss_{'left' if side > 0 else 'right'}",
        )
    head.inertial = Inertial.from_geometry(
        Sphere(radius=0.11),
        mass=0.9,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.026, length=0.026),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="hub_shell",
    )
    rotor.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=warm_metal,
        name="hub_nose",
    )
    for index in range(4):
        angle = index * (pi / 2.0)
        rotor.visual(
            Box((0.026, 0.088, 0.005)),
            origin=Origin(
                xyz=(0.010, 0.052 * cos(angle), 0.052 * sin(angle)),
                rpy=(angle, 0.14, 0.0),
            ),
            material=warm_metal,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.052),
        mass=0.25,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.032, 0.0, 0.244)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-0.55,
            upper=0.85,
        ),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.076, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=28.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    with ctx.pose({tilt: 0.0}):
        ctx.expect_contact(
            head,
            rotor,
            elem_a="spindle_nose",
            elem_b="hub_shell",
            name="rotor hub seats on spindle nose",
        )
        ctx.expect_within(
            rotor,
            head,
            axes="yz",
            outer_elem="front_guard_rim",
            margin=0.002,
            name="rotor stays inside guard aperture at rest",
        )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({tilt: 0.65}):
        ctx.expect_within(
            rotor,
            head,
            axes="y",
            outer_elem="front_guard_rim",
            margin=0.002,
            name="rotor stays centered within guard width when tilted up",
        )
        tilted_pos = ctx.part_world_position(rotor)

    ctx.check(
        "positive tilt raises the fan head",
        rest_pos is not None and tilted_pos is not None and tilted_pos[2] > rest_pos[2] + 0.03,
        details=f"rest={rest_pos}, tilted={tilted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
