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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixed_mount_church_bell")

    oak = model.material("oak", rgba=(0.45, 0.31, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.44, 0.46, 0.49, 1.0))
    bronze = model.material("bell_bronze", rgba=(0.63, 0.43, 0.20, 1.0))
    iron = model.material("dark_iron", rgba=(0.17, 0.18, 0.19, 1.0))

    belfry_frame = model.part("belfry_frame")
    belfry_frame.visual(
        Box((1.30, 0.84, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=oak,
        name="platform_base",
    )
    for name, x_pos, y_pos in (
        ("front_left_post", -0.53, 0.30),
        ("front_right_post", 0.53, 0.30),
        ("rear_left_post", -0.53, -0.30),
        ("rear_right_post", 0.53, -0.30),
    ):
        belfry_frame.visual(
            Box((0.14, 0.14, 1.72)),
            origin=Origin(xyz=(x_pos, y_pos, 0.94)),
            material=oak,
            name=name,
        )
    belfry_frame.visual(
        Box((1.18, 0.24, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 1.69)),
        material=oak,
        name="carrier_beam",
    )
    belfry_frame.visual(
        Box((0.18, 0.68, 0.18)),
        origin=Origin(xyz=(-0.53, 0.0, 1.71)),
        material=oak,
        name="left_top_tie",
    )
    belfry_frame.visual(
        Box((0.18, 0.68, 0.18)),
        origin=Origin(xyz=(0.53, 0.0, 1.71)),
        material=oak,
        name="right_top_tie",
    )
    belfry_frame.visual(
        Box((1.06, 0.16, 0.14)),
        origin=Origin(xyz=(0.0, 0.30, 0.26)),
        material=oak,
        name="front_lower_tie",
    )
    belfry_frame.visual(
        Box((1.06, 0.16, 0.14)),
        origin=Origin(xyz=(0.0, -0.30, 0.26)),
        material=oak,
        name="rear_lower_tie",
    )
    belfry_frame.visual(
        Box((0.16, 0.60, 0.14)),
        origin=Origin(xyz=(-0.53, 0.0, 0.26)),
        material=oak,
        name="left_lower_tie",
    )
    belfry_frame.visual(
        Box((0.16, 0.60, 0.14)),
        origin=Origin(xyz=(0.53, 0.0, 0.26)),
        material=oak,
        name="right_lower_tie",
    )
    belfry_frame.inertial = Inertial.from_geometry(
        Box((1.30, 0.84, 1.80)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
    )

    steel_yoke = model.part("steel_yoke")
    steel_yoke.visual(
        Box((0.70, 0.22, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=steel,
        name="yoke_head",
    )
    steel_yoke.visual(
        Box((0.10, 0.16, 0.16)),
        origin=Origin(xyz=(-0.23, 0.0, -0.14)),
        material=steel,
        name="left_hanger_plate",
    )
    steel_yoke.visual(
        Box((0.10, 0.16, 0.16)),
        origin=Origin(xyz=(0.23, 0.0, -0.14)),
        material=steel,
        name="right_hanger_plate",
    )
    steel_yoke.visual(
        Box((0.36, 0.18, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.24)),
        material=steel,
        name="yoke_saddle",
    )
    steel_yoke.visual(
        Cylinder(radius=0.018, length=0.22),
        origin=Origin(xyz=(-0.23, 0.0, -0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_bolt_tube",
    )
    steel_yoke.visual(
        Cylinder(radius=0.018, length=0.22),
        origin=Origin(xyz=(0.23, 0.0, -0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_bolt_tube",
    )
    steel_yoke.inertial = Inertial.from_geometry(
        Box((0.70, 0.22, 0.26)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, -0.13)),
    )

    bell = model.part("bell")
    bell_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.10, 0.00),
            (0.13, -0.05),
            (0.20, -0.12),
            (0.29, -0.24),
            (0.35, -0.36),
            (0.39, -0.50),
            (0.42, -0.60),
            (0.43, -0.62),
        ],
        [
            (0.06, 0.00),
            (0.09, -0.04),
            (0.15, -0.11),
            (0.23, -0.23),
            (0.29, -0.35),
            (0.33, -0.49),
            (0.36, -0.59),
            (0.37, -0.62),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    bell.visual(
        _mesh("bell_shell", bell_shell_geom),
        origin=Origin(xyz=(0.0, 0.0, -0.25)),
        material=bronze,
        name="bell_shell",
    )
    bell.visual(
        Box((0.28, 0.18, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=bronze,
        name="crown_head",
    )
    bell.visual(
        Box((0.05, 0.18, 0.20)),
        origin=Origin(xyz=(-0.10, 0.0, -0.15)),
        material=bronze,
        name="left_crown_cheek",
    )
    bell.visual(
        Box((0.05, 0.18, 0.20)),
        origin=Origin(xyz=(0.10, 0.0, -0.15)),
        material=bronze,
        name="right_crown_cheek",
    )
    bell.visual(
        Box((0.18, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, 0.09, -0.11)),
        material=bronze,
        name="front_clapper_lug",
    )
    bell.visual(
        Box((0.18, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, -0.09, -0.11)),
        material=bronze,
        name="rear_clapper_lug",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.43, length=0.87),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, -0.44)),
    )

    clapper = model.part("clapper")
    clapper.visual(
        Cylinder(radius=0.024, length=0.14),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="clapper_eye",
    )
    clapper.visual(
        Cylinder(radius=0.022, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
        material=iron,
        name="clapper_rod_upper",
    )
    clapper.visual(
        Cylinder(radius=0.030, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        material=iron,
        name="clapper_rod_lower",
    )
    clapper.visual(
        Sphere(radius=0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.56)),
        material=iron,
        name="clapper_ball",
    )
    clapper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.66),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.33)),
    )

    model.articulation(
        "frame_to_yoke",
        ArticulationType.FIXED,
        parent=belfry_frame,
        child=steel_yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.58)),
    )
    model.articulation(
        "yoke_to_bell",
        ArticulationType.FIXED,
        parent=steel_yoke,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, -0.26)),
    )
    model.articulation(
        "bell_to_clapper",
        ArticulationType.REVOLUTE,
        parent=bell,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=2.0,
            lower=-0.40,
            upper=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    belfry_frame = object_model.get_part("belfry_frame")
    steel_yoke = object_model.get_part("steel_yoke")
    bell = object_model.get_part("bell")
    clapper = object_model.get_part("clapper")
    clapper_joint = object_model.get_articulation("bell_to_clapper")

    ctx.expect_contact(
        steel_yoke,
        belfry_frame,
        elem_a="yoke_head",
        elem_b="carrier_beam",
        name="yoke head is seated against the belfry carrier beam",
    )
    ctx.expect_contact(
        bell,
        steel_yoke,
        elem_a="crown_head",
        elem_b="yoke_saddle",
        name="bell crown is rigidly clamped into the steel yoke",
    )
    ctx.expect_contact(
        clapper,
        bell,
        elem_a="clapper_eye",
        elem_b="front_clapper_lug",
        name="clapper eye bears on the front crown lug at the pivot",
    )
    ctx.expect_contact(
        clapper,
        bell,
        elem_a="clapper_eye",
        elem_b="rear_clapper_lug",
        name="clapper eye bears on the rear crown lug at the pivot",
    )
    ctx.expect_within(
        clapper,
        bell,
        axes="xy",
        inner_elem="clapper_ball",
        outer_elem="bell_shell",
        margin=0.0,
        name="clapper ball hangs within the bell mouth footprint at rest",
    )
    ctx.check(
        "clapper joint uses a horizontal cross-bell axis",
        tuple(round(value, 6) for value in clapper_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={clapper_joint.axis}",
    )

    rest_ball = ctx.part_element_world_aabb(clapper, elem="clapper_ball")
    upper_limit = 0.0
    if clapper_joint.motion_limits is not None and clapper_joint.motion_limits.upper is not None:
        upper_limit = clapper_joint.motion_limits.upper

    with ctx.pose({clapper_joint: upper_limit}):
        ctx.expect_within(
            clapper,
            bell,
            axes="xy",
            inner_elem="clapper_ball",
            outer_elem="bell_shell",
            margin=0.0,
            name="swung clapper ball remains inside the bell shell footprint",
        )
        swung_ball = ctx.part_element_world_aabb(clapper, elem="clapper_ball")

    rest_center = _aabb_center(rest_ball)
    swung_center = _aabb_center(swung_ball)
    ctx.check(
        "positive clapper motion swings the ball toward one side of the bell",
        rest_center is not None
        and swung_center is not None
        and swung_center[0] > rest_center[0] + 0.12,
        details=f"rest_center={rest_center}, swung_center={swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
