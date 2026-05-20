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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def _circle_loop(radius: float, z: float, segments: int = 40):
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
            z,
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_warehouse_floor_lamp")

    cast_iron = model.material("cast_iron", rgba=(0.14, 0.14, 0.15, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    steel_pipe = model.material("steel_pipe", rgba=(0.43, 0.45, 0.48, 1.0))
    enamel = model.material("enamel", rgba=(0.78, 0.79, 0.76, 1.0))
    reflector_trim = model.material("reflector_trim", rgba=(0.63, 0.65, 0.68, 1.0))

    reflector_shell_mesh = _mesh(
        "reflector_shell",
        LoftGeometry(
            [
                _circle_loop(0.030, 0.000),
                _circle_loop(0.054, 0.050),
                _circle_loop(0.086, 0.110),
                _circle_loop(0.122, 0.180),
                _circle_loop(0.160, 0.260),
            ],
            cap=False,
            closed=True,
        ).rotate_y(math.pi / 2.0),
    )
    front_rim_mesh = _mesh(
        "reflector_rim",
        TorusGeometry(radius=0.158, tube=0.008, radial_segments=18, tubular_segments=72).rotate_y(math.pi / 2.0),
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.188, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=cast_iron,
        name="base_floor_pad",
    )
    stand.visual(
        Cylinder(radius=0.176, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="base_body",
    )
    stand.visual(
        Cylinder(radius=0.095, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=cast_iron,
        name="base_top_hub",
    )
    stand.visual(
        Cylinder(radius=0.021, length=1.420),
        origin=Origin(xyz=(0.0, 0.0, 0.755)),
        material=steel_pipe,
        name="column_pipe",
    )
    stand.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 1.461)),
        material=dark_steel,
        name="column_collar",
    )
    stand.visual(
        Cylinder(radius=0.033, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.475)),
        material=dark_steel,
        name="top_cap",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.376, 0.376, 1.485)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.7425)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.035, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_steel,
        name="pivot_collar",
    )
    yoke.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_steel,
        name="yoke_post",
    )
    yoke.visual(
        Box((0.060, 0.060, 0.034)),
        origin=Origin(xyz=(0.030, 0.0, 0.094)),
        material=dark_steel,
        name="head_block",
    )
    yoke.visual(
        Box((0.036, 0.176, 0.022)),
        origin=Origin(xyz=(0.044, 0.0, 0.122)),
        material=dark_steel,
        name="cross_bridge",
    )
    yoke.visual(
        Box((0.094, 0.016, 0.108)),
        origin=Origin(xyz=(0.089, 0.096, 0.070)),
        material=dark_steel,
        name="left_arm",
    )
    yoke.visual(
        Box((0.094, 0.016, 0.108)),
        origin=Origin(xyz=(0.089, -0.096, 0.070)),
        material=dark_steel,
        name="right_arm",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.136, 0.192, 0.140)),
        mass=3.2,
        origin=Origin(xyz=(0.048, 0.0, 0.070)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.012, length=0.144),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_barrel",
    )
    shade.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.080, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    shade.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, -0.080, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    shade.visual(
        Cylinder(radius=0.037, length=0.110),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="socket_housing",
    )
    shade.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="neck_flare",
    )
    shade.visual(
        reflector_shell_mesh,
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=enamel,
        name="reflector_shell",
    )
    shade.visual(
        front_rim_mesh,
        origin=Origin(xyz=(0.4376, 0.0, 0.0)),
        material=reflector_trim,
        name="front_rim",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.480, 0.330, 0.330)),
        mass=4.6,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
    )

    model.articulation(
        "column_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.485)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-2.40,
            upper=2.40,
        ),
    )
    model.articulation(
        "yoke_to_shade_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=shade,
        origin=Origin(xyz=(0.112, 0.0, 0.078)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-1.05,
            upper=0.72,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    shade = object_model.get_part("shade")
    pan = object_model.get_articulation("column_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_shade_tilt")

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

    ctx.expect_contact(
        yoke,
        stand,
        elem_a="pivot_collar",
        elem_b="top_cap",
        contact_tol=1e-5,
        name="yoke collar seats on the stand top cap",
    )
    ctx.expect_contact(
        shade,
        yoke,
        elem_a="left_trunnion",
        elem_b="left_arm",
        contact_tol=1e-5,
        name="left trunnion is carried by the left yoke arm",
    )
    ctx.expect_contact(
        shade,
        yoke,
        elem_a="right_trunnion",
        elem_b="right_arm",
        contact_tol=1e-5,
        name="right trunnion is carried by the right yoke arm",
    )
    ctx.expect_contact(
        yoke,
        yoke,
        elem_a="head_block",
        elem_b="cross_bridge",
        contact_tol=1e-6,
        name="yoke bridge is fused into the head block",
    )
    ctx.expect_contact(
        shade,
        shade,
        elem_a="neck_flare",
        elem_b="reflector_shell",
        contact_tol=1e-6,
        name="reflector shell connects to the neck flare",
    )
    ctx.expect_contact(
        shade,
        shade,
        elem_a="reflector_shell",
        elem_b="front_rim",
        contact_tol=1e-6,
        name="front rim is attached to the reflector shell lip",
    )
    ctx.expect_gap(
        shade,
        yoke,
        axis="x",
        positive_elem="trunnion_barrel",
        negative_elem="yoke_post",
        min_gap=0.03,
        name="tilt barrel clears the pan post",
    )

    stand_aabb = ctx.part_world_aabb(stand)
    stand_height = None if stand_aabb is None else stand_aabb[1][2] - stand_aabb[0][2]
    stand_width = None if stand_aabb is None else stand_aabb[1][0] - stand_aabb[0][0]
    ctx.check(
        "stand proportions read as a full-size floor lamp",
        stand_height is not None
        and stand_width is not None
        and 1.45 <= stand_height <= 1.52
        and 0.35 <= stand_width <= 0.39,
        details=f"stand_height={stand_height}, stand_width={stand_width}",
    )

    rest_rim_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="front_rim"))
    with ctx.pose({pan: 1.10}):
        turned_rim_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="front_rim"))
    ctx.check(
        "pan joint swings the reflector around the column",
        rest_rim_center is not None
        and turned_rim_center is not None
        and turned_rim_center[1] > rest_rim_center[1] + 0.16
        and turned_rim_center[0] < rest_rim_center[0] - 0.08,
        details=f"rest={rest_rim_center}, turned={turned_rim_center}",
    )

    with ctx.pose({tilt: 0.55}):
        raised_rim_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="front_rim"))
    ctx.check(
        "positive tilt raises the wide reflector mouth",
        rest_rim_center is not None
        and raised_rim_center is not None
        and raised_rim_center[2] > rest_rim_center[2] + 0.11,
        details=f"rest={rest_rim_center}, raised={raised_rim_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
