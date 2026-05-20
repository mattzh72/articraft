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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _beam_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    thickness: float,
    material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(xyz=mid, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_ground_dish")

    pedestal_gray = model.material("pedestal_gray", rgba=(0.43, 0.45, 0.48, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.88, 0.90, 0.92, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.15, 0.17, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.08, 0.09, 0.10, 1.0))

    reflector_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.028, -0.140),
                (0.070, -0.126),
                (0.165, -0.088),
                (0.235, -0.038),
                (0.260, 0.000),
            ],
            [
                (0.010, -0.124),
                (0.050, -0.116),
                (0.145, -0.082),
                (0.218, -0.036),
                (0.248, -0.010),
            ],
            segments=64,
            start_cap="round",
            end_cap="flat",
            lip_samples=8,
        ),
        "reflector_shell",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.30, 0.26, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=pedestal_gray,
        name="base_plinth",
    )
    pedestal.visual(
        Box((0.24, 0.20, 0.280)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=pedestal_gray,
        name="pedestal_box",
    )
    pedestal.visual(
        Cylinder(radius=0.090, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        material=machinery_gray,
        name="pedestal_collar",
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        Cylinder(radius=0.085, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=bearing_black,
        name="bearing_drum",
    )
    top_plate.visual(
        Cylinder(radius=0.110, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=machinery_gray,
        name="rotating_plate",
    )
    top_plate.visual(
        Cylinder(radius=0.042, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_metal,
        name="center_cap",
    )

    bracket = model.part("elevation_bracket")
    bracket.visual(
        Box((0.13, 0.12, 0.050)),
        origin=Origin(xyz=(-0.08, 0.0, 0.025)),
        material=machinery_gray,
        name="bracket_base",
    )
    bracket.visual(
        Box((0.055, 0.11, 0.180)),
        origin=Origin(xyz=(-0.125, 0.0, 0.140)),
        material=machinery_gray,
        name="center_mast",
    )
    bracket.visual(
        Box((0.055, 0.560, 0.035)),
        origin=Origin(xyz=(-0.125, 0.0, 0.200)),
        material=machinery_gray,
        name="rear_crosshead",
    )
    bracket.visual(
        Box((0.160, 0.050, 0.080)),
        origin=Origin(xyz=(-0.060, 0.287, 0.200)),
        material=machinery_gray,
        name="left_yoke_arm",
    )
    bracket.visual(
        Box((0.160, 0.050, 0.080)),
        origin=Origin(xyz=(-0.060, -0.287, 0.200)),
        material=machinery_gray,
        name="right_yoke_arm",
    )
    _beam_between(
        bracket,
        (-0.095, 0.0, 0.050),
        (-0.100, 0.245, 0.160),
        thickness=0.030,
        material=dark_metal,
        name="left_brace",
    )
    _beam_between(
        bracket,
        (-0.095, 0.0, 0.050),
        (-0.100, -0.245, 0.160),
        thickness=0.030,
        material=dark_metal,
        name="right_brace",
    )

    reflector = model.part("reflector")
    reflector.visual(
        Cylinder(radius=0.014, length=0.524),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_shaft",
    )
    reflector.visual(
        Cylinder(radius=0.055, length=0.120),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="hub_barrel",
    )
    reflector.visual(
        reflector_mesh,
        origin=Origin(xyz=(0.170, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=reflector_white,
        name="reflector_shell",
    )
    _beam_between(
        reflector,
        (0.010, 0.0, -0.030),
        (0.300, 0.0, 0.0),
        thickness=0.016,
        material=dark_metal,
        name="feed_support_arm",
    )
    reflector.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.330, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_black,
        name="feed_horn",
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5),
    )
    model.articulation(
        "top_plate_to_bracket",
        ArticulationType.FIXED,
        parent=top_plate,
        child=bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=reflector,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.0,
            lower=-0.20,
            upper=1.10,
        ),
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

    pedestal = object_model.get_part("pedestal")
    top_plate = object_model.get_part("top_plate")
    bracket = object_model.get_part("elevation_bracket")
    reflector = object_model.get_part("reflector")

    azimuth = object_model.get_articulation("azimuth_axis")
    elevation = object_model.get_articulation("elevation_axis")

    bearing_drum = top_plate.get_visual("bearing_drum")
    bracket_base = bracket.get_visual("bracket_base")
    left_yoke_arm = bracket.get_visual("left_yoke_arm")
    right_yoke_arm = bracket.get_visual("right_yoke_arm")
    trunnion_shaft = reflector.get_visual("trunnion_shaft")
    reflector_shell = reflector.get_visual("reflector_shell")
    feed_horn = reflector.get_visual("feed_horn")

    ctx.expect_contact(
        top_plate,
        pedestal,
        elem_a=bearing_drum,
        elem_b="pedestal_collar",
        name="azimuth turntable sits on the pedestal collar",
    )
    ctx.expect_contact(
        bracket,
        top_plate,
        elem_a=bracket_base,
        elem_b="center_cap",
        name="elevation bracket is mounted to the rotating top plate",
    )
    ctx.expect_contact(
        bracket,
        reflector,
        elem_a=left_yoke_arm,
        elem_b=trunnion_shaft,
        name="left trunnion is seated against the yoke arm",
    )
    ctx.expect_contact(
        bracket,
        reflector,
        elem_a=right_yoke_arm,
        elem_b=trunnion_shaft,
        name="right trunnion is seated against the yoke arm",
    )
    with ctx.pose({elevation: 0.85}):
        ctx.expect_gap(
            bracket,
            reflector,
            axis="y",
            positive_elem=left_yoke_arm,
            negative_elem=reflector_shell,
            min_gap=0.001,
            name="dish shell clears the left yoke arm at elevated pose",
        )

    rest_shell_aabb = ctx.part_element_world_aabb(reflector, elem=reflector_shell)
    with ctx.pose({azimuth: math.pi / 2.0}):
        az_shell_aabb = ctx.part_element_world_aabb(reflector, elem=reflector_shell)

    rest_shell_center = None
    az_shell_center = None
    if rest_shell_aabb is not None:
        rest_shell_center = tuple(
            0.5 * (rest_shell_aabb[0][index] + rest_shell_aabb[1][index]) for index in range(3)
        )
    if az_shell_aabb is not None:
        az_shell_center = tuple(
            0.5 * (az_shell_aabb[0][index] + az_shell_aabb[1][index]) for index in range(3)
        )
    ctx.check(
        "azimuth rotation swings the reflector around the pedestal",
        rest_shell_center is not None
        and az_shell_center is not None
        and abs(rest_shell_center[1]) < 0.02
        and abs(az_shell_center[1]) > 0.06
        and abs(math.hypot(rest_shell_center[0], rest_shell_center[1]) - math.hypot(az_shell_center[0], az_shell_center[1])) < 0.03,
        details=f"rest_shell_center={rest_shell_center}, az_shell_center={az_shell_center}",
    )

    rest_horn_aabb = ctx.part_element_world_aabb(reflector, elem=feed_horn)
    with ctx.pose({elevation: 0.85}):
        elevated_horn_aabb = ctx.part_element_world_aabb(reflector, elem=feed_horn)
    ctx.check(
        "positive elevation raises the feed horn",
        rest_horn_aabb is not None
        and elevated_horn_aabb is not None
        and 0.5 * (elevated_horn_aabb[0][2] + elevated_horn_aabb[1][2])
        > 0.5 * (rest_horn_aabb[0][2] + rest_horn_aabb[1][2]) + 0.12,
        details=f"rest_horn_aabb={rest_horn_aabb}, elevated_horn_aabb={elevated_horn_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
