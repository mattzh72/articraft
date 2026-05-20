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
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_dish")

    pedestal_gray = model.material("pedestal_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.55, 0.58, 0.61, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.90, 0.92, 0.94, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.14, 0.15, 0.17, 1.0))

    base_ring_mesh = _mesh(
        "azimuth_base_ring",
        TorusGeometry(radius=0.17, tube=0.028, radial_segments=18, tubular_segments=56),
    )
    rim_mesh = _mesh(
        "reflector_rim",
        TorusGeometry(radius=0.386, tube=0.0105, radial_segments=14, tubular_segments=72),
    )
    reflector_shell_mesh = _mesh(
        "reflector_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.020, -0.006),
                (0.080, 0.000),
                (0.155, 0.012),
                (0.245, 0.034),
                (0.330, 0.068),
                (0.390, 0.104),
            ],
            [
                (0.000, 0.000),
                (0.070, 0.006),
                (0.145, 0.018),
                (0.235, 0.040),
                (0.320, 0.072),
                (0.382, 0.098),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.16, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=pedestal_gray,
        name="ground_pad",
    )
    pedestal.visual(
        Cylinder(radius=0.10, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=pedestal_gray,
        name="bearing_drum",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=trim_dark,
        name="azimuth_stub",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.32, 0.32, 0.11)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(
        base_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=mount_gray,
        name="base_ring",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=trim_dark,
        name="hub_core",
    )
    azimuth_stage.visual(
        Box((0.30, 0.035, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=mount_gray,
        name="spoke_x",
    )
    azimuth_stage.visual(
        Box((0.035, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=mount_gray,
        name="spoke_y",
    )
    azimuth_stage.visual(
        Box((0.05, 0.10, 0.42)),
        origin=Origin(xyz=(0.185, -0.015, 0.23)),
        material=mount_gray,
        name="left_cheek",
    )
    azimuth_stage.visual(
        Box((0.05, 0.10, 0.42)),
        origin=Origin(xyz=(-0.185, -0.015, 0.23)),
        material=mount_gray,
        name="right_cheek",
    )
    azimuth_stage.visual(
        Box((0.38, 0.06, 0.14)),
        origin=Origin(xyz=(0.0, -0.08, 0.18)),
        material=trim_dark,
        name="rear_bridge",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.048, length=0.04),
        origin=Origin(xyz=(0.18, 0.0, 0.35), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="left_bearing_boss",
    )
    azimuth_stage.visual(
        Cylinder(radius=0.048, length=0.04),
        origin=Origin(xyz=(-0.18, 0.0, 0.35), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="right_bearing_boss",
    )
    azimuth_stage.inertial = Inertial.from_geometry(
        Box((0.42, 0.36, 0.46)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )

    reflector_assembly = model.part("reflector_assembly")
    reflector_assembly.visual(
        Cylinder(radius=0.016, length=0.26),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="trunnion_shaft",
    )
    reflector_assembly.visual(
        Cylinder(radius=0.03, length=0.03),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="left_trunnion_end",
    )
    reflector_assembly.visual(
        Cylinder(radius=0.03, length=0.03),
        origin=Origin(xyz=(-0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="right_trunnion_end",
    )
    reflector_assembly.visual(
        Box((0.08, 0.10, 0.26)),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        material=mount_gray,
        name="back_frame",
    )
    reflector_assembly.visual(
        Cylinder(radius=0.05, length=0.16),
        origin=Origin(xyz=(0.0, 0.12, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mount_gray,
        name="hub_barrel",
    )
    reflector_assembly.visual(
        reflector_shell_mesh,
        origin=Origin(xyz=(0.0, 0.197, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=reflector_white,
        name="reflector_shell",
    )
    reflector_assembly.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, 0.297, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="reflector_rim",
    )
    reflector_assembly.inertial = Inertial.from_geometry(
        Box((0.80, 0.34, 0.80)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.20, 0.0)),
    )

    model.articulation(
        "pedestal_to_azimuth_stage",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=azimuth_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5),
    )
    model.articulation(
        "azimuth_stage_to_reflector",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=reflector_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=math.radians(-10.0),
            upper=math.radians(75.0),
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
    azimuth_stage = object_model.get_part("azimuth_stage")
    reflector = object_model.get_part("reflector_assembly")
    azimuth = object_model.get_articulation("pedestal_to_azimuth_stage")
    elevation = object_model.get_articulation("azimuth_stage_to_reflector")

    ctx.expect_overlap(
        azimuth_stage,
        pedestal,
        axes="xy",
        min_overlap=0.10,
        name="azimuth stage stays centered over the pedestal",
    )
    ctx.expect_gap(
        azimuth_stage,
        pedestal,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="rotating stage sits tightly on the pedestal bearing",
    )
    ctx.expect_gap(
        azimuth_stage,
        reflector,
        axis="x",
        positive_elem="left_bearing_boss",
        negative_elem="left_trunnion_end",
        min_gap=0.0,
        max_gap=0.001,
        name="left trunnion seats against the left cheek boss",
    )
    ctx.expect_gap(
        reflector,
        azimuth_stage,
        axis="x",
        positive_elem="right_trunnion_end",
        negative_elem="right_bearing_boss",
        min_gap=0.0,
        max_gap=0.001,
        name="right trunnion seats against the right cheek boss",
    )

    left_cheek_aabb = ctx.part_element_world_aabb(azimuth_stage, elem="left_cheek")
    right_cheek_aabb = ctx.part_element_world_aabb(azimuth_stage, elem="right_cheek")
    shaft_aabb = ctx.part_element_world_aabb(reflector, elem="trunnion_shaft")
    shaft_between_cheeks = (
        left_cheek_aabb is not None
        and right_cheek_aabb is not None
        and shaft_aabb is not None
        and shaft_aabb[1][0] <= left_cheek_aabb[0][0] - 0.02
        and shaft_aabb[0][0] >= right_cheek_aabb[1][0] + 0.02
    )
    ctx.check(
        "trunnion shaft stays between the side cheeks",
        shaft_between_cheeks,
        details=(
            f"left_cheek={left_cheek_aabb}, right_cheek={right_cheek_aabb}, "
            f"shaft={shaft_aabb}"
        ),
    )

    rest_shell = ctx.part_element_world_aabb(reflector, elem="reflector_shell")
    rest_center = _aabb_center(rest_shell)

    with ctx.pose({azimuth: math.pi / 2.0}):
        az_shell = ctx.part_element_world_aabb(reflector, elem="reflector_shell")
        az_center = _aabb_center(az_shell)
    ctx.check(
        "azimuth rotation swings the reflector around the vertical axis",
        rest_center is not None
        and az_center is not None
        and abs(rest_center[1]) > 0.18
        and abs(az_center[0]) > 0.18
        and abs(az_center[1]) < 0.08,
        details=f"rest_center={rest_center}, azimuth_center={az_center}",
    )

    with ctx.pose({elevation: math.radians(60.0)}):
        elevated_shell = ctx.part_element_world_aabb(reflector, elem="reflector_shell")
        elevated_center = _aabb_center(elevated_shell)
    ctx.check(
        "elevation rotation raises the reflector upward",
        rest_center is not None
        and elevated_center is not None
        and elevated_center[2] > rest_center[2] + 0.10
        and elevated_center[1] < rest_center[1] - 0.08,
        details=f"rest_center={rest_center}, elevated_center={elevated_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
