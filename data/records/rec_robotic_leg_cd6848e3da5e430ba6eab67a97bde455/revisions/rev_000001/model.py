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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _lateral_cylinder(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    painted_gray = model.material("painted_gray", rgba=(0.57, 0.60, 0.63, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.17, 0.19, 0.21, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.10, 0.11, 0.12, 1.0))

    hip_housing = model.part("hip_housing")
    hip_housing.visual(
        Box((0.240, 0.180, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=dark_steel,
        name="root_mount_plate",
    )
    hip_housing.visual(
        Box((0.210, 0.150, 0.150)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=painted_gray,
        name="upper_leg_housing",
    )
    hip_housing.visual(
        Box((0.090, 0.026, 0.110)),
        origin=Origin(xyz=(0.0, 0.067, -0.005)),
        material=painted_gray,
        name="hip_left_cheek",
    )
    hip_housing.visual(
        Box((0.090, 0.026, 0.110)),
        origin=Origin(xyz=(0.0, -0.067, -0.005)),
        material=painted_gray,
        name="hip_right_cheek",
    )
    hip_housing.visual(
        Box((0.050, 0.070, 0.070)),
        origin=Origin(xyz=(-0.060, 0.0, 0.005)),
        material=gunmetal,
        name="hip_bearing_block",
    )
    hip_side_cap_geom, hip_side_cap_origin = _lateral_cylinder(0.038, 0.028)
    hip_housing.visual(
        hip_side_cap_geom,
        origin=Origin(xyz=(0.0, 0.067, 0.000), rpy=hip_side_cap_origin.rpy),
        material=gunmetal,
        name="hip_left_cap",
    )
    hip_housing.visual(
        hip_side_cap_geom,
        origin=Origin(xyz=(0.0, -0.067, 0.000), rpy=hip_side_cap_origin.rpy),
        material=gunmetal,
        name="hip_right_cap",
    )
    hip_housing.visual(
        Box((0.110, 0.120, 0.050)),
        origin=Origin(xyz=(0.045, 0.0, 0.090)),
        material=black_polymer,
        name="service_cover",
    )
    hip_housing.inertial = Inertial.from_geometry(
        Box((0.240, 0.180, 0.220)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    thigh_module = model.part("thigh_module")
    hip_barrel_geom, hip_barrel_origin = _lateral_cylinder(0.031, 0.106)
    thigh_module.visual(
        hip_barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=hip_barrel_origin.rpy),
        material=gunmetal,
        name="hip_barrel",
    )
    thigh_module.visual(
        Box((0.070, 0.106, 0.055)),
        origin=Origin(xyz=(0.030, 0.0, -0.016)),
        material=gunmetal,
        name="hip_module_block",
    )
    thigh_module.visual(
        Box((0.120, 0.090, 0.360)),
        origin=Origin(xyz=(0.0, 0.0, -0.215)),
        material=painted_gray,
        name="thigh_body",
    )
    thigh_module.visual(
        Box((0.060, 0.110, 0.180)),
        origin=Origin(xyz=(0.030, 0.0, -0.170)),
        material=black_polymer,
        name="thigh_front_cover",
    )
    thigh_module.visual(
        Box((0.075, 0.024, 0.090)),
        origin=Origin(xyz=(0.0, 0.054, -0.395)),
        material=painted_gray,
        name="knee_left_cheek",
    )
    thigh_module.visual(
        Box((0.075, 0.024, 0.090)),
        origin=Origin(xyz=(0.0, -0.054, -0.395)),
        material=painted_gray,
        name="knee_right_cheek",
    )
    thigh_module.visual(
        Box((0.082, 0.080, 0.055)),
        origin=Origin(xyz=(-0.012, 0.0, -0.360)),
        material=gunmetal,
        name="knee_mount_block",
    )
    knee_cap_geom, knee_cap_origin = _lateral_cylinder(0.034, 0.026)
    thigh_module.visual(
        knee_cap_geom,
        origin=Origin(xyz=(0.0, 0.054, -0.420), rpy=knee_cap_origin.rpy),
        material=gunmetal,
        name="knee_left_cap",
    )
    thigh_module.visual(
        knee_cap_geom,
        origin=Origin(xyz=(0.0, -0.054, -0.420), rpy=knee_cap_origin.rpy),
        material=gunmetal,
        name="knee_right_cap",
    )
    thigh_module.inertial = Inertial.from_geometry(
        Box((0.140, 0.120, 0.460)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, -0.230)),
    )

    shank_module = model.part("shank_module")
    knee_barrel_geom, knee_barrel_origin = _lateral_cylinder(0.028, 0.078)
    shank_module.visual(
        knee_barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=knee_barrel_origin.rpy),
        material=gunmetal,
        name="knee_barrel",
    )
    shank_module.visual(
        Box((0.062, 0.078, 0.050)),
        origin=Origin(xyz=(0.026, 0.0, -0.014)),
        material=gunmetal,
        name="knee_module_block",
    )
    shank_module.visual(
        Box((0.100, 0.080, 0.275)),
        origin=Origin(xyz=(0.0, 0.0, -0.168)),
        material=painted_gray,
        name="shank_body",
    )
    shank_module.visual(
        Box((0.050, 0.096, 0.150)),
        origin=Origin(xyz=(0.028, 0.0, -0.138)),
        material=black_polymer,
        name="shin_cover",
    )
    shank_module.visual(
        Box((0.060, 0.022, 0.070)),
        origin=Origin(xyz=(0.0, 0.047, -0.305)),
        material=painted_gray,
        name="ankle_left_cheek",
    )
    shank_module.visual(
        Box((0.060, 0.022, 0.070)),
        origin=Origin(xyz=(0.0, -0.047, -0.305)),
        material=painted_gray,
        name="ankle_right_cheek",
    )
    shank_module.visual(
        Box((0.068, 0.068, 0.050)),
        origin=Origin(xyz=(-0.008, 0.0, -0.280)),
        material=gunmetal,
        name="ankle_mount_block",
    )
    ankle_cap_geom, ankle_cap_origin = _lateral_cylinder(0.029, 0.024)
    shank_module.visual(
        ankle_cap_geom,
        origin=Origin(xyz=(0.0, 0.047, -0.335), rpy=ankle_cap_origin.rpy),
        material=gunmetal,
        name="ankle_left_cap",
    )
    shank_module.visual(
        ankle_cap_geom,
        origin=Origin(xyz=(0.0, -0.047, -0.335), rpy=ankle_cap_origin.rpy),
        material=gunmetal,
        name="ankle_right_cap",
    )
    shank_module.inertial = Inertial.from_geometry(
        Box((0.120, 0.110, 0.360)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
    )

    foot_module = model.part("foot_module")
    ankle_barrel_geom, ankle_barrel_origin = _lateral_cylinder(0.026, 0.072)
    foot_module.visual(
        ankle_barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=ankle_barrel_origin.rpy),
        material=gunmetal,
        name="ankle_barrel",
    )
    foot_module.visual(
        Box((0.058, 0.066, 0.048)),
        origin=Origin(xyz=(0.018, 0.0, -0.014)),
        material=gunmetal,
        name="ankle_module_block",
    )
    foot_module.visual(
        Box((0.046, 0.060, 0.070)),
        origin=Origin(xyz=(0.010, 0.0, -0.060)),
        material=painted_gray,
        name="ankle_pylon",
    )
    foot_module.visual(
        Box((0.170, 0.085, 0.040)),
        origin=Origin(xyz=(0.070, 0.0, -0.105)),
        material=black_polymer,
        name="foot_core",
    )
    foot_module.visual(
        Box((0.100, 0.080, 0.028)),
        origin=Origin(xyz=(0.118, 0.0, -0.136)),
        material=black_polymer,
        name="toe_shell",
    )
    foot_module.visual(
        Box((0.060, 0.070, 0.035)),
        origin=Origin(xyz=(-0.030, 0.0, -0.105)),
        material=dark_steel,
        name="heel_block",
    )
    foot_module.inertial = Inertial.from_geometry(
        Box((0.200, 0.100, 0.150)),
        mass=3.5,
        origin=Origin(xyz=(0.055, 0.0, -0.090)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_housing,
        child=thigh_module,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=2.0,
            lower=math.radians(-50.0),
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_module,
        child=shank_module,
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=280.0,
            velocity=2.3,
            lower=0.0,
            upper=math.radians(130.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_module,
        child=foot_module,
        origin=Origin(xyz=(0.0, 0.0, -0.335)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.8,
            lower=math.radians(-40.0),
            upper=math.radians(30.0),
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

    hip_housing = object_model.get_part("hip_housing")
    thigh_module = object_model.get_part("thigh_module")
    hip_pitch = object_model.get_articulation("hip_pitch")

    ctx.expect_gap(
        hip_housing,
        thigh_module,
        axis="y",
        positive_elem="hip_left_cheek",
        negative_elem="hip_barrel",
        min_gap=0.0005,
        max_gap=0.003,
        name="left hip cheek clears thigh barrel",
    )
    ctx.expect_gap(
        thigh_module,
        hip_housing,
        axis="y",
        positive_elem="hip_barrel",
        negative_elem="hip_right_cheek",
        min_gap=0.0005,
        max_gap=0.003,
        name="right hip cheek clears thigh barrel",
    )

    rest_aabb = ctx.part_world_aabb(thigh_module)
    with ctx.pose({hip_pitch: math.radians(35.0)}):
        flexed_aabb = ctx.part_world_aabb(thigh_module)
    rest_center_x = None
    flexed_center_x = None
    if rest_aabb is not None:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5
    if flexed_aabb is not None:
        flexed_center_x = (flexed_aabb[0][0] + flexed_aabb[1][0]) * 0.5
    ctx.check(
        "hip positive rotation swings thigh forward",
        rest_center_x is not None and flexed_center_x is not None and flexed_center_x > rest_center_x + 0.08,
        details=f"rest_center_x={rest_center_x}, flexed_center_x={flexed_center_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
