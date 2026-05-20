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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.66, 1.0))
    mast_gray = model.material("mast_gray", rgba=(0.31, 0.34, 0.37, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.55, 0.58, 0.61, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.15, 0.17, 1.0))
    glass = model.material("glass", rgba=(0.82, 0.89, 0.94, 0.45))

    lamp_shell_profile_outer = [
        (0.17, -0.20),
        (0.18, -0.15),
        (0.20, -0.08),
        (0.23, 0.08),
        (0.24, 0.28),
        (0.255, 0.40),
    ]
    lamp_shell_profile_inner = [
        (0.13, -0.19),
        (0.14, -0.13),
        (0.16, -0.06),
        (0.19, 0.08),
        (0.20, 0.27),
        (0.215, 0.39),
    ]
    lamp_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            lamp_shell_profile_outer,
            lamp_shell_profile_inner,
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "searchlight_shell",
    )

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((1.50, 1.50, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="foundation_block",
    )
    tower_base.visual(
        Box((0.74, 0.74, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=dark_metal,
        name="anchor_plinth",
    )
    tower_base.visual(
        Cylinder(radius=0.22, length=1.82),
        origin=Origin(xyz=(0.0, 0.0, 1.23)),
        material=mast_gray,
        name="mast_column",
    )
    tower_base.visual(
        Cylinder(radius=0.29, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.14)),
        material=dark_metal,
        name="mast_cap",
    )
    tower_base.visual(
        Box((0.34, 0.46, 0.42)),
        origin=Origin(xyz=(-0.29, 0.0, 0.51)),
        material=housing_gray,
        name="service_box",
    )
    tower_base.visual(
        Box((0.16, 0.08, 0.54)),
        origin=Origin(xyz=(-0.15, 0.0, 0.72)),
        material=dark_metal,
        name="service_box_stem",
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((1.50, 1.50, 2.22)),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.11)),
    )

    bearing_module = model.part("bearing_module")
    bearing_module.visual(
        Cylinder(radius=0.33, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="slewing_ring_lower",
    )
    bearing_module.visual(
        Cylinder(radius=0.30, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=housing_gray,
        name="slewing_ring_upper",
    )
    bearing_module.visual(
        Box((0.52, 0.40, 0.20)),
        origin=Origin(xyz=(0.03, 0.0, 0.26)),
        material=housing_gray,
        name="bearing_housing",
    )
    bearing_module.visual(
        Box((0.22, 0.24, 0.18)),
        origin=Origin(xyz=(-0.29, 0.0, 0.23)),
        material=dark_metal,
        name="drive_pack",
    )
    bearing_module.visual(
        Box((0.40, 0.30, 0.08)),
        origin=Origin(xyz=(0.02, 0.0, 0.40)),
        material=dark_metal,
        name="yoke_mount_plate",
    )
    bearing_module.inertial = Inertial.from_geometry(
        Box((0.60, 0.45, 0.46)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )

    yoke_frame = model.part("yoke_frame")
    yoke_frame.visual(
        Box((0.24, 0.20, 0.54)),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=mast_gray,
        name="yoke_kingpost",
    )
    yoke_frame.visual(
        Box((0.32, 0.26, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=dark_metal,
        name="yoke_saddle",
    )
    yoke_frame.visual(
        Box((0.44, 0.96, 0.10)),
        origin=Origin(xyz=(0.18, 0.0, 0.67)),
        material=mast_gray,
        name="yoke_crossbeam",
    )
    yoke_frame.visual(
        Box((0.16, 0.08, 0.50)),
        origin=Origin(xyz=(0.35, 0.44, 0.40)),
        material=mast_gray,
        name="left_arm",
    )
    yoke_frame.visual(
        Box((0.16, 0.08, 0.50)),
        origin=Origin(xyz=(0.35, -0.44, 0.40)),
        material=mast_gray,
        name="right_arm",
    )
    yoke_frame.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(xyz=(0.35, 0.385, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion_collar",
    )
    yoke_frame.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(xyz=(0.35, -0.385, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion_collar",
    )
    yoke_frame.inertial = Inertial.from_geometry(
        Box((0.40, 1.00, 0.76)),
        mass=180.0,
        origin=Origin(xyz=(0.04, 0.0, 0.38)),
    )

    lamp_body = model.part("lamp_body")
    lamp_body.visual(
        lamp_shell_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_gray,
        name="lamp_shell",
    )
    lamp_body.visual(
        Cylinder(radius=0.26, length=0.04),
        origin=Origin(xyz=(0.39, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    lamp_body.visual(
        Cylinder(radius=0.235, length=0.03),
        origin=Origin(xyz=(0.405, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    lamp_body.visual(
        Cylinder(radius=0.155, length=0.08),
        origin=Origin(xyz=(-0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_cap",
    )
    lamp_body.visual(
        Box((0.18, 0.18, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, -0.21)),
        material=dark_metal,
        name="underslung_gearbox",
    )
    lamp_body.visual(
        Cylinder(radius=0.055, length=0.15),
        origin=Origin(xyz=(0.0, 0.28, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    lamp_body.visual(
        Cylinder(radius=0.055, length=0.15),
        origin=Origin(xyz=(0.0, -0.28, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    lamp_body.inertial = Inertial.from_geometry(
        Box((0.74, 0.70, 0.62)),
        mass=85.0,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=tower_base,
        child=bearing_module,
        origin=Origin(xyz=(0.0, 0.0, 2.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.7,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "bearing_to_yoke",
        ArticulationType.FIXED,
        parent=bearing_module,
        child=yoke_frame,
        origin=Origin(xyz=(0.02, 0.0, 0.44)),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=yoke_frame,
        child=lamp_body,
        origin=Origin(xyz=(0.35, 0.0, 0.30)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=0.9,
            lower=-math.radians(35.0),
            upper=math.radians(70.0),
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

    base = object_model.get_part("tower_base")
    bearing = object_model.get_part("bearing_module")
    yoke = object_model.get_part("yoke_frame")
    lamp = object_model.get_part("lamp_body")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    ctx.expect_gap(
        bearing,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="bearing module seats on mast cap",
    )
    ctx.expect_gap(
        yoke,
        bearing,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="yoke frame seats on rotating bearing module",
    )
    ctx.expect_gap(
        yoke,
        lamp,
        axis="y",
        positive_elem="left_trunnion_collar",
        negative_elem="left_trunnion",
        max_gap=0.01,
        max_penetration=1e-6,
        name="left lamp trunnion sits close to left yoke collar",
    )
    ctx.expect_gap(
        lamp,
        yoke,
        axis="y",
        positive_elem="right_trunnion",
        negative_elem="right_trunnion_collar",
        max_gap=0.01,
        max_penetration=1e-6,
        name="right lamp trunnion sits close to right yoke collar",
    )

    def elem_center_z(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def elem_center_xy(part_name: str, elem_name: str) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
        )

    rest_front_z = elem_center_z("lamp_body", "front_bezel")
    with ctx.pose({tilt_axis: math.radians(45.0)}):
        tilted_front_z = elem_center_z("lamp_body", "front_bezel")
    ctx.check(
        "positive tilt raises the front bezel",
        rest_front_z is not None
        and tilted_front_z is not None
        and tilted_front_z > rest_front_z + 0.10,
        details=f"rest_front_z={rest_front_z}, tilted_front_z={tilted_front_z}",
    )

    rest_front_xy = elem_center_xy("lamp_body", "front_bezel")
    with ctx.pose({pan_axis: math.radians(60.0)}):
        panned_front_xy = elem_center_xy("lamp_body", "front_bezel")
    ctx.check(
        "positive pan swings the lamp toward +Y",
        rest_front_xy is not None
        and panned_front_xy is not None
        and panned_front_xy[1] > rest_front_xy[1] + 0.25
        and panned_front_xy[0] < rest_front_xy[0] - 0.10,
        details=f"rest_front_xy={rest_front_xy}, panned_front_xy={panned_front_xy}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
