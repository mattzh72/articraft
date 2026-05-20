from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_cctv_pole")

    galvanized_steel = model.material("galvanized_steel", rgba=(0.44, 0.46, 0.49, 1.0))
    painted_housing = model.material("painted_housing", rgba=(0.84, 0.85, 0.87, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.20, 1.0))
    smoked_dome = model.material("smoked_dome", rgba=(0.18, 0.21, 0.24, 0.55))

    base_plate_size = 0.24
    base_plate_thickness = 0.02
    pole_radius = 0.048
    pole_length = 1.70
    mast_cap_radius = 0.06
    mast_cap_length = 0.04
    mast_top_z = base_plate_thickness + pole_length + mast_cap_length

    mast = model.part("mast")
    mast.visual(
        Box((base_plate_size, base_plate_size, base_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness * 0.5)),
        material=galvanized_steel,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.09, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness + 0.006)),
        material=galvanized_steel,
        name="pole_flange",
    )
    mast.visual(
        Cylinder(radius=pole_radius, length=pole_length),
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness + pole_length * 0.5)),
        material=galvanized_steel,
        name="pole_shaft",
    )
    mast.visual(
        Cylinder(radius=mast_cap_radius, length=mast_cap_length),
        origin=Origin(xyz=(0.0, 0.0, mast_top_z - mast_cap_length * 0.5)),
        material=galvanized_steel,
        name="mast_cap",
    )

    yoke = model.part("pan_yoke")
    yoke.visual(
        Cylinder(radius=0.06, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=painted_housing,
        name="pan_housing",
    )
    yoke.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=painted_housing,
        name="pan_cap",
    )
    yoke.visual(
        Box((0.21, 0.066, 0.062)),
        origin=Origin(xyz=(0.105, 0.0, 0.055)),
        material=painted_housing,
        name="front_arm",
    )
    yoke.visual(
        Box((0.15, 0.04, 0.04)),
        origin=Origin(xyz=(0.075, 0.0, 0.025)),
        material=galvanized_steel,
        name="arm_gusset",
    )
    yoke.visual(
        Box((0.04, 0.012, 0.10)),
        origin=Origin(xyz=(0.24, -0.071, 0.05)),
        material=painted_housing,
        name="left_cheek",
    )
    yoke.visual(
        Box((0.04, 0.012, 0.10)),
        origin=Origin(xyz=(0.24, 0.071, 0.05)),
        material=painted_housing,
        name="right_cheek",
    )
    yoke.visual(
        Box((0.032, 0.154, 0.02)),
        origin=Origin(xyz=(0.224, 0.0, 0.088)),
        material=painted_housing,
        name="cheek_bridge",
    )

    camera = model.part("camera_head")
    camera.visual(
        Cylinder(radius=0.012, length=0.13),
        origin=Origin(rpy=(-pi * 0.5, 0.0, 0.0)),
        material=dark_trim,
        name="tilt_trunnion",
    )
    camera.visual(
        Box((0.10, 0.045, 0.03)),
        origin=Origin(xyz=(0.05, 0.0, -0.015)),
        material=painted_housing,
        name="camera_stem",
    )
    camera.visual(
        Box((0.12, 0.09, 0.018)),
        origin=Origin(xyz=(0.07, 0.0, -0.009)),
        material=painted_housing,
        name="camera_shroud",
    )
    camera.visual(
        Cylinder(radius=0.075, length=0.05),
        origin=Origin(xyz=(0.115, 0.0, -0.045)),
        material=painted_housing,
        name="camera_can",
    )
    camera.visual(
        Cylinder(radius=0.08, length=0.014),
        origin=Origin(xyz=(0.115, 0.0, -0.063)),
        material=dark_trim,
        name="dome_ring",
    )

    dome_geom = DomeGeometry(0.072, radial_segments=36, height_segments=18, closed=True)
    dome_geom.rotate_x(pi)
    dome_geom.translate(0.115, 0.0, -0.07)
    camera.visual(
        mesh_from_geometry(dome_geom, "camera_dome_shell"),
        material=smoked_dome,
        name="camera_dome",
    )

    model.articulation(
        "mast_to_pan",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, mast_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "yoke_to_camera",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=camera,
        origin=Origin(xyz=(0.24, 0.0, 0.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-1.1,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    yoke = object_model.get_part("pan_yoke")
    camera = object_model.get_part("camera_head")
    pan = object_model.get_articulation("mast_to_pan")
    tilt = object_model.get_articulation("yoke_to_camera")

    def aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

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

    ctx.expect_gap(
        yoke,
        mast,
        axis="z",
        positive_elem="pan_housing",
        negative_elem="mast_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan housing seats on mast cap",
    )
    ctx.expect_overlap(
        yoke,
        mast,
        axes="xy",
        elem_a="pan_housing",
        elem_b="mast_cap",
        min_overlap=0.10,
        name="pan housing stays centered over mast cap",
    )
    ctx.expect_contact(
        camera,
        yoke,
        elem_a="tilt_trunnion",
        elem_b="left_cheek",
        name="left cheek supports tilt trunnion",
    )
    ctx.expect_contact(
        camera,
        yoke,
        elem_a="tilt_trunnion",
        elem_b="right_cheek",
        name="right cheek supports tilt trunnion",
    )

    ctx.check(
        "pan joint uses vertical axis",
        pan.axis == (0.0, 0.0, 1.0)
        and pan.motion_limits is not None
        and pan.motion_limits.lower is not None
        and pan.motion_limits.upper is not None
        and pan.motion_limits.lower < 0.0 < pan.motion_limits.upper,
        details=f"axis={pan.axis}, limits={pan.motion_limits}",
    )
    ctx.check(
        "tilt joint uses horizontal axis",
        tilt.axis == (0.0, -1.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper,
        details=f"axis={tilt.axis}, limits={tilt.motion_limits}",
    )

    rest_camera_pos = ctx.part_world_position(camera)
    with ctx.pose({pan: 1.2}):
        panned_camera_pos = ctx.part_world_position(camera)
    ctx.check(
        "pan rotation swings camera around mast",
        rest_camera_pos is not None
        and panned_camera_pos is not None
        and panned_camera_pos[1] > rest_camera_pos[1] + 0.18
        and panned_camera_pos[0] < rest_camera_pos[0] - 0.10,
        details=f"rest={rest_camera_pos}, panned={panned_camera_pos}",
    )

    rest_dome = ctx.part_element_world_aabb(camera, elem="camera_dome")
    with ctx.pose({tilt: 0.45}):
        raised_dome = ctx.part_element_world_aabb(camera, elem="camera_dome")
    rest_dome_z = aabb_center_z(rest_dome)
    raised_dome_z = aabb_center_z(raised_dome)
    ctx.check(
        "positive tilt raises dome housing",
        rest_dome_z is not None and raised_dome_z is not None and raised_dome_z > rest_dome_z + 0.03,
        details=f"rest_z={rest_dome_z}, raised_z={raised_dome_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
