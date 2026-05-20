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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_anchor_adjustable_spotlight")

    white_paint = model.material("white_paint", rgba=(0.93, 0.93, 0.91, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.19, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.88, 0.90, 0.92, 0.55))

    head_shell = _mesh(
        "lamp_head_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.046, -0.100),
                (0.044, -0.088),
                (0.042, -0.020),
                (0.041, 0.006),
                (0.036, 0.024),
            ],
            [
                (0.038, -0.094),
                (0.036, -0.082),
                (0.034, -0.018),
                (0.032, 0.004),
                (0.025, 0.018),
            ],
            segments=64,
            lip_samples=8,
        ),
    )

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Cylinder(radius=0.062, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=white_paint,
        name="ceiling_plate",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.050, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=white_paint,
        name="canopy_cup",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=dark_metal,
        name="swivel_socket",
    )
    ceiling_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.062, length=0.044),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=dark_metal,
        name="yoke_hub",
    )
    yoke.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=dark_metal,
        name="yoke_stem",
    )
    yoke.visual(
        Box((0.018, 0.122, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=white_paint,
        name="yoke_bridge",
    )
    yoke.visual(
        Box((0.016, 0.014, 0.088)),
        origin=Origin(xyz=(0.0, 0.060, -0.100)),
        material=white_paint,
        name="yoke_left_arm",
    )
    yoke.visual(
        Box((0.016, 0.014, 0.088)),
        origin=Origin(xyz=(0.0, -0.060, -0.100)),
        material=white_paint,
        name="yoke_right_arm",
    )
    yoke.visual(
        Box((0.020, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.060, -0.112)),
        material=dark_metal,
        name="yoke_left_clevis",
    )
    yoke.visual(
        Box((0.020, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, -0.060, -0.112)),
        material=dark_metal,
        name="yoke_right_clevis",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.050, 0.130, 0.115)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.006, length=0.102),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_shaft",
    )
    lamp_head.visual(
        head_shell,
        material=white_paint,
        name="head_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=white_paint,
        name="rear_cap",
    )
    lamp_head.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=satin_black,
        name="driver_cap",
    )
    lamp_head.visual(
        Cylinder(radius=0.0385, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.091)),
        material=glass,
        name="lens_face",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.128),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
    )

    model.articulation(
        "mount_pan",
        ArticulationType.CONTINUOUS,
        parent=ceiling_mount,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.5),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, -0.112)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-1.05,
            upper=1.05,
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

    ceiling_mount = object_model.get_part("ceiling_mount")
    yoke = object_model.get_part("yoke")
    lamp_head = object_model.get_part("lamp_head")
    mount_pan = object_model.get_articulation("mount_pan")
    yoke_tilt = object_model.get_articulation("yoke_tilt")

    ctx.expect_gap(
        ceiling_mount,
        yoke,
        axis="z",
        positive_elem="swivel_socket",
        negative_elem="yoke_hub",
        max_gap=0.001,
        max_penetration=0.0,
        name="yoke hub seats directly under the ceiling socket",
    )
    ctx.expect_gap(
        yoke,
        lamp_head,
        axis="y",
        positive_elem="yoke_left_clevis",
        negative_elem="trunnion_shaft",
        max_gap=0.0015,
        max_penetration=0.0,
        name="left clevis captures the head trunnion",
    )
    ctx.expect_gap(
        lamp_head,
        yoke,
        axis="y",
        positive_elem="trunnion_shaft",
        negative_elem="yoke_right_clevis",
        max_gap=0.0015,
        max_penetration=0.0,
        name="right clevis captures the head trunnion",
    )

    rest_lens = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="lens_face"))
    with ctx.pose({yoke_tilt: 0.70}):
        tilted_lens = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="lens_face"))
    ctx.check(
        "tilt articulation pitches the lamp head forward and upward",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[0] > rest_lens[0] + 0.04
        and tilted_lens[2] > rest_lens[2] + 0.015,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    with ctx.pose({yoke_tilt: 0.70, mount_pan: 0.0}):
        pan0_lens = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="lens_face"))
    with ctx.pose({yoke_tilt: 0.70, mount_pan: math.pi / 2.0}):
        pan90_lens = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="lens_face"))

    radius0 = None if pan0_lens is None else math.hypot(pan0_lens[0], pan0_lens[1])
    radius90 = None if pan90_lens is None else math.hypot(pan90_lens[0], pan90_lens[1])
    ctx.check(
        "pan articulation yaws the tilted lamp around the vertical axis",
        pan0_lens is not None
        and pan90_lens is not None
        and radius0 is not None
        and radius90 is not None
        and pan0_lens[0] > 0.04
        and abs(pan0_lens[1]) < 0.01
        and pan90_lens[1] > 0.04
        and abs(pan90_lens[0]) < 0.01
        and abs(radius0 - radius90) < 0.005,
        details=f"pan0={pan0_lens}, pan90={pan90_lens}, radius0={radius0}, radius90={radius90}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
