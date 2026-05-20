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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    concrete = model.material("concrete", rgba=(0.66, 0.67, 0.69, 1.0))
    tower_paint = model.material("tower_paint", rgba=(0.77, 0.79, 0.81, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.78, 0.88, 0.96, 0.55))

    support = model.part("support")
    support.visual(
        Box((0.92, 0.92, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="foundation_pad",
    )
    support.visual(
        Cylinder(radius=0.09, length=1.20),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=tower_paint,
        name="mast_column",
    )
    support.visual(
        Box((0.18, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.43)),
        material=machinery_gray,
        name="mast_cap",
    )
    support.visual(
        Box((0.36, 0.14, 0.08)),
        origin=Origin(xyz=(0.18, 0.0, 1.43)),
        material=machinery_gray,
        name="offset_boom",
    )
    support.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.18, 0.0, 1.51)),
        material=dark_metal,
        name="pan_pedestal",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.08, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="slew_ring",
    )
    yoke.visual(
        Cylinder(radius=0.05, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=machinery_gray,
        name="yoke_post",
    )
    yoke.visual(
        Box((0.12, 0.18, 0.06)),
        origin=Origin(xyz=(0.02, 0.0, 0.25)),
        material=machinery_gray,
        name="crosshead",
    )
    yoke.visual(
        Box((0.12, 0.03, 0.18)),
        origin=Origin(xyz=(0.14, 0.095, 0.19)),
        material=tower_paint,
        name="left_arm",
    )
    yoke.visual(
        Box((0.12, 0.03, 0.18)),
        origin=Origin(xyz=(0.14, -0.095, 0.19)),
        material=tower_paint,
        name="right_arm",
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=yoke,
        origin=Origin(xyz=(0.18, 0.0, 1.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.8,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_shaft",
    )
    lamp.visual(
        Cylinder(radius=0.08, length=0.24),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tower_paint,
        name="lamp_shell",
    )
    lamp.visual(
        Cylinder(radius=0.10, length=0.04),
        origin=Origin(xyz=(0.25, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.088, length=0.008),
        origin=Origin(xyz=(0.266, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Cylinder(radius=0.06, length=0.07),
        origin=Origin(xyz=(-0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="rear_cap",
    )
    lamp.visual(
        Box((0.09, 0.12, 0.04)),
        origin=Origin(xyz=(0.02, 0.0, 0.10)),
        material=machinery_gray,
        name="ballast_box",
    )

    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.15, 0.0, 0.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.7,
            lower=math.radians(-35.0),
            upper=math.radians(65.0),
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
    support = object_model.get_part("support")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    ctx.expect_origin_gap(
        yoke,
        support,
        axis="x",
        min_gap=0.12,
        name="pan assembly is offset from the mast centerline",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="y",
        margin=0.0,
        name="lamp body stays between the yoke arms",
    )

    rest_lamp_pos = ctx.part_world_position(lamp)
    with ctx.pose({pan_axis: math.radians(70.0)}):
        panned_lamp_pos = ctx.part_world_position(lamp)
    ctx.check(
        "pan joint swings the lamp laterally",
        rest_lamp_pos is not None
        and panned_lamp_pos is not None
        and panned_lamp_pos[1] > rest_lamp_pos[1] + 0.06,
        details=f"rest={rest_lamp_pos}, panned={panned_lamp_pos}",
    )

    def center_z(aabb):
        return 0.5 * (aabb[0][2] + aabb[1][2]) if aabb is not None else None

    rest_bezel = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({tilt_axis: math.radians(60.0)}):
        tilted_bezel = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    rest_bezel_z = center_z(rest_bezel)
    tilted_bezel_z = center_z(tilted_bezel)
    ctx.check(
        "tilt joint raises the searchlight nose",
        rest_bezel_z is not None
        and tilted_bezel_z is not None
        and tilted_bezel_z > rest_bezel_z + 0.10,
        details=f"rest_z={rest_bezel_z}, tilted_z={tilted_bezel_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
