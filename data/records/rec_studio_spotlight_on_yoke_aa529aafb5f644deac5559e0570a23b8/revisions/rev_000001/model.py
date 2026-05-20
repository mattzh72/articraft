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
    model = ArticulatedObject(name="studio_spotlight_on_yoke_stand")

    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.19, 0.20, 0.22, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.48, 0.49, 0.52, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.78, 0.88, 0.98, 0.45))

    lamp_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.118, 0.000),
                (0.125, 0.014),
                (0.122, 0.055),
                (0.118, 0.175),
                (0.108, 0.260),
                (0.091, 0.318),
            ],
            inner_profile=[
                (0.101, 0.012),
                (0.108, 0.030),
                (0.105, 0.055),
                (0.101, 0.175),
                (0.092, 0.255),
                (0.076, 0.306),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "lamp_shell",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.210, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=satin_steel,
        name="base_collar",
    )
    stand.visual(
        Cylinder(radius=0.022, length=1.220),
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        material=satin_steel,
        name="main_stand_tube",
    )
    stand.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        material=dark_steel,
        name="height_lock",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 1.300)),
        material=satin_steel,
        name="upper_spigot",
    )
    stand.visual(
        Cylinder(radius=0.056, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 1.390)),
        material=dark_steel,
        name="top_receiver",
    )
    stand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.210, length=1.440),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="pan_spindle",
    )
    yoke.visual(
        Cylinder(radius=0.078, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_steel,
        name="pan_plate",
    )
    yoke.visual(
        Box((0.040, 0.080, 0.080)),
        origin=Origin(xyz=(-0.070, 0.0, 0.075)),
        material=dark_steel,
        name="rear_web",
    )
    yoke.visual(
        Box((0.040, 0.200, 0.040)),
        origin=Origin(xyz=(-0.070, 0.0, 0.110)),
        material=dark_steel,
        name="lower_crossbar",
    )
    yoke.visual(
        Box((0.040, 0.100, 0.060)),
        origin=Origin(xyz=(-0.070, -0.110, 0.130)),
        material=dark_steel,
        name="left_bracket",
    )
    yoke.visual(
        Box((0.040, 0.100, 0.060)),
        origin=Origin(xyz=(-0.070, 0.110, 0.130)),
        material=dark_steel,
        name="right_bracket",
    )
    yoke.visual(
        Box((0.045, 0.024, 0.250)),
        origin=Origin(xyz=(-0.030, -0.160, 0.220)),
        material=dark_steel,
        name="left_arm",
    )
    yoke.visual(
        Box((0.045, 0.024, 0.250)),
        origin=Origin(xyz=(-0.030, 0.160, 0.220)),
        material=dark_steel,
        name="right_arm",
    )
    yoke.visual(
        Box((0.045, 0.344, 0.022)),
        origin=Origin(xyz=(-0.030, 0.0, 0.334)),
        material=dark_steel,
        name="top_bridge",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.140, 0.360, 0.360)),
        mass=2.8,
        origin=Origin(xyz=(-0.040, 0.0, 0.180)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        lamp_shell,
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=matte_black,
        name="can_shell",
    )
    lamp.visual(
        Cylinder(radius=0.102, length=0.006),
        origin=Origin(xyz=(0.255, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Cylinder(radius=0.078, length=0.040),
        origin=Origin(xyz=(-0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_cap",
    )
    lamp.visual(
        Cylinder(radius=0.037, length=0.020),
        origin=Origin(xyz=(0.0, -0.112, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion_hub",
    )
    lamp.visual(
        Cylinder(radius=0.037, length=0.020),
        origin=Origin(xyz=(0.0, 0.112, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion_hub",
    )
    lamp.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, -0.135, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="left_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, 0.135, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="right_trunnion",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.330, 0.260, 0.260)),
        mass=4.6,
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-3.0,
            upper=3.0,
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-1.05,
            upper=1.15,
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

    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("stand_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_lamp_tilt")

    ctx.expect_origin_distance(
        yoke,
        stand,
        axes="xy",
        max_dist=0.001,
        name="yoke pan axis stays centered on stand",
    )
    ctx.expect_origin_distance(
        lamp,
        stand,
        axes="xy",
        max_dist=0.001,
        name="lamp tilt axis stays centered on stand",
    )
    ctx.expect_gap(
        yoke,
        stand,
        axis="z",
        positive_elem="pan_spindle",
        negative_elem="top_receiver",
        max_gap=0.0005,
        max_penetration=0.0,
        name="pan spindle seats on receiver",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a="left_trunnion",
        elem_b="left_arm",
        name="left trunnion reaches left yoke arm",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a="right_trunnion",
        elem_b="right_arm",
        name="right trunnion reaches right yoke arm",
    )

    lens_rest = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({pan: 1.1, tilt: 0.65}):
        ctx.expect_origin_distance(
            lamp,
            stand,
            axes="xy",
            max_dist=0.001,
            name="lamp stays centered while panned",
        )
        lens_tilted = ctx.part_element_world_aabb(lamp, elem="front_lens")

    lens_rest_z = None if lens_rest is None else 0.5 * (lens_rest[0][2] + lens_rest[1][2])
    lens_tilted_z = None if lens_tilted is None else 0.5 * (lens_tilted[0][2] + lens_tilted[1][2])
    ctx.check(
        "positive tilt lifts beam center",
        lens_rest_z is not None
        and lens_tilted_z is not None
        and lens_tilted_z > lens_rest_z + 0.05,
        details=f"rest_z={lens_rest_z}, tilted_z={lens_tilted_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
