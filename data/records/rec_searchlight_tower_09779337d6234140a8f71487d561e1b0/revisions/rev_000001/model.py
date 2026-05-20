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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    concrete = model.material("concrete", rgba=(0.64, 0.65, 0.66, 1.0))
    tower_gray = model.material("tower_gray", rgba=(0.43, 0.46, 0.49, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    glass = model.material("glass", rgba=(0.74, 0.84, 0.92, 0.72))

    tower = model.part("tower")
    tower.visual(
        Box((1.40, 1.40, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=concrete,
        name="foundation",
    )
    tower.visual(
        Cylinder(radius=0.34, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=tower_gray,
        name="pedestal",
    )
    tower.visual(
        Box((0.28, 0.28, 4.60)),
        origin=Origin(xyz=(0.0, 0.0, 2.96)),
        material=tower_gray,
        name="mast",
    )
    tower.visual(
        Cylinder(radius=0.22, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 5.37)),
        material=dark_metal,
        name="pan_bearing_housing",
    )
    tower.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 5.52)),
        material=tower_gray,
        name="mast_cap",
    )
    tower.inertial = Inertial.from_geometry(
        Box((1.40, 1.40, 5.56)),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 2.78)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.26, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_metal,
        name="turntable_drum",
    )
    yoke.visual(
        Cylinder(radius=0.18, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=housing_gray,
        name="rotary_collar",
    )
    yoke.visual(
        Box((0.16, 0.70, 0.10)),
        origin=Origin(xyz=(-0.12, 0.0, 0.93)),
        material=housing_gray,
        name="crosshead",
    )
    yoke.visual(
        Box((0.22, 0.10, 0.86)),
        origin=Origin(xyz=(-0.05, 0.30, 0.49)),
        material=housing_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.22, 0.10, 0.86)),
        origin=Origin(xyz=(-0.05, -0.30, 0.49)),
        material=housing_gray,
        name="right_arm",
    )
    yoke.visual(
        Cylinder(radius=0.10, length=0.08),
        origin=Origin(xyz=(0.02, 0.29, 0.56), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_tilt_housing",
    )
    yoke.visual(
        Cylinder(radius=0.10, length=0.08),
        origin=Origin(xyz=(0.02, -0.29, 0.56), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_tilt_housing",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.66, 0.66, 0.84)),
        mass=180.0,
        origin=Origin(xyz=(-0.01, 0.0, 0.42)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.20, length=0.46),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tower_gray,
        name="barrel",
    )
    lamp.visual(
        Cylinder(radius=0.25, length=0.05),
        origin=Origin(xyz=(0.29, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_gray,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.23, length=0.008),
        origin=Origin(xyz=(0.319, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    lamp.visual(
        Box((0.20, 0.28, 0.22)),
        origin=Origin(xyz=(-0.16, 0.0, 0.0)),
        material=dark_metal,
        name="rear_box",
    )
    lamp.visual(
        Cylinder(radius=0.13, length=0.10),
        origin=Origin(xyz=(-0.24, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_cap",
    )
    lamp.visual(
        Box((0.18, 0.18, 0.06)),
        origin=Origin(xyz=(-0.02, 0.0, 0.22)),
        material=dark_metal,
        name="top_vent_housing",
    )
    lamp.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.02, 0.21, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_gray,
        name="left_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.02, -0.21, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_gray,
        name="right_trunnion",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.63, 0.50, 0.50)),
        mass=90.0,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 5.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.02, 0.0, 0.56)),
        # The lamp body points along local +X at rest; -Y makes positive
        # articulation values raise the beam toward +Z.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.9,
            lower=-0.30,
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
    tower = object_model.get_part("tower")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("tower_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_lamp_tilt")

    ctx.expect_gap(
        yoke,
        tower,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="pan turntable seats on mast cap",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="y",
        margin=0.0,
        name="lamp stays between yoke arms in width",
    )

    lens_rest = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({tilt: 0.85}):
        lens_tilted = ctx.part_element_world_aabb(lamp, elem="front_lens")
    rest_lens_z = None if lens_rest is None else 0.5 * (lens_rest[0][2] + lens_rest[1][2])
    tilted_lens_z = None if lens_tilted is None else 0.5 * (lens_tilted[0][2] + lens_tilted[1][2])
    ctx.check(
        "positive tilt raises searchlight nose",
        rest_lens_z is not None and tilted_lens_z is not None and tilted_lens_z > rest_lens_z + 0.10,
        details=f"rest_z={rest_lens_z}, tilted_z={tilted_lens_z}",
    )

    with ctx.pose({pan: 1.20}):
        lens_panned = ctx.part_element_world_aabb(lamp, elem="front_lens")
    rest_lens_y = None if lens_rest is None else 0.5 * (lens_rest[0][1] + lens_rest[1][1])
    panned_lens_y = None if lens_panned is None else 0.5 * (lens_panned[0][1] + lens_panned[1][1])
    ctx.check(
        "positive pan swings lamp toward positive y",
        rest_lens_y is not None and panned_lens_y is not None and panned_lens_y > rest_lens_y + 0.18,
        details=f"rest_y={rest_lens_y}, panned_y={panned_lens_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
