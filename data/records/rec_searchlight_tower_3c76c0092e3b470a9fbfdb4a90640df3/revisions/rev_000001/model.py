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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    concrete = model.material("concrete", rgba=(0.67, 0.68, 0.70, 1.0))
    mast_gray = model.material("mast_gray", rgba=(0.46, 0.49, 0.52, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.31, 0.34, 0.37, 1.0))
    painted_white = model.material("painted_white", rgba=(0.86, 0.87, 0.84, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.13, 0.15, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.80, 0.89, 0.95, 0.45))

    tower = model.part("tower")
    tower.visual(
        Box((1.80, 1.80, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=concrete,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=0.34, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=machinery_gray,
        name="mast_pedestal",
    )
    tower.visual(
        Box((0.82, 0.82, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.98)),
        material=machinery_gray,
        name="service_platform",
    )
    tower.visual(
        Box((0.52, 0.52, 1.20)),
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        material=mast_gray,
        name="mast_cladding",
    )
    tower.visual(
        Cylinder(radius=0.22, length=1.90),
        origin=Origin(xyz=(0.0, 0.0, 1.27)),
        material=painted_white,
        name="mast_column",
    )
    tower.visual(
        Box((0.62, 0.62, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, 2.18)),
        material=machinery_gray,
        name="top_support_block",
    )
    tower.visual(
        Cylinder(radius=0.34, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 2.19)),
        material=charcoal,
        name="top_bearing_housing",
    )
    tower.inertial = Inertial.from_geometry(
        Box((1.80, 1.80, 2.34)),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.17)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.31, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=charcoal,
        name="turntable_drum",
    )
    yoke.visual(
        Cylinder(radius=0.44, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=machinery_gray,
        name="rotary_stage",
    )
    yoke.visual(
        Box((0.94, 0.72, 0.20)),
        origin=Origin(xyz=(0.02, 0.0, 0.48)),
        material=machinery_gray,
        name="yoke_saddle",
    )
    yoke.visual(
        Box((0.42, 0.20, 0.48)),
        origin=Origin(xyz=(0.03, 0.40, 0.65)),
        material=machinery_gray,
        name="left_arm_base",
    )
    yoke.visual(
        Box((0.42, 0.20, 0.48)),
        origin=Origin(xyz=(0.03, -0.40, 0.65)),
        material=machinery_gray,
        name="right_arm_base",
    )
    yoke.visual(
        Box((0.28, 0.18, 0.70)),
        origin=Origin(xyz=(0.17, 0.43, 1.01)),
        material=painted_white,
        name="left_arm_upright",
    )
    yoke.visual(
        Box((0.28, 0.18, 0.70)),
        origin=Origin(xyz=(0.17, -0.43, 1.01)),
        material=painted_white,
        name="right_arm_upright",
    )
    yoke.visual(
        Box((0.30, 0.28, 0.26)),
        origin=Origin(xyz=(-0.18, 0.31, 0.55)),
        material=charcoal,
        name="left_pan_drive_housing",
    )
    yoke.visual(
        Box((0.30, 0.28, 0.26)),
        origin=Origin(xyz=(-0.18, -0.31, 0.55)),
        material=charcoal,
        name="right_pan_drive_housing",
    )
    yoke.visual(
        Box((0.28, 0.86, 0.16)),
        origin=Origin(xyz=(-0.16, 0.0, 0.56)),
        material=charcoal,
        name="rear_tie_block",
    )
    yoke.visual(
        Cylinder(radius=0.11, length=0.10),
        origin=Origin(xyz=(0.18, 0.39, 0.98), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_tilt_bearing",
    )
    yoke.visual(
        Cylinder(radius=0.11, length=0.10),
        origin=Origin(xyz=(0.18, -0.39, 0.98), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_tilt_bearing",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.94, 0.98, 1.36)),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.27, length=0.76),
        origin=Origin(xyz=(0.26, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_white,
        name="lamp_shell",
    )
    lamp.visual(
        Cylinder(radius=0.31, length=0.08),
        origin=Origin(xyz=(0.68, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.28, length=0.02),
        origin=Origin(xyz=(0.73, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Sphere(radius=0.20),
        origin=Origin(xyz=(-0.10, 0.0, 0.0)),
        material=machinery_gray,
        name="rear_cap",
    )
    lamp.visual(
        Box((0.22, 0.40, 0.30)),
        origin=Origin(xyz=(-0.15, 0.0, 0.0)),
        material=charcoal,
        name="rear_motor_box",
    )
    lamp.visual(
        Box((0.26, 0.22, 0.12)),
        origin=Origin(xyz=(0.00, 0.0, 0.24)),
        material=machinery_gray,
        name="top_access_cover",
    )
    lamp.visual(
        Cylinder(radius=0.15, length=0.06),
        origin=Origin(xyz=(0.0, 0.23, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="left_side_hub",
    )
    lamp.visual(
        Cylinder(radius=0.15, length=0.06),
        origin=Origin(xyz=(0.0, -0.23, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="right_side_hub",
    )
    lamp.visual(
        Cylinder(radius=0.048, length=0.08),
        origin=Origin(xyz=(0.0, 0.30, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.048, length=0.08),
        origin=Origin(xyz=(0.0, -0.30, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_trunnion",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.96, 0.62, 0.62)),
        mass=260.0,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 2.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2800.0,
            velocity=0.9,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.18, 0.0, 0.98)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1900.0,
            velocity=0.8,
            lower=math.radians(-20.0),
            upper=math.radians(65.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("tower_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_lamp_tilt")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.expect_gap(
        yoke,
        tower,
        axis="z",
        positive_elem="turntable_drum",
        negative_elem="top_bearing_housing",
        max_gap=0.005,
        max_penetration=0.0,
        name="turntable sits on the mast bearing",
    )
    ctx.expect_gap(
        yoke,
        lamp,
        axis="y",
        positive_elem="left_tilt_bearing",
        negative_elem="left_trunnion",
        max_gap=0.005,
        max_penetration=0.0,
        name="left trunnion reaches the left tilt bearing",
    )
    ctx.expect_gap(
        lamp,
        yoke,
        axis="y",
        positive_elem="right_trunnion",
        negative_elem="right_tilt_bearing",
        max_gap=0.005,
        max_penetration=0.0,
        name="right trunnion reaches the right tilt bearing",
    )

    rest_front = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    with ctx.pose({tilt: math.radians(55.0)}):
        tilted_front = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "positive tilt raises the searchlight beam",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.25,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    with ctx.pose({pan: math.pi / 2.0}):
        panned_front = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "positive pan sweeps the beam toward +y",
        rest_front is not None
        and panned_front is not None
        and panned_front[1] > rest_front[1] + 0.45,
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
