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
    model = ArticulatedObject(name="mobile_over_bed_desk")

    powder_coat = model.material("powder_coat", rgba=(0.22, 0.23, 0.25, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    laminate = model.material("laminate", rgba=(0.74, 0.63, 0.48, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.84, 0.85, 0.87, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((0.055, 0.50, 0.05)),
        origin=Origin(xyz=(-0.300, 0.000, 0.055)),
        material=powder_coat,
        name="outer_base_rail",
    )
    base.visual(
        Box((0.52, 0.055, 0.05)),
        origin=Origin(xyz=(-0.040, 0.220, 0.055)),
        material=powder_coat,
        name="front_base_leg",
    )
    base.visual(
        Box((0.52, 0.055, 0.05)),
        origin=Origin(xyz=(-0.040, -0.220, 0.055)),
        material=powder_coat,
        name="rear_base_leg",
    )
    base.visual(
        Box((0.060, 0.060, 0.70)),
        origin=Origin(xyz=(-0.300, -0.220, 0.430)),
        material=powder_coat,
        name="rear_post",
    )
    base.visual(
        Box((0.64, 0.050, 0.050)),
        origin=Origin(xyz=(0.020, -0.220, 0.760)),
        material=powder_coat,
        name="upper_support_arm",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.120),
        origin=Origin(
            xyz=(0.020, -0.220, 0.802),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=charcoal,
        name="hinge_knuckle_center",
    )
    base.visual(
        Box((0.140, 0.050, 0.010)),
        origin=Origin(xyz=(0.020, -0.220, 0.785)),
        material=charcoal,
        name="hinge_mount_center",
    )
    base.visual(
        Box((0.050, 0.050, 0.320)),
        origin=Origin(
            xyz=(-0.205, -0.220, 0.645),
            rpy=(0.0, math.pi / 4.0, 0.0),
        ),
        material=charcoal,
        name="rear_diagonal_brace",
    )

    caster_positions = (
        (-0.300, -0.220),
        (-0.300, 0.220),
        (0.220, -0.220),
        (0.220, 0.220),
    )
    for index, (x_pos, y_pos) in enumerate(caster_positions):
        base.visual(
            Cylinder(radius=0.028, length=0.020),
            origin=Origin(
                xyz=(x_pos, y_pos, 0.028),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=wheel_gray,
            name=f"caster_wheel_{index}",
        )
        base.visual(
            Box((0.038, 0.024, 0.024)),
            origin=Origin(xyz=(x_pos, y_pos, 0.049)),
            material=charcoal,
            name=f"caster_mount_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.70, 0.55, 0.82)),
        mass=12.0,
        origin=Origin(xyz=(-0.020, 0.000, 0.410)),
    )

    desktop = model.part("desktop")
    desktop.visual(
        Box((0.82, 0.42, 0.024)),
        origin=Origin(xyz=(0.000, 0.224, 0.020)),
        material=laminate,
        name="top_panel",
    )
    desktop.visual(
        Box((0.030, 0.36, 0.022)),
        origin=Origin(xyz=(-0.395, 0.220, -0.001)),
        material=powder_coat,
        name="left_apron",
    )
    desktop.visual(
        Box((0.030, 0.080, 0.022)),
        origin=Origin(xyz=(0.395, 0.045, -0.001)),
        material=powder_coat,
        name="right_rear_apron",
    )
    desktop.visual(
        Box((0.030, 0.065, 0.022)),
        origin=Origin(xyz=(0.395, 0.3875, -0.001)),
        material=powder_coat,
        name="right_front_apron",
    )
    desktop.visual(
        Box((0.74, 0.030, 0.022)),
        origin=Origin(xyz=(0.000, 0.395, -0.001)),
        material=powder_coat,
        name="front_stretcher",
    )
    desktop.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(
            xyz=(-0.110, 0.000, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=charcoal,
        name="hinge_knuckle_left",
    )
    desktop.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(
            xyz=(0.110, 0.000, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=charcoal,
        name="hinge_knuckle_right",
    )
    desktop.visual(
        Box((0.080, 0.020, 0.012)),
        origin=Origin(xyz=(-0.110, 0.018, 0.010)),
        material=charcoal,
        name="hinge_leaf_left",
    )
    desktop.visual(
        Box((0.080, 0.020, 0.012)),
        origin=Origin(xyz=(0.110, 0.018, 0.010)),
        material=charcoal,
        name="hinge_leaf_right",
    )
    desktop.visual(
        Box((0.236, 0.010, 0.012)),
        origin=Origin(xyz=(0.270, 0.086, 0.005)),
        material=charcoal,
        name="drawer_rail_back",
    )
    desktop.visual(
        Box((0.236, 0.010, 0.012)),
        origin=Origin(xyz=(0.270, 0.334, 0.005)),
        material=charcoal,
        name="drawer_rail_front",
    )
    desktop.inertial = Inertial.from_geometry(
        Box((0.82, 0.42, 0.08)),
        mass=5.4,
        origin=Origin(xyz=(0.000, 0.224, 0.000)),
    )

    model.articulation(
        "base_to_desktop",
        ArticulationType.REVOLUTE,
        parent=base,
        child=desktop,
        origin=Origin(xyz=(0.020, -0.220, 0.802)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=0.90,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.018, 0.248, 0.078)),
        origin=Origin(xyz=(-0.009, 0.000, -0.039)),
        material=drawer_gray,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.236, 0.236, 0.006)),
        origin=Origin(xyz=(-0.121, 0.000, -0.075)),
        material=drawer_gray,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.230, 0.006, 0.060)),
        origin=Origin(xyz=(-0.121, -0.115, -0.048)),
        material=drawer_gray,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((0.230, 0.006, 0.060)),
        origin=Origin(xyz=(-0.121, 0.115, -0.048)),
        material=drawer_gray,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((0.006, 0.236, 0.060)),
        origin=Origin(xyz=(-0.236, 0.000, -0.048)),
        material=drawer_gray,
        name="drawer_back_wall",
    )
    drawer.visual(
        Box((0.220, 0.010, 0.020)),
        origin=Origin(xyz=(-0.122, -0.121, -0.011)),
        material=drawer_gray,
        name="drawer_back_runner",
    )
    drawer.visual(
        Box((0.220, 0.010, 0.020)),
        origin=Origin(xyz=(-0.122, 0.121, -0.011)),
        material=drawer_gray,
        name="drawer_front_runner",
    )
    drawer.visual(
        Box((0.014, 0.090, 0.018)),
        origin=Origin(xyz=(0.007, 0.000, -0.039)),
        material=charcoal,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.25, 0.25, 0.09)),
        mass=1.1,
        origin=Origin(xyz=(-0.120, 0.000, -0.045)),
    )

    model.articulation(
        "desktop_to_drawer",
        ArticulationType.PRISMATIC,
        parent=desktop,
        child=drawer,
        origin=Origin(xyz=(0.410, 0.210, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.25,
            lower=0.0,
            upper=0.155,
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

    base = object_model.get_part("base_frame")
    desktop = object_model.get_part("desktop")
    drawer = object_model.get_part("drawer")
    top_hinge = object_model.get_articulation("base_to_desktop")
    drawer_slide = object_model.get_articulation("desktop_to_drawer")

    ctx.expect_gap(
        desktop,
        base,
        axis="z",
        positive_elem="top_panel",
        negative_elem="upper_support_arm",
        min_gap=0.020,
        max_gap=0.030,
        name="desktop clears the rear support arm in the flat pose",
    )
    ctx.expect_gap(
        desktop,
        drawer,
        axis="z",
        positive_elem="top_panel",
        negative_elem="drawer_front",
        min_gap=0.006,
        max_gap=0.020,
        name="drawer tucks beneath the desktop without rubbing",
    )

    top_aabb = ctx.part_element_world_aabb(desktop, elem="top_panel")
    drawer_front_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    flush_ok = (
        top_aabb is not None
        and drawer_front_aabb is not None
        and abs(top_aabb[1][0] - drawer_front_aabb[1][0]) <= 0.0025
    )
    ctx.check(
        "drawer front closes flush with the desktop side",
        flush_ok,
        details=f"top={top_aabb}, drawer_front={drawer_front_aabb}",
    )

    front_rest = ctx.part_element_world_aabb(desktop, elem="front_stretcher")
    with ctx.pose({top_hinge: 0.78}):
        front_open = ctx.part_element_world_aabb(desktop, elem="front_stretcher")
        ctx.expect_gap(
            desktop,
            base,
            axis="z",
            positive_elem="top_panel",
            negative_elem="upper_support_arm",
            min_gap=0.025,
            name="desktop stays above the rear support arm while tilted",
        )
    tilt_ok = (
        front_rest is not None
        and front_open is not None
        and front_open[0][2] > front_rest[0][2] + 0.18
    )
    ctx.check(
        "desktop front edge rises when the hinge opens",
        tilt_ok,
        details=f"rest={front_rest}, open={front_open}",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.155}):
        drawer_open = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            desktop,
            axes="x",
            elem_a="drawer_bottom",
            elem_b="drawer_rail_back",
            min_overlap=0.055,
            name="drawer keeps retained insertion on its rails at full extension",
        )
    drawer_motion_ok = (
        drawer_rest is not None
        and drawer_open is not None
        and drawer_open[0] > drawer_rest[0] + 0.14
    )
    ctx.check(
        "drawer slides outward to the side",
        drawer_motion_ok,
        details=f"rest={drawer_rest}, open={drawer_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
