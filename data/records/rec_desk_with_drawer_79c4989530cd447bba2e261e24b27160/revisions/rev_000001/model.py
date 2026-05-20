from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_desk_with_cable_tray_and_pedestal_drawer")

    desktop_finish = model.material("desktop_finish", rgba=(0.23, 0.16, 0.11, 1.0))
    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    graphite = model.material("graphite", rgba=(0.26, 0.27, 0.29, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.18, 0.19, 0.20, 1.0))

    desktop = model.part("desktop")
    desktop.visual(
        Box((1.60, 0.75, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=desktop_finish,
        name="desktop_surface",
    )
    desktop.visual(
        Box((0.90, 0.06, 0.05)),
        origin=Origin(xyz=(-0.125, 0.320, 0.695)),
        material=powder_black,
        name="rear_support_beam",
    )
    desktop.visual(
        Box((0.022, 0.24, 0.014)),
        origin=Origin(xyz=(-0.485, -0.140, 0.713)),
        material=graphite,
        name="tray_rail_left",
    )
    desktop.visual(
        Box((0.022, 0.24, 0.014)),
        origin=Origin(xyz=(0.225, -0.140, 0.713)),
        material=graphite,
        name="tray_rail_right",
    )
    desktop.inertial = Inertial.from_geometry(
        Box((1.60, 0.75, 0.10)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
    )

    left_frame = model.part("left_frame")
    left_frame.visual(
        Box((0.05, 0.62, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=powder_black,
        name="base_skid",
    )
    left_frame.visual(
        Box((0.05, 0.05, 0.65)),
        origin=Origin(xyz=(0.0, -0.23, 0.365)),
        material=powder_black,
        name="front_upright",
    )
    left_frame.visual(
        Box((0.05, 0.05, 0.65)),
        origin=Origin(xyz=(0.0, 0.23, 0.365)),
        material=powder_black,
        name="rear_upright",
    )
    left_frame.visual(
        Box((0.08, 0.56, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=powder_black,
        name="top_mount_rail",
    )
    left_frame.inertial = Inertial.from_geometry(
        Box((0.08, 0.62, 0.72)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )
    model.articulation(
        "desktop_to_left_frame",
        ArticulationType.FIXED,
        parent=desktop,
        child=left_frame,
        origin=Origin(xyz=(-0.63, 0.0, 0.0)),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.018, 0.60, 0.72)),
        origin=Origin(xyz=(-0.191, 0.0, 0.0)),
        material=graphite,
        name="pedestal_left_side",
    )
    pedestal.visual(
        Box((0.018, 0.60, 0.72)),
        origin=Origin(xyz=(0.191, 0.0, 0.0)),
        material=graphite,
        name="pedestal_right_side",
    )
    pedestal.visual(
        Box((0.364, 0.018, 0.72)),
        origin=Origin(xyz=(0.0, 0.291, 0.0)),
        material=graphite,
        name="pedestal_back",
    )
    pedestal.visual(
        Box((0.364, 0.582, 0.018)),
        origin=Origin(xyz=(0.0, -0.009, -0.351)),
        material=graphite,
        name="pedestal_bottom",
    )
    pedestal.visual(
        Box((0.364, 0.582, 0.018)),
        origin=Origin(xyz=(0.0, -0.009, 0.351)),
        material=graphite,
        name="pedestal_top",
    )
    pedestal.visual(
        Box((0.364, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.291, 0.147)),
        material=graphite,
        name="drawer_opening_sill",
    )
    pedestal.visual(
        Box((0.364, 0.018, 0.410)),
        origin=Origin(xyz=(0.0, -0.291, -0.077)),
        material=graphite,
        name="lower_front_panel",
    )
    pedestal.visual(
        Box((0.008, 0.46, 0.035)),
        origin=Origin(xyz=(-0.178, 0.010, 0.225)),
        material=powder_black,
        name="drawer_rail_left",
    )
    pedestal.visual(
        Box((0.008, 0.46, 0.035)),
        origin=Origin(xyz=(0.178, 0.010, 0.225)),
        material=powder_black,
        name="drawer_rail_right",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.40, 0.60, 0.72)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "desktop_to_pedestal",
        ArticulationType.FIXED,
        parent=desktop,
        child=pedestal,
        origin=Origin(xyz=(0.55, 0.0, 0.360)),
    )

    cable_tray = model.part("cable_tray")
    cable_tray.visual(
        Box((0.700, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=graphite,
        name="tray_bottom",
    )
    cable_tray.visual(
        Box((0.012, 0.180, 0.070)),
        origin=Origin(xyz=(-0.344, 0.0, 0.0)),
        material=graphite,
        name="tray_left_wall",
    )
    cable_tray.visual(
        Box((0.012, 0.180, 0.070)),
        origin=Origin(xyz=(0.344, 0.0, 0.0)),
        material=graphite,
        name="tray_right_wall",
    )
    cable_tray.visual(
        Box((0.700, 0.012, 0.048)),
        origin=Origin(xyz=(0.0, -0.084, -0.005)),
        material=graphite,
        name="tray_front_lip",
    )
    cable_tray.visual(
        Box((0.700, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.084, -0.010)),
        material=graphite,
        name="tray_rear_lip",
    )
    cable_tray.visual(
        Box((0.018, 0.220, 0.018)),
        origin=Origin(xyz=(-0.335, 0.0, 0.033)),
        material=powder_black,
        name="tray_runner_left",
    )
    cable_tray.visual(
        Box((0.018, 0.220, 0.018)),
        origin=Origin(xyz=(0.335, 0.0, 0.033)),
        material=powder_black,
        name="tray_runner_right",
    )
    cable_tray.inertial = Inertial.from_geometry(
        Box((0.70, 0.22, 0.08)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "desktop_to_cable_tray",
        ArticulationType.PRISMATIC,
        parent=desktop,
        child=cable_tray,
        origin=Origin(xyz=(-0.130, -0.140, 0.664)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.30,
            lower=0.0,
            upper=0.18,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.340, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, -0.250, 0.0)),
        material=drawer_face,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.012, 0.476, 0.090)),
        origin=Origin(xyz=(-0.164, -0.004, 0.0)),
        material=drawer_face,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((0.012, 0.476, 0.090)),
        origin=Origin(xyz=(0.164, -0.004, 0.0)),
        material=drawer_face,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((0.316, 0.476, 0.012)),
        origin=Origin(xyz=(0.0, -0.004, -0.039)),
        material=drawer_face,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.316, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, 0.234, 0.0)),
        material=drawer_face,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.004, 0.440, 0.028)),
        origin=Origin(xyz=(-0.172, 0.0, 0.0)),
        material=powder_black,
        name="drawer_runner_left",
    )
    drawer.visual(
        Box((0.004, 0.440, 0.028)),
        origin=Origin(xyz=(0.172, 0.0, 0.0)),
        material=powder_black,
        name="drawer_runner_right",
    )
    drawer.visual(
        Box((0.120, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.268, 0.0)),
        material=powder_black,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.34, 0.50, 0.12)),
        mass=4.0,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
    )
    model.articulation(
        "pedestal_to_drawer",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.041, 0.225)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.35,
            lower=0.0,
            upper=0.26,
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

    desktop = object_model.get_part("desktop")
    pedestal = object_model.get_part("pedestal")
    cable_tray = object_model.get_part("cable_tray")
    drawer = object_model.get_part("drawer")

    tray_slide = object_model.get_articulation("desktop_to_cable_tray")
    drawer_slide = object_model.get_articulation("pedestal_to_drawer")

    tray_limits = tray_slide.motion_limits
    drawer_limits = drawer_slide.motion_limits

    ctx.check(
        "cable tray slide uses forward prismatic travel",
        tray_slide.articulation_type == ArticulationType.PRISMATIC
        and tray_slide.axis == (0.0, -1.0, 0.0)
        and tray_limits is not None
        and tray_limits.lower == 0.0
        and tray_limits.upper is not None
        and tray_limits.upper >= 0.18,
        details=f"type={tray_slide.articulation_type}, axis={tray_slide.axis}, limits={tray_limits}",
    )
    ctx.check(
        "pedestal drawer slide uses forward prismatic travel",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC
        and drawer_slide.axis == (0.0, -1.0, 0.0)
        and drawer_limits is not None
        and drawer_limits.lower == 0.0
        and drawer_limits.upper is not None
        and drawer_limits.upper >= 0.25,
        details=f"type={drawer_slide.articulation_type}, axis={drawer_slide.axis}, limits={drawer_limits}",
    )

    rest_tray_pos = None
    extended_tray_pos = None
    rest_drawer_pos = None
    extended_drawer_pos = None

    with ctx.pose({tray_slide: 0.0, drawer_slide: 0.0}):
        ctx.expect_contact(
            cable_tray,
            desktop,
            elem_a="tray_runner_left",
            elem_b="tray_rail_left",
            name="left cable tray runner sits on the left desk rail",
        )
        ctx.expect_contact(
            cable_tray,
            desktop,
            elem_a="tray_runner_right",
            elem_b="tray_rail_right",
            name="right cable tray runner sits on the right desk rail",
        )
        ctx.expect_gap(
            desktop,
            cable_tray,
            axis="z",
            positive_elem="desktop_surface",
            negative_elem="tray_left_wall",
            min_gap=0.003,
            max_gap=0.030,
            name="cable tray sits just below desktop",
        )
        ctx.expect_overlap(
            cable_tray,
            desktop,
            axes="x",
            min_overlap=0.68,
            elem_a="tray_bottom",
            elem_b="desktop_surface",
            name="cable tray spans a broad section under the desktop",
        )
        ctx.expect_within(
            drawer,
            pedestal,
            axes="xz",
            margin=0.0,
            name="drawer stays centered inside the pedestal opening",
        )
        ctx.expect_overlap(
            drawer,
            pedestal,
            axes="y",
            min_overlap=0.45,
            name="closed drawer remains deeply inserted in the pedestal",
        )
        rest_tray_pos = ctx.part_world_position(cable_tray)
        rest_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({tray_slide: 0.18, drawer_slide: 0.26}):
        ctx.expect_contact(
            cable_tray,
            desktop,
            elem_a="tray_runner_left",
            elem_b="tray_rail_left",
            name="left cable tray runner stays engaged when extended",
        )
        ctx.expect_contact(
            cable_tray,
            desktop,
            elem_a="tray_runner_right",
            elem_b="tray_rail_right",
            name="right cable tray runner stays engaged when extended",
        )
        ctx.expect_gap(
            desktop,
            cable_tray,
            axis="z",
            positive_elem="desktop_surface",
            negative_elem="tray_left_wall",
            min_gap=0.003,
            max_gap=0.030,
            name="extended cable tray stays tucked below the desktop",
        )
        ctx.expect_within(
            drawer,
            pedestal,
            axes="xz",
            margin=0.0,
            name="open drawer stays aligned on the pedestal rails",
        )
        ctx.expect_overlap(
            drawer,
            pedestal,
            axes="y",
            min_overlap=0.20,
            name="open drawer retains insertion on its rails",
        )
        extended_tray_pos = ctx.part_world_position(cable_tray)
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "cable tray pulls out toward the seated user",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] < rest_tray_pos[1] - 0.15,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )
    ctx.check(
        "drawer opens forward from the pedestal",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.20,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
