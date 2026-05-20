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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xz_section(y: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, z in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_monitor")

    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin = model.material("satin", rgba=(0.36, 0.37, 0.39, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.06, 0.09, 0.11, 0.94))

    base = model.part("base")
    foot_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.230, 0.170, 0.018), 0.014),
        "monitor_foot",
    )
    base.visual(
        foot_mesh,
        origin=Origin(xyz=(0.0, -0.010, 0.007)),
        material=graphite,
        name="foot",
    )
    base.visual(
        Box((0.070, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.034, 0.024)),
        material=graphite,
        name="foot_riser",
    )

    sleeve_y = 0.034
    sleeve_bottom = 0.034
    sleeve_height = 0.168
    sleeve_center_z = sleeve_bottom + sleeve_height / 2.0
    sleeve_outer_x = 0.034
    sleeve_outer_y = 0.030
    sleeve_inner_x = 0.022
    sleeve_inner_y = 0.018
    wall_x = (sleeve_outer_x - sleeve_inner_x) / 2.0
    wall_y = (sleeve_outer_y - sleeve_inner_y) / 2.0

    base.visual(
        Box((wall_x, sleeve_outer_y, sleeve_height)),
        origin=Origin(xyz=(-(sleeve_inner_x / 2.0 + wall_x / 2.0), sleeve_y, sleeve_center_z)),
        material=charcoal,
        name="sleeve_left_wall",
    )
    base.visual(
        Box((wall_x, sleeve_outer_y, sleeve_height)),
        origin=Origin(xyz=((sleeve_inner_x / 2.0 + wall_x / 2.0), sleeve_y, sleeve_center_z)),
        material=charcoal,
        name="sleeve_right_wall",
    )
    base.visual(
        Box((sleeve_inner_x, wall_y, sleeve_height)),
        origin=Origin(xyz=(0.0, sleeve_y - (sleeve_inner_y / 2.0 + wall_y / 2.0), sleeve_center_z)),
        material=charcoal,
        name="sleeve_front_wall",
    )
    base.visual(
        Box((sleeve_inner_x, wall_y, sleeve_height)),
        origin=Origin(xyz=(0.0, sleeve_y + (sleeve_inner_y / 2.0 + wall_y / 2.0), sleeve_center_z)),
        material=charcoal,
        name="sleeve_back_wall",
    )
    base.visual(
        Box((0.060, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, sleeve_y + 0.019, sleeve_bottom + sleeve_height - 0.001)),
        material=charcoal,
        name="sleeve_rear_cap",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(
            xyz=(0.030, sleeve_y + 0.010, sleeve_bottom + sleeve_height - 0.002),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin,
        name="height_lock_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.230, 0.190, 0.198)),
        mass=3.4,
        origin=Origin(xyz=(0.0, -0.002, 0.099)),
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.016, 0.016, 0.360)),
        origin=Origin(xyz=(0.0, 0.008, 0.030)),
        material=satin,
        name="inner_mast",
    )
    mast.visual(
        Box((0.018, 0.014, 0.052)),
        origin=Origin(xyz=(0.0, 0.017, 0.227)),
        material=charcoal,
        name="tilt_post",
    )
    mast.visual(
        Box((0.052, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, 0.255)),
        material=charcoal,
        name="tilt_bridge",
    )
    mast.visual(
        Box((0.008, 0.020, 0.050)),
        origin=Origin(xyz=(-0.018, 0.004, 0.224)),
        material=charcoal,
        name="left_tilt_cheek",
    )
    mast.visual(
        Box((0.008, 0.020, 0.050)),
        origin=Origin(xyz=(0.018, 0.004, 0.224)),
        material=charcoal,
        name="right_tilt_cheek",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.056, 0.030, 0.392)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.008, 0.098)),
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, sleeve_y - sleeve_inner_y / 2.0, sleeve_bottom + sleeve_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=0.100),
    )

    panel = model.part("panel")
    rear_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _xz_section(-0.006, 0.540, 0.322, 0.018),
                _xz_section(0.002, 0.500, 0.292, 0.022),
                _xz_section(0.010, 0.420, 0.238, 0.026),
            ]
        ),
        "monitor_rear_shell",
    )
    panel.visual(
        rear_shell_mesh,
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=charcoal,
        name="rear_shell",
    )
    panel.visual(
        Box((0.540, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.053, 0.155)),
        material=black,
        name="top_bezel",
    )
    panel.visual(
        Box((0.540, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.050, -0.152)),
        material=black,
        name="bottom_bezel",
    )
    panel.visual(
        Box((0.010, 0.010, 0.322)),
        origin=Origin(xyz=(-0.265, -0.053, 0.0)),
        material=black,
        name="left_bezel",
    )
    panel.visual(
        Box((0.010, 0.010, 0.322)),
        origin=Origin(xyz=(0.265, -0.053, 0.0)),
        material=black,
        name="right_bezel",
    )
    panel.visual(
        Box((0.520, 0.004, 0.286)),
        origin=Origin(xyz=(0.0, -0.054, 0.0)),
        material=screen_glass,
        name="screen",
    )
    panel.visual(
        Box((0.082, 0.018, 0.110)),
        origin=Origin(xyz=(0.0, -0.043, 0.0)),
        material=charcoal,
        name="mount_block",
    )
    panel.visual(
        Box((0.022, 0.038, 0.134)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=charcoal,
        name="tilt_spine",
    )
    panel.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="tilt_barrel",
    )
    panel.visual(
        Box((0.010, 0.010, 0.014)),
        origin=Origin(xyz=(0.203, -0.050, -0.168)),
        material=black,
        name="wheel_left_tab",
    )
    panel.visual(
        Box((0.010, 0.010, 0.014)),
        origin=Origin(xyz=(0.241, -0.050, -0.168)),
        material=black,
        name="wheel_right_tab",
    )
    panel.inertial = Inertial.from_geometry(
        Box((0.540, 0.060, 0.322)),
        mass=2.7,
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
    )

    model.articulation(
        "mast_to_panel",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.18,
            upper=0.34,
        ),
    )

    control_wheel = model.part("control_wheel")
    control_wheel.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="wheel_tire",
    )
    control_wheel.visual(
        Cylinder(radius=0.0025, length=0.032),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="wheel_axle",
    )
    control_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.009, length=0.032),
        mass=0.03,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "panel_to_control_wheel",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=control_wheel,
        origin=Origin(xyz=(0.222, -0.050, -0.171)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
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

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    panel = object_model.get_part("panel")
    wheel = object_model.get_part("control_wheel")
    lift = object_model.get_articulation("base_to_mast")
    tilt = object_model.get_articulation("mast_to_panel")
    wheel_joint = object_model.get_articulation("panel_to_control_wheel")

    ctx.expect_gap(
        panel,
        wheel,
        axis="z",
        positive_elem="bottom_bezel",
        negative_elem="wheel_tire",
        min_gap=0.0005,
        max_gap=0.010,
        name="control wheel hangs just under the bottom bezel",
    )

    ctx.expect_overlap(
        panel,
        mast,
        axes="z",
        elem_a="mount_block",
        elem_b="tilt_bridge",
        min_overlap=0.016,
        name="tilt bracket overlaps the mast head vertically",
    )

    rest_panel_pos = ctx.part_world_position(panel)
    with ctx.pose({lift: lift.motion_limits.upper}):
        raised_panel_pos = ctx.part_world_position(panel)
        mast_aabb = ctx.part_element_world_aabb(mast, elem="inner_mast")
        sleeve_aabb = ctx.part_element_world_aabb(base, elem="sleeve_left_wall")
        retained = (
            mast_aabb is not None
            and sleeve_aabb is not None
            and mast_aabb[0][2] < sleeve_aabb[1][2] - 0.020
        )
        ctx.check(
            "mast keeps retained insertion at full height",
            retained,
            details=f"mast_aabb={mast_aabb}, sleeve_aabb={sleeve_aabb}",
        )
        ctx.check(
            "panel rises when mast extends",
            rest_panel_pos is not None
            and raised_panel_pos is not None
            and raised_panel_pos[2] > rest_panel_pos[2] + 0.090,
            details=f"rest={rest_panel_pos}, raised={raised_panel_pos}",
        )

    rest_top_bezel_aabb = ctx.part_element_world_aabb(panel, elem="top_bezel")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_top_bezel_aabb = ctx.part_element_world_aabb(panel, elem="top_bezel")
        ctx.check(
            "panel tilts backward at the upper hinge limit",
            rest_top_bezel_aabb is not None
            and tilted_top_bezel_aabb is not None
            and tilted_top_bezel_aabb[1][1] > rest_top_bezel_aabb[1][1] + 0.030,
            details=f"rest={rest_top_bezel_aabb}, tilted={tilted_top_bezel_aabb}",
        )

    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: 1.2}):
        spun_wheel_pos = ctx.part_world_position(wheel)
        ctx.check(
            "control wheel spins in place under the frame",
            rest_wheel_pos is not None
            and spun_wheel_pos is not None
            and max(abs(a - b) for a, b in zip(rest_wheel_pos, spun_wheel_pos)) < 1e-6,
            details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
