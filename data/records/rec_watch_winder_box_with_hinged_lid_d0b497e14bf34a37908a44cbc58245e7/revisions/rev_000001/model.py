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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("walnut", rgba=(0.33, 0.22, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.18, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.61, 0.34, 1.0))
    suede = model.material("suede", rgba=(0.70, 0.63, 0.52, 1.0))
    charcoal_fabric = model.material("charcoal_fabric", rgba=(0.26, 0.27, 0.30, 1.0))

    outer_w = 0.240
    outer_d = 0.180
    body_h = 0.150
    wall_t = 0.018
    floor_t = 0.018

    inner_w = outer_w - (2.0 * wall_t)
    inner_d = outer_d - (2.0 * wall_t)
    wall_h = body_h - floor_t

    main_shell = model.part("main_shell")
    main_shell.visual(
        Box((outer_w, outer_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=walnut,
        name="base_panel",
    )
    main_shell.visual(
        Box((wall_t, outer_d, wall_h)),
        origin=Origin(
            xyz=(-(outer_w / 2.0) + (wall_t / 2.0), 0.0, floor_t + (wall_h / 2.0))
        ),
        material=walnut,
        name="left_wall",
    )
    main_shell.visual(
        Box((wall_t, outer_d, wall_h)),
        origin=Origin(
            xyz=((outer_w / 2.0) - (wall_t / 2.0), 0.0, floor_t + (wall_h / 2.0))
        ),
        material=walnut,
        name="right_wall",
    )
    main_shell.visual(
        Box((inner_w, wall_t, wall_h)),
        origin=Origin(
            xyz=(0.0, -(outer_d / 2.0) + (wall_t / 2.0), floor_t + (wall_h / 2.0))
        ),
        material=walnut,
        name="rear_wall",
    )
    main_shell.visual(
        Box((inner_w, wall_t, wall_h)),
        origin=Origin(
            xyz=(0.0, (outer_d / 2.0) - (wall_t / 2.0), floor_t + (wall_h / 2.0))
        ),
        material=walnut,
        name="front_wall",
    )
    main_shell.visual(
        Box((inner_w - 0.018, inner_d - 0.022, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, floor_t + 0.003)),
        material=suede,
        name="base_liner",
    )
    main_shell.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    reinforcement_band = model.part("reinforcement_band")
    reinforcement_band.visual(
        Box((outer_w, wall_t, 0.070)),
        origin=Origin(xyz=(0.0, -(outer_d / 2.0) - (wall_t / 2.0), 0.115)),
        material=satin_black,
        name="rear_band",
    )
    reinforcement_band.visual(
        Box((wall_t, 0.030, 0.070)),
        origin=Origin(
            xyz=(-(outer_w / 2.0) - (wall_t / 2.0), -(outer_d / 2.0) + 0.015, 0.115)
        ),
        material=satin_black,
        name="left_return",
    )
    reinforcement_band.visual(
        Box((wall_t, 0.030, 0.070)),
        origin=Origin(
            xyz=((outer_w / 2.0) + (wall_t / 2.0), -(outer_d / 2.0) + 0.015, 0.115)
        ),
        material=satin_black,
        name="right_return",
    )
    for index, x_pos in enumerate((-0.090, 0.0, 0.090)):
        reinforcement_band.visual(
            Cylinder(radius=0.007, length=0.026),
            origin=Origin(
                xyz=(x_pos, -(outer_d / 2.0) - 0.025, body_h),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"hinge_knuckle_{index}",
        )
    reinforcement_band.inertial = Inertial.from_geometry(
        Box((outer_w, 0.036, 0.070)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -(outer_d / 2.0), 0.115)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((outer_w - 0.004, outer_d - 0.004, 0.008)),
        origin=Origin(xyz=(0.0, (outer_d - 0.004) / 2.0, 0.028)),
        material=walnut,
        name="top_panel",
    )
    lid.visual(
        Box((outer_w - 0.004, 0.014, 0.028)),
        origin=Origin(xyz=(0.0, outer_d - 0.009, 0.014)),
        material=walnut,
        name="front_rail",
    )
    lid.visual(
        Box((0.014, outer_d - 0.018, 0.028)),
        origin=Origin(xyz=(-(outer_w / 2.0) + 0.007, (outer_d - 0.018) / 2.0, 0.014)),
        material=walnut,
        name="left_rail",
    )
    lid.visual(
        Box((0.014, outer_d - 0.018, 0.028)),
        origin=Origin(xyz=((outer_w / 2.0) - 0.007, (outer_d - 0.018) / 2.0, 0.014)),
        material=walnut,
        name="right_rail",
    )
    for index, x_pos in enumerate((-0.040, 0.040)):
        lid.visual(
            Box((0.050, 0.010, 0.028)),
            origin=Origin(xyz=(x_pos, 0.005, 0.014)),
            material=walnut,
            name=f"hinge_leaf_{index}",
        )
    lid.visual(
        Box((inner_w - 0.010, inner_d - 0.020, 0.004)),
        origin=Origin(xyz=(0.0, (outer_d - 0.004) / 2.0, 0.022)),
        material=suede,
        name="lid_liner",
    )
    for index, x_pos in enumerate((-0.040, 0.040)):
        lid.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"lid_knuckle_{index}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, 0.036)),
        mass=0.9,
        origin=Origin(xyz=(0.0, outer_d / 2.0, 0.018)),
    )

    spindle_mount = model.part("spindle_mount")
    spindle_mount.visual(
        Box((0.070, 0.028, 0.050)),
        origin=Origin(xyz=(0.0, -(outer_d / 2.0) + wall_t + 0.014, 0.078)),
        material=satin_black,
        name="motor_housing",
    )
    spindle_mount.visual(
        Box((0.040, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.035, 0.078)),
        material=satin_black,
        name="support_pedestal",
    )
    spindle_mount.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.0, -0.020, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="spindle",
    )
    spindle_mount.inertial = Inertial.from_geometry(
        Box((0.070, 0.050, 0.050)),
        mass=0.28,
        origin=Origin(xyz=(0.0, -0.049, 0.078)),
    )

    cushion_geometry = ExtrudeGeometry.centered(
        rounded_rect_profile(0.070, 0.090, 0.016),
        0.055,
    )
    cushion_geometry.rotate_x(math.pi / 2.0)
    cushion_mesh = mesh_from_geometry(cushion_geometry, "cradle_cushion")

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hub_cap",
    )
    cradle.visual(
        Box((0.050, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=satin_black,
        name="rear_plate",
    )
    cradle.visual(
        Box((0.012, 0.035, 0.050)),
        origin=Origin(xyz=(-0.024, 0.030, 0.0)),
        material=satin_black,
        name="left_arm",
    )
    cradle.visual(
        Box((0.012, 0.035, 0.050)),
        origin=Origin(xyz=(0.024, 0.030, 0.0)),
        material=satin_black,
        name="right_arm",
    )
    cradle.visual(
        cushion_mesh,
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=charcoal_fabric,
        name="cushion_shell",
    )
    cradle.visual(
        Box((0.034, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.078, 0.030)),
        material=suede,
        name="watch_stop",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.080, 0.085, 0.095)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
    )

    model.articulation(
        "shell_to_band",
        ArticulationType.FIXED,
        parent=main_shell,
        child=reinforcement_band,
        origin=Origin(),
    )
    model.articulation(
        "band_to_lid",
        ArticulationType.REVOLUTE,
        parent=reinforcement_band,
        child=lid,
        origin=Origin(xyz=(0.0, -(outer_d / 2.0) - 0.025, body_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "shell_to_spindle_mount",
        ArticulationType.FIXED,
        parent=main_shell,
        child=spindle_mount,
        origin=Origin(),
    )
    model.articulation(
        "spindle_mount_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=spindle_mount,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.020, 0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
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

    main_shell = object_model.get_part("main_shell")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    hinge = object_model.get_articulation("band_to_lid")
    cradle_spin = object_model.get_articulation("spindle_mount_to_cradle")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            main_shell,
            axis="z",
            max_gap=0.0025,
            max_penetration=0.0002,
            positive_elem="front_rail",
            name="closed lid seats on the shell",
        )
        ctx.expect_overlap(
            lid,
            main_shell,
            axes="xy",
            min_overlap=0.12,
            name="closed lid covers the shell opening",
        )
        ctx.expect_gap(
            cradle,
            main_shell,
            axis="z",
            min_gap=0.008,
            positive_elem="cushion_shell",
            negative_elem="base_liner",
            name="cradle cushion clears the floor liner",
        )
        ctx.expect_overlap(
            cradle,
            main_shell,
            axes="xy",
            min_overlap=0.050,
            name="cradle stays within the box footprint",
        )

    closed_front = None
    open_front = None
    with ctx.pose({hinge: 0.0}):
        closed_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    with ctx.pose({hinge: math.radians(80.0)}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_rail")

    ctx.check(
        "lid front edge rises when opened",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.09,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    cradle_stop_rest = None
    cradle_stop_turned = None
    with ctx.pose({cradle_spin: 0.0}):
        cradle_stop_rest = ctx.part_element_world_aabb(cradle, elem="watch_stop")
    with ctx.pose({cradle_spin: math.pi / 2.0}):
        cradle_stop_turned = ctx.part_element_world_aabb(cradle, elem="watch_stop")

    rest_center = None
    turned_center = None
    if cradle_stop_rest is not None:
        rest_center = tuple(
            (cradle_stop_rest[0][axis] + cradle_stop_rest[1][axis]) / 2.0 for axis in range(3)
        )
    if cradle_stop_turned is not None:
        turned_center = tuple(
            (cradle_stop_turned[0][axis] + cradle_stop_turned[1][axis]) / 2.0
            for axis in range(3)
        )

    ctx.check(
        "cradle visibly rotates on the spindle",
        rest_center is not None
        and turned_center is not None
        and abs(turned_center[0] - rest_center[0]) > 0.02
        and abs(turned_center[2] - rest_center[2]) > 0.02,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
