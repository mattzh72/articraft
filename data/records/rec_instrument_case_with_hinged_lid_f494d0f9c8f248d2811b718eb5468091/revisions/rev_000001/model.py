from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="flute_case")

    shell_black = model.material("shell_black", rgba=(0.12, 0.12, 0.13, 1.0))
    hardware = model.material("hardware", rgba=(0.65, 0.66, 0.68, 1.0))
    grip_dark = model.material("grip_dark", rgba=(0.20, 0.20, 0.21, 1.0))

    tray_length = 0.475
    tray_width = 0.090
    tray_height = 0.027
    lid_length = 0.488
    lid_width = 0.105
    lid_height = 0.046
    wall_t = 0.003
    panel_t = 0.003
    hinge_r = 0.0045
    pivot_r = 0.0048

    tray = model.part("lower_tray")
    tray.visual(
        Box((tray_length, tray_width, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_t / 2.0)),
        material=shell_black,
        name="tray_bottom",
    )
    tray.visual(
        Box((tray_length, wall_t, tray_height)),
        origin=Origin(xyz=(0.0, -(tray_width / 2.0) + (wall_t / 2.0), tray_height / 2.0)),
        material=shell_black,
        name="tray_back_wall",
    )
    tray.visual(
        Box((tray_length, wall_t, tray_height)),
        origin=Origin(xyz=(0.0, (tray_width / 2.0) - (wall_t / 2.0), tray_height / 2.0)),
        material=shell_black,
        name="tray_front_wall",
    )
    tray.visual(
        Box((wall_t, tray_width, tray_height)),
        origin=Origin(xyz=(-(tray_length / 2.0) + (wall_t / 2.0), 0.0, tray_height / 2.0)),
        material=shell_black,
        name="tray_left_wall",
    )
    tray.visual(
        Box((wall_t, tray_width, tray_height)),
        origin=Origin(xyz=((tray_length / 2.0) - (wall_t / 2.0), 0.0, tray_height / 2.0)),
        material=shell_black,
        name="tray_right_wall",
    )
    tray.visual(
        Cylinder(radius=hinge_r, length=0.095),
        origin=Origin(
            xyz=(-0.132, -(tray_width / 2.0), tray_height),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=hardware,
        name="tray_hinge_left",
    )
    tray.visual(
        Cylinder(radius=hinge_r, length=0.095),
        origin=Origin(
            xyz=(0.132, -(tray_width / 2.0), tray_height),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=hardware,
        name="tray_hinge_right",
    )
    tray.inertial = Inertial.from_geometry(
        Box((tray_length, tray_width, tray_height)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, tray_height / 2.0)),
    )

    lid = model.part("top_lid")
    lid.visual(
        Box((lid_length, lid_width, panel_t)),
        origin=Origin(xyz=(0.0, lid_width / 2.0, lid_height - (panel_t / 2.0))),
        material=shell_black,
        name="lid_top_panel",
    )
    lid.visual(
        Box((lid_length, wall_t, lid_height)),
        origin=Origin(xyz=(0.0, wall_t / 2.0, lid_height / 2.0)),
        material=shell_black,
        name="lid_back_wall",
    )
    lid.visual(
        Box((lid_length, wall_t, lid_height)),
        origin=Origin(xyz=(0.0, lid_width - (wall_t / 2.0), lid_height / 2.0)),
        material=shell_black,
        name="lid_front_wall",
    )
    lid.visual(
        Box((wall_t, lid_width, lid_height)),
        origin=Origin(xyz=(-(lid_length / 2.0) + (wall_t / 2.0), lid_width / 2.0, lid_height / 2.0)),
        material=shell_black,
        name="lid_left_wall",
    )
    lid.visual(
        Box((wall_t, lid_width, lid_height)),
        origin=Origin(xyz=((lid_length / 2.0) - (wall_t / 2.0), lid_width / 2.0, lid_height / 2.0)),
        material=shell_black,
        name="lid_right_wall",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=0.170),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=hardware,
        name="lid_hinge_center",
    )
    lid.visual(
        Cylinder(radius=pivot_r, length=0.012),
        origin=Origin(
            xyz=(-0.080, lid_width, 0.018),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=hardware,
        name="lid_pivot_left",
    )
    lid.visual(
        Cylinder(radius=pivot_r, length=0.012),
        origin=Origin(
            xyz=(0.080, lid_width, 0.018),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=hardware,
        name="lid_pivot_right",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, lid_height)),
        mass=0.9,
        origin=Origin(xyz=(0.0, lid_width / 2.0, lid_height / 2.0)),
    )

    handle = model.part("front_handle")
    handle.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(
            xyz=(-0.091, 0.0, 0.0),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=hardware,
        name="handle_washer_left",
    )
    handle.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(
            xyz=(0.091, 0.0, 0.0),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=hardware,
        name="handle_washer_right",
    )
    handle.visual(
        Box((0.020, 0.008, 0.030)),
        origin=Origin(xyz=(-0.078, 0.006, -0.015)),
        material=hardware,
        name="handle_left_strap",
    )
    handle.visual(
        Box((0.020, 0.008, 0.030)),
        origin=Origin(xyz=(0.078, 0.006, -0.015)),
        material=hardware,
        name="handle_right_strap",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.158),
        origin=Origin(
            xyz=(0.0, 0.013, -0.030),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=grip_dark,
        name="handle_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.185, 0.028, 0.040)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.009, -0.016)),
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lid,
        origin=Origin(xyz=(0.0, -(tray_width / 2.0), tray_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=2.15,
        ),
    )
    model.articulation(
        "handle_pivots",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, lid_width, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=2.0,
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

    tray = object_model.get_part("lower_tray")
    lid = object_model.get_part("top_lid")
    handle = object_model.get_part("front_handle")
    rear_hinge = object_model.get_articulation("rear_hinge")
    handle_pivots = object_model.get_articulation("handle_pivots")

    with ctx.pose({rear_hinge: 0.0, handle_pivots: 0.0}):
        ctx.expect_gap(
            lid,
            tray,
            axis="z",
            positive_elem="lid_front_wall",
            negative_elem="tray_front_wall",
            max_gap=0.0005,
            max_penetration=0.0,
            name="closed lid seats on the tray rim",
        )
        ctx.expect_overlap(
            lid,
            tray,
            axes="x",
            min_overlap=0.45,
            name="lid spans the tray length when closed",
        )
        ctx.expect_overlap(
            lid,
            tray,
            axes="y",
            min_overlap=0.09,
            name="lid covers the narrow tray width when closed",
        )
        ctx.expect_contact(
            handle,
            lid,
            elem_a="handle_washer_left",
            elem_b="lid_pivot_left",
            name="left handle pivot stays mounted to the lid boss",
        )
        ctx.expect_contact(
            handle,
            lid,
            elem_a="handle_washer_right",
            elem_b="lid_pivot_right",
            name="right handle pivot stays mounted to the lid boss",
        )

    closed_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_wall")
    with ctx.pose({rear_hinge: 1.45, handle_pivots: 0.0}):
        opened_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_wall")
    ctx.check(
        "rear hinge lifts the lid front upward",
        closed_lid_front is not None
        and opened_lid_front is not None
        and opened_lid_front[1][2] > closed_lid_front[1][2] + 0.05,
        details=f"closed={closed_lid_front}, opened={opened_lid_front}",
    )

    closed_handle_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({rear_hinge: 0.0, handle_pivots: 1.55}):
        raised_handle_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    ctx.check(
        "handle pivots lift the grip above its folded-flat pose",
        closed_handle_grip is not None
        and raised_handle_grip is not None
        and raised_handle_grip[1][2] > closed_handle_grip[1][2] + 0.018,
        details=f"closed={closed_handle_grip}, raised={raised_handle_grip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
