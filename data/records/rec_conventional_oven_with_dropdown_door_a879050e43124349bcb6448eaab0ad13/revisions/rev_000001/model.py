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


BODY_W = 0.43
BODY_D = 0.31
BODY_H = 0.25
SHELL_T = 0.006

DOOR_W = 0.326
DOOR_H = 0.176
DOOR_T = 0.018
DOOR_Y = BODY_D * 0.5
HINGE_Z = 0.034

TRAY_W = 0.296
TRAY_D = 0.228
TRAY_TRAVEL = 0.110
TRAY_Z = 0.000
TRAY_Y_CLOSED = 0.027


def _axis_matches(
    axis: tuple[float, float, float], expected: tuple[float, float, float]
) -> bool:
    return all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    chrome = model.material("chrome", rgba=(0.82, 0.83, 0.85, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    black_trim = model.material("black_trim", rgba=(0.10, 0.10, 0.11, 1.0))
    oven_glass = model.material("oven_glass", rgba=(0.52, 0.67, 0.76, 0.28))
    interior_dark = model.material("interior_dark", rgba=(0.13, 0.13, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        Box((SHELL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(-(BODY_W * 0.5 - SHELL_T * 0.5), 0.0, BODY_H * 0.5)),
        material=chrome,
        name="left_side",
    )
    body.visual(
        Box((SHELL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W * 0.5 - SHELL_T * 0.5, 0.0, BODY_H * 0.5)),
        material=chrome,
        name="right_side",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - SHELL_T * 0.5)),
        material=chrome,
        name="top_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, BODY_H - SHELL_T)),
        origin=Origin(
            xyz=(0.0, -(BODY_D * 0.5 - SHELL_T * 0.5), (BODY_H - SHELL_T) * 0.5)
        ),
        material=chrome,
        name="back_panel",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, BODY_D - 0.024, 0.006)),
        origin=Origin(xyz=(0.0, -0.012, 0.041)),
        material=interior_dark,
        name="chamber_floor",
    )
    body.visual(
        Box((0.049, 0.024, 0.176)),
        origin=Origin(xyz=(-0.1875, 0.143, 0.122)),
        material=chrome,
        name="front_left_frame",
    )
    body.visual(
        Box((0.049, 0.024, 0.176)),
        origin=Origin(xyz=(0.1875, 0.143, 0.122)),
        material=chrome,
        name="front_right_frame",
    )
    body.visual(
        Box((0.375, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, 0.143, 0.027)),
        material=chrome,
        name="front_lower_sill",
    )
    body.visual(
        Box((0.375, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.143, 0.222)),
        material=chrome,
        name="front_top_cap",
    )
    body.visual(
        Box((0.006, 0.215, 0.024)),
        origin=Origin(xyz=(-0.147, -0.002, 0.026)),
        material=dark_steel,
        name="left_guide",
    )
    body.visual(
        Box((0.006, 0.215, 0.024)),
        origin=Origin(xyz=(0.147, -0.002, 0.026)),
        material=dark_steel,
        name="right_guide",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.235),
        origin=Origin(
            xyz=(-0.120, -(BODY_D * 0.5 - 0.028), 0.152),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="upper_rear_element_left",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.235),
        origin=Origin(
            xyz=(0.120, -(BODY_D * 0.5 - 0.028), 0.152),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="upper_rear_element_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, 0.016)),
        origin=Origin(xyz=(0.0, DOOR_T * 0.5, 0.008)),
        material=chrome,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_W, DOOR_T, 0.018)),
        origin=Origin(xyz=(0.0, DOOR_T * 0.5, DOOR_H - 0.009)),
        material=chrome,
        name="top_rail",
    )
    door.visual(
        Box((0.020, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-(DOOR_W * 0.5 - 0.010), DOOR_T * 0.5, DOOR_H * 0.5)),
        material=chrome,
        name="left_stile",
    )
    door.visual(
        Box((0.020, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_W * 0.5 - 0.010, DOOR_T * 0.5, DOOR_H * 0.5)),
        material=chrome,
        name="right_stile",
    )
    door.visual(
        Box((DOOR_W - 0.040, 0.004, DOOR_H - 0.034)),
        origin=Origin(xyz=(0.0, 0.007, 0.087)),
        material=oven_glass,
        name="glass_panel",
    )
    door.visual(
        Cylinder(radius=0.005, length=DOOR_W - 0.008),
        origin=Origin(xyz=(0.0, 0.011, 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(-0.055, 0.021, 0.165)),
        material=chrome,
        name="handle_mount_left",
    )
    door.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.055, 0.021, 0.165)),
        material=chrome,
        name="handle_mount_right",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.150),
        origin=Origin(xyz=(0.0, 0.028, 0.165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_trim,
        name="pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, 0.030, DOOR_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.015, DOOR_H * 0.5)),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((TRAY_W, TRAY_D, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=dark_steel,
        name="tray_floor",
    )
    crumb_tray.visual(
        Box((0.003, TRAY_D, 0.014)),
        origin=Origin(xyz=(-(TRAY_W * 0.5 - 0.0015), 0.0, 0.007)),
        material=dark_steel,
        name="left_wall",
    )
    crumb_tray.visual(
        Box((0.003, TRAY_D, 0.014)),
        origin=Origin(xyz=(TRAY_W * 0.5 - 0.0015, 0.0, 0.007)),
        material=dark_steel,
        name="right_wall",
    )
    crumb_tray.visual(
        Box((TRAY_W, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, -(TRAY_D * 0.5 - 0.0015), 0.006)),
        material=dark_steel,
        name="rear_wall",
    )
    crumb_tray.visual(
        Box((TRAY_W + 0.014, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, TRAY_D * 0.5 + 0.007, 0.009)),
        material=black_trim,
        name="tray_front_lip",
    )
    crumb_tray.inertial = Inertial.from_geometry(
        Box((TRAY_W + 0.014, TRAY_D + 0.014, 0.018)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.007, 0.009)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, DOOR_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "crumb_tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=crumb_tray,
        origin=Origin(xyz=(0.0, TRAY_Y_CLOSED, TRAY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.20,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    crumb_tray = object_model.get_part("crumb_tray")
    door_hinge = object_model.get_articulation("door_hinge")
    crumb_tray_slide = object_model.get_articulation("crumb_tray_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "toaster_parts_present",
        all(part is not None for part in (body, door, crumb_tray)),
        "Expected body, door, and crumb tray parts.",
    )
    ctx.check(
        "door_hinge_axis_and_limits",
        _axis_matches(door_hinge.axis, (-1.0, 0.0, 0.0))
        and door_hinge.motion_limits is not None
        and math.isclose(door_hinge.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper >= 1.15,
        f"axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "crumb_tray_slide_axis_and_limits",
        _axis_matches(crumb_tray_slide.axis, (0.0, 1.0, 0.0))
        and crumb_tray_slide.motion_limits is not None
        and math.isclose(crumb_tray_slide.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and crumb_tray_slide.motion_limits.upper is not None
        and crumb_tray_slide.motion_limits.upper >= 0.10,
        f"axis={crumb_tray_slide.axis}, limits={crumb_tray_slide.motion_limits}",
    )

    with ctx.pose({door_hinge: 0.0, crumb_tray_slide: 0.0}):
        ctx.expect_contact(
            door,
            body,
            name="door_connected_to_body_when_closed",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.17,
            name="door_covers_main_opening",
        )
        door_closed_aabb = ctx.part_world_aabb(door)
        front_frame_aabb = ctx.part_element_world_aabb(body, elem="front_left_frame")
        ctx.check(
            "door_front_sits_proud_of_chrome_frame",
            door_closed_aabb is not None
            and front_frame_aabb is not None
            and door_closed_aabb[1][1] > front_frame_aabb[1][1] + 0.015,
            f"door_aabb={door_closed_aabb}, frame_aabb={front_frame_aabb}",
        )
        ctx.expect_gap(
            body,
            crumb_tray,
            axis="z",
            positive_elem="chamber_floor",
            negative_elem="tray_front_lip",
            min_gap=0.018,
            max_gap=0.0225,
            name="crumb_tray_stays_below_oven_floor",
        )
        ctx.expect_contact(
            body,
            crumb_tray,
            elem_a="left_guide",
            elem_b="left_wall",
            name="left_guide_clearance",
        )
        ctx.expect_contact(
            body,
            crumb_tray,
            elem_a="right_guide",
            elem_b="right_wall",
            name="right_guide_clearance",
        )
        ctx.expect_overlap(
            crumb_tray,
            body,
            axes="xy",
            min_overlap=0.18,
            name="crumb_tray_nested_in_base",
        )
        crumb_tray_closed_pos = ctx.part_world_position(crumb_tray)

    with ctx.pose({door_hinge: 1.15}):
        door_open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door_drops_down_when_opened",
            door_closed_aabb is not None
            and door_open_aabb is not None
            and door_open_aabb[1][1] > door_closed_aabb[1][1] + 0.10
            and door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.09,
            f"closed_aabb={door_closed_aabb}, open_aabb={door_open_aabb}",
        )

    with ctx.pose({crumb_tray_slide: TRAY_TRAVEL}):
        crumb_tray_open_pos = ctx.part_world_position(crumb_tray)
        ctx.expect_overlap(
            crumb_tray,
            body,
            axes="x",
            min_overlap=0.28,
            name="crumb_tray_remains_captured_between_side_guides",
        )
        ctx.expect_contact(
            body,
            crumb_tray,
            elem_a="left_guide",
            elem_b="left_wall",
            name="left_guide_clearance_at_extension",
        )
        ctx.check(
            "crumb_tray_pulls_forward",
            crumb_tray_closed_pos is not None
            and crumb_tray_open_pos is not None
            and crumb_tray_open_pos[1] > crumb_tray_closed_pos[1] + 0.09,
            f"closed_pos={crumb_tray_closed_pos}, open_pos={crumb_tray_open_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
