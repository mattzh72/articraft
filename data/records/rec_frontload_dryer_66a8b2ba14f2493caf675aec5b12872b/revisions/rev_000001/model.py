from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 56,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos(2.0 * pi * step / segments),
            cy + radius * sin(2.0 * pi * step / segments),
        )
        for step in range(segments)
    ]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    return [
        (-half_w, 0.0),
        (half_w, 0.0),
        (half_w, height),
        (-half_w, height),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drop_door_tumble_dryer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    fascia_dark = model.material("fascia_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    glass = model.material("glass", rgba=(0.53, 0.67, 0.78, 0.38))
    gasket_black = model.material("gasket_black", rgba=(0.07, 0.07, 0.08, 1.0))
    foot_black = model.material("foot_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body_width = 0.60
    body_depth = 0.58
    body_height = 0.84
    panel_t = 0.018
    front_y = body_depth * 0.5
    rear_y = -body_depth * 0.5

    opening_radius = 0.205
    opening_center_z = 0.46

    drum_radius = 0.255
    drum_depth = 0.46
    shaft_radius = 0.024
    shaft_length = 0.060

    door_width = 0.54
    door_height = 0.52
    door_t = 0.032
    door_hinge_z = 0.20
    door_window_radius = 0.170

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    front_panel_mesh = _save_mesh(
        "dryer_front_panel",
        ExtrudeWithHolesGeometry(
            _rect_profile(body_width, body_height),
            [_circle_profile(opening_radius, center=(0.0, opening_center_z))],
            height=panel_t,
            center=False,
        ),
    )
    body.visual(
        front_panel_mesh,
        origin=Origin(xyz=(0.0, front_y, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=cabinet_white,
        name="front_panel",
    )
    body.visual(
        Box((body_width, body_depth, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, panel_t * 0.5)),
        material=cabinet_white,
        name="base_pan",
    )
    body.visual(
        Box((body_width, body_depth, panel_t)),
        origin=Origin(xyz=(0.0, 0.0, body_height - panel_t * 0.5)),
        material=cabinet_white,
        name="top_panel",
    )
    body.visual(
        Box((panel_t, body_depth, body_height)),
        origin=Origin(xyz=(body_width * 0.5 - panel_t * 0.5, 0.0, body_height * 0.5)),
        material=cabinet_white,
        name="right_side_panel",
    )
    body.visual(
        Box((panel_t, body_depth, body_height)),
        origin=Origin(xyz=(-body_width * 0.5 + panel_t * 0.5, 0.0, body_height * 0.5)),
        material=cabinet_white,
        name="left_side_panel",
    )
    back_panel_mesh = _save_mesh(
        "dryer_back_panel",
        ExtrudeWithHolesGeometry(
            _rect_profile(body_width, body_height),
            [_circle_profile(0.048, center=(0.0, opening_center_z), segments=48)],
            height=panel_t,
            center=False,
        ),
    )
    body.visual(
        back_panel_mesh,
        origin=Origin(xyz=(0.0, rear_y, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=cabinet_white,
        name="back_panel",
    )

    body.visual(
        Box((0.44, 0.05, 0.09)),
        origin=Origin(xyz=(0.0, front_y - 0.025, body_height - 0.075)),
        material=fascia_dark,
        name="control_fascia",
    )
    body.visual(
        Cylinder(radius=0.038, length=0.022),
        origin=Origin(xyz=(-0.18, front_y + 0.011, body_height - 0.075), rpy=(pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="cycle_dial",
    )
    body.visual(
        Box((0.12, 0.012, 0.028)),
        origin=Origin(xyz=(0.12, front_y + 0.006, body_height - 0.076)),
        material=steel,
        name="display_strip",
    )

    body.visual(
        Box((0.11, 0.064, 0.11)),
        origin=Origin(xyz=(0.0, rear_y - 0.032, opening_center_z)),
        material=dark_steel,
        name="rear_bearing_housing",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, rear_y - 0.002, opening_center_z), rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bearing_insert",
    )

    for foot_name, foot_x in (("front_left_foot", -0.22), ("front_right_foot", 0.22)):
        body.visual(
            Cylinder(radius=0.028, length=0.022),
            origin=Origin(xyz=(foot_x, 0.22, -0.011)),
            material=foot_black,
            name=foot_name,
        )
    for foot_name, foot_x in (("rear_left_foot", -0.22), ("rear_right_foot", 0.22)):
        body.visual(
            Cylinder(radius=0.028, length=0.022),
            origin=Origin(xyz=(foot_x, -0.22, -0.011)),
            material=foot_black,
            name=foot_name,
        )

    drum = model.part("drum")
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_radius, length=drum_depth),
        mass=9.0,
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
    )
    drum_shell = _save_mesh(
        "dryer_drum_shell",
        CylinderGeometry(drum_radius, drum_depth, radial_segments=56, closed=False).rotate_x(pi * 0.5),
    )
    drum_front_lip = _save_mesh(
        "dryer_drum_front_lip",
        CylinderGeometry(drum_radius, 0.032, radial_segments=56, closed=False).rotate_x(pi * 0.5),
    )
    drum.visual(drum_shell, material=steel, name="drum_shell")
    drum.visual(
        drum_front_lip,
        origin=Origin(xyz=(0.0, drum_depth * 0.5 - 0.016, 0.0)),
        material=dark_steel,
        name="drum_front_lip",
    )
    drum.visual(
        Cylinder(radius=drum_radius, length=0.012),
        origin=Origin(xyz=(0.0, -drum_depth * 0.5 + 0.006, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="drum_back_plate",
    )
    drum.visual(
        Cylinder(radius=shaft_radius, length=shaft_length),
        origin=Origin(
            xyz=(0.0, -drum_depth * 0.5 - shaft_length * 0.5 + 0.002, 0.0),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=dark_steel,
        name="axle_stub",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_t, door_height)),
        mass=4.2,
        origin=Origin(xyz=(0.0, door_t * 0.5, door_height * 0.5)),
    )
    door_frame_mesh = _save_mesh(
        "dryer_drop_door_frame",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(door_width, door_height, 0.028, corner_segments=8),
            [_circle_profile(door_window_radius, center=(0.0, 0.0), segments=64)],
            height=door_t,
            center=False,
        ),
    )
    door.visual(
        door_frame_mesh,
        origin=Origin(xyz=(0.0, door_t, door_height * 0.5), rpy=(pi * 0.5, 0.0, 0.0)),
        material=fascia_dark,
        name="door_frame",
    )
    door.visual(
        Cylinder(radius=door_window_radius * 1.04, length=0.006),
        origin=Origin(
            xyz=(0.0, door_t * 0.68, door_height * 0.5),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(door_width * 0.5 - 0.07, door_t + 0.011, door_height * 0.67), rpy=(pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="handle_upper_post",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(door_width * 0.5 - 0.07, door_t + 0.011, door_height * 0.40), rpy=(pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="handle_lower_post",
    )
    door.visual(
        Cylinder(radius=0.010, length=door_height * 0.29),
        origin=Origin(
            xyz=(door_width * 0.5 - 0.07, door_t + 0.022, door_height * 0.535),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=steel,
        name="pull_bar",
    )
    door.visual(
        Box((0.075, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.006, door_height - 0.010)),
        material=gasket_black,
        name="top_latch_tongue",
    )

    model.articulation(
        "drum_spin",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, opening_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=10.0, lower=-6.283, upper=6.283),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, front_y, door_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.45, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_spin = object_model.get_articulation("drum_spin")
    door_hinge = object_model.get_articulation("door_hinge")

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

    ctx.expect_contact(door, body, name="door_contacts_front_frame_when_closed")
    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.44, name="door_covers_front_aperture")
    ctx.expect_contact(
        drum,
        body,
        elem_a="axle_stub",
        elem_b="rear_bearing_insert",
        name="drum_axle_bears_on_rear_housing",
    )
    ctx.expect_within(drum, body, axes="xz", margin=0.0, name="drum_stays_within_cabinet_span")
    ctx.expect_gap(
        body,
        drum,
        axis="y",
        min_gap=0.02,
        max_gap=0.08,
        positive_elem="front_panel",
        name="drum_sits_behind_front_panel_opening",
    )

    ctx.check(
        "door_hinge_axis_is_horizontal",
        abs(door_hinge.axis[0]) > 0.99 and abs(door_hinge.axis[1]) < 1e-6 and abs(door_hinge.axis[2]) < 1e-6,
        details=f"door hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "drum_axis_runs_front_to_back",
        abs(drum_spin.axis[1]) > 0.99 and abs(drum_spin.axis[0]) < 1e-6 and abs(drum_spin.axis[2]) < 1e-6,
        details=f"drum axis was {drum_spin.axis}",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    closed_body_aabb = ctx.part_world_aabb(body)
    if closed_door_aabb is None or closed_body_aabb is None:
        ctx.fail("rest_pose_aabbs_available", "missing door or body AABB in rest pose")
    else:
        with ctx.pose({door_hinge: -1.40}):
            open_door_aabb = ctx.part_world_aabb(door)
        if open_door_aabb is None:
            ctx.fail("open_pose_door_aabb_available", "missing door AABB in open pose")
        else:
            ctx.check(
                "door_drops_forward_into_loading_shelf_pose",
                open_door_aabb[1][1] > closed_body_aabb[1][1] + 0.18
                and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.18,
                details=(
                    f"open door max y/z={open_door_aabb[1][1]:.3f}/{open_door_aabb[1][2]:.3f}, "
                    f"closed door max z={closed_door_aabb[1][2]:.3f}, "
                    f"body max y={closed_body_aabb[1][1]:.3f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
