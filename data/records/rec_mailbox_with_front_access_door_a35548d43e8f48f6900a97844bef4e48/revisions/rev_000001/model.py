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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rect_profile(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> list[tuple[float, float]]:
    return [
        (x_min, y_min),
        (x_max, y_min),
        (x_max, y_max),
        (x_min, y_max),
    ]


def _circle_profile(
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 28,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_fascia_mesh(
    *,
    width: float,
    z_min: float,
    z_max: float,
    slot_width: float,
    slot_height: float,
    slot_center_z: float,
    thickness: float,
):
    fascia = ExtrudeWithHolesGeometry(
        _rect_profile(-width * 0.5, width * 0.5, z_min, z_max),
        [
            _translate_profile(
                rounded_rect_profile(
                    slot_width,
                    slot_height,
                    radius=min(slot_height * 0.45, 0.010),
                    corner_segments=6,
                ),
                dy=slot_center_z,
            )
        ],
        thickness,
        center=True,
    )
    fascia.rotate_x(math.pi / 2.0)
    return fascia


def _build_door_face_mesh(
    *,
    width: float,
    z_bottom: float,
    height: float,
    thickness: float,
    lock_center_z: float,
    lock_hole_radius: float,
    panel_center_y: float,
):
    panel = ExtrudeWithHolesGeometry(
        _rect_profile(-width * 0.5, width * 0.5, z_bottom, z_bottom + height),
        [_circle_profile(lock_hole_radius, cy=lock_center_z, segments=32)],
        thickness,
        center=True,
    )
    panel.rotate_x(math.pi / 2.0)
    panel.translate(0.0, panel_center_y, 0.0)
    return panel


def _build_hinge_tube_mesh(
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
):
    tube = LatheGeometry.from_shell_profiles(
        [  # outer
            (outer_radius, -length * 0.5),
            (outer_radius, length * 0.5),
        ],
        [  # inner
            (inner_radius, -length * 0.5),
            (inner_radius, length * 0.5),
        ],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )
    tube.rotate_y(math.pi / 2.0)
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_wall_mailbox")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.59, 0.28, 1.0))
    dark_insert = model.material("dark_insert", rgba=(0.10, 0.10, 0.10, 1.0))

    width = 0.36
    depth = 0.18
    height = 0.45
    shell_t = 0.003
    door_width = 0.332
    door_height = 0.268
    door_t = 0.014
    door_panel_back = 0.003
    door_panel_center_y = door_panel_back + door_t * 0.5
    hinge_pin_radius = 0.0085
    hinge_tube_outer = 0.0105
    hinge_axis_z = 0.018
    door_panel_bottom = 0.004
    door_top_z = door_panel_bottom + door_height
    slot_width = 0.250
    slot_height = 0.028
    slot_center_z = 0.372
    lock_center_z = door_panel_bottom + door_height * 0.53
    lock_hole_radius = 0.0082

    body = model.part("body")
    body.visual(
        Box((width, shell_t, height)),
        origin=Origin(xyz=(0.0, shell_t * 0.5, height * 0.5)),
        material=body_finish,
        name="back_panel",
    )
    body.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=(-width * 0.5 + shell_t * 0.5, depth * 0.5, height * 0.5)),
        material=body_finish,
        name="left_wall",
    )
    body.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=(width * 0.5 - shell_t * 0.5, depth * 0.5, height * 0.5)),
        material=body_finish,
        name="right_wall",
    )
    body.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, depth * 0.5, shell_t * 0.5)),
        material=body_finish,
        name="bottom_panel",
    )
    body.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, depth * 0.5, height - shell_t * 0.5)),
        material=body_finish,
        name="top_panel",
    )
    body.visual(
        Box((0.020, shell_t, door_top_z)),
        origin=Origin(
            xyz=(
                -width * 0.5 + 0.010,
                depth - shell_t * 0.5,
                door_top_z * 0.5,
            )
        ),
        material=body_finish,
        name="left_front_stile",
    )
    body.visual(
        Box((0.020, shell_t, door_top_z)),
        origin=Origin(
            xyz=(
                width * 0.5 - 0.010,
                depth - shell_t * 0.5,
                door_top_z * 0.5,
            )
        ),
        material=body_finish,
        name="right_front_stile",
    )
    body.visual(
        Box((width - 0.012, shell_t, 0.020)),
        origin=Origin(xyz=(0.0, depth - shell_t * 0.5, 0.010)),
        material=body_finish,
        name="threshold_rail",
    )
    body.visual(
        mesh_from_geometry(
            _build_fascia_mesh(
                width=width - 2.0 * shell_t,
                z_min=door_top_z + 0.004,
                z_max=height,
                slot_width=slot_width,
                slot_height=slot_height,
                slot_center_z=slot_center_z,
                thickness=shell_t,
            ),
            "mailbox_front_fascia",
        ),
        origin=Origin(xyz=(0.0, depth - shell_t * 0.5, 0.0)),
        material=body_finish,
        name="front_fascia",
    )
    body.visual(
        Box((slot_width + 0.040, 0.018, shell_t)),
        origin=Origin(
            xyz=(
                0.0,
                depth + 0.009,
                slot_center_z + slot_height * 0.5 + 0.014,
            )
        ),
        material=steel,
        name="slot_rain_hood",
    )
    body.visual(
        Box((slot_width + 0.010, shell_t, 0.050)),
        origin=Origin(
            xyz=(
                0.0,
                depth - 0.015,
                slot_center_z - 0.010,
            ),
            rpy=(0.45, 0.0, 0.0),
        ),
        material=steel,
        name="slot_baffle",
    )
    body.visual(
        Cylinder(radius=hinge_pin_radius, length=width - 0.048),
        origin=Origin(
            xyz=(0.0, depth - shell_t, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="hinge_pin",
    )
    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=5.4,
        origin=Origin(xyz=(0.0, depth * 0.5, height * 0.5)),
    )

    door = model.part("retrieval_door")
    door.visual(
        mesh_from_geometry(
            _build_door_face_mesh(
                width=door_width,
                z_bottom=door_panel_bottom,
                height=door_height,
                thickness=door_t,
                lock_center_z=lock_center_z,
                lock_hole_radius=lock_hole_radius,
                panel_center_y=door_panel_center_y,
            ),
            "mailbox_retrieval_door_face",
        ),
        material=body_finish,
        name="door_face",
    )
    door.visual(
        Box((door_width, 0.006, 0.018)),
        origin=Origin(
            xyz=(
                0.0,
                0.006,
                door_panel_bottom + door_height - 0.009,
            )
        ),
        material=body_finish,
        name="top_return",
    )
    door.visual(
        Box((0.018, 0.006, door_height - 0.010)),
        origin=Origin(
            xyz=(
                -door_width * 0.5 + 0.009,
                0.006,
                door_panel_bottom + door_height * 0.5,
            )
        ),
        material=body_finish,
        name="left_return",
    )
    door.visual(
        Box((0.018, 0.006, door_height - 0.010)),
        origin=Origin(
            xyz=(
                door_width * 0.5 - 0.009,
                0.006,
                door_panel_bottom + door_height * 0.5,
            )
        ),
        material=body_finish,
        name="right_return",
    )
    door.visual(
        Box((door_width - 0.070, 0.006, 0.018)),
        origin=Origin(
            xyz=(
                0.0,
                0.006,
                door_panel_bottom + 0.050,
            )
        ),
        material=body_finish,
        name="lower_stiffener",
    )
    door.visual(
        mesh_from_geometry(
            _build_hinge_tube_mesh(
                length=width - 0.060,
                outer_radius=hinge_tube_outer,
                inner_radius=hinge_pin_radius + 0.001,
            ),
            "mailbox_door_hinge_tube",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.020, door_height + 0.020)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.008, door_height * 0.5)),
    )

    model.articulation(
        "body_to_retrieval_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, depth - shell_t, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-1.35,
            upper=0.0,
        ),
    )

    lock = model.part("lock_cylinder")
    lock.visual(
        Cylinder(radius=0.0075, length=door_t + 0.014),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="spindle",
    )
    lock.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.010, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="lock_bezel",
    )
    lock.visual(
        Cylinder(radius=0.0105, length=0.010),
        origin=Origin(
            xyz=(0.0, -0.012, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="rear_hub",
    )
    lock.visual(
        Box((0.070, 0.004, 0.014)),
        origin=Origin(xyz=(0.026, -0.019, 0.0)),
        material=steel,
        name="cam_latch",
    )
    lock.visual(
        Box((0.003, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, 0.0145, 0.0)),
        material=dark_insert,
        name="keyway",
    )
    lock.inertial = Inertial.from_geometry(
        Box((0.080, 0.030, 0.030)),
        mass=0.18,
        origin=Origin(xyz=(0.015, -0.003, 0.0)),
    )

    model.articulation(
        "door_to_lock_cylinder",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lock,
        origin=Origin(xyz=(0.0, door_panel_center_y, lock_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("retrieval_door")
    lock = object_model.get_part("lock_cylinder")
    door_hinge = object_model.get_articulation("body_to_retrieval_door")
    lock_joint = object_model.get_articulation("door_to_lock_cylinder")

    def _check_hinge_capture(name: str) -> bool:
        pin_aabb = ctx.part_element_world_aabb(body, elem="hinge_pin")
        tube_aabb = ctx.part_element_world_aabb(door, elem="hinge_tube")
        if pin_aabb is None or tube_aabb is None:
            return ctx.fail(name, "Missing hinge pin or hinge tube geometry for capture check.")
        pin_min, pin_max = pin_aabb
        tube_min, tube_max = tube_aabb
        x_overlap = min(pin_max[0], tube_max[0]) - max(pin_min[0], tube_min[0])
        pin_center_y = (pin_min[1] + pin_max[1]) * 0.5
        pin_center_z = (pin_min[2] + pin_max[2]) * 0.5
        tube_center_y = (tube_min[1] + tube_max[1]) * 0.5
        tube_center_z = (tube_min[2] + tube_max[2]) * 0.5
        return ctx.check(
            name,
            x_overlap > 0.290
            and abs(pin_center_y - tube_center_y) < 0.004
            and abs(pin_center_z - tube_center_z) < 0.004,
            "Hinge tube should remain wrapped around the hinge pin so the door stays supported.",
        )

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
        "mailbox_primary_articulations",
        door_hinge.axis == (1.0, 0.0, 0.0)
        and lock_joint.axis == (0.0, 1.0, 0.0)
        and door_hinge.motion_limits is not None
        and lock_joint.motion_limits is not None
        and door_hinge.motion_limits.upper == 0.0
        and door_hinge.motion_limits.lower is not None
        and door_hinge.motion_limits.lower < -1.0
        and lock_joint.motion_limits.lower == -math.pi / 2.0
        and lock_joint.motion_limits.upper == math.pi / 2.0,
        "Expected a bottom-hinged pull-down door and a face-normal rotating lock cylinder.",
    )

    _check_hinge_capture("door_hinge_captured_closed")
    ctx.expect_contact(
        lock,
        door,
        elem_a="lock_bezel",
        elem_b="door_face",
        name="lock_seated_in_door_face",
    )

    closed_face_aabb = ctx.part_element_world_aabb(door, elem="door_face")
    body_aabb = ctx.part_world_aabb(body)
    with ctx.pose({door_hinge: -1.20}):
        _check_hinge_capture("door_hinge_captured_open")
        open_face_aabb = ctx.part_element_world_aabb(door, elem="door_face")
        if closed_face_aabb is not None and open_face_aabb is not None and body_aabb is not None:
            ctx.check(
                "door_swings_forward_when_opened",
                open_face_aabb[1][1] > body_aabb[1][1] + 0.070,
                "Open door should project clearly forward of the mailbox body.",
            )
            ctx.check(
                "door_swings_downward_when_opened",
                open_face_aabb[1][2] < closed_face_aabb[1][2] - 0.120,
                "Open door should rotate downward from the lower hinge axis.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
