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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_turntable_mesh():
    outer_profile = [
        (0.013, 0.009),
        (0.040, 0.0048),
        (0.090, 0.0042),
        (0.135, 0.0047),
        (0.147, 0.0052),
        (0.149, 0.0082),
    ]
    inner_profile = [
        (0.013, 0.0041),
        (0.040, 0.0017),
        (0.090, 0.0012),
        (0.135, 0.0017),
        (0.145, 0.0021),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "microwave_turntable_glass",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_microwave")

    body_steel = model.material("body_steel", rgba=(0.43, 0.45, 0.47, 1.0))
    fascia_dark = model.material("fascia_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    cavity_enamel = model.material("cavity_enamel", rgba=(0.82, 0.83, 0.86, 1.0))
    door_glass = model.material("door_glass", rgba=(0.16, 0.20, 0.22, 0.42))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.20, 0.21, 0.22, 1.0))
    pointer_white = model.material("pointer_white", rgba=(0.92, 0.92, 0.90, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.68, 0.84, 0.90, 0.34))
    spindle_dark = model.material("spindle_dark", rgba=(0.24, 0.24, 0.25, 1.0))

    outer_w = 0.560
    outer_d = 0.490
    outer_h = 0.380
    shell_t = 0.018
    front_frame_d = 0.038
    front_face_y = -outer_d * 0.5

    control_h = 0.092
    opening_w = 0.410
    opening_h = 0.238
    opening_bottom = 0.084
    opening_top = opening_bottom + opening_h
    side_margin = (outer_w - opening_w) * 0.5

    housing = model.part("housing")
    housing.visual(
        Box((outer_w, outer_d, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=body_steel,
        name="bottom_shell",
    )
    housing.visual(
        Box((shell_t, outer_d, outer_h - 0.001)),
        origin=Origin(
            xyz=(-outer_w * 0.5 + shell_t * 0.5, 0.0, (outer_h - 0.001) * 0.5)
        ),
        material=body_steel,
        name="left_shell",
    )
    housing.visual(
        Box((shell_t, outer_d, outer_h - 0.001)),
        origin=Origin(
            xyz=(outer_w * 0.5 - shell_t * 0.5, 0.0, (outer_h - 0.001) * 0.5)
        ),
        material=body_steel,
        name="right_shell",
    )
    housing.visual(
        Box((outer_w - 2.0 * shell_t + 0.002, outer_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, outer_h - shell_t * 0.5)),
        material=body_steel,
        name="top_shell",
    )
    housing.visual(
        Box((outer_w - 2.0 * shell_t + 0.002, shell_t, outer_h - 0.020)),
        origin=Origin(
            xyz=(0.0, outer_d * 0.5 - shell_t * 0.5, 0.020 + (outer_h - 0.020) * 0.5)
        ),
        material=body_steel,
        name="rear_shell",
    )
    housing.visual(
        Box((outer_w - 2.0 * shell_t + 0.002, front_frame_d, control_h)),
        origin=Origin(xyz=(0.0, front_face_y + front_frame_d * 0.5, control_h * 0.5)),
        material=body_steel,
        name="lower_fascia",
    )
    housing.visual(
        Box((side_margin + 0.002, front_frame_d, opening_h + 0.010)),
        origin=Origin(
            xyz=(
                -outer_w * 0.5 + (side_margin + 0.002) * 0.5,
                front_face_y + front_frame_d * 0.5,
                opening_bottom + (opening_h + 0.010) * 0.5,
            )
        ),
        material=body_steel,
        name="left_front_pillar",
    )
    housing.visual(
        Box((side_margin + 0.002, front_frame_d, opening_h + 0.010)),
        origin=Origin(
            xyz=(
                outer_w * 0.5 - (side_margin + 0.002) * 0.5,
                front_face_y + front_frame_d * 0.5,
                opening_bottom + (opening_h + 0.010) * 0.5,
            )
        ),
        material=body_steel,
        name="right_front_pillar",
    )
    housing.visual(
        Box((opening_w + 0.040, front_frame_d, outer_h - opening_top + 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y + front_frame_d * 0.5,
                opening_top + (outer_h - opening_top + 0.010) * 0.5,
            )
        ),
        material=body_steel,
        name="top_front_lintel",
    )
    housing.visual(
        Box((0.356, 0.006, 0.060)),
        origin=Origin(xyz=(0.0, front_face_y - 0.003, 0.046)),
        material=fascia_dark,
        name="control_panel",
    )
    housing.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, outer_h * 0.5)),
    )

    cavity = model.part("cavity")
    cavity_floor_top = 0.084
    cavity_floor_th = 0.006
    cavity_w = 0.398
    cavity_d = 0.372
    cavity_h = 0.214
    cavity_front_y = front_face_y + 0.040
    cavity_center_y = cavity_front_y + cavity_d * 0.5
    cavity.visual(
        Box((cavity_w, cavity_d, cavity_floor_th)),
        origin=Origin(
            xyz=(0.0, cavity_center_y, cavity_floor_top - cavity_floor_th * 0.5)
        ),
        material=cavity_enamel,
        name="cavity_floor",
    )
    cavity.visual(
        Box((0.004, cavity_d, cavity_h)),
        origin=Origin(
            xyz=(-cavity_w * 0.5 + 0.002, cavity_center_y, cavity_floor_top + cavity_h * 0.5)
        ),
        material=cavity_enamel,
        name="left_wall",
    )
    cavity.visual(
        Box((0.004, cavity_d, cavity_h)),
        origin=Origin(
            xyz=(cavity_w * 0.5 - 0.002, cavity_center_y, cavity_floor_top + cavity_h * 0.5)
        ),
        material=cavity_enamel,
        name="right_wall",
    )
    cavity.visual(
        Box((cavity_w, cavity_d, 0.004)),
        origin=Origin(
            xyz=(0.0, cavity_center_y, cavity_floor_top + cavity_h + 0.002)
        ),
        material=cavity_enamel,
        name="cavity_ceiling",
    )
    cavity.visual(
        Box((cavity_w, 0.004, cavity_h + 0.004)),
        origin=Origin(
            xyz=(
                0.0,
                cavity_front_y + cavity_d - 0.002,
                cavity_floor_top + (cavity_h + 0.004) * 0.5,
            )
        ),
        material=cavity_enamel,
        name="rear_wall",
    )
    cavity.visual(
        Box((cavity_w, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, cavity_front_y + 0.009, cavity_floor_top + 0.005)),
        material=cavity_enamel,
        name="front_sill",
    )
    cavity.visual(
        Box((0.160, 0.160, cavity_floor_top - 0.020)),
        origin=Origin(
            xyz=(
                0.0,
                cavity_center_y + 0.060,
                0.020 + (cavity_floor_top - 0.020) * 0.5,
            )
        ),
        material=trim_black,
        name="support_plinth",
    )
    cavity.visual(
        Cylinder(radius=0.010, length=0.009),
        origin=Origin(xyz=(0.0, cavity_center_y, cavity_floor_top + 0.0045)),
        material=spindle_dark,
        name="spindle_shaft",
    )
    cavity.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, cavity_center_y, cavity_floor_top + 0.011)),
        material=spindle_dark,
        name="spindle_cap",
    )
    cavity.inertial = Inertial.from_geometry(
        Box((cavity_w, cavity_d, cavity_h)),
        mass=4.5,
        origin=Origin(xyz=(0.0, cavity_center_y, cavity_floor_top + cavity_h * 0.5)),
    )
    model.articulation(
        "housing_to_cavity",
        ArticulationType.FIXED,
        parent=housing,
        child=cavity,
        origin=Origin(),
    )

    door = model.part("door")
    door_w = 0.446
    door_h = 0.272
    frame_d = 0.041
    frame_center_y = -0.0205
    frame_x0 = 0.022
    left_frame_w = 0.024
    right_frame_w = 0.078
    rail_span = door_w - frame_x0
    rail_center_x = frame_x0 + rail_span * 0.5
    door.visual(
        Box((left_frame_w, frame_d, door_h)),
        origin=Origin(
            xyz=(frame_x0 + left_frame_w * 0.5, frame_center_y, door_h * 0.5)
        ),
        material=body_steel,
        name="left_frame_stile",
    )
    door.visual(
        Box((right_frame_w, frame_d, door_h)),
        origin=Origin(
            xyz=(door_w - right_frame_w * 0.5, frame_center_y, door_h * 0.5)
        ),
        material=body_steel,
        name="handle_stile",
    )
    door.visual(
        Box((rail_span, frame_d, 0.036)),
        origin=Origin(xyz=(rail_center_x, frame_center_y, door_h - 0.018)),
        material=body_steel,
        name="top_rail",
    )
    door.visual(
        Box((rail_span, frame_d, 0.040)),
        origin=Origin(xyz=(rail_center_x, frame_center_y, 0.020)),
        material=body_steel,
        name="bottom_rail",
    )
    door.visual(
        Box((0.292, 0.006, 0.198)),
        origin=Origin(xyz=(0.210, -0.028, 0.136)),
        material=door_glass,
        name="window_glass",
    )
    door.visual(
        Box((0.312, 0.010, 0.214)),
        origin=Origin(xyz=(0.206, -0.017, 0.136)),
        material=trim_black,
        name="inner_bezel",
    )
    handle_x = 0.376
    door.visual(
        Box((0.024, 0.032, 0.028)),
        origin=Origin(xyz=(handle_x, -0.053, 0.074)),
        material=trim_black,
        name="lower_handle_mount",
    )
    door.visual(
        Box((0.024, 0.032, 0.028)),
        origin=Origin(xyz=(handle_x, -0.053, 0.210)),
        material=trim_black,
        name="upper_handle_mount",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.190),
        origin=Origin(xyz=(handle_x, -0.080, 0.142)),
        material=trim_black,
        name="pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.086, door_h)),
        mass=6.5,
        origin=Origin(xyz=(door_w * 0.5, -0.043, door_h * 0.5)),
    )
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-0.228, front_face_y, opening_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.8,
            lower=-math.radians(105.0),
            upper=0.0,
        ),
    )

    turntable = model.part("turntable")
    turntable.visual(
        _build_turntable_mesh(),
        origin=Origin(),
        material=glass_clear,
        name="turntable_glass",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.149, length=0.010),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )
    model.articulation(
        "cavity_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=cavity,
        child=turntable,
        origin=Origin(xyz=(0.0, cavity_center_y, cavity_floor_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=10.0),
    )

    def add_knob(
        part_name: str,
        articulation_name: str,
        *,
        center_x: float,
    ) -> None:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.0285, length=0.030),
            origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_cap,
            name="knob_face",
        )
        knob.visual(
            Box((0.004, 0.006, 0.018)),
            origin=Origin(xyz=(0.0, -0.035, 0.019)),
            material=pointer_white,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.029, length=0.030),
            mass=0.12,
            origin=Origin(xyz=(0.0, -0.015, 0.0)),
        )
        model.articulation(
            articulation_name,
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(center_x, front_face_y - 0.004, 0.046)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=3.0,
                lower=-2.35,
                upper=2.35,
            ),
        )

    add_knob(
        "left_selector_knob",
        "housing_to_left_selector_knob",
        center_x=-0.100,
    )
    add_knob(
        "right_selector_knob",
        "housing_to_right_selector_knob",
        center_x=0.100,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cavity = object_model.get_part("cavity")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    left_knob = object_model.get_part("left_selector_knob")
    right_knob = object_model.get_part("right_selector_knob")

    door_hinge = object_model.get_articulation("housing_to_door")
    turntable_spin = object_model.get_articulation("cavity_to_turntable")
    left_knob_joint = object_model.get_articulation("housing_to_left_selector_knob")
    right_knob_joint = object_model.get_articulation("housing_to_right_selector_knob")

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

    ctx.expect_contact(cavity, housing, contact_tol=0.0015)
    ctx.expect_contact(door, housing, contact_tol=0.0015)
    ctx.expect_contact(left_knob, housing, contact_tol=0.0015)
    ctx.expect_contact(right_knob, housing, contact_tol=0.0015)
    ctx.expect_contact(turntable, cavity, elem_b="spindle_cap", contact_tol=0.0015)
    ctx.expect_gap(
        turntable,
        cavity,
        axis="z",
        positive_elem="turntable_glass",
        negative_elem="cavity_floor",
        min_gap=0.0005,
        max_gap=0.008,
        name="turntable_clear_of_floor",
    )
    ctx.expect_within(
        turntable,
        cavity,
        axes="xy",
        inner_elem="turntable_glass",
        outer_elem="cavity_floor",
        margin=0.0,
    )

    ctx.check(
        "door_is_vertical_hinge_revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_hinge.axis) == (0.0, 0.0, 1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower is not None
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.lower < 0.0 <= door_hinge.motion_limits.upper
        and (door_hinge.motion_limits.upper - door_hinge.motion_limits.lower) > math.radians(90.0),
        "Door should open on a vertical side hinge with a realistic opening range.",
    )
    ctx.check(
        "turntable_is_vertical_continuous",
        turntable_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(turntable_spin.axis) == (0.0, 0.0, 1.0),
        "Turntable should rotate continuously about a vertical spindle.",
    )
    ctx.check(
        "selector_knobs_rotate_on_panel_normal",
        left_knob_joint.articulation_type == ArticulationType.REVOLUTE
        and right_knob_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(left_knob_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(right_knob_joint.axis) == (0.0, -1.0, 0.0),
        "Both selector knobs should rotate about their front-to-back local axes.",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    if closed_door_aabb is not None:
        with ctx.pose({door_hinge: -math.radians(100.0)}):
            open_door_aabb = ctx.part_world_aabb(door)
            if open_door_aabb is not None:
                ctx.check(
                    "door_swings_outward",
                    open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10,
                    "Door did not swing outward far enough from the front face.",
                )
            else:
                ctx.fail("door_swings_outward", "Open-pose door AABB was unavailable.")
            ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_with_door_open")

    rest_pointer_aabb = ctx.part_element_world_aabb(left_knob, elem="pointer")
    if rest_pointer_aabb is not None:
        with ctx.pose({left_knob_joint: 1.6}):
            turned_pointer_aabb = ctx.part_element_world_aabb(left_knob, elem="pointer")
            if turned_pointer_aabb is not None:
                moved_in_x = abs(turned_pointer_aabb[1][0] - rest_pointer_aabb[1][0]) > 0.010
                moved_in_z = abs(turned_pointer_aabb[1][2] - rest_pointer_aabb[1][2]) > 0.010
                ctx.check(
                    "left_knob_pointer_moves_when_rotated",
                    moved_in_x or moved_in_z,
                    "Selector knob pointer did not move in world space when posed.",
                )
            else:
                ctx.fail(
                    "left_knob_pointer_moves_when_rotated",
                    "Rotated left knob pointer AABB was unavailable.",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
