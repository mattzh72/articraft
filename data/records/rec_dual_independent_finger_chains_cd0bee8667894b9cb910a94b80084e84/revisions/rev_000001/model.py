from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PALM_WIDTH = 0.118
PALM_DEPTH = 0.050
PALM_BASE_HEIGHT = 0.012
PALM_UPPER_HEIGHT = 0.020
PALM_TOP_Z = PALM_BASE_HEIGHT + PALM_UPPER_HEIGHT

LEFT_PEDESTAL_ORIGIN = (-0.029, 0.002, PALM_TOP_Z)
RIGHT_PEDESTAL_ORIGIN = (0.029, 0.002, PALM_TOP_Z)

LEFT_PED_WIDTH = 0.030
LEFT_PED_DEPTH = 0.028
LEFT_PED_HEIGHT = 0.028

RIGHT_PED_WIDTH = 0.026
RIGHT_PED_DEPTH = 0.024
RIGHT_PED_HEIGHT = 0.024


def _box(size_x: float, size_y: float, size_z: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((cx - length / 2.0, cy, cz))
    )


def make_palm() -> cq.Workplane:
    lower = _box(PALM_WIDTH, PALM_DEPTH, PALM_BASE_HEIGHT, (0.0, 0.0, PALM_BASE_HEIGHT / 2.0))
    rear_bridge = _box(0.096, 0.018, 0.012, (0.0, -0.013, PALM_BASE_HEIGHT + 0.006))
    left_block = _box(0.046, 0.044, PALM_UPPER_HEIGHT, (-0.029, 0.000, PALM_BASE_HEIGHT + PALM_UPPER_HEIGHT / 2.0))
    right_block = _box(0.040, 0.042, PALM_UPPER_HEIGHT, (0.029, 0.000, PALM_BASE_HEIGHT + PALM_UPPER_HEIGHT / 2.0))
    left_front_boss = _box(0.032, 0.018, 0.008, (-0.029, 0.014, PALM_TOP_Z - 0.004))
    right_front_boss = _box(0.028, 0.016, 0.008, (0.029, 0.014, PALM_TOP_Z - 0.004))
    return lower.union(rear_bridge).union(left_block).union(right_block).union(left_front_boss).union(right_front_boss)


def make_pedestal(width: float, depth: float, height: float) -> cq.Workplane:
    lower = _box(width, depth, height * 0.68, (0.0, 0.0, height * 0.34))
    upper = _box(width * 0.78, depth * 0.54, height * 0.22, (0.0, depth * 0.06, height * 0.79))
    rear_shoulder = _box(width * 0.54, depth * 0.28, height * 0.18, (0.0, -depth * 0.20, height * 0.77))
    front_nose = _box(width * 0.58, depth * 0.14, height * 0.16, (0.0, depth * 0.34, height * 0.80))
    hinge_cap = _box(width * 0.66, depth * 0.18, height * 0.12, (0.0, depth * 0.48, height * 0.93))
    return lower.union(upper).union(rear_shoulder).union(front_nose).union(hinge_cap)


def make_link_body(
    *,
    axis_length: float,
    barrel_radius: float,
    barrel_length: float,
    plate_thickness: float,
    plate_height: float,
    body_stop: float,
    bridge_height: float,
) -> cq.Workplane:
    plate_gap = barrel_length - 2.0 * plate_thickness
    plate_start = barrel_radius * 0.55
    plate_length = max(axis_length - body_stop - plate_start, barrel_radius * 1.5)
    plate_center_y = plate_start + plate_length / 2.0
    side_x = plate_gap / 2.0 + plate_thickness / 2.0
    distal_bridge_length = min(max(axis_length * 0.11, 0.006), 0.009)

    barrel = _x_cylinder(barrel_radius, barrel_length, (0.0, 0.0, 0.0))
    left_plate = _box(plate_thickness, plate_length, plate_height, (side_x, plate_center_y, 0.0))
    right_plate = _box(plate_thickness, plate_length, plate_height, (-side_x, plate_center_y, 0.0))
    rear_web = _box(barrel_length * 0.72, max(barrel_radius * 0.95, 0.006), plate_height * 0.66, (0.0, barrel_radius * 0.55, 0.0))
    mid_bridge = _box(
        barrel_length * 0.82,
        min(max(axis_length * 0.16, 0.008), 0.012),
        bridge_height,
        (0.0, axis_length * 0.58, -0.0005),
    )
    distal_bridge = _box(
        barrel_length * 0.70,
        distal_bridge_length,
        bridge_height * 0.92,
        (0.0, axis_length - body_stop - distal_bridge_length / 2.0, -0.0008),
    )
    return barrel.union(left_plate).union(right_plate).union(rear_web).union(mid_bridge).union(distal_bridge)


def make_round_pad(width: float, axis_length: float) -> cq.Workplane:
    adapter = _box(width * 0.46, 0.010, 0.006, (0.0, axis_length - 0.001, -0.0022))
    pad_block = _box(width * 0.78, 0.010, 0.009, (0.0, axis_length + 0.003, -0.002))
    pad_nose = _x_cylinder(0.0055, width * 0.78, (0.0, axis_length + 0.006, -0.002))
    return adapter.union(pad_block).union(pad_nose)


def make_flat_nib(width: float, axis_length: float) -> cq.Workplane:
    support = _box(width * 0.56, 0.009, 0.007, (0.0, axis_length - 0.0005, -0.001))
    nib = _box(width * 0.72, 0.014, 0.0042, (0.0, axis_length + 0.0065, -0.003))
    return support.union(nib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_finger_manipulator_core")

    model.material("palm_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("pedestal_dark", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("link_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("rubber_pad", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("nib_bright", rgba=(0.80, 0.82, 0.84, 1.0))

    left_outer_width = 0.028
    right_outer_width = 0.024

    left_lengths = (0.050, 0.040, 0.032)
    right_lengths = (0.042, 0.032, 0.024)

    left_barrel_radius = 0.007
    right_barrel_radius = 0.006

    left_base_joint_origin = (
        0.0,
        LEFT_PED_DEPTH / 2.0 + left_barrel_radius - 0.0005,
        LEFT_PED_HEIGHT + left_barrel_radius - 0.0013,
    )
    right_base_joint_origin = (
        0.0,
        RIGHT_PED_DEPTH / 2.0 + right_barrel_radius - 0.0005,
        RIGHT_PED_HEIGHT + right_barrel_radius - 0.0009,
    )

    palm = model.part("palm")
    palm.visual(mesh_from_cadquery(make_palm(), "split_palm_block"), material="palm_dark", name="palm_shell")
    palm.inertial = Inertial.from_geometry(
        Box((PALM_WIDTH, PALM_DEPTH, PALM_TOP_Z)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, PALM_TOP_Z / 2.0)),
    )

    left_pedestal = model.part("left_pedestal")
    left_pedestal.visual(
        mesh_from_cadquery(make_pedestal(LEFT_PED_WIDTH, LEFT_PED_DEPTH, LEFT_PED_HEIGHT), "left_pedestal"),
        material="pedestal_dark",
        name="left_pedestal_shell",
    )
    left_pedestal.inertial = Inertial.from_geometry(
        Box((LEFT_PED_WIDTH, LEFT_PED_DEPTH, LEFT_PED_HEIGHT)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, LEFT_PED_HEIGHT / 2.0)),
    )

    right_pedestal = model.part("right_pedestal")
    right_pedestal.visual(
        mesh_from_cadquery(make_pedestal(RIGHT_PED_WIDTH, RIGHT_PED_DEPTH, RIGHT_PED_HEIGHT), "right_pedestal"),
        material="pedestal_dark",
        name="right_pedestal_shell",
    )
    right_pedestal.inertial = Inertial.from_geometry(
        Box((RIGHT_PED_WIDTH, RIGHT_PED_DEPTH, RIGHT_PED_HEIGHT)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, RIGHT_PED_HEIGHT / 2.0)),
    )

    left_proximal = model.part("left_proximal")
    left_proximal.visual(
        mesh_from_cadquery(
            make_link_body(
                axis_length=left_lengths[0],
                barrel_radius=left_barrel_radius,
                barrel_length=left_outer_width,
                plate_thickness=0.004,
                plate_height=0.017,
                body_stop=0.0066,
                bridge_height=0.010,
            ),
            "left_proximal",
        ),
        material="link_steel",
        name="left_proximal_body",
    )
    left_proximal.inertial = Inertial.from_geometry(
        Box((left_outer_width, left_lengths[0], 0.018)),
        mass=0.09,
        origin=Origin(xyz=(0.0, left_lengths[0] / 2.0, 0.0)),
    )

    left_middle = model.part("left_middle")
    left_middle.visual(
        mesh_from_cadquery(
            make_link_body(
                axis_length=left_lengths[1],
                barrel_radius=0.0066,
                barrel_length=left_outer_width * 0.96,
                plate_thickness=0.0038,
                plate_height=0.0155,
                body_stop=0.0062,
                bridge_height=0.009,
            ),
            "left_middle",
        ),
        material="link_steel",
        name="left_middle_body",
    )
    left_middle.inertial = Inertial.from_geometry(
        Box((left_outer_width * 0.96, left_lengths[1], 0.017)),
        mass=0.07,
        origin=Origin(xyz=(0.0, left_lengths[1] / 2.0, 0.0)),
    )

    left_distal = model.part("left_distal")
    left_distal.visual(
        mesh_from_cadquery(
            make_link_body(
                axis_length=left_lengths[2],
                barrel_radius=0.0062,
                barrel_length=left_outer_width * 0.90,
                plate_thickness=0.0036,
                plate_height=0.0145,
                body_stop=0.0022,
                bridge_height=0.008,
            ),
            "left_distal_body",
        ),
        material="link_steel",
        name="left_distal_body",
    )
    left_distal.visual(
        mesh_from_cadquery(make_round_pad(left_outer_width * 0.90, left_lengths[2]), "left_round_pad"),
        material="rubber_pad",
        name="left_round_pad",
    )
    left_distal.inertial = Inertial.from_geometry(
        Box((left_outer_width * 0.90, left_lengths[2] + 0.012, 0.016)),
        mass=0.05,
        origin=Origin(xyz=(0.0, left_lengths[2] / 2.0 + 0.004, -0.001)),
    )

    right_proximal = model.part("right_proximal")
    right_proximal.visual(
        mesh_from_cadquery(
            make_link_body(
                axis_length=right_lengths[0],
                barrel_radius=right_barrel_radius,
                barrel_length=right_outer_width,
                plate_thickness=0.0038,
                plate_height=0.015,
                body_stop=0.0058,
                bridge_height=0.009,
            ),
            "right_proximal",
        ),
        material="link_steel",
        name="right_proximal_body",
    )
    right_proximal.inertial = Inertial.from_geometry(
        Box((right_outer_width, right_lengths[0], 0.016)),
        mass=0.075,
        origin=Origin(xyz=(0.0, right_lengths[0] / 2.0, 0.0)),
    )

    right_middle = model.part("right_middle")
    right_middle.visual(
        mesh_from_cadquery(
            make_link_body(
                axis_length=right_lengths[1],
                barrel_radius=0.0058,
                barrel_length=right_outer_width * 0.94,
                plate_thickness=0.0035,
                plate_height=0.0135,
                body_stop=0.0052,
                bridge_height=0.008,
            ),
            "right_middle",
        ),
        material="link_steel",
        name="right_middle_body",
    )
    right_middle.inertial = Inertial.from_geometry(
        Box((right_outer_width * 0.94, right_lengths[1], 0.015)),
        mass=0.058,
        origin=Origin(xyz=(0.0, right_lengths[1] / 2.0, 0.0)),
    )

    right_distal = model.part("right_distal")
    right_distal.visual(
        mesh_from_cadquery(
            make_link_body(
                axis_length=right_lengths[2],
                barrel_radius=0.0052,
                barrel_length=right_outer_width * 0.88,
                plate_thickness=0.0033,
                plate_height=0.0125,
                body_stop=0.0018,
                bridge_height=0.007,
            ),
            "right_distal_body",
        ),
        material="link_steel",
        name="right_distal_body",
    )
    right_distal.visual(
        mesh_from_cadquery(make_flat_nib(right_outer_width * 0.88, right_lengths[2]), "right_flat_nib"),
        material="nib_bright",
        name="right_flat_nib",
    )
    right_distal.inertial = Inertial.from_geometry(
        Box((right_outer_width * 0.88, right_lengths[2] + 0.013, 0.014)),
        mass=0.04,
        origin=Origin(xyz=(0.0, right_lengths[2] / 2.0 + 0.004, -0.002)),
    )

    model.articulation(
        "palm_to_left_pedestal",
        ArticulationType.FIXED,
        parent=palm,
        child=left_pedestal,
        origin=Origin(xyz=LEFT_PEDESTAL_ORIGIN),
    )
    model.articulation(
        "palm_to_right_pedestal",
        ArticulationType.FIXED,
        parent=palm,
        child=right_pedestal,
        origin=Origin(xyz=RIGHT_PEDESTAL_ORIGIN),
    )

    model.articulation(
        "left_base_joint",
        ArticulationType.REVOLUTE,
        parent=left_pedestal,
        child=left_proximal,
        origin=Origin(xyz=left_base_joint_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.5, lower=-0.15, upper=1.25),
    )
    model.articulation(
        "left_middle_joint",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(0.0, left_lengths[0], 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "left_distal_joint",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(0.0, left_lengths[1], 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.5, velocity=3.0, lower=0.0, upper=1.30),
    )

    model.articulation(
        "right_base_joint",
        ArticulationType.REVOLUTE,
        parent=right_pedestal,
        child=right_proximal,
        origin=Origin(xyz=right_base_joint_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.6, lower=-0.12, upper=1.20),
    )
    model.articulation(
        "right_middle_joint",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(0.0, right_lengths[0], 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.5, velocity=2.8, lower=0.0, upper=1.38),
    )
    model.articulation(
        "right_distal_joint",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(0.0, right_lengths[1], 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.8, velocity=3.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")
    left_pedestal = object_model.get_part("left_pedestal")
    right_pedestal = object_model.get_part("right_pedestal")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_base_joint = object_model.get_articulation("left_base_joint")
    left_middle_joint = object_model.get_articulation("left_middle_joint")
    left_distal_joint = object_model.get_articulation("left_distal_joint")
    right_base_joint = object_model.get_articulation("right_base_joint")
    right_middle_joint = object_model.get_articulation("right_middle_joint")
    right_distal_joint = object_model.get_articulation("right_distal_joint")
    palm_to_left_pedestal = object_model.get_articulation("palm_to_left_pedestal")
    palm_to_right_pedestal = object_model.get_articulation("palm_to_right_pedestal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0007)
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

    expected_parts = {
        "palm",
        "left_pedestal",
        "right_pedestal",
        "left_proximal",
        "left_middle",
        "left_distal",
        "right_proximal",
        "right_middle",
        "right_distal",
    }
    ctx.check(
        "all_expected_parts_present",
        {part.name for part in object_model.parts} == expected_parts,
        f"got parts {[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "expected_articulation_count",
        len(object_model.articulations) == 8,
        f"got {len(object_model.articulations)} articulations",
    )
    ctx.check(
        "pedestal_mounts_are_fixed",
        palm_to_left_pedestal.articulation_type == ArticulationType.FIXED
        and palm_to_right_pedestal.articulation_type == ArticulationType.FIXED,
        "pedestals should be rigidly mounted to the split palm",
    )
    ctx.check(
        "left_chain_topology",
        left_base_joint.parent == "left_pedestal"
        and left_base_joint.child == "left_proximal"
        and left_middle_joint.parent == "left_proximal"
        and left_middle_joint.child == "left_middle"
        and left_distal_joint.parent == "left_middle"
        and left_distal_joint.child == "left_distal",
        "left finger chain should be independent and sequential",
    )
    ctx.check(
        "right_chain_topology",
        right_base_joint.parent == "right_pedestal"
        and right_base_joint.child == "right_proximal"
        and right_middle_joint.parent == "right_proximal"
        and right_middle_joint.child == "right_middle"
        and right_distal_joint.parent == "right_middle"
        and right_distal_joint.child == "right_distal",
        "right finger chain should be independent and sequential",
    )

    for joint in (
        left_base_joint,
        left_middle_joint,
        left_distal_joint,
        right_base_joint,
        right_middle_joint,
        right_distal_joint,
    ):
        ctx.check(
            f"{joint.name}_axis_is_x",
            tuple(round(value, 6) for value in joint.axis) == (1.0, 0.0, 0.0),
            f"{joint.name} axis was {joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_has_limits",
            limits is not None and limits.lower is not None and limits.upper is not None and limits.upper > limits.lower,
            f"{joint.name} limits were {limits}",
        )

    ctx.expect_gap(
        left_pedestal,
        palm,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="left_pedestal_seated_on_palm",
    )
    ctx.expect_gap(
        right_pedestal,
        palm,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="right_pedestal_seated_on_palm",
    )
    ctx.expect_overlap(
        left_pedestal,
        palm,
        axes="xy",
        min_overlap=0.018,
        name="left_pedestal_footprint_supported",
    )
    ctx.expect_overlap(
        right_pedestal,
        palm,
        axes="xy",
        min_overlap=0.016,
        name="right_pedestal_footprint_supported",
    )
    ctx.expect_origin_gap(
        right_pedestal,
        left_pedestal,
        axis="x",
        min_gap=0.050,
        max_gap=0.065,
        name="pedestals_side_by_side",
    )
    ctx.expect_origin_gap(
        right_proximal,
        left_proximal,
        axis="x",
        min_gap=0.050,
        max_gap=0.065,
        name="finger_chains_side_by_side",
    )
    ctx.expect_contact(
        left_pedestal,
        left_proximal,
        contact_tol=0.0007,
        name="left_base_knuckle_supported",
    )
    ctx.expect_contact(
        left_proximal,
        left_middle,
        contact_tol=0.0007,
        name="left_middle_knuckle_supported",
    )
    ctx.expect_contact(
        left_middle,
        left_distal,
        contact_tol=0.0007,
        name="left_distal_knuckle_supported",
    )
    ctx.expect_contact(
        right_pedestal,
        right_proximal,
        contact_tol=0.0007,
        name="right_base_knuckle_supported",
    )
    ctx.expect_contact(
        right_proximal,
        right_middle,
        contact_tol=0.0007,
        name="right_middle_knuckle_supported",
    )
    ctx.expect_contact(
        right_middle,
        right_distal,
        contact_tol=0.0007,
        name="right_distal_knuckle_supported",
    )

    left_distal_rest = ctx.part_world_position(left_distal)
    right_distal_rest = ctx.part_world_position(right_distal)
    ctx.check(
        "left_chain_reaches_farther_than_right",
        left_distal_rest is not None
        and right_distal_rest is not None
        and left_distal_rest[1] > right_distal_rest[1] + 0.015,
        f"left distal rest={left_distal_rest}, right distal rest={right_distal_rest}",
    )

    with ctx.pose(
        {
            left_base_joint: 0.55,
            left_middle_joint: 0.80,
            left_distal_joint: 0.58,
        }
    ):
        left_distal_flex = ctx.part_world_position(left_distal)
        right_distal_static = ctx.part_world_position(right_distal)

    ctx.check(
        "left_finger_curls_in_bending_plane",
        left_distal_rest is not None
        and left_distal_flex is not None
        and left_distal_flex[1] < left_distal_rest[1] - 0.030
        and left_distal_flex[2] > left_distal_rest[2] + 0.015,
        f"left distal rest={left_distal_rest}, flexed={left_distal_flex}",
    )
    ctx.check(
        "right_chain_ignores_left_finger_pose",
        right_distal_rest is not None
        and right_distal_static is not None
        and max(abs(a - b) for a, b in zip(right_distal_rest, right_distal_static)) < 1e-9,
        f"right distal changed from {right_distal_rest} to {right_distal_static}",
    )

    with ctx.pose(
        {
            right_base_joint: 0.48,
            right_middle_joint: 0.72,
            right_distal_joint: 0.50,
        }
    ):
        right_distal_flex = ctx.part_world_position(right_distal)
        left_distal_static = ctx.part_world_position(left_distal)

    ctx.check(
        "right_finger_curls_in_bending_plane",
        right_distal_rest is not None
        and right_distal_flex is not None
        and right_distal_flex[1] < right_distal_rest[1] - 0.020
        and right_distal_flex[2] > right_distal_rest[2] + 0.010,
        f"right distal rest={right_distal_rest}, flexed={right_distal_flex}",
    )
    ctx.check(
        "left_chain_ignores_right_finger_pose",
        left_distal_rest is not None
        and left_distal_static is not None
        and max(abs(a - b) for a, b in zip(left_distal_rest, left_distal_static)) < 1e-9,
        f"left distal changed from {left_distal_rest} to {left_distal_static}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
