from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _build_knob_part(part, *, knob_material, steel_material) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_material,
        name="shaft_stub",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_material,
        name="collar",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.0, 0.038, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_body",
    )
    part.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(0.0, 0.053, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="grip_skirt",
    )
    part.visual(
        Box((0.006, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.053, 0.022)),
        material=steel_material,
        name="pointer",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.066, 0.060, 0.066)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_toaster_oven")

    cabinet = model.material("cabinet", rgba=(0.24, 0.25, 0.27, 1.0))
    panel = model.material("panel", rgba=(0.60, 0.62, 0.64, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.37, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.12, 0.13, 1.0))
    glass = model.material("glass", rgba=(0.54, 0.67, 0.76, 0.28))
    rail_steel = model.material("rail_steel", rgba=(0.68, 0.69, 0.71, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    width = 0.56
    depth = 0.43
    height = 0.34
    foot_h = 0.018
    base_t = 0.018
    roof_t = 0.018
    side_t = 0.016
    front_t = 0.024
    rear_t = 0.018
    front_y = depth * 0.5
    front_face_y = front_y - front_t * 0.5
    rear_y = -depth * 0.5
    rear_face_y = rear_y + rear_t * 0.5
    shell_bottom = foot_h + base_t
    shell_top = height - roof_t

    door_x = -0.055
    door_w = 0.424
    door_h = 0.226
    hinge_z = 0.056
    hinge_y = 0.208

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )
    body.visual(
        Box((width, depth, base_t)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + base_t * 0.5)),
        material=cabinet,
        name="base_plate",
    )
    for idx, (fx, fy) in enumerate(
        (
            (-0.215, -0.145),
            (0.215, -0.145),
            (-0.215, 0.145),
            (0.215, 0.145),
        )
    ):
        body.visual(
            Box((0.058, 0.070, foot_h)),
            origin=Origin(xyz=(fx, fy, foot_h * 0.5)),
            material=rubber,
            name=f"foot_{idx}",
        )
    body.visual(
        Box((side_t, depth - 0.036, shell_top - shell_bottom)),
        origin=Origin(
            xyz=(
                -width * 0.5 + side_t * 0.5,
                0.0,
                (shell_bottom + shell_top) * 0.5,
            )
        ),
        material=cabinet,
        name="left_wall",
    )
    body.visual(
        Box((side_t, depth - 0.036, shell_top - shell_bottom)),
        origin=Origin(
            xyz=(
                width * 0.5 - side_t * 0.5,
                0.0,
                (shell_bottom + shell_top) * 0.5,
            )
        ),
        material=cabinet,
        name="right_wall",
    )
    body.visual(
        Box((width, depth, roof_t)),
        origin=Origin(xyz=(0.0, 0.0, height - roof_t * 0.5)),
        material=cabinet,
        name="roof_panel",
    )
    body.visual(
        Box((0.52, rear_t, 0.034)),
        origin=Origin(xyz=(0.0, rear_face_y, 0.305)),
        material=panel,
        name="rear_top_bar",
    )
    body.visual(
        Box((0.050, rear_t, 0.246)),
        origin=Origin(xyz=(-0.251, rear_face_y, 0.199)),
        material=panel,
        name="rear_left_post",
    )
    body.visual(
        Box((0.050, rear_t, 0.246)),
        origin=Origin(xyz=(0.251, rear_face_y, 0.199)),
        material=panel,
        name="rear_right_post",
    )
    body.visual(
        Box((0.52, rear_t, 0.040)),
        origin=Origin(xyz=(0.0, rear_face_y, 0.056)),
        material=panel,
        name="rear_lower_bar",
    )
    body.visual(
        Box((0.424, front_t, 0.046)),
        origin=Origin(xyz=(door_x, front_face_y, 0.299)),
        material=panel,
        name="front_lintel",
    )
    body.visual(
        Box((0.022, front_t, 0.266)),
        origin=Origin(xyz=(-0.272, front_face_y, 0.189)),
        material=panel,
        name="front_left_jamb",
    )
    body.visual(
        Box((0.022, front_t, 0.304)),
        origin=Origin(xyz=(0.162, front_face_y, 0.170)),
        material=panel,
        name="center_stile",
    )
    body.visual(
        Box((0.118, front_t, 0.304)),
        origin=Origin(xyz=(0.221, front_face_y, 0.170)),
        material=panel,
        name="control_panel",
    )
    body.visual(
        Box((0.062, front_t, 0.038)),
        origin=Origin(xyz=(-0.236, front_face_y, 0.037)),
        material=panel,
        name="left_sill_block",
    )
    body.visual(
        Box((0.212, front_t, 0.038)),
        origin=Origin(xyz=(-0.031, front_face_y, 0.037)),
        material=panel,
        name="center_sill_block",
    )
    body.visual(
        Box((0.022, front_t, 0.038)),
        origin=Origin(xyz=(0.146, front_face_y, 0.037)),
        material=panel,
        name="right_sill_block",
    )
    body.visual(
        Box((0.018, 0.260, 0.018)),
        origin=Origin(xyz=(-0.255, -0.010, 0.156)),
        material=rail_steel,
        name="left_rail",
    )
    body.visual(
        Box((0.018, 0.260, 0.018)),
        origin=Origin(xyz=(0.255, -0.010, 0.156)),
        material=rail_steel,
        name="right_rail",
    )
    for boss_name, boss_z in (("top_boss", 0.268), ("middle_boss", 0.192), ("bottom_boss", 0.116)):
        body.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(0.221, front_y - 0.006, boss_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=boss_name,
        )
    body.visual(
        Box((0.014, 0.020, 0.056)),
        origin=Origin(xyz=(door_x - 0.1345, 0.189, 0.028)),
        material=dark_steel,
        name="left_hinge_ear_outer",
    )
    body.visual(
        Box((0.014, 0.020, 0.056)),
        origin=Origin(xyz=(door_x - 0.0975, 0.189, 0.028)),
        material=dark_steel,
        name="left_hinge_ear_inner",
    )
    body.visual(
        Box((0.014, 0.020, 0.056)),
        origin=Origin(xyz=(door_x + 0.1455, 0.189, 0.028)),
        material=dark_steel,
        name="right_hinge_ear_inner",
    )
    body.visual(
        Box((0.014, 0.020, 0.056)),
        origin=Origin(xyz=(door_x + 0.1825, 0.189, 0.028)),
        material=dark_steel,
        name="right_hinge_ear_outer",
    )
    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.464, 0.006, 0.228)),
        material=panel,
        name="panel_sheet",
    )
    service_panel.visual(
        Box((0.028, 0.012, 0.178)),
        origin=Origin(xyz=(-0.146, -0.008, 0.0)),
        material=dark_steel,
        name="left_rib",
    )
    service_panel.visual(
        Box((0.028, 0.012, 0.178)),
        origin=Origin(xyz=(0.146, -0.008, 0.0)),
        material=dark_steel,
        name="right_rib",
    )
    service_panel.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(-0.052, -0.017, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="handle_left_post",
    )
    service_panel.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(0.052, -0.017, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="handle_right_post",
    )
    service_panel.visual(
        Cylinder(radius=0.008, length=0.160),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="service_handle",
    )
    service_panel.inertial = Inertial.from_geometry(
        Box((0.464, 0.050, 0.228)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((0.424, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, 0.018, 0.020)),
        material=dark_steel,
        name="bottom_rail",
    )
    door.visual(
        Box((0.360, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, 0.018, 0.206)),
        material=dark_steel,
        name="top_rail",
    )
    door.visual(
        Box((0.032, 0.022, 0.186)),
        origin=Origin(xyz=(-0.196, 0.018, 0.133)),
        material=dark_steel,
        name="left_stile",
    )
    door.visual(
        Box((0.032, 0.022, 0.186)),
        origin=Origin(xyz=(0.196, 0.018, 0.133)),
        material=dark_steel,
        name="right_stile",
    )
    door.visual(
        Box((0.310, 0.006, 0.138)),
        origin=Origin(xyz=(0.0, 0.010, 0.124)),
        material=glass,
        name="glass_panel",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.080),
        origin=Origin(xyz=(-0.135, 0.047, 0.184), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="left_handle_post",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.080),
        origin=Origin(xyz=(0.135, 0.047, 0.184), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="right_handle_post",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.290),
        origin=Origin(xyz=(0.0, 0.078, 0.184), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="handle_bar",
    )
    door.visual(
        Box((0.052, 0.014, 0.060)),
        origin=Origin(xyz=(-0.116, 0.010, 0.030)),
        material=dark_steel,
        name="left_hinge_leaf",
    )
    door.visual(
        Box((0.052, 0.014, 0.060)),
        origin=Origin(xyz=(0.164, 0.010, 0.030)),
        material=dark_steel,
        name="right_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.116, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="left_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.164, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="right_hinge_barrel",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.070, door_h)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.032, door_h * 0.5)),
    )

    knob_top = model.part("knob_top")
    _build_knob_part(knob_top, knob_material=knob_black, steel_material=stainless)
    knob_mid = model.part("knob_middle")
    _build_knob_part(knob_mid, knob_material=knob_black, steel_material=stainless)
    knob_bottom = model.part("knob_bottom")
    _build_knob_part(knob_bottom, knob_material=knob_black, steel_material=stainless)

    model.articulation(
        "body_to_service_panel",
        ArticulationType.FIXED,
        parent=body,
        child=service_panel,
        origin=Origin(xyz=(0.0, rear_y - 0.003, 0.182)),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_x, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    for child, joint_name, z_pos in (
        (knob_top, "body_to_knob_top", 0.268),
        (knob_mid, "body_to_knob_middle", 0.192),
        (knob_bottom, "body_to_knob_bottom", 0.116),
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=child,
            origin=Origin(xyz=(0.221, front_y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    service_panel = object_model.get_part("service_panel")
    knob_top = object_model.get_part("knob_top")
    knob_mid = object_model.get_part("knob_middle")
    knob_bottom = object_model.get_part("knob_bottom")
    door_hinge = object_model.get_articulation("body_to_door")
    knob_joints = [
        object_model.get_articulation("body_to_knob_top"),
        object_model.get_articulation("body_to_knob_middle"),
        object_model.get_articulation("body_to_knob_bottom"),
    ]

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
        "door_hinge_axis_is_bottom_pivot",
        door_hinge.axis == (-1.0, 0.0, 0.0),
        f"expected (-1, 0, 0), got {door_hinge.axis}",
    )
    ctx.check(
        "knob_axes_are_shaft_aligned",
        all(joint.axis == (0.0, 1.0, 0.0) for joint in knob_joints),
        "all control shafts should rotate about the front-to-back axis",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="left_stile",
        elem_b="front_left_jamb",
        name="door_left_stile_contacts_jamb",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="right_stile",
        elem_b="center_stile",
        name="door_right_stile_contacts_center_stile",
    )
    ctx.expect_contact(
        service_panel,
        body,
        elem_a="panel_sheet",
        elem_b="rear_top_bar",
        name="service_panel_covers_rear_frame",
    )
    ctx.expect_contact(
        knob_top,
        body,
        elem_a="shaft_stub",
        elem_b="top_boss",
        name="top_knob_is_supported_by_boss",
    )
    ctx.expect_contact(
        knob_mid,
        body,
        elem_a="shaft_stub",
        elem_b="middle_boss",
        name="middle_knob_is_supported_by_boss",
    )
    ctx.expect_contact(
        knob_bottom,
        body,
        elem_a="shaft_stub",
        elem_b="bottom_boss",
        name="bottom_knob_is_supported_by_boss",
    )

    with ctx.pose({door_hinge: 1.35}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="handle_bar",
            negative_elem="front_lintel",
            min_gap=0.09,
            name="door_handle_swings_clear_of_front_face",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
