from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _annular_ring_mesh(outer_radius: float, inner_radius: float, length: float):
    half_length = length / 2.0
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_length), (outer_radius, half_length)],
        [(inner_radius, -half_length), (inner_radius, half_length)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    body_w = 0.76
    body_d = 0.66
    body_h = 0.85
    base_h = 0.055
    shell_t = 0.018
    front_t = 0.022
    opening_z = 0.41
    opening_r = 0.255

    drum_r = 0.242
    drum_len = 0.47
    drum_wall = 0.010
    drum_center_y = -0.045

    door_outer_r = 0.285
    door_window_r = 0.228
    door_glass_r = 0.215
    door_t = 0.065
    door_hinge_x = 0.305
    door_axis_y = body_d / 2.0 + door_t / 2.0 + 0.0255
    door_face_offset_y = -0.025

    knob_r = 0.045
    knob_t = 0.03

    model = ArticulatedObject(name="condenser_dryer", assets=ASSETS)

    cabinet_grey = model.material("cabinet_grey", rgba=(0.65, 0.67, 0.70, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.58, 0.67, 0.77, 0.30))
    drum_metal = model.material("drum_metal", rgba=(0.77, 0.79, 0.81, 1.0))
    shadow_dark = model.material("shadow_dark", rgba=(0.08, 0.09, 0.11, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.84, 0.86, 0.88, 1.0))

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, base_h)),
        origin=Origin(xyz=(0.0, 0.0, base_h / 2.0)),
        material=cabinet_grey,
        name="base_plinth",
    )
    body.visual(
        Box((shell_t, body_d, body_h - base_h)),
        origin=Origin(xyz=(-(body_w - shell_t) / 2.0, 0.0, base_h + (body_h - base_h) / 2.0)),
        material=cabinet_grey,
        name="left_wall",
    )
    body.visual(
        Box((shell_t, body_d, body_h - base_h)),
        origin=Origin(xyz=((body_w - shell_t) / 2.0, 0.0, base_h + (body_h - base_h) / 2.0)),
        material=cabinet_grey,
        name="right_wall",
    )
    body.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - shell_t / 2.0)),
        material=cabinet_grey,
        name="top_shell",
    )
    body.visual(
        Box((body_w - 2.0 * shell_t, shell_t, body_h - base_h)),
        origin=Origin(
            xyz=(0.0, -(body_d - shell_t) / 2.0, base_h + (body_h - base_h) / 2.0),
        ),
        material=cabinet_grey,
        name="back_panel",
    )
    body.visual(
        Box((body_w, front_t, body_h - (opening_z + opening_r))),
        origin=Origin(
            xyz=(
                0.0,
                body_d / 2.0 - front_t / 2.0,
                opening_z + opening_r + (body_h - (opening_z + opening_r)) / 2.0,
            ),
        ),
        material=cabinet_grey,
        name="front_upper",
    )
    body.visual(
        Box((body_w, front_t, opening_z - opening_r - base_h)),
        origin=Origin(
            xyz=(
                0.0,
                body_d / 2.0 - front_t / 2.0,
                base_h + (opening_z - opening_r - base_h) / 2.0,
            ),
        ),
        material=cabinet_grey,
        name="front_lower",
    )
    body.visual(
        Box(((body_w - 2.0 * opening_r) / 2.0, front_t, 2.0 * opening_r)),
        origin=Origin(
            xyz=(
                -opening_r - (body_w - 2.0 * opening_r) / 4.0,
                body_d / 2.0 - front_t / 2.0,
                opening_z,
            ),
        ),
        material=cabinet_grey,
        name="front_left",
    )
    body.visual(
        Box(((body_w - 2.0 * opening_r) / 2.0, front_t, 2.0 * opening_r)),
        origin=Origin(
            xyz=(
                opening_r + (body_w - 2.0 * opening_r) / 4.0,
                body_d / 2.0 - front_t / 2.0,
                opening_z,
            ),
        ),
        material=cabinet_grey,
        name="front_right",
    )
    body.visual(
        Box((0.35, 0.018, 0.10)),
        origin=Origin(xyz=(0.115, body_d / 2.0 - 0.009, 0.76)),
        material=trim_dark,
        name="control_fascia",
    )
    body.visual(
        Box((0.11, 0.018, 0.07)),
        origin=Origin(xyz=(-0.25, body_d / 2.0 - 0.009, 0.765)),
        material=cabinet_grey,
        name="condenser_drawer_face",
    )
    body.visual(
        Box((0.020, 0.069, 0.20)),
        origin=Origin(xyz=(door_hinge_x, 0.3535, opening_z)),
        material=trim_dark,
        name="hinge_mount",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, -0.303, opening_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="rear_bearing",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    drum = model.part("drum")
    drum.visual(
        _mesh("drum_shell.obj", _annular_ring_mesh(drum_r, drum_r - drum_wall, drum_len)),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=drum_r - 0.004, length=0.014),
        origin=Origin(xyz=(0.0, drum_len / 2.0 - 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="front_lip",
    )
    drum.visual(
        Cylinder(radius=drum_r, length=0.024),
        origin=Origin(xyz=(0.0, -0.225, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_metal,
        name="rear_wall",
    )
    drum.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, -0.239, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="rear_hub",
    )
    for index, angle in enumerate((0.0, math.tau / 3.0, 2.0 * math.tau / 3.0)):
        drum.visual(
            Box((0.03, 0.34, 0.05)),
            origin=Origin(
                xyz=(0.220 * math.sin(angle), 0.0, 0.220 * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=drum_metal,
            name=f"lifter_{index}",
        )
    drum.inertial = Inertial.from_geometry(Cylinder(radius=drum_r, length=drum_len), mass=11.0)

    door = model.part("door")
    door.visual(
        _mesh("door_bezel.obj", _annular_ring_mesh(door_outer_r, door_window_r, door_t)),
        origin=Origin(xyz=(-door_hinge_x, door_face_offset_y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="outer_bezel",
    )
    door.visual(
        _mesh("door_retainer.obj", _annular_ring_mesh(0.242, 0.205, 0.018)),
        origin=Origin(xyz=(-door_hinge_x, door_face_offset_y - 0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="inner_retainer",
    )
    door.visual(
        Cylinder(radius=door_glass_r, length=0.01),
        origin=Origin(xyz=(-door_hinge_x, door_face_offset_y - 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_knuckle",
    )
    door.visual(
        Box((0.022, 0.020, 0.18)),
        origin=Origin(xyz=(-0.021, -0.010, 0.0)),
        material=trim_dark,
        name="hinge_boss",
    )
    door.visual(
        Box((0.05, 0.018, 0.18)),
        origin=Origin(xyz=(-0.51, door_face_offset_y + 0.014, 0.0)),
        material=trim_dark,
        name="handle_trim",
    )
    door.visual(
        Box((0.028, 0.026, 0.11)),
        origin=Origin(xyz=(-0.515, door_face_offset_y - 0.014, 0.0)),
        material=shadow_dark,
        name="handle_cavity",
    )
    door.inertial = Inertial.from_geometry(Cylinder(radius=door_outer_r, length=door_t), mass=3.5)

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=knob_r, length=knob_t),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_silver,
        name="knob_disc",
    )
    selector_knob.visual(
        Box((0.008, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.004, 0.032)),
        material=trim_dark,
        name="pointer",
    )
    selector_knob.inertial = Inertial.from_geometry(Cylinder(radius=knob_r, length=knob_t), mass=0.15)

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, drum_center_y, opening_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=10.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_axis_y, opening_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.55),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.22, body_d / 2.0 + knob_t / 2.0, 0.765)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-2.4, upper=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    selector_knob = object_model.get_part("selector_knob")

    drum_axle = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")
    knob_joint = object_model.get_articulation("body_to_selector_knob")

    front_upper = body.get_visual("front_upper")
    hinge_mount = body.get_visual("hinge_mount")
    rear_bearing = body.get_visual("rear_bearing")
    control_fascia = body.get_visual("control_fascia")

    front_lip = drum.get_visual("front_lip")
    rear_hub = drum.get_visual("rear_hub")
    lifter_0 = drum.get_visual("lifter_0")

    outer_bezel = door.get_visual("outer_bezel")
    door_glass = door.get_visual("door_glass")
    hinge_knuckle = door.get_visual("hinge_knuckle")
    hinge_boss = door.get_visual("hinge_boss")
    handle_trim = door.get_visual("handle_trim")
    handle_cavity = door.get_visual("handle_cavity")

    knob_disc = selector_knob.get_visual("knob_disc")
    pointer = selector_knob.get_visual("pointer")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        door,
        body,
        elem_a=hinge_knuckle,
        elem_b=hinge_mount,
        reason="Door hinge knuckle wraps the cabinet-side hinge bracket with a small pinned overlap.",
    )
    ctx.allow_overlap(
        door,
        body,
        elem_a=outer_bezel,
        elem_b=hinge_mount,
        reason="The molded door bezel includes an integrated right-side hinge carrier that wraps around the cabinet hinge pin through the swing range.",
    )
    ctx.allow_overlap(
        drum,
        body,
        elem_a=rear_hub,
        elem_b=rear_bearing,
        reason="The rear drum hub seats slightly into the cabinet bearing support.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=48, ignore_adjacent=False, ignore_fixed=True)

    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    body_size = tuple(body_aabb[1][idx] - body_aabb[0][idx] for idx in range(3))
    ctx.check(
        "body_realistic_appliance_size",
        body_size[0] >= 0.72 and body_size[1] >= 0.62 and body_size[2] >= 0.82,
        details=f"body dimensions were {body_size}",
    )

    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem=outer_bezel,
        negative_elem=front_upper,
    )
    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.54, elem_a=outer_bezel)
    ctx.expect_overlap(door, drum, axes="xz", min_overlap=0.40, elem_a=door_glass, elem_b=front_lip)
    ctx.expect_gap(
        door,
        drum,
        axis="y",
        min_gap=0.10,
        positive_elem=door_glass,
        negative_elem=front_lip,
    )
    ctx.expect_contact(door, body, elem_a=hinge_knuckle, elem_b=hinge_mount)
    ctx.expect_contact(door, door, elem_a=hinge_knuckle, elem_b=hinge_boss)

    ctx.expect_contact(drum, body, elem_a=rear_hub, elem_b=rear_bearing)
    ctx.expect_within(drum, body, axes="xz", margin=0.02)

    ctx.expect_within(door, door, axes="xz", inner_elem=handle_cavity, outer_elem=handle_trim)
    ctx.expect_gap(
        door,
        door,
        axis="y",
        min_gap=0.003,
        positive_elem=handle_trim,
        negative_elem=handle_cavity,
    )
    ctx.expect_gap(
        door,
        door,
        axis="x",
        min_gap=0.45,
        positive_elem=hinge_knuckle,
        negative_elem=handle_trim,
    )

    ctx.expect_contact(selector_knob, body, elem_a=knob_disc, elem_b=control_fascia)
    ctx.expect_gap(
        selector_knob,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=knob_disc,
        negative_elem=control_fascia,
    )

    door_limits = door_hinge.motion_limits
    assert door_limits is not None
    assert door_limits.upper is not None

    closed_handle_aabb = ctx.part_element_world_aabb(door, elem=handle_trim)
    assert closed_handle_aabb is not None
    with ctx.pose({door_hinge: door_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.expect_contact(door, body, elem_a=hinge_knuckle, elem_b=hinge_mount)
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=0.18,
            positive_elem=handle_trim,
        )
        open_handle_aabb = ctx.part_element_world_aabb(door, elem=handle_trim)
        assert open_handle_aabb is not None
        ctx.check(
            "door_swings_outward_from_right_hinge",
            open_handle_aabb[1][1] > closed_handle_aabb[1][1] + 0.20,
            details=f"closed handle max y={closed_handle_aabb[1][1]:.4f}, open handle max y={open_handle_aabb[1][1]:.4f}",
        )

    drum_lifter_rest = ctx.part_element_world_aabb(drum, elem=lifter_0)
    assert drum_lifter_rest is not None
    with ctx.pose({drum_axle: math.pi / 2.0}):
        ctx.expect_contact(drum, body, elem_a=rear_hub, elem_b=rear_bearing)
        drum_lifter_quarter = ctx.part_element_world_aabb(drum, elem=lifter_0)
        assert drum_lifter_quarter is not None
        ctx.check(
            "drum_rotates_about_axle",
            drum_lifter_quarter[1][0] > drum_lifter_rest[1][0] + 0.16,
            details=f"rest lifter max x={drum_lifter_rest[1][0]:.4f}, quarter-turn max x={drum_lifter_quarter[1][0]:.4f}",
        )

    knob_pointer_rest = ctx.part_element_world_aabb(selector_knob, elem=pointer)
    assert knob_pointer_rest is not None
    with ctx.pose({knob_joint: 1.3}):
        ctx.expect_contact(selector_knob, body, elem_a=knob_disc, elem_b=control_fascia)
        knob_pointer_turned = ctx.part_element_world_aabb(selector_knob, elem=pointer)
        assert knob_pointer_turned is not None
        ctx.check(
            "selector_knob_rotates",
            knob_pointer_turned[1][0] > knob_pointer_rest[1][0] + 0.02,
            details=f"rest pointer max x={knob_pointer_rest[1][0]:.4f}, turned pointer max x={knob_pointer_turned[1][0]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
