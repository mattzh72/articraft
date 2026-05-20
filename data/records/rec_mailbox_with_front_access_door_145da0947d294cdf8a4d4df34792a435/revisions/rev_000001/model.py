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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_LENGTH = 0.52
BODY_WIDTH = 0.22
BODY_SHOULDER_HEIGHT = 0.15
BODY_HEIGHT = 0.26
WALL_THICKNESS = 0.012
DOOR_DEPTH = 0.018
POST_HEIGHT = 1.15
BODY_MOUNT_X = 0.34
DOOR_PANEL_THICKNESS = 0.010
HINGE_RADIUS = 0.006


def arched_mailbox_profile(
    width: float,
    shoulder_height: float,
    *,
    bottom_z: float = 0.0,
    arc_segments: int = 18,
) -> list[tuple[float, float]]:
    half_width = width / 2.0
    points = [(-half_width, bottom_z), (half_width, bottom_z), (half_width, shoulder_height)]
    for index in range(1, arc_segments):
        angle = math.pi * index / arc_segments
        points.append(
            (
                half_width * math.cos(angle),
                shoulder_height + half_width * math.sin(angle),
            )
        )
    points.append((-half_width, shoulder_height))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curbside_mailbox")

    post_material = model.material("post_material", rgba=(0.39, 0.28, 0.18, 1.0))
    mailbox_material = model.material("mailbox_material", rgba=(0.72, 0.74, 0.77, 1.0))
    hardware_material = model.material("hardware_material", rgba=(0.18, 0.20, 0.22, 1.0))
    flag_material = model.material("flag_material", rgba=(0.82, 0.10, 0.10, 1.0))

    outer_profile = arched_mailbox_profile(BODY_WIDTH, BODY_SHOULDER_HEIGHT)
    inner_profile = arched_mailbox_profile(
        BODY_WIDTH - 2.0 * WALL_THICKNESS,
        BODY_SHOULDER_HEIGHT,
        bottom_z=WALL_THICKNESS,
    )

    post_assembly = model.part("post_assembly")
    post_assembly.visual(
        Box((0.10, 0.10, POST_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT / 2.0)),
        material=post_material,
        name="post",
    )
    post_assembly.visual(
        Box((0.36, 0.08, 0.05)),
        origin=Origin(xyz=(0.18, 0.0, 0.94)),
        material=post_material,
        name="arm",
    )
    post_assembly.visual(
        Box((0.16, 0.18, 0.28)),
        origin=Origin(xyz=(0.44, 0.0, 0.94)),
        material=post_material,
        name="rear_support",
    )
    post_assembly.inertial = Inertial.from_geometry(
        Box((0.60, 0.18, POST_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.24, 0.0, POST_HEIGHT / 2.0)),
    )

    mailbox_body = model.part("mailbox_body")
    body_shell_geometry = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        BODY_LENGTH,
        cap=False,
        center=True,
        closed=True,
    )
    body_shell_geometry.rotate_z(math.pi / 2.0).rotate_y(math.pi / 2.0).translate(BODY_LENGTH / 2.0, 0.0, 0.0)
    mailbox_body.visual(
        mesh_from_geometry(body_shell_geometry, "mailbox_shell_v2"),
        material=mailbox_material,
        name="shell",
    )

    rear_cap_geometry = ExtrudeGeometry(outer_profile, WALL_THICKNESS, cap=True, center=True, closed=True)
    rear_cap_geometry.rotate_z(math.pi / 2.0).rotate_y(math.pi / 2.0).translate(
        BODY_LENGTH - WALL_THICKNESS / 2.0,
        0.0,
        0.0,
    )
    mailbox_body.visual(
        mesh_from_geometry(rear_cap_geometry, "mailbox_rear_cap_v2"),
        material=mailbox_material,
        name="rear_cap",
    )
    mailbox_body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(BODY_LENGTH / 2.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    model.articulation(
        "post_to_body",
        ArticulationType.FIXED,
        parent=post_assembly,
        child=mailbox_body,
        origin=Origin(xyz=(BODY_MOUNT_X, 0.0, 1.08)),
    )

    front_door = model.part("front_door")
    door_profile = arched_mailbox_profile(BODY_WIDTH - 0.010, BODY_SHOULDER_HEIGHT - 0.005, bottom_z=0.0)
    door_geometry = ExtrudeGeometry(door_profile, DOOR_PANEL_THICKNESS, cap=True, center=True, closed=True)
    door_geometry.rotate_z(math.pi / 2.0).rotate_y(math.pi / 2.0).translate(
        HINGE_RADIUS - DOOR_PANEL_THICKNESS / 2.0,
        0.0,
        -HINGE_RADIUS,
    )
    front_door.visual(
        mesh_from_geometry(door_geometry, "mailbox_front_door_v3"),
        material=mailbox_material,
        name="door_panel",
    )
    front_door.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_WIDTH - 0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="door_hinge",
    )
    front_door.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(-0.008, 0.0, 0.102), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_material,
        name="door_handle",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((0.04, BODY_WIDTH - 0.008, BODY_HEIGHT)),
        mass=0.85,
        origin=Origin(xyz=(-0.020, 0.0, BODY_HEIGHT / 2.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=mailbox_body,
        child=front_door,
        origin=Origin(xyz=(-HINGE_RADIUS, 0.0, HINGE_RADIUS)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    signal_flag = model.part("signal_flag")
    signal_flag.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="flag_hub",
    )
    signal_flag.visual(
        Box((0.11, 0.004, 0.012)),
        origin=Origin(xyz=(0.055, 0.014, 0.0)),
        material=flag_material,
        name="flag_bar",
    )
    signal_flag.visual(
        Box((0.055, 0.004, 0.09)),
        origin=Origin(xyz=(0.135, 0.014, 0.045)),
        material=flag_material,
        name="flag_plate",
    )
    signal_flag.inertial = Inertial.from_geometry(
        Box((0.18, 0.02, 0.10)),
        mass=0.22,
        origin=Origin(xyz=(0.090, 0.010, 0.040)),
    )

    model.articulation(
        "body_to_flag",
        ArticulationType.REVOLUTE,
        parent=mailbox_body,
        child=signal_flag,
        origin=Origin(xyz=(0.14, BODY_WIDTH / 2.0, 0.135)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post_assembly = object_model.get_part("post_assembly")
    mailbox_body = object_model.get_part("mailbox_body")
    front_door = object_model.get_part("front_door")
    signal_flag = object_model.get_part("signal_flag")

    body_shell = mailbox_body.get_visual("shell")
    rear_support = post_assembly.get_visual("rear_support")
    door_panel = front_door.get_visual("door_panel")
    door_hinge = front_door.get_visual("door_hinge")
    flag_hub = signal_flag.get_visual("flag_hub")
    flag_plate = signal_flag.get_visual("flag_plate")

    body_to_door = object_model.get_articulation("body_to_door")
    body_to_flag = object_model.get_articulation("body_to_flag")

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

    ctx.expect_contact(mailbox_body, post_assembly, elem_a=body_shell, elem_b=rear_support, name="body_mounted_to_post")
    ctx.expect_gap(
        mailbox_body,
        front_door,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=body_shell,
        negative_elem=door_hinge,
        name="door_hinge_aligned_with_front_rim",
    )
    ctx.expect_overlap(
        front_door,
        mailbox_body,
        axes="y",
        min_overlap=0.18,
        elem_a=door_hinge,
        elem_b=body_shell,
        name="door_hinge_spans_front_rim",
    )
    ctx.expect_contact(signal_flag, mailbox_body, elem_a=flag_hub, elem_b=body_shell, name="flag_pivot_attached_to_side_wall")

    ctx.check(
        "door_axis_is_lower_edge_crossbar",
        abs(body_to_door.axis[0]) < 1e-9 and abs(abs(body_to_door.axis[1]) - 1.0) < 1e-9 and abs(body_to_door.axis[2]) < 1e-9,
        f"Unexpected door hinge axis: {body_to_door.axis}",
    )
    ctx.check(
        "flag_axis_is_side_pivot",
        abs(body_to_flag.axis[0]) < 1e-9 and abs(body_to_flag.axis[1] + 1.0) < 1e-9 and abs(body_to_flag.axis[2]) < 1e-9,
        f"Unexpected flag pivot axis: {body_to_flag.axis}",
    )
    ctx.expect_origin_gap(
        signal_flag,
        mailbox_body,
        axis="y",
        min_gap=(BODY_WIDTH / 2.0) - 1e-6,
        max_gap=(BODY_WIDTH / 2.0) + 1e-6,
        name="flag_pivot_is_on_right_side_wall",
    )

    body_aabb = ctx.part_world_aabb(mailbox_body)
    if body_aabb is None:
        ctx.fail("mailbox_body_has_geometry", "mailbox_body world AABB unavailable")
    else:
        body_dims = tuple(body_aabb[1][axis] - body_aabb[0][axis] for axis in range(3))
        ctx.check(
            "mailbox_body_length_realistic",
            0.48 <= body_dims[0] <= 0.56,
            f"Unexpected mailbox body length: {body_dims[0]:.3f} m",
        )
        ctx.check(
            "mailbox_body_width_realistic",
            0.20 <= body_dims[1] <= 0.24,
            f"Unexpected mailbox body width: {body_dims[1]:.3f} m",
        )
        ctx.check(
            "mailbox_body_height_realistic",
            0.24 <= body_dims[2] <= 0.28,
            f"Unexpected mailbox body height: {body_dims[2]:.3f} m",
        )

    post_aabb = ctx.part_world_aabb(post_assembly)
    if post_aabb is None:
        ctx.fail("post_assembly_has_geometry", "post_assembly world AABB unavailable")
    else:
        post_height = post_aabb[1][2] - post_aabb[0][2]
        ctx.check(
            "post_height_realistic",
            1.0 <= post_height <= 1.2,
            f"Unexpected post height: {post_height:.3f} m",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    with ctx.pose({body_to_door: 0.0}):
        ctx.expect_gap(
            mailbox_body,
            front_door,
            axis="x",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=body_shell,
            negative_elem=door_panel,
            name="door_closed_flush_to_front_opening",
        )
        ctx.expect_overlap(
            front_door,
            mailbox_body,
            axes="yz",
            min_overlap=0.20,
            elem_a=door_panel,
            elem_b=body_shell,
            name="door_closed_covers_mail_opening",
        )

    door_limits = body_to_door.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({body_to_door: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="door_lower_no_floating")

        closed_door_aabb = ctx.part_element_world_aabb(front_door, elem=door_panel.name)
        if closed_door_aabb is None:
            ctx.fail("door_closed_panel_aabb_available", "Closed door panel AABB unavailable")
        with ctx.pose({body_to_door: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_upper_no_floating")
            ctx.expect_gap(
                mailbox_body,
                front_door,
                axis="x",
                max_gap=0.0005,
                max_penetration=0.0,
                positive_elem=body_shell,
                negative_elem=door_hinge,
                name="door_upper_pose_hinge_alignment",
            )
            ctx.expect_overlap(
                front_door,
                mailbox_body,
                axes="y",
                min_overlap=0.18,
                elem_a=door_hinge,
                elem_b=body_shell,
                name="door_upper_pose_hinge_span",
            )
            open_door_aabb = ctx.part_element_world_aabb(front_door, elem=door_panel.name)
            if closed_door_aabb is None or open_door_aabb is None:
                ctx.fail("door_open_panel_aabb_available", "Open door panel AABB unavailable")
            else:
                ctx.check(
                    "door_swings_forward",
                    open_door_aabb[0][0] < closed_door_aabb[0][0] - 0.14,
                    f"Closed min x={closed_door_aabb[0][0]:.3f}, open min x={open_door_aabb[0][0]:.3f}",
                )
                ctx.check(
                    "door_swings_downward",
                    open_door_aabb[0][2] < closed_door_aabb[0][2] - 0.07,
                    f"Closed min z={closed_door_aabb[0][2]:.3f}, open min z={open_door_aabb[0][2]:.3f}",
                )

    flag_limits = body_to_flag.motion_limits
    if flag_limits is not None and flag_limits.lower is not None and flag_limits.upper is not None:
        with ctx.pose({body_to_flag: flag_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="flag_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="flag_lower_no_floating")
            ctx.expect_contact(
                signal_flag,
                mailbox_body,
                elem_a=flag_hub,
                elem_b=body_shell,
                name="flag_lower_pose_hub_contact",
            )

        closed_flag_aabb = ctx.part_element_world_aabb(signal_flag, elem=flag_plate.name)
        if closed_flag_aabb is None:
            ctx.fail("flag_closed_plate_aabb_available", "Closed flag plate AABB unavailable")
        with ctx.pose({body_to_flag: flag_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="flag_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="flag_upper_no_floating")
            ctx.expect_contact(
                signal_flag,
                mailbox_body,
                elem_a=flag_hub,
                elem_b=body_shell,
                name="flag_upper_pose_hub_contact",
            )
            raised_flag_aabb = ctx.part_element_world_aabb(signal_flag, elem=flag_plate.name)
            if closed_flag_aabb is None or raised_flag_aabb is None:
                ctx.fail("flag_raised_plate_aabb_available", "Raised flag plate AABB unavailable")
            else:
                ctx.check(
                    "flag_raises_upward",
                    raised_flag_aabb[1][2] > closed_flag_aabb[1][2] + 0.07,
                    f"Closed max z={closed_flag_aabb[1][2]:.3f}, raised max z={raised_flag_aabb[1][2]:.3f}",
                )
                ctx.check(
                    "flag_remains_outboard",
                    raised_flag_aabb[1][1] > (BODY_WIDTH / 2.0) + 0.012,
                    f"Raised flag max y={raised_flag_aabb[1][1]:.3f}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
