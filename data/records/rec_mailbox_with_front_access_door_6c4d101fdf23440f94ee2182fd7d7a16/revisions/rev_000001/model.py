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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _triangulate_fan(geom: MeshGeometry, loop_ids: list[int], *, reverse: bool) -> None:
    for index in range(1, len(loop_ids) - 1):
        if reverse:
            geom.add_face(loop_ids[0], loop_ids[index + 1], loop_ids[index])
        else:
            geom.add_face(loop_ids[0], loop_ids[index], loop_ids[index + 1])


def _bridge_loops(geom: MeshGeometry, loop_a: list[int], loop_b: list[int]) -> None:
    count = len(loop_a)
    for index in range(count):
        nxt = (index + 1) % count
        _add_quad(geom, loop_a[index], loop_a[nxt], loop_b[nxt], loop_b[index])


def _mailbox_profile(
    *,
    width: float,
    shoulder_height: float,
    bottom_z: float,
    arc_segments: int,
) -> list[tuple[float, float]]:
    half_width = width * 0.5
    profile: list[tuple[float, float]] = [
        (-half_width, bottom_z),
        (half_width, bottom_z),
        (half_width, shoulder_height),
    ]
    for step in range(1, arc_segments):
        angle = math.pi * step / arc_segments
        profile.append(
            (
                half_width * math.cos(angle),
                shoulder_height + half_width * math.sin(angle),
            )
        )
    profile.append((-half_width, shoulder_height))
    return profile


def _extrude_profile_along_x(
    profile: list[tuple[float, float]],
    *,
    x_min: float,
    x_max: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    back_loop = [geom.add_vertex(x_min, y, z) for y, z in profile]
    front_loop = [geom.add_vertex(x_max, y, z) for y, z in profile]
    _bridge_loops(geom, back_loop, front_loop)
    _triangulate_fan(geom, back_loop, reverse=True)
    _triangulate_fan(geom, front_loop, reverse=False)
    return geom


def _build_mailbox_shell(
    *,
    body_length: float,
    outer_width: float,
    shoulder_height: float,
    wall_thickness: float,
    arc_segments: int,
) -> MeshGeometry:
    front_x = body_length * 0.5
    back_x = -body_length * 0.5
    outer_profile = _mailbox_profile(
        width=outer_width,
        shoulder_height=shoulder_height,
        bottom_z=0.0,
        arc_segments=arc_segments,
    )
    inner_profile = _mailbox_profile(
        width=outer_width - 2.0 * wall_thickness,
        shoulder_height=shoulder_height,
        bottom_z=wall_thickness,
        arc_segments=arc_segments,
    )

    geom = MeshGeometry()
    outer_back = [geom.add_vertex(back_x, y, z) for y, z in outer_profile]
    outer_front = [geom.add_vertex(front_x, y, z) for y, z in outer_profile]
    inner_back = [geom.add_vertex(back_x, y, z) for y, z in inner_profile]
    inner_front = [geom.add_vertex(front_x, y, z) for y, z in inner_profile]

    _bridge_loops(geom, outer_back, outer_front)
    _bridge_loops(geom, inner_front, inner_back)
    _bridge_loops(geom, outer_front, inner_front)
    _bridge_loops(geom, inner_back, outer_back)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rural_mailbox")

    galvanized = model.material("galvanized_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    shadow_steel = model.material("shadow_steel", rgba=(0.33, 0.35, 0.37, 1.0))
    flag_red = model.material("flag_red", rgba=(0.78, 0.15, 0.11, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.53, 0.40, 0.28, 1.0))
    handle_black = model.material("handle_black", rgba=(0.12, 0.12, 0.13, 1.0))

    body_length = 0.53
    body_width = 0.205
    shoulder_height = 0.120
    wall_thickness = 0.0025
    body_height = shoulder_height + body_width * 0.5
    front_x = body_length * 0.5
    hinge_radius = 0.005
    door_thickness = 0.014

    post_mount = model.part("post_mount")
    post_mount.visual(
        Box((0.10, 0.10, 0.90)),
        origin=Origin(xyz=(-0.035, 0.0, 0.45)),
        material=weathered_wood,
        name="post",
    )
    post_mount.visual(
        Box((0.16, 0.12, 0.04)),
        origin=Origin(xyz=(0.000, 0.0, 0.92)),
        material=weathered_wood,
        name="cap_block",
    )
    post_mount.visual(
        Box((0.34, 0.14, 0.02)),
        origin=Origin(xyz=(0.030, 0.0, 0.95)),
        material=weathered_wood,
        name="mounting_plank",
    )
    post_mount.visual(
        Box((0.14, 0.12, 0.04)),
        origin=Origin(xyz=(0.030, 0.0, 0.98)),
        material=weathered_wood,
        name="saddle_block",
    )
    post_mount.inertial = Inertial.from_geometry(
        Box((0.34, 0.14, 1.00)),
        mass=7.5,
        origin=Origin(xyz=(0.020, 0.0, 0.50)),
    )

    shell_mesh = mesh_from_geometry(
        _build_mailbox_shell(
            body_length=body_length,
            outer_width=body_width,
            shoulder_height=shoulder_height,
            wall_thickness=wall_thickness,
            arc_segments=18,
        ),
        "mailbox_shell",
    )
    rear_panel_profile = _mailbox_profile(
        width=body_width - 2.0 * wall_thickness + 0.0008,
        shoulder_height=shoulder_height,
        bottom_z=wall_thickness,
        arc_segments=18,
    )
    rear_panel_mesh = mesh_from_geometry(
        _extrude_profile_along_x(
            rear_panel_profile,
            x_min=-body_length * 0.5 - 0.0002,
            x_max=-body_length * 0.5 + 0.0032,
        ),
        "mailbox_rear_panel",
    )

    body = model.part("body")
    body.visual(shell_mesh, material=galvanized, name="shell")
    body.visual(rear_panel_mesh, material=galvanized, name="rear_panel")
    body.visual(
        Box((0.16, 0.060, 0.012)),
        origin=Origin(xyz=(0.015, 0.0, 0.0085)),
        material=shadow_steel,
        name="mounting_channel",
    )
    for name, y_center in (("left", -0.1175), ("right", 0.1175)):
        body.visual(
            Cylinder(radius=hinge_radius, length=0.030),
            origin=Origin(
                xyz=(front_x, y_center, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=shadow_steel,
            name=f"body_hinge_{name}",
        )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    door_profile = _mailbox_profile(
        width=body_width - 0.002,
        shoulder_height=shoulder_height,
        bottom_z=0.0,
        arc_segments=18,
    )
    door_mesh = mesh_from_geometry(
        _extrude_profile_along_x(
            door_profile,
            x_min=0.0,
            x_max=door_thickness,
        ),
        "mailbox_door",
    )

    door = model.part("door")
    door.visual(door_mesh, material=galvanized, name="door_panel")
    door.visual(
        Cylinder(radius=hinge_radius, length=0.068),
        origin=Origin(xyz=(hinge_radius + 0.0008, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=shadow_steel,
        name="door_hinge_center",
    )
    door.visual(
        Box((0.014, 0.082, 0.012)),
        origin=Origin(
            xyz=(door_thickness - hinge_radius + 0.007, 0.0, 0.105),
        ),
        material=handle_black,
        name="pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_thickness + 0.010, body_width, body_height)),
        mass=0.55,
        origin=Origin(xyz=(0.004, 0.0, body_height * 0.5)),
    )

    flag_bracket = model.part("flag_bracket")
    flag_bracket.visual(
        Box((0.036, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=shadow_steel,
        name="bracket_plate",
    )
    flag_bracket.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(
            xyz=(0.0, -0.003, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=shadow_steel,
        name="pivot_boss",
    )
    flag_bracket.inertial = Inertial.from_geometry(
        Box((0.036, 0.012, 0.050)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
    )

    flag = model.part("flag")
    flag.visual(
        Box((0.150, 0.003, 0.045)),
        origin=Origin(xyz=(0.075, 0.0045, 0.0)),
        material=flag_red,
        name="flag_blade",
    )
    flag.visual(
        Cylinder(radius=0.0225, length=0.003),
        origin=Origin(
            xyz=(0.150, 0.0045, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=flag_red,
        name="flag_tip",
    )
    flag.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(
            xyz=(0.0, 0.002, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=shadow_steel,
        name="flag_collar",
    )
    flag.inertial = Inertial.from_geometry(
        Box((0.155, 0.010, 0.050)),
        mass=0.09,
        origin=Origin(xyz=(0.078, 0.004, 0.0)),
    )

    body_mount_height = 1.00
    model.articulation(
        "post_to_body",
        ArticulationType.FIXED,
        parent=post_mount,
        child=body,
        origin=Origin(xyz=(0.030, 0.0, body_mount_height)),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(front_x, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.80,
        ),
    )
    model.articulation(
        "body_to_flag_bracket",
        ArticulationType.FIXED,
        parent=body,
        child=flag_bracket,
        origin=Origin(xyz=(0.115, body_width * 0.5 + 0.008, 0.128)),
    )
    model.articulation(
        "bracket_to_flag",
        ArticulationType.REVOLUTE,
        parent=flag_bracket,
        child=flag,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=-1.35,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post_mount = object_model.get_part("post_mount")
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    flag_bracket = object_model.get_part("flag_bracket")
    flag = object_model.get_part("flag")
    door_hinge = object_model.get_articulation("body_to_door")
    flag_pivot = object_model.get_articulation("bracket_to_flag")

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
        "door hinge axis is horizontal",
        tuple(door_hinge.axis) == (0.0, 1.0, 0.0),
        f"expected door hinge axis (0, 1, 0), got {door_hinge.axis}",
    )
    ctx.check(
        "flag pivot axis is lateral",
        tuple(flag_pivot.axis) == (0.0, 1.0, 0.0),
        f"expected flag pivot axis (0, 1, 0), got {flag_pivot.axis}",
    )

    ctx.expect_contact(body, post_mount, contact_tol=0.001, name="body sits on post mount")
    ctx.expect_contact(
        flag_bracket,
        body,
        contact_tol=0.001,
        name="flag bracket is mounted to body",
    )

    with ctx.pose({door_hinge: 0.0, flag_pivot: 0.0}):
        ctx.expect_contact(door, body, contact_tol=0.001, name="door closes onto shell")
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="door_panel",
            negative_elem="shell",
            min_gap=0.0,
            max_gap=0.001,
            max_penetration=0.0,
            name="door sits at mailbox front edge",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.18,
            name="door covers rounded opening",
        )
        ctx.expect_contact(
            flag,
            flag_bracket,
            contact_tol=0.001,
            name="flag remains clipped to bracket pivot",
        )

    with ctx.pose({door_hinge: 1.55}):
        body_aabb = ctx.part_world_aabb(body)
        door_aabb = ctx.part_world_aabb(door)
        if body_aabb is None or door_aabb is None:
            ctx.fail("door open pose measurable", "missing body or door AABB in open pose")
        else:
            ctx.check(
                "door swings downward and forward",
                door_aabb[1][0] > body_aabb[1][0] + 0.10
                and door_aabb[1][2] < body_aabb[1][2] - 0.12,
                f"door_aabb={door_aabb}, body_aabb={body_aabb}",
            )

    with ctx.pose({flag_pivot: -1.20}):
        body_aabb = ctx.part_world_aabb(body)
        shell_aabb = ctx.part_element_world_aabb(body, elem="shell")
        flag_aabb = ctx.part_world_aabb(flag)
        if body_aabb is None or shell_aabb is None or flag_aabb is None:
            ctx.fail("flag raised pose measurable", "missing body or flag AABB in raised pose")
        else:
            ctx.check(
                "flag raises above the roofline",
                flag_aabb[1][2] > shell_aabb[1][2] - 0.005
                and flag_aabb[0][1] > shell_aabb[1][1],
                f"flag_aabb={flag_aabb}, shell_aabb={shell_aabb}, body_aabb={body_aabb}",
            )

    with ctx.pose({door_hinge: 1.55, flag_pivot: -1.20}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in operating pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
