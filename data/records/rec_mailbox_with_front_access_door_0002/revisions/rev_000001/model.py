from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_LENGTH = 0.48
BODY_WIDTH = 0.21
BODY_SIDE_HEIGHT = 0.095
BODY_RADIUS = BODY_WIDTH * 0.5
BODY_HEIGHT = BODY_SIDE_HEIGHT + BODY_RADIUS
SHELL_THICKNESS = 0.008
BACK_WALL_THICKNESS = 0.010

DOOR_WIDTH = BODY_WIDTH - 0.010
DOOR_SIDE_HEIGHT = BODY_SIDE_HEIGHT - 0.002
DOOR_THICKNESS = 0.008

POST_WIDTH = 0.095
POST_DEPTH = 0.095
POST_HEIGHT = 0.58
PLATE_LENGTH = 0.26
PLATE_WIDTH = 0.14
PLATE_THICKNESS = 0.018

HANDLE_STEM_RADIUS = 0.003
HANDLE_STEM_LENGTH = 0.008
HANDLE_SPAN = 0.032
HANDLE_Z = 0.040


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _arch_profile(width: float, side_height: float, *, bottom_z: float, arc_segments: int) -> list[tuple[float, float]]:
    half_width = width * 0.5
    radius = half_width
    points: list[tuple[float, float]] = [
        (half_width, bottom_z),
        (half_width, side_height),
    ]
    for index in range(1, arc_segments):
        angle = math.pi * index / arc_segments
        points.append((radius * math.cos(angle), side_height + radius * math.sin(angle)))
    points.extend(
        [
            (-half_width, side_height),
            (-half_width, bottom_z),
        ]
    )
    return points


def _extrusion_to_world(mesh: MeshGeometry, *, x_shift: float = 0.0) -> MeshGeometry:
    return MeshGeometry(
        vertices=[(z + x_shift, x, y) for x, y, z in mesh.vertices],
        faces=list(mesh.faces),
    )


def _build_body_shell_mesh() -> MeshGeometry:
    outer = ExtrudeGeometry.from_z0(
        _arch_profile(
            BODY_WIDTH,
            BODY_SIDE_HEIGHT,
            bottom_z=0.0,
            arc_segments=28,
        ),
        BODY_LENGTH,
        cap=True,
        closed=True,
    )
    inner = ExtrudeGeometry.from_z0(
        _arch_profile(
            BODY_WIDTH - (2.0 * SHELL_THICKNESS),
            BODY_SIDE_HEIGHT,
            bottom_z=SHELL_THICKNESS,
            arc_segments=24,
        ),
        BODY_LENGTH - BACK_WALL_THICKNESS + 0.002,
        cap=True,
        closed=True,
    )
    inner.translate(0.0, 0.0, BACK_WALL_THICKNESS)
    shell_local = boolean_difference(outer, inner)
    return _extrusion_to_world(shell_local, x_shift=-(BODY_LENGTH * 0.5))


def _build_door_panel_mesh() -> MeshGeometry:
    panel = ExtrudeGeometry.from_z0(
        _arch_profile(
            DOOR_WIDTH,
            DOOR_SIDE_HEIGHT,
            bottom_z=0.0,
            arc_segments=24,
        ),
        DOOR_THICKNESS,
        cap=True,
        closed=True,
    )
    return _extrusion_to_world(panel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curbside_mailbox", assets=ASSETS)

    galvanized_steel = model.material("galvanized_steel", rgba=(0.69, 0.72, 0.76, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    post_wood = model.material("post_wood", rgba=(0.47, 0.32, 0.19, 1.0))
    mount_grey = model.material("mount_grey", rgba=(0.44, 0.46, 0.49, 1.0))

    support = model.part("support")
    support.visual(
        Box((POST_WIDTH, POST_DEPTH, POST_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT * 0.5)),
        material=post_wood,
        name="post",
    )
    support.visual(
        Box((PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT + (PLATE_THICKNESS * 0.5))),
        material=mount_grey,
        name="mount_plate",
    )
    support.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, POST_HEIGHT + PLATE_THICKNESS)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, (POST_HEIGHT + PLATE_THICKNESS) * 0.5)),
    )

    body = model.part("mailbox_body")
    body.visual(
        _save_mesh(_build_body_shell_mesh(), "mailbox_body_shell.obj"),
        material=galvanized_steel,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    door = model.part("front_door")
    door.visual(
        _save_mesh(_build_door_panel_mesh(), "mailbox_front_door.obj"),
        material=galvanized_steel,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=HANDLE_STEM_RADIUS, length=HANDLE_STEM_LENGTH),
        origin=Origin(
            xyz=(DOOR_THICKNESS + (HANDLE_STEM_LENGTH * 0.5), -(HANDLE_SPAN * 0.5), HANDLE_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=handle_dark,
        name="handle_left_stem",
    )
    door.visual(
        Cylinder(radius=HANDLE_STEM_RADIUS, length=HANDLE_STEM_LENGTH),
        origin=Origin(
            xyz=(DOOR_THICKNESS + (HANDLE_STEM_LENGTH * 0.5), HANDLE_SPAN * 0.5, HANDLE_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=handle_dark,
        name="handle_right_stem",
    )
    door.visual(
        Cylinder(radius=HANDLE_STEM_RADIUS, length=HANDLE_SPAN),
        origin=Origin(
            xyz=(DOOR_THICKNESS + HANDLE_STEM_LENGTH + HANDLE_STEM_RADIUS, 0.0, HANDLE_Z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=handle_dark,
        name="handle_grip",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_THICKNESS + HANDLE_STEM_LENGTH + (2.0 * HANDLE_STEM_RADIUS), DOOR_WIDTH, BODY_HEIGHT)),
        mass=0.55,
        origin=Origin(
            xyz=(
                0.5 * (DOOR_THICKNESS + HANDLE_STEM_LENGTH + (2.0 * HANDLE_STEM_RADIUS)),
                0.0,
                BODY_HEIGHT * 0.5,
            )
        ),
    )

    model.articulation(
        "support_to_mailbox",
        ArticulationType.FIXED,
        parent=support,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT + PLATE_THICKNESS)),
    )
    model.articulation(
        "body_to_front_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(BODY_LENGTH * 0.5, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=math.pi * 0.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support = object_model.get_part("support")
    body = object_model.get_part("mailbox_body")
    door = object_model.get_part("front_door")
    door_hinge = object_model.get_articulation("body_to_front_door")

    mount_plate = support.get_visual("mount_plate")
    body_shell = body.get_visual("body_shell")
    door_panel = door.get_visual("door_panel")
    handle_grip = door.get_visual("handle_grip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "door_hinge_axis_is_lateral",
        tuple(round(value, 6) for value in door_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"expected hinge axis (0, 1, 0), got {door_hinge.axis}",
    )
    ctx.check(
        "door_hinge_travel_is_quarter_turn",
        abs(door_hinge.motion_limits.lower - 0.0) < 1e-6
        and abs(door_hinge.motion_limits.upper - (math.pi * 0.5)) < 1e-6,
        details=(
            "front door should open from closed at 0 rad to roughly 90 degrees; "
            f"got limits {door_hinge.motion_limits.lower}..{door_hinge.motion_limits.upper}"
        ),
    )

    ctx.expect_origin_distance(body, support, axes="xy", max_dist=0.001)
    ctx.expect_gap(
        body,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=body_shell,
        negative_elem=mount_plate,
        name="mailbox_body_sits_flush_on_mount_plate",
    )
    ctx.expect_contact(
        body,
        support,
        elem_a=body_shell,
        elem_b=mount_plate,
        name="mailbox_body_contacts_mount_plate",
    )
    ctx.expect_overlap(
        body,
        support,
        axes="xy",
        min_overlap=0.12,
        elem_a=body_shell,
        elem_b=mount_plate,
        name="mailbox_body_footprint_overlaps_mount_plate",
    )

    ctx.expect_gap(
        door,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=door_panel,
        negative_elem=body_shell,
        name="front_door_closes_against_mailbox_front",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        min_overlap=0.16,
        elem_a=door_panel,
        elem_b=body_shell,
        name="front_door_covers_mailbox_opening",
    )
    ctx.expect_origin_distance(
        door,
        body,
        axes="yz",
        max_dist=0.006,
        name="front_door_centered_on_mailbox_face",
    )
    ctx.expect_gap(
        door,
        body,
        axis="x",
        min_gap=0.008,
        positive_elem=handle_grip,
        negative_elem=body_shell,
        name="handle_proud_of_closed_door_face",
    )

    with ctx.pose({door_hinge: math.pi * 0.5}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            min_gap=0.035,
            positive_elem=handle_grip,
            negative_elem=body_shell,
            name="open_door_swings_handle_forward",
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            min_gap=0.002,
            positive_elem=body_shell,
            negative_elem=handle_grip,
            name="open_door_drops_below_mailbox_body",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
