from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Mesh,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.26
BODY_DEPTH = 0.24
BODY_HEIGHT = 0.68
BODY_RADIUS = 0.034
WALL = 0.0045
BASE_PLINTH_HEIGHT = 0.014

REAR_OPENING_WIDTH = 0.188
REAR_OPENING_HEIGHT = 0.525
REAR_OPENING_BOTTOM = 0.082
REAR_OPENING_CENTER_Z = REAR_OPENING_BOTTOM + REAR_OPENING_HEIGHT / 2.0

FRONT_GRILLE_WIDTH = 0.165
FRONT_GRILLE_HEIGHT = 0.455
FRONT_GRILLE_CENTER_Z = 0.315

TOP_VENT_WIDTH = 0.165
TOP_VENT_DEPTH = 0.115
TOP_VENT_CENTER_Z = BODY_HEIGHT - 0.0035

DOOR_WIDTH = 0.198
DOOR_HEIGHT = 0.540
DOOR_THICKNESS = 0.005
DOOR_BOTTOM = 0.074

FILTER_WIDTH = 0.176
FILTER_HEIGHT = 0.500
FILTER_DEPTH = 0.136
FILTER_BOTTOM = 0.094
FILTER_TRAVEL = 0.092


def _body_shell_mesh() -> Mesh:
    outer = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(BODY_RADIUS)
        .edges(">Z")
        .fillet(0.012)
    )

    inner = (
        cq.Workplane("XY")
        .box(BODY_WIDTH - 2.0 * WALL, BODY_DEPTH - 2.0 * WALL, BODY_HEIGHT - 0.078)
        .translate((0.0, 0.0, 0.040 + (BODY_HEIGHT - 0.078) / 2.0))
        .edges("|Z")
        .fillet(max(BODY_RADIUS - WALL, 0.008))
    )

    rear_cut = (
        cq.Workplane("XY")
        .box(REAR_OPENING_WIDTH - 0.010, 0.030, REAR_OPENING_HEIGHT - 0.014)
        .translate(
            (
                0.0,
                -BODY_DEPTH / 2.0 + 0.010,
                REAR_OPENING_BOTTOM + REAR_OPENING_HEIGHT / 2.0,
            )
        )
    )

    shell = outer.cut(inner).cut(rear_cut)
    return mesh_from_cadquery(shell, "tower_purifier_body_shell")


def _base_plinth_mesh() -> Mesh:
    outer = (
        cq.Workplane("XY")
        .box(BODY_WIDTH + 0.020, BODY_DEPTH + 0.018, BASE_PLINTH_HEIGHT)
        .translate((0.0, 0.0, BASE_PLINTH_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.022)
    )
    inner = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BASE_PLINTH_HEIGHT + 0.002)
        .translate((0.0, 0.0, BASE_PLINTH_HEIGHT / 2.0))
    )
    plinth = outer.cut(inner)
    return mesh_from_cadquery(plinth, "tower_purifier_base_plinth")


def _dial_mesh() -> Mesh:
    dial = KnobGeometry(
        0.058,
        0.020,
        body_style="skirted",
        top_diameter=0.048,
        skirt=KnobSkirt(0.066, 0.004, flare=0.05),
        grip=KnobGrip(style="fluted", count=24, depth=0.001),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
        center=False,
    )
    return mesh_from_geometry(dial, "tower_purifier_dial")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.90, 0.91, 0.89, 1.0))
    grille_gray = model.material("grille_gray", rgba=(0.34, 0.37, 0.39, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.52, 0.55, 0.56, 1.0))
    filter_media = model.material("filter_media", rgba=(0.76, 0.81, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        _body_shell_mesh(),
        material=shell_white,
        name="shell",
    )

    front_grille = SlotPatternPanelGeometry(
        (FRONT_GRILLE_WIDTH, FRONT_GRILLE_HEIGHT),
        0.003,
        slot_size=(0.090, 0.0055),
        pitch=(0.110, 0.013),
        frame=0.007,
        corner_radius=0.020,
        center=True,
    )
    body.visual(
        mesh_from_geometry(front_grille, "tower_purifier_front_grille"),
        origin=Origin(
            xyz=(0.0, BODY_DEPTH / 2.0 - 0.0015, FRONT_GRILLE_CENTER_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_gray,
        name="front_grille",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * WALL, 0.150, 0.010)),
        origin=Origin(xyz=(0.0, -0.018, 0.081)),
        material=shell_white,
        name="filter_tray",
    )

    top_vent_geometry = VentGrilleGeometry(
        (TOP_VENT_WIDTH, TOP_VENT_DEPTH),
        frame=0.010,
        face_thickness=0.003,
        duct_depth=0.008,
        duct_wall=0.002,
        slat_pitch=0.016,
        slat_width=0.008,
        slat_angle_deg=22.0,
        corner_radius=0.015,
        slats=VentGrilleSlats(profile="flat", direction="down", inset=0.002, divider_count=2),
        frame_profile=VentGrilleFrame(style="radiused", depth=0.001),
        sleeve=VentGrilleSleeve(style="short", depth=0.008, wall=0.002),
        center=True,
    )
    top_vent = model.part("top_vent")
    top_vent.visual(
        mesh_from_geometry(top_vent_geometry, "tower_purifier_top_vent"),
        origin=Origin(xyz=(0.0, 0.0, TOP_VENT_CENTER_Z)),
        material=grille_gray,
        name="vent_face",
    )
    model.articulation(
        "body_to_top_vent",
        ArticulationType.FIXED,
        parent=body,
        child=top_vent,
        origin=Origin(),
    )

    dial = model.part("dial")
    dial.visual(
        _dial_mesh(),
        material=charcoal,
        name="dial_body",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=shell_white,
        name="door_panel",
    )
    filter_door.visual(
        Box((DOOR_WIDTH - 0.026, 0.003, DOOR_HEIGHT - 0.030)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0 + 0.002, -0.001, DOOR_HEIGHT / 2.0)),
        material=grille_gray,
        name="door_inset",
    )
    filter_door.visual(
        Box((0.015, 0.012, 0.104)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.022, -0.0035, DOOR_HEIGHT * 0.58)),
        material=grille_gray,
        name="door_handle",
    )

    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(
            xyz=(
                -REAR_OPENING_WIDTH / 2.0 - 0.004,
                -BODY_DEPTH / 2.0 - DOOR_THICKNESS / 2.0,
                DOOR_BOTTOM,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    filter_cartridge = model.part("filter_cartridge")
    filter_cartridge.visual(
        Box((FILTER_WIDTH, FILTER_DEPTH, FILTER_HEIGHT)),
        origin=Origin(xyz=(0.0, FILTER_DEPTH / 2.0, FILTER_HEIGHT / 2.0)),
        material=filter_media,
        name="filter_block",
    )
    filter_cartridge.visual(
        Box((FILTER_WIDTH + 0.010, 0.012, FILTER_HEIGHT + 0.016)),
        origin=Origin(xyz=(0.0, 0.006, FILTER_HEIGHT / 2.0)),
        material=filter_gray,
        name="filter_frame",
    )
    filter_cartridge.visual(
        Box((0.086, 0.014, 0.038)),
        origin=Origin(xyz=(0.0, -0.007, FILTER_HEIGHT * 0.82)),
        material=filter_gray,
        name="filter_handle",
    )

    model.articulation(
        "body_to_filter_cartridge",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_cartridge,
        origin=Origin(
            xyz=(
                0.0,
                -BODY_DEPTH / 2.0 + 0.018,
                FILTER_BOTTOM,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.18,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    filter_door = object_model.get_part("filter_door")
    filter_cartridge = object_model.get_part("filter_cartridge")
    top_vent = object_model.get_part("top_vent")
    door_joint = object_model.get_articulation("body_to_filter_door")
    filter_joint = object_model.get_articulation("body_to_filter_cartridge")
    dial_joint = object_model.get_articulation("body_to_dial")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Expected body bounds.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("body_has_tower_scale", 0.24 <= size[0] <= 0.30 and 0.22 <= size[1] <= 0.28 and 0.66 <= size[2] <= 0.70, f"size={size!r}")

    dial_aabb = ctx.part_world_aabb(dial)
    ctx.check("dial_aabb_present", dial_aabb is not None, "Expected dial bounds.")
    if dial_aabb is not None and body_aabb is not None:
        dial_mins, dial_maxs = dial_aabb
        _, body_maxs = body_aabb
        ctx.check(
            "dial_sits_near_top_surface",
            dial_mins[2] >= body_maxs[2] - 0.010,
            f"dial_z={dial_mins[2]:.4f}, body_top={body_maxs[2]:.4f}",
        )
        ctx.check(
            "dial_has_home_appliance_size",
            0.055 <= (dial_maxs[0] - dial_mins[0]) <= 0.070 and 0.018 <= (dial_maxs[2] - dial_mins[2]) <= 0.026,
            f"dial_size={(dial_maxs[0] - dial_mins[0], dial_maxs[1] - dial_mins[1], dial_maxs[2] - dial_mins[2])!r}",
        )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "dial_joint_is_continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS and dial_limits is not None and dial_limits.lower is None and dial_limits.upper is None,
        f"type={dial_joint.articulation_type!r}, limits={dial_limits!r}",
    )

    ctx.expect_contact(
        dial,
        top_vent,
        elem_a="dial_body",
        elem_b="vent_face",
        contact_tol=0.0035,
        name="dial seats onto top vent deck",
    )

    shell_aabb = ctx.part_element_world_aabb(body, elem="shell")
    closed_door_aabb = ctx.part_element_world_aabb(filter_door, elem="door_panel")
    ctx.check("door_panel_aabb_present", closed_door_aabb is not None, "Expected closed door panel AABB.")
    if shell_aabb is not None and closed_door_aabb is not None:
        shell_mins, shell_maxs = shell_aabb
        door_mins, door_maxs = closed_door_aabb
        ctx.check(
            "door_closed_flush_to_rear_shell",
            abs(door_maxs[1] - shell_mins[1]) <= 0.0025 and abs(door_maxs[2] - (DOOR_BOTTOM + DOOR_HEIGHT)) <= 0.002,
            f"door_mins={door_mins!r}, shell_mins={shell_mins!r}, door_maxs={door_maxs!r}",
        )
        ctx.check(
            "door_covers_rear_service_zone",
            (door_maxs[0] - door_mins[0]) >= REAR_OPENING_WIDTH and (door_maxs[2] - door_mins[2]) >= REAR_OPENING_HEIGHT,
            f"door_size={(door_maxs[0] - door_mins[0], door_maxs[1] - door_mins[1], door_maxs[2] - door_mins[2])!r}",
        )

    ctx.expect_within(
        filter_cartridge,
        body,
        axes="xz",
        elem_a="filter_block",
        elem_b="shell",
        margin=0.015,
        name="filter cartridge stays centered in the body cavity",
    )
    ctx.expect_overlap(
        filter_cartridge,
        body,
        axes="y",
        elem_a="filter_block",
        elem_b="shell",
        min_overlap=0.105,
        name="resting filter remains deeply inserted into the purifier body",
    )

    rest_filter_aabb = ctx.part_element_world_aabb(filter_cartridge, elem="filter_handle")
    with ctx.pose({door_joint: math.radians(90.0)}):
        open_door_aabb = ctx.part_element_world_aabb(filter_door, elem="door_panel")
    ctx.check("open_door_aabb_present", open_door_aabb is not None, "Expected open door panel AABB.")
    if closed_door_aabb is not None and open_door_aabb is not None:
        ctx.check(
            "door_swings_outward_from_rear_opening",
            open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.070,
            f"closed={closed_door_aabb!r}, open={open_door_aabb!r}",
        )

    with ctx.pose({door_joint: math.radians(105.0), filter_joint: FILTER_TRAVEL}):
        extended_filter_aabb = ctx.part_element_world_aabb(filter_cartridge, elem="filter_handle")
        extended_open_door_aabb = ctx.part_element_world_aabb(filter_door, elem="door_panel")
        ctx.expect_within(
            filter_cartridge,
            body,
            axes="xz",
            elem_a="filter_block",
            elem_b="shell",
            margin=0.015,
            name="extended filter stays laterally aligned with the service opening",
        )
        ctx.expect_overlap(
            filter_cartridge,
            body,
            axes="y",
            elem_a="filter_block",
            elem_b="shell",
            min_overlap=0.040,
            name="extended filter retains insertion inside the purifier cavity",
        )
        ctx.check(
            "opened door clears the extended filter handle",
            extended_open_door_aabb is not None
            and extended_filter_aabb is not None
            and extended_open_door_aabb[1][0] < extended_filter_aabb[0][0] - 0.010,
            details=f"door={extended_open_door_aabb!r}, filter={extended_filter_aabb!r}",
        )

    ctx.check("extended_filter_aabb_present", extended_filter_aabb is not None, "Expected extended filter handle AABB.")
    if rest_filter_aabb is not None and extended_filter_aabb is not None:
        ctx.check(
            "filter_slides_out_toward_rear",
            extended_filter_aabb[0][1] < rest_filter_aabb[0][1] - 0.070,
            f"rest={rest_filter_aabb!r}, extended={extended_filter_aabb!r}",
        )

    return ctx.report()


object_model = build_object_model()
