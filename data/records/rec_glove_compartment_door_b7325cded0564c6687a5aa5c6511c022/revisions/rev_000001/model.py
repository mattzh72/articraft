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
    rounded_rect_profile,
    section_loft,
)


FASCIA_WIDTH = 0.74
FASCIA_HEIGHT = 0.30
FASCIA_FRAME_THICKNESS = 0.008

OPENING_WIDTH = 0.38
OPENING_HEIGHT = 0.13
OPENING_CENTER_Z = 0.12

BOX_WIDTH = 0.372
BOX_HEIGHT = 0.124
BOX_DEPTH = 0.176
BOX_WALL = 0.005
BOX_FLANGE_THICKNESS = 0.004
BOX_FLANGE_OUTER_WIDTH = 0.404
BOX_FLANGE_OUTER_HEIGHT = 0.148
BOX_FLANGE_INNER_WIDTH = 0.354
BOX_FLANGE_INNER_HEIGHT = 0.106

DOOR_WIDTH = 0.37
DOOR_HEIGHT = 0.122
DOOR_THICKNESS = 0.008


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _fascia_frame_geometry():
    opening_offset = OPENING_CENTER_Z - (FASCIA_HEIGHT * 0.5)
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(FASCIA_WIDTH, FASCIA_HEIGHT, 0.03, corner_segments=8),
        [
            _offset_profile(
                rounded_rect_profile(OPENING_WIDTH, OPENING_HEIGHT, 0.012, corner_segments=8),
                dy=opening_offset,
            )
        ],
        FASCIA_FRAME_THICKNESS,
        center=False,
    )
    geom.rotate_x(math.pi * 0.5)
    geom.translate(0.0, 0.0, FASCIA_HEIGHT * 0.5)
    return geom


def _door_geometry():
    geom = ExtrudeGeometry(
        rounded_rect_profile(DOOR_WIDTH, DOOR_HEIGHT, 0.01, corner_segments=8),
        DOOR_THICKNESS,
        center=False,
    )
    geom.rotate_x(math.pi * 0.5)
    geom.translate(0.0, 0.0, -DOOR_HEIGHT * 0.5)
    return geom


def _section_loop(width: float, y_pos: float, z_low: float, z_high: float) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    return [
        (-half_width, y_pos, z_low),
        (half_width, y_pos, z_low),
        (half_width, y_pos, z_high),
        (-half_width, y_pos, z_high),
    ]


def _dashboard_cowl_geometry():
    return section_loft(
        [
            _section_loop(FASCIA_WIDTH * 0.98, -FASCIA_FRAME_THICKNESS, 0.192, 0.300),
            _section_loop(FASCIA_WIDTH * 0.90, -0.080, 0.212, 0.310),
            _section_loop(FASCIA_WIDTH * 0.78, -0.160, 0.236, 0.286),
        ]
    )


def _storage_flange_geometry():
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(BOX_FLANGE_OUTER_WIDTH, BOX_FLANGE_OUTER_HEIGHT, 0.012, corner_segments=8),
        [
            rounded_rect_profile(
                BOX_FLANGE_INNER_WIDTH,
                BOX_FLANGE_INNER_HEIGHT,
                0.008,
                corner_segments=8,
            )
        ],
        BOX_FLANGE_THICKNESS,
        center=False,
    )
    geom.rotate_x(math.pi * 0.5)
    geom.translate(0.0, -FASCIA_FRAME_THICKNESS, OPENING_CENTER_Z)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aircraft_glove_compartment")

    fascia_gray = model.material("fascia_gray", rgba=(0.50, 0.53, 0.56, 1.0))
    bin_gray = model.material("bin_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    door_gray = model.material("door_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.15, 0.16, 0.18, 1.0))

    fascia = model.part("dashboard_fascia")
    fascia.visual(
        mesh_from_geometry(_fascia_frame_geometry(), "dashboard_fascia_frame"),
        material=fascia_gray,
        name="fascia_frame",
    )
    fascia.visual(
        mesh_from_geometry(_dashboard_cowl_geometry(), "dashboard_upper_cowl"),
        material=fascia_gray,
        name="upper_cowl",
    )
    fascia.visual(
        Box((0.72, 0.085, 0.050)),
        origin=Origin(xyz=(0.0, -0.0425, 0.025)),
        material=fascia_gray,
        name="lower_sill",
    )
    fascia.visual(
        Box((0.052, 0.110, 0.230)),
        origin=Origin(xyz=(-0.344, -0.055, 0.145)),
        material=fascia_gray,
        name="left_cheek",
    )
    fascia.visual(
        Box((0.052, 0.110, 0.230)),
        origin=Origin(xyz=(0.344, -0.055, 0.145)),
        material=fascia_gray,
        name="right_cheek",
    )
    fascia.visual(
        Box((0.40, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.013, OPENING_CENTER_Z - (OPENING_HEIGHT * 0.5) - 0.007)),
        material=trim_dark,
        name="opening_sill_trim",
    )
    fascia.inertial = Inertial.from_geometry(
        Box((0.74, 0.17, 0.31)),
        mass=7.0,
        origin=Origin(xyz=(0.0, -0.070, 0.155)),
    )

    storage_box = model.part("storage_box")
    storage_box.visual(
        mesh_from_geometry(_storage_flange_geometry(), "glovebox_storage_flange_v2"),
        material=trim_dark,
        name="front_flange",
    )
    storage_box.visual(
        Box((BOX_WIDTH, BOX_DEPTH, BOX_WALL)),
        origin=Origin(
            xyz=(
                0.0,
                -FASCIA_FRAME_THICKNESS - BOX_FLANGE_THICKNESS - (BOX_DEPTH * 0.5),
                OPENING_CENTER_Z - (BOX_HEIGHT * 0.5) + (BOX_WALL * 0.5),
            )
        ),
        material=bin_gray,
        name="bottom_wall",
    )
    storage_box.visual(
        Box((BOX_WIDTH, BOX_DEPTH, BOX_WALL)),
        origin=Origin(
            xyz=(
                0.0,
                -FASCIA_FRAME_THICKNESS - BOX_FLANGE_THICKNESS - (BOX_DEPTH * 0.5),
                OPENING_CENTER_Z + (BOX_HEIGHT * 0.5) - (BOX_WALL * 0.5),
            )
        ),
        material=bin_gray,
        name="top_wall",
    )
    storage_box.visual(
        Box((BOX_WALL, BOX_DEPTH, BOX_HEIGHT - (2.0 * BOX_WALL))),
        origin=Origin(
            xyz=(
                -(BOX_WIDTH * 0.5) + (BOX_WALL * 0.5),
                -FASCIA_FRAME_THICKNESS - BOX_FLANGE_THICKNESS - (BOX_DEPTH * 0.5),
                OPENING_CENTER_Z,
            )
        ),
        material=bin_gray,
        name="left_wall",
    )
    storage_box.visual(
        Box((BOX_WALL, BOX_DEPTH, BOX_HEIGHT - (2.0 * BOX_WALL))),
        origin=Origin(
            xyz=(
                (BOX_WIDTH * 0.5) - (BOX_WALL * 0.5),
                -FASCIA_FRAME_THICKNESS - BOX_FLANGE_THICKNESS - (BOX_DEPTH * 0.5),
                OPENING_CENTER_Z,
            )
        ),
        material=bin_gray,
        name="right_wall",
    )
    storage_box.visual(
        Box((BOX_WIDTH, BOX_WALL, BOX_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -FASCIA_FRAME_THICKNESS - BOX_FLANGE_THICKNESS - BOX_DEPTH + (BOX_WALL * 0.5),
                OPENING_CENTER_Z,
            )
        ),
        material=bin_gray,
        name="back_wall",
    )
    storage_box.inertial = Inertial.from_geometry(
        Box((BOX_WIDTH, BOX_DEPTH + BOX_FLANGE_THICKNESS, BOX_HEIGHT)),
        mass=1.1,
        origin=Origin(
            xyz=(
                0.0,
                -FASCIA_FRAME_THICKNESS - 0.5 * (BOX_FLANGE_THICKNESS + BOX_DEPTH),
                OPENING_CENTER_Z,
            )
        ),
    )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(_door_geometry(), "glovebox_door"),
        material=door_gray,
        name="door_panel",
    )
    door.visual(
        Box((0.092, 0.002, 0.036)),
        origin=Origin(xyz=(0.0, 0.001, -0.094)),
        material=trim_dark,
        name="handle_bezel",
    )
    door.visual(
        Box((0.30, 0.002, 0.090)),
        origin=Origin(xyz=(0.0, -0.007, -0.065)),
        material=trim_dark,
        name="inner_stiffener",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=0.85,
        origin=Origin(xyz=(0.0, -0.004, -DOOR_HEIGHT * 0.5)),
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.0045, length=0.074),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=handle_dark,
        name="pivot_barrel",
    )
    latch_handle.visual(
        Box((0.074, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, 0.005, -0.016)),
        material=handle_dark,
        name="grip_paddle",
    )
    latch_handle.visual(
        Box((0.050, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.010, -0.029)),
        material=handle_dark,
        name="finger_lip",
    )
    latch_handle.inertial = Inertial.from_geometry(
        Box((0.080, 0.018, 0.038)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.006, -0.017)),
    )

    model.articulation(
        "fascia_to_storage_box",
        ArticulationType.FIXED,
        parent=fascia,
        child=storage_box,
        origin=Origin(),
    )
    model.articulation(
        "fascia_to_door",
        ArticulationType.REVOLUTE,
        parent=fascia,
        child=door,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                OPENING_CENTER_Z + (DOOR_HEIGHT * 0.5),
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.4,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "door_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_handle,
        origin=Origin(xyz=(0.0, 0.0045, -0.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.5,
            lower=0.0,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fascia = object_model.get_part("dashboard_fascia")
    storage_box = object_model.get_part("storage_box")
    door = object_model.get_part("door")
    latch_handle = object_model.get_part("latch_handle")

    door_hinge = object_model.get_articulation("fascia_to_door")
    latch_pivot = object_model.get_articulation("door_to_latch_handle")

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
        "door_hinge_axis_is_horizontal",
        door_hinge.axis == (1.0, 0.0, 0.0),
        details=f"Expected door hinge axis (1, 0, 0), got {door_hinge.axis}.",
    )
    ctx.check(
        "latch_axis_is_horizontal",
        latch_pivot.axis == (1.0, 0.0, 0.0),
        details=f"Expected latch axis (1, 0, 0), got {latch_pivot.axis}.",
    )

    ctx.expect_contact(storage_box, fascia)
    ctx.expect_contact(latch_handle, door)
    ctx.expect_within(storage_box, fascia, axes="xz", margin=0.0)

    with ctx.pose({door_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_gap(
            door,
            storage_box,
            axis="y",
            min_gap=0.0,
            max_gap=0.0005,
            name="door_seats_against_storage_flange",
        )
        ctx.expect_overlap(
            door,
            storage_box,
            axes="xz",
            min_overlap=0.10,
            name="door_covers_storage_opening",
        )
        door_closed_aabb = ctx.part_world_aabb(door)
        latch_closed_aabb = ctx.part_world_aabb(latch_handle)
        assert door_closed_aabb is not None
        assert latch_closed_aabb is not None

    with ctx.pose({door_hinge: 1.15}):
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        assert door_open_aabb[0][2] > door_closed_aabb[0][2] + 0.04
        assert door_open_aabb[1][1] > door_closed_aabb[1][1] + 0.08

    with ctx.pose({door_hinge: 0.0, latch_pivot: 0.60}):
        latch_open_aabb = ctx.part_world_aabb(latch_handle)
        assert latch_open_aabb is not None
        assert latch_open_aabb[1][1] > latch_closed_aabb[1][1] + 0.008

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
