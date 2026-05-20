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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


JAMB_OUTER_WIDTH = 0.620
OPENING_WIDTH = 0.556
JAMB_MEMBER = 0.032
JAMB_DEPTH = 0.110
OPENING_HEIGHT = 2.000

DOOR_WIDTH = 0.550
DOOR_HEIGHT = 1.990
DOOR_THICKNESS = 0.032
DOOR_Y = 0.015

STILE_WIDTH = 0.060
TOP_RAIL = 0.090
BOTTOM_RAIL = 0.110
MID_RAIL = 0.080
KICK_PANEL_HEIGHT = 0.500
KICK_PANEL_THICKNESS = 0.014
LOUVER_COUNT = 21
LOUVER_PITCH = 0.056
LOUVER_BLADE_HEIGHT = 0.052
LOUVER_BLADE_THICKNESS = 0.008
LOUVER_BLADE_LENGTH = DOOR_WIDTH - 2.0 * STILE_WIDTH - 0.020
LOUVER_PIN_LENGTH = 0.010
LOUVER_PIN_RADIUS = 0.003
LOUVER_BASE_TILT = math.radians(18.0)


def _panel_mesh(width: float, height: float, thickness: float, name: str):
    profile = rounded_rect_profile(width, height, radius=min(0.010, width * 0.08, height * 0.08))
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, center=True, closed=True, cap=True),
        name,
    )


def _louver_section(x_pos: float, height: float, thickness: float) -> list[tuple[float, float, float]]:
    half_height = 0.5 * height
    half_thickness = 0.5 * thickness
    return [
        (x_pos, 0.00 * half_thickness, 1.00 * half_height),
        (x_pos, 0.52 * half_thickness, 0.72 * half_height),
        (x_pos, 0.92 * half_thickness, 0.18 * half_height),
        (x_pos, 1.00 * half_thickness, 0.00 * half_height),
        (x_pos, 0.84 * half_thickness, -0.34 * half_height),
        (x_pos, 0.36 * half_thickness, -0.76 * half_height),
        (x_pos, 0.00 * half_thickness, -1.00 * half_height),
        (x_pos, -0.36 * half_thickness, -0.76 * half_height),
        (x_pos, -0.84 * half_thickness, -0.34 * half_height),
        (x_pos, -1.00 * half_thickness, 0.00 * half_height),
        (x_pos, -0.92 * half_thickness, 0.18 * half_height),
        (x_pos, -0.52 * half_thickness, 0.72 * half_height),
    ]


def _louver_blade_mesh(length: float, height: float, thickness: float, name: str):
    half_length = 0.5 * length
    return mesh_from_geometry(
        section_loft(
            [
                _louver_section(-half_length, height, thickness),
                _louver_section(half_length, height, thickness),
            ]
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="closet_shutter_door")

    painted_white = model.material("painted_white", rgba=(0.95, 0.95, 0.93, 1.0))
    jamb_white = model.material("jamb_white", rgba=(0.93, 0.93, 0.91, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.66, 0.64, 0.58, 1.0))
    louver_white = model.material("louver_white", rgba=(0.97, 0.97, 0.95, 1.0))

    jamb = model.part("jamb")
    jamb.visual(
        Box((JAMB_MEMBER, JAMB_DEPTH, OPENING_HEIGHT + JAMB_MEMBER)),
        origin=Origin(
            xyz=(
                -0.5 * OPENING_WIDTH - 0.5 * JAMB_MEMBER,
                0.0,
                0.5 * (OPENING_HEIGHT + JAMB_MEMBER),
            )
        ),
        material=jamb_white,
        name="left_jamb",
    )
    jamb.visual(
        Box((JAMB_MEMBER, JAMB_DEPTH, OPENING_HEIGHT + JAMB_MEMBER)),
        origin=Origin(
            xyz=(
                0.5 * OPENING_WIDTH + 0.5 * JAMB_MEMBER,
                0.0,
                0.5 * (OPENING_HEIGHT + JAMB_MEMBER),
            )
        ),
        material=jamb_white,
        name="right_jamb",
    )
    jamb.visual(
        Box((JAMB_OUTER_WIDTH, JAMB_DEPTH, JAMB_MEMBER)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT + 0.5 * JAMB_MEMBER)),
        material=jamb_white,
        name="head_jamb",
    )
    for index, hinge_z in enumerate((0.280, 1.000, 1.720)):
        jamb.visual(
            Box((0.003, 0.028, 0.095)),
            origin=Origin(xyz=(-0.5 * OPENING_WIDTH + 0.0015, 0.028, hinge_z)),
            material=hinge_metal,
            name=f"jamb_hinge_leaf_{index}",
        )
    jamb.inertial = Inertial.from_geometry(
        Box((JAMB_OUTER_WIDTH, JAMB_DEPTH, OPENING_HEIGHT + JAMB_MEMBER)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 1.016)),
    )

    door = model.part("door")
    door.visual(
        Box((STILE_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.5 * STILE_WIDTH, 0.0, 0.5 * DOOR_HEIGHT)),
        material=painted_white,
        name="left_stile",
    )
    door.visual(
        Box((STILE_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.5 * STILE_WIDTH, 0.0, 0.5 * DOOR_HEIGHT)),
        material=painted_white,
        name="right_stile",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, TOP_RAIL)),
        origin=Origin(xyz=(0.5 * DOOR_WIDTH, 0.0, DOOR_HEIGHT - 0.5 * TOP_RAIL)),
        material=painted_white,
        name="top_rail",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, BOTTOM_RAIL)),
        origin=Origin(xyz=(0.5 * DOOR_WIDTH, 0.0, 0.5 * BOTTOM_RAIL)),
        material=painted_white,
        name="bottom_rail",
    )
    mid_rail_center_z = BOTTOM_RAIL + KICK_PANEL_HEIGHT + 0.5 * MID_RAIL
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, MID_RAIL)),
        origin=Origin(xyz=(0.5 * DOOR_WIDTH, 0.0, mid_rail_center_z)),
        material=painted_white,
        name="mid_rail",
    )
    door.visual(
        _panel_mesh(
            DOOR_WIDTH - 2.0 * STILE_WIDTH,
            KICK_PANEL_HEIGHT,
            KICK_PANEL_THICKNESS,
            "kick_panel_mesh",
        ),
        origin=Origin(
            xyz=(
                0.5 * DOOR_WIDTH,
                0.0,
                BOTTOM_RAIL + 0.5 * KICK_PANEL_HEIGHT,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=painted_white,
        name="kick_panel",
    )
    for index, hinge_z in enumerate((0.280, 1.000, 1.720)):
        door.visual(
            Box((0.003, 0.024, 0.090)),
            origin=Origin(xyz=(0.0015, 0.002, hinge_z)),
            material=hinge_metal,
            name=f"door_hinge_leaf_{index}",
        )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=12.0,
        origin=Origin(xyz=(0.5 * DOOR_WIDTH, 0.0, 0.5 * DOOR_HEIGHT)),
    )

    model.articulation(
        "jamb_to_door",
        ArticulationType.REVOLUTE,
        parent=jamb,
        child=door,
        origin=Origin(xyz=(-0.5 * OPENING_WIDTH + 0.003, DOOR_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    upper_opening_bottom = BOTTOM_RAIL + KICK_PANEL_HEIGHT + MID_RAIL
    upper_opening_top = DOOR_HEIGHT - TOP_RAIL
    first_louver_z = upper_opening_bottom + 0.5 * (upper_opening_top - upper_opening_bottom - (LOUVER_COUNT - 1) * LOUVER_PITCH)
    louver_mesh = _louver_blade_mesh(
        LOUVER_BLADE_LENGTH,
        LOUVER_BLADE_HEIGHT,
        LOUVER_BLADE_THICKNESS,
        "louver_blade_mesh",
    )

    for index in range(LOUVER_COUNT):
        louver = model.part(f"louver_{index}")
        louver.visual(
            louver_mesh,
            origin=Origin(rpy=(LOUVER_BASE_TILT, 0.0, 0.0)),
            material=louver_white,
            name="slat_blade",
        )
        pin_offset = 0.5 * LOUVER_BLADE_LENGTH + 0.5 * LOUVER_PIN_LENGTH
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=(-pin_offset, 0.0, 0.0),
                rpy=(LOUVER_BASE_TILT, math.pi / 2.0, 0.0),
            ),
            material=hinge_metal,
            name="left_pivot_pin",
        )
        louver.visual(
            Cylinder(radius=LOUVER_PIN_RADIUS, length=LOUVER_PIN_LENGTH),
            origin=Origin(
                xyz=(pin_offset, 0.0, 0.0),
                rpy=(LOUVER_BASE_TILT, math.pi / 2.0, 0.0),
            ),
            material=hinge_metal,
            name="right_pivot_pin",
        )
        louver.inertial = Inertial.from_geometry(
            Box(
                (
                    LOUVER_BLADE_LENGTH + 2.0 * LOUVER_PIN_LENGTH,
                    0.018,
                    LOUVER_BLADE_HEIGHT,
                )
            ),
            mass=0.18,
            origin=Origin(),
        )
        model.articulation(
            f"door_to_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=louver,
            origin=Origin(
                xyz=(
                    0.5 * DOOR_WIDTH,
                    0.0,
                    first_louver_z + index * LOUVER_PITCH,
                )
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=1.5,
                lower=math.radians(-28.0),
                upper=math.radians(52.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    jamb = object_model.get_part("jamb")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("jamb_to_door")

    first_louver = object_model.get_part("louver_0")
    top_louver = object_model.get_part(f"louver_{LOUVER_COUNT - 1}")
    first_louver_joint = object_model.get_articulation("door_to_louver_0")

    ctx.expect_gap(
        jamb,
        door,
        axis="x",
        positive_elem="right_jamb",
        negative_elem="right_stile",
        min_gap=0.001,
        max_gap=0.008,
        name="door clears the strike-side jamb",
    )
    ctx.expect_gap(
        first_louver,
        door,
        axis="z",
        positive_elem="slat_blade",
        negative_elem="mid_rail",
        min_gap=0.010,
        max_gap=0.030,
        name="lowest louver sits above the mid rail",
    )
    ctx.expect_gap(
        door,
        top_louver,
        axis="z",
        positive_elem="top_rail",
        negative_elem="slat_blade",
        min_gap=0.010,
        max_gap=0.030,
        name="top louver clears the top rail",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="right_stile")
    with ctx.pose({hinge: math.radians(90.0)}):
        open_aabb = ctx.part_element_world_aabb(door, elem="right_stile")
    ctx.check(
        "door swings outward from the jamb",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.35
        and open_aabb[0][0] < closed_aabb[0][0] - 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    louver_rest = ctx.part_element_world_aabb(first_louver, elem="slat_blade")
    with ctx.pose({first_louver_joint: math.radians(40.0)}):
        louver_open = ctx.part_element_world_aabb(first_louver, elem="slat_blade")
    ctx.check(
        "louver tilts about its long axis",
        louver_rest is not None
        and louver_open is not None
        and (louver_open[1][1] - louver_open[0][1]) > (louver_rest[1][1] - louver_rest[0][1]) + 0.010,
        details=f"rest={louver_rest}, open={louver_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
