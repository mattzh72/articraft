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


BASE_LENGTH = 0.42
BASE_WIDTH = 0.28
BASE_THICKNESS = 0.016
BASE_RAIL_LENGTH = 0.34
BASE_RAIL_PLINTH_WIDTH = 0.042
BASE_RAIL_PLINTH_HEIGHT = 0.008
BASE_RAIL_CROWN_WIDTH = 0.020
BASE_RAIL_CROWN_HEIGHT = 0.008
BASE_RAIL_OFFSET_Y = 0.084
BASE_RAIL_TOP_Z = (
    BASE_THICKNESS + BASE_RAIL_PLINTH_HEIGHT + BASE_RAIL_CROWN_HEIGHT
)

LOWER_LENGTH = 0.30
LOWER_WIDTH = 0.22
LOWER_BODY_HEIGHT = 0.042
LOWER_TOP_RAIL_WIDTH = 0.024
LOWER_TOP_RAIL_LENGTH = 0.19
LOWER_TOP_RAIL_HEIGHT = 0.008
LOWER_TOP_RAIL_OFFSET_X = 0.058
LOWER_RAIL_TOP_Z = LOWER_BODY_HEIGHT + LOWER_TOP_RAIL_HEIGHT
LOWER_TRAVEL = 0.05

CROSS_LENGTH = 0.18
CROSS_WIDTH = 0.16
CROSS_BODY_HEIGHT = 0.038
CROSS_GUIDE_OUTER_X = 0.080
CROSS_GUIDE_OUTER_Y = 0.052
CROSS_GUIDE_HEIGHT = 0.100
CROSS_GUIDE_CHEEK_THICKNESS = 0.014
CROSS_TRAVEL = 0.045

RAM_BODY_X = 0.046
RAM_BODY_Y = 0.026
RAM_GUIDE_SHOE_THICKNESS = 0.003
RAM_GUIDE_SHOE_WIDTH_Y = 0.020
RAM_HIDDEN_LENGTH = 0.08
RAM_VISIBLE_LENGTH = 0.16
RAM_TOTAL_LENGTH = RAM_HIDDEN_LENGTH + RAM_VISIBLE_LENGTH
RAM_TRAVEL = 0.05


def _box(
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z, centered=(True, True, False))
        .translate((x, y, z))
    )


def _base_shape() -> cq.Workplane:
    shape = _box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
    for rail_y in (BASE_RAIL_OFFSET_Y, -BASE_RAIL_OFFSET_Y):
        shape = shape.union(
            _box(
                BASE_RAIL_LENGTH,
                BASE_RAIL_PLINTH_WIDTH,
                BASE_RAIL_PLINTH_HEIGHT,
                y=rail_y,
                z=BASE_THICKNESS,
            )
        )
        shape = shape.union(
            _box(
                BASE_RAIL_LENGTH,
                BASE_RAIL_CROWN_WIDTH,
                BASE_RAIL_CROWN_HEIGHT,
                y=rail_y,
                z=BASE_THICKNESS + BASE_RAIL_PLINTH_HEIGHT,
            )
        )

    shape = shape.cut(_box(0.22, 0.06, 0.006, z=BASE_THICKNESS - 0.006))
    shape = shape.cut(_box(0.10, 0.10, 0.008, z=0.004))
    return shape


def _lower_carriage_shape() -> cq.Workplane:
    shape = _box(LOWER_LENGTH, LOWER_WIDTH, LOWER_BODY_HEIGHT)
    shape = shape.cut(
        _box(
            0.18,
            0.014,
            0.020,
            y=LOWER_WIDTH / 2.0 - 0.007,
            z=0.011,
        )
    )
    shape = shape.cut(
        _box(
            0.18,
            0.014,
            0.020,
            y=-(LOWER_WIDTH / 2.0 - 0.007),
            z=0.011,
        )
    )
    shape = shape.cut(_box(0.16, 0.090, 0.010, z=LOWER_BODY_HEIGHT - 0.010))
    for rail_x in (LOWER_TOP_RAIL_OFFSET_X, -LOWER_TOP_RAIL_OFFSET_X):
        shape = shape.union(
            _box(
                LOWER_TOP_RAIL_WIDTH,
                LOWER_TOP_RAIL_LENGTH,
                LOWER_TOP_RAIL_HEIGHT,
                x=rail_x,
                z=LOWER_BODY_HEIGHT,
            )
        )
    return shape


def _cross_slide_shape() -> cq.Workplane:
    shape = _box(CROSS_LENGTH, CROSS_WIDTH, CROSS_BODY_HEIGHT)
    shape = shape.cut(
        _box(
            0.012,
            0.10,
            0.018,
            x=CROSS_LENGTH / 2.0 - 0.006,
            z=0.010,
        )
    )
    shape = shape.cut(
        _box(
            0.012,
            0.10,
            0.018,
            x=-(CROSS_LENGTH / 2.0 - 0.006),
            z=0.010,
        )
    )
    cheek_x = (CROSS_GUIDE_OUTER_X - CROSS_GUIDE_CHEEK_THICKNESS) / 2.0
    for guide_x in (cheek_x, -cheek_x):
        shape = shape.union(
            _box(
                CROSS_GUIDE_CHEEK_THICKNESS,
                CROSS_GUIDE_OUTER_Y,
                CROSS_GUIDE_HEIGHT,
                x=guide_x,
                z=CROSS_BODY_HEIGHT,
            )
        )
    shape = shape.union(
        _box(
            CROSS_GUIDE_OUTER_X,
            0.010,
            0.050,
            y=-(CROSS_GUIDE_OUTER_Y / 2.0 - 0.005),
            z=CROSS_BODY_HEIGHT,
        )
    )
    shape = shape.cut(
        _box(
            0.036,
            CROSS_GUIDE_OUTER_Y / 2.0,
            0.050,
            y=CROSS_GUIDE_OUTER_Y / 4.0,
            z=CROSS_BODY_HEIGHT + 0.018,
        )
    )
    return shape


def _ram_shape() -> cq.Workplane:
    shape = _box(
        RAM_BODY_X,
        RAM_BODY_Y,
        RAM_TOTAL_LENGTH,
        z=-RAM_HIDDEN_LENGTH,
    )
    shoe_height = 0.12
    shoe_bottom = -RAM_HIDDEN_LENGTH
    for shoe_x in (
        RAM_BODY_X / 2.0 + RAM_GUIDE_SHOE_THICKNESS / 2.0,
        -(RAM_BODY_X / 2.0 + RAM_GUIDE_SHOE_THICKNESS / 2.0),
    ):
        shape = shape.union(
            _box(
                RAM_GUIDE_SHOE_THICKNESS,
                RAM_GUIDE_SHOE_WIDTH_Y,
                shoe_height,
                x=shoe_x,
                z=shoe_bottom,
            )
        )
    shape = shape.cut(_box(0.018, RAM_BODY_Y + 0.002, 0.09, z=0.02))
    shape = shape.cut(_box(RAM_BODY_X + 0.002, 0.010, 0.06, y=0.008, z=-0.08))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_xyz_stage")

    model.material("cast_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("dark_way", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("ram_steel", rgba=(0.84, 0.85, 0.87, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base"),
        material="cast_gray",
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_RAIL_TOP_Z)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_TOP_Z / 2.0)),
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        mesh_from_cadquery(_lower_carriage_shape(), "lower_carriage"),
        material="dark_way",
        name="lower_body",
    )
    lower_carriage.inertial = Inertial.from_geometry(
        Box((LOWER_LENGTH, LOWER_WIDTH, LOWER_RAIL_TOP_Z)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, LOWER_RAIL_TOP_Z / 2.0)),
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        Box((CROSS_LENGTH, CROSS_WIDTH, CROSS_BODY_HEIGHT)),
        material="cast_gray",
        origin=Origin(xyz=(0.0, 0.0, CROSS_BODY_HEIGHT / 2.0)),
        name="cross_table",
    )
    guide_cheek_x = (CROSS_GUIDE_OUTER_X - CROSS_GUIDE_CHEEK_THICKNESS) / 2.0
    guide_cheek_origin_z = CROSS_BODY_HEIGHT + CROSS_GUIDE_HEIGHT / 2.0
    for cheek_name, cheek_x in (
        ("guide_left", guide_cheek_x),
        ("guide_right", -guide_cheek_x),
    ):
        cross_slide.visual(
            Box(
                (
                    CROSS_GUIDE_CHEEK_THICKNESS,
                    CROSS_GUIDE_OUTER_Y,
                    CROSS_GUIDE_HEIGHT,
                )
            ),
            material="cast_gray",
            origin=Origin(xyz=(cheek_x, 0.0, guide_cheek_origin_z)),
            name=cheek_name,
        )
    cross_slide.visual(
        Box((CROSS_GUIDE_OUTER_X, 0.010, 0.050)),
        material="cast_gray",
        origin=Origin(
            xyz=(
                0.0,
                -(CROSS_GUIDE_OUTER_Y / 2.0 - 0.005),
                CROSS_BODY_HEIGHT + 0.025,
            )
        ),
        name="guide_back",
    )
    cross_slide.inertial = Inertial.from_geometry(
        Box((CROSS_LENGTH, CROSS_WIDTH, CROSS_BODY_HEIGHT + CROSS_GUIDE_HEIGHT)),
        mass=4.2,
        origin=Origin(
            xyz=(0.0, 0.0, (CROSS_BODY_HEIGHT + CROSS_GUIDE_HEIGHT) / 2.0)
        ),
    )

    ram = model.part("vertical_ram")
    ram.visual(
        Box((RAM_BODY_X, RAM_BODY_Y, RAM_TOTAL_LENGTH)),
        material="ram_steel",
        origin=Origin(xyz=(0.0, 0.0, (RAM_VISIBLE_LENGTH - RAM_HIDDEN_LENGTH) / 2.0)),
        name="ram_core",
    )
    shoe_origin_z = -RAM_HIDDEN_LENGTH + 0.06
    for shoe_name, shoe_x in (
        (
            "guide_shoe_left",
            RAM_BODY_X / 2.0 + RAM_GUIDE_SHOE_THICKNESS / 2.0,
        ),
        (
            "guide_shoe_right",
            -(RAM_BODY_X / 2.0 + RAM_GUIDE_SHOE_THICKNESS / 2.0),
        ),
    ):
        ram.visual(
            Box((RAM_GUIDE_SHOE_THICKNESS, RAM_GUIDE_SHOE_WIDTH_Y, 0.12)),
            material="ram_steel",
            origin=Origin(xyz=(shoe_x, 0.0, shoe_origin_z)),
            name=shoe_name,
        )
    ram.visual(
        Box((0.060, 0.040, 0.012)),
        material="ram_steel",
        origin=Origin(xyz=(0.0, 0.0, RAM_VISIBLE_LENGTH - 0.006)),
        name="tool_plate",
    )
    ram.inertial = Inertial.from_geometry(
        Box(
            (
                RAM_BODY_X + 2.0 * RAM_GUIDE_SHOE_THICKNESS,
                RAM_BODY_Y,
                RAM_TOTAL_LENGTH,
            )
        ),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, (RAM_VISIBLE_LENGTH - RAM_HIDDEN_LENGTH) / 2.0)),
    )

    model.articulation(
        "base_to_lower_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-LOWER_TRAVEL,
            upper=LOWER_TRAVEL,
            effort=900.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "lower_to_cross_y",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, LOWER_RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CROSS_TRAVEL,
            upper=CROSS_TRAVEL,
            effort=700.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "cross_to_ram_z",
        ArticulationType.PRISMATIC,
        parent=cross_slide,
        child=ram,
        origin=Origin(xyz=(0.0, 0.0, CROSS_BODY_HEIGHT + CROSS_GUIDE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=RAM_TRAVEL,
            effort=500.0,
            velocity=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_carriage = object_model.get_part("lower_carriage")
    cross_slide = object_model.get_part("cross_slide")
    ram = object_model.get_part("vertical_ram")
    lower_stage = object_model.get_articulation("base_to_lower_x")
    middle_stage = object_model.get_articulation("lower_to_cross_y")
    upper_stage = object_model.get_articulation("cross_to_ram_z")
    lower_upper = lower_stage.motion_limits.upper if lower_stage.motion_limits else LOWER_TRAVEL
    middle_upper = (
        middle_stage.motion_limits.upper if middle_stage.motion_limits else CROSS_TRAVEL
    )
    upper_upper = upper_stage.motion_limits.upper if upper_stage.motion_limits else RAM_TRAVEL

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

    ctx.expect_gap(
        lower_carriage,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="lower carriage seats on base rails",
    )
    ctx.expect_gap(
        cross_slide,
        lower_carriage,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="cross slide seats on lower carriage ways",
    )
    ctx.expect_contact(
        ram,
        cross_slide,
        contact_tol=0.0005,
        name="vertical ram engages the guide housing",
    )
    ctx.expect_within(
        lower_carriage,
        base,
        axes="y",
        margin=0.0,
        name="lower carriage stays inside the base rail span",
    )
    ctx.expect_within(
        cross_slide,
        lower_carriage,
        axes="x",
        margin=0.0,
        name="cross slide stays within the lower carriage width in X",
    )
    ctx.expect_within(
        ram,
        cross_slide,
        axes="xy",
        margin=0.025,
        name="ram stays over the cross slide footprint",
    )

    lower_rest = ctx.part_world_position(lower_carriage)
    with ctx.pose({lower_stage: lower_upper}):
        ctx.expect_overlap(
            lower_carriage,
            base,
            axes="x",
            min_overlap=0.24,
            name="lower carriage retains long rail support at max X travel",
        )
        lower_extended = ctx.part_world_position(lower_carriage)

    middle_rest = ctx.part_world_position(cross_slide)
    with ctx.pose({middle_stage: middle_upper}):
        ctx.expect_overlap(
            cross_slide,
            lower_carriage,
            axes="y",
            min_overlap=0.11,
            name="cross slide retains guide engagement at max Y travel",
        )
        middle_extended = ctx.part_world_position(cross_slide)

    upper_rest = ctx.part_world_position(ram)
    with ctx.pose({upper_stage: upper_upper}):
        ctx.expect_contact(
            ram,
            cross_slide,
            contact_tol=0.0005,
            name="ram still contacts its guide at max Z travel",
        )
        ctx.expect_overlap(
            ram,
            cross_slide,
            axes="z",
            min_overlap=0.028,
            name="ram retains insertion in the guide at max Z travel",
        )
        upper_extended = ctx.part_world_position(ram)

    with ctx.pose(
        {
            lower_stage: lower_upper,
            middle_stage: middle_upper,
            upper_stage: upper_upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at the max positive service pose")

    ctx.check(
        "lower carriage moves along +X",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[0] > lower_rest[0] + 0.04,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )
    ctx.check(
        "cross slide moves along +Y",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[1] > middle_rest[1] + 0.03,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "vertical ram moves along +Z",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[2] > upper_rest[2] + 0.035,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
