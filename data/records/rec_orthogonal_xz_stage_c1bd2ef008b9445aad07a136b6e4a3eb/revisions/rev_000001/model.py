from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 1.18
BASE_DEPTH = 0.20
BASE_HEIGHT = 0.10

COLUMN_WIDTH = 0.12
COLUMN_DEPTH = 0.14
COLUMN_HEIGHT = 0.74
COLUMN_X = (BASE_LEN / 2.0) - 0.17

BRIDGE_LEN = 0.94
BRIDGE_DEPTH = 0.18
BRIDGE_HEIGHT = 0.12
BRIDGE_BOTTOM_Z = BASE_HEIGHT + COLUMN_HEIGHT

RAIL_LEN = 0.84
RAIL_WIDTH = 0.030
RAIL_HEIGHT = 0.016
RAIL_OFFSET_Y = 0.055
RAIL_BOTTOM_Z = BRIDGE_BOTTOM_Z - RAIL_HEIGHT

CARRIAGE_LEN = 0.20
CARRIAGE_BODY_DEPTH = 0.15
CARRIAGE_BODY_HEIGHT = 0.09
SHOE_LEN = 0.16
SHOE_WIDTH = 0.032
SHOE_HEIGHT = 0.020
X_TIE_LEN = 0.020
X_TIE_DEPTH = (2.0 * RAIL_OFFSET_Y) + SHOE_WIDTH
X_TIE_HEIGHT = 0.070
X_TIE_CENTER_X = 0.065
GUIDE_CHEEK_THICKNESS = 0.020
GUIDE_CHEEK_DEPTH = 0.080
GUIDE_CHEEK_HEIGHT = 0.180
GUIDE_TOP_Z = -SHOE_HEIGHT

RAM_WIDTH = 0.070
RAM_DEPTH = 0.036
RAM_HEIGHT = 0.30
TOOL_PLATE_WIDTH = 0.094
TOOL_PLATE_DEPTH = 0.050
TOOL_PLATE_HEIGHT = 0.030
TOOL_NOSE_RADIUS = 0.014
TOOL_NOSE_LENGTH = 0.050

X_TRAVEL = 0.30
Z_TRAVEL = 0.18
RAM_JOINT_Z = GUIDE_TOP_Z


def _frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BASE_LEN, BASE_DEPTH, BASE_HEIGHT, centered=(True, True, False)
    )
    left_column = (
        cq.Workplane("XY")
        .box(COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT, centered=(True, True, False))
        .translate((COLUMN_X, 0.0, BASE_HEIGHT))
    )
    right_column = (
        cq.Workplane("XY")
        .box(COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT, centered=(True, True, False))
        .translate((-COLUMN_X, 0.0, BASE_HEIGHT))
    )
    bridge = (
        cq.Workplane("XY")
        .box(BRIDGE_LEN, BRIDGE_DEPTH, BRIDGE_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BRIDGE_BOTTOM_Z))
    )
    left_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (COLUMN_X - 0.06, BASE_HEIGHT),
                (COLUMN_X - 0.06, BASE_HEIGHT + 0.18),
                (COLUMN_X - 0.18, BRIDGE_BOTTOM_Z),
                (COLUMN_X - 0.06, BRIDGE_BOTTOM_Z),
            ]
        )
        .close()
        .extrude(0.016, both=True)
    )
    right_gusset = left_gusset.mirror("YZ")
    return (
        base.union(left_column)
        .union(right_column)
        .union(bridge)
        .union(left_gusset)
        .union(right_gusset)
    )


def _carriage_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LEN,
            CARRIAGE_BODY_DEPTH,
            CARRIAGE_BODY_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -CARRIAGE_BODY_HEIGHT - SHOE_HEIGHT))
    )
    left_shoe = (
        cq.Workplane("XY")
        .box(SHOE_LEN, SHOE_WIDTH, SHOE_HEIGHT, centered=(True, True, False))
        .translate((0.0, RAIL_OFFSET_Y, -SHOE_HEIGHT))
    )
    right_shoe = (
        cq.Workplane("XY")
        .box(SHOE_LEN, SHOE_WIDTH, SHOE_HEIGHT, centered=(True, True, False))
        .translate((0.0, -RAIL_OFFSET_Y, -SHOE_HEIGHT))
    )
    left_side_plate = (
        cq.Workplane("XY")
        .box(
            SHOE_LEN,
            SIDE_PLATE_THICKNESS,
            SIDE_PLATE_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (0.0, (BRIDGE_DEPTH / 2.0) + (SIDE_PLATE_THICKNESS / 2.0), -SHOE_HEIGHT)
        )
    )
    right_side_plate = (
        cq.Workplane("XY")
        .box(
            SHOE_LEN,
            SIDE_PLATE_THICKNESS,
            SIDE_PLATE_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (0.0, -(BRIDGE_DEPTH / 2.0) - (SIDE_PLATE_THICKNESS / 2.0), -SHOE_HEIGHT)
        )
    )
    housing = (
        cq.Workplane("XY")
        .box(HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, HOUSING_TOP_Z - HOUSING_HEIGHT))
    )

    carriage = (
        body.union(left_shoe)
        .union(right_shoe)
        .union(left_side_plate)
        .union(right_side_plate)
        .union(housing)
    )
    slide_slot = (
        cq.Workplane("XY")
        .box(RAM_WIDTH, 0.046, HOUSING_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, HOUSING_TOP_Z - HOUSING_HEIGHT))
    )
    return carriage.cut(slide_slot)


def _ram_shape() -> cq.Workplane:
    ram_body = (
        cq.Workplane("XY")
        .box(RAM_WIDTH, RAM_DEPTH, RAM_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, -RAM_HEIGHT))
    )
    tool_plate = (
        cq.Workplane("XY")
        .box(
            TOOL_PLATE_WIDTH,
            TOOL_PLATE_DEPTH,
            TOOL_PLATE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -RAM_HEIGHT - TOOL_PLATE_HEIGHT))
    )
    return ram_body.union(tool_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_xz_positioning_unit")

    model.material("frame_gray", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("rail_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("carriage_graphite", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("ram_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("tool_steel", rgba=(0.40, 0.43, 0.47, 1.0))
    model.material("accent_blue", rgba=(0.16, 0.42, 0.76, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame_shell"),
        material="frame_gray",
        name="frame_shell",
    )
    frame.visual(
        Box((RAIL_LEN, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_OFFSET_Y, RAIL_BOTTOM_Z + (RAIL_HEIGHT / 2.0))),
        material="rail_steel",
        name="rail_left",
    )
    frame.visual(
        Box((RAIL_LEN, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -RAIL_OFFSET_Y, RAIL_BOTTOM_Z + (RAIL_HEIGHT / 2.0))),
        material="rail_steel",
        name="rail_right",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((SHOE_LEN, SHOE_WIDTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_OFFSET_Y, -(SHOE_HEIGHT / 2.0))),
        material="carriage_graphite",
        name="left_shoe",
    )
    carriage.visual(
        Box((SHOE_LEN, SHOE_WIDTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, -RAIL_OFFSET_Y, -(SHOE_HEIGHT / 2.0))),
        material="carriage_graphite",
        name="right_shoe",
    )
    carriage.visual(
        Box((X_TIE_LEN, X_TIE_DEPTH, X_TIE_HEIGHT)),
        origin=Origin(
            xyz=(
                X_TIE_CENTER_X,
                0.0,
                -SHOE_HEIGHT - (X_TIE_HEIGHT / 2.0),
            )
        ),
        material="carriage_graphite",
        name="front_tie",
    )
    carriage.visual(
        Box((X_TIE_LEN, X_TIE_DEPTH, X_TIE_HEIGHT)),
        origin=Origin(
            xyz=(
                -X_TIE_CENTER_X,
                0.0,
                -SHOE_HEIGHT - (X_TIE_HEIGHT / 2.0),
            )
        ),
        material="carriage_graphite",
        name="rear_tie",
    )
    carriage.visual(
        Box((GUIDE_CHEEK_THICKNESS, GUIDE_CHEEK_DEPTH, GUIDE_CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                (RAM_WIDTH / 2.0) + (GUIDE_CHEEK_THICKNESS / 2.0),
                0.0,
                GUIDE_TOP_Z - (GUIDE_CHEEK_HEIGHT / 2.0),
            )
        ),
        material="carriage_graphite",
        name="right_guide",
    )
    carriage.visual(
        Box((GUIDE_CHEEK_THICKNESS, GUIDE_CHEEK_DEPTH, GUIDE_CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                -((RAM_WIDTH / 2.0) + (GUIDE_CHEEK_THICKNESS / 2.0)),
                0.0,
                GUIDE_TOP_Z - (GUIDE_CHEEK_HEIGHT / 2.0),
            )
        ),
        material="carriage_graphite",
        name="left_guide",
    )
    carriage.visual(
        Box((0.076, 0.008, 0.12)),
        origin=Origin(
            xyz=(0.0, (GUIDE_CHEEK_DEPTH / 2.0) + 0.004, GUIDE_TOP_Z - 0.09)
        ),
        material="accent_blue",
        name="carriage_face",
    )

    ram = model.part("ram")
    ram.visual(
        Box((RAM_WIDTH, RAM_DEPTH, RAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -(RAM_HEIGHT / 2.0))),
        material="ram_silver",
        name="ram_shell",
    )
    ram.visual(
        Box((TOOL_PLATE_WIDTH, TOOL_PLATE_DEPTH, TOOL_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, -RAM_HEIGHT - (TOOL_PLATE_HEIGHT / 2.0))
        ),
        material="tool_steel",
        name="tool_plate",
    )
    ram.visual(
        Cylinder(radius=TOOL_NOSE_RADIUS, length=TOOL_NOSE_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, -RAM_HEIGHT - TOOL_PLATE_HEIGHT - (TOOL_NOSE_LENGTH / 2.0))
        ),
        material="tool_steel",
        name="tool_nose",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=500.0,
            velocity=0.70,
        ),
    )
    model.articulation(
        "carriage_to_ram",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=ram,
        origin=Origin(xyz=(0.0, 0.0, RAM_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=300.0,
            velocity=0.40,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    ram = object_model.get_part("ram")
    x_slide = object_model.get_articulation("frame_to_carriage")
    z_slide = object_model.get_articulation("carriage_to_ram")
    rail_left = frame.get_visual("rail_left")
    rail_right = frame.get_visual("rail_right")

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
        "x_slide_axis_and_type",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected PRISMATIC +X slide, got type={x_slide.articulation_type} axis={x_slide.axis}",
    )
    ctx.check(
        "z_slide_axis_and_type",
        z_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(z_slide.axis) == (0.0, 0.0, -1.0),
        details=f"expected PRISMATIC -Z slide, got type={z_slide.articulation_type} axis={z_slide.axis}",
    )

    ctx.expect_contact(
        carriage,
        frame,
        elem_b=rail_left,
        name="carriage_contacts_left_rail",
    )
    ctx.expect_contact(
        carriage,
        frame,
        elem_b=rail_right,
        name="carriage_contacts_right_rail",
    )
    ctx.expect_contact(
        ram,
        carriage,
        name="ram_contacts_carriage_guide",
    )

    with ctx.pose({x_slide: -X_TRAVEL}):
        low_x = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            frame,
            name="carriage_stays_supported_at_left_limit",
        )
    with ctx.pose({x_slide: X_TRAVEL}):
        high_x = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            frame,
            name="carriage_stays_supported_at_right_limit",
        )

    ctx.check(
        "carriage_moves_horizontally",
        low_x is not None
        and high_x is not None
        and high_x[0] > low_x[0] + 0.50
        and abs(high_x[2] - low_x[2]) < 1e-6,
        details=f"expected large +X travel without Z drift, got left={low_x} right={high_x}",
    )

    with ctx.pose({z_slide: 0.0}):
        retracted_ram = ctx.part_world_position(ram)
    with ctx.pose({z_slide: Z_TRAVEL}):
        extended_ram = ctx.part_world_position(ram)
        ctx.expect_contact(
            ram,
            carriage,
            name="ram_stays_guided_at_full_extension",
        )

    ctx.check(
        "ram_moves_downward",
        retracted_ram is not None
        and extended_ram is not None
        and extended_ram[2] < retracted_ram[2] - 0.15
        and abs(extended_ram[0] - retracted_ram[0]) < 1e-6,
        details=f"expected downward Z extension without X drift, got retracted={retracted_ram} extended={extended_ram}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
