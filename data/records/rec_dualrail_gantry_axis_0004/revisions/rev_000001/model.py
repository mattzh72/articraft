from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_LENGTH = 0.38
BASE_WIDTH = 0.22
BASE_HEIGHT = 0.03

RAIL_LENGTH = 0.34
RAIL_WIDTH = 0.018
RAIL_HEIGHT = 0.014
RAIL_OFFSET_X = 0.078

GANTRY_TRAVEL = 0.25
TOOL_TRAVEL = 0.12

SHOE_WIDTH = 0.034
SHOE_DEPTH = 0.07
BEAM_LENGTH = 0.22
BEAM_DEPTH = 0.05
BEAM_HEIGHT = 0.032
GANTRY_HEIGHT = 0.092
BEAM_BOTTOM = GANTRY_HEIGHT - BEAM_HEIGHT

TRUCK_WIDTH = 0.058
TRUCK_DEPTH = 0.046
TRUCK_BODY_HEIGHT = 0.03
TRUCK_DROP = 0.048


def _base_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        BASE_WIDTH,
        BASE_LENGTH,
        BASE_HEIGHT,
        centered=(True, True, False),
    )
    pocket = (
        cq.Workplane("XY")
        .box(
            BASE_WIDTH - 0.09,
            BASE_LENGTH - 0.11,
            0.012,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_HEIGHT - 0.009))
    )
    end_feet = (
        cq.Workplane("XY")
        .box(BASE_WIDTH - 0.03, 0.038, 0.012, centered=(True, True, False))
        .translate((0.0, BASE_LENGTH * 0.5 - 0.03, 0.0))
        .union(
            cq.Workplane("XY")
            .box(BASE_WIDTH - 0.03, 0.038, 0.012, centered=(True, True, False))
            .translate((0.0, -BASE_LENGTH * 0.5 + 0.03, 0.0))
        )
    )
    return body.union(end_feet).cut(pocket)


def _rail_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(
        RAIL_WIDTH,
        RAIL_LENGTH,
        RAIL_HEIGHT,
        centered=(True, True, False),
    )


def _gantry_shape() -> cq.Workplane:
    left_shoe = (
        cq.Workplane("XY")
        .box(SHOE_WIDTH, SHOE_DEPTH, GANTRY_HEIGHT, centered=(True, True, False))
        .translate((-RAIL_OFFSET_X, 0.0, 0.0))
    )
    right_shoe = (
        cq.Workplane("XY")
        .box(SHOE_WIDTH, SHOE_DEPTH, GANTRY_HEIGHT, centered=(True, True, False))
        .translate((RAIL_OFFSET_X, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BEAM_BOTTOM))
    )
    beam_window = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH - 0.05, BEAM_DEPTH + 0.002, BEAM_HEIGHT - 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, BEAM_BOTTOM + 0.006))
    )
    left_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-RAIL_OFFSET_X - SHOE_WIDTH * 0.5, 0.0),
                (-RAIL_OFFSET_X + SHOE_WIDTH * 0.5, 0.0),
                (-RAIL_OFFSET_X + SHOE_WIDTH * 0.5, BEAM_BOTTOM),
                (-BEAM_LENGTH * 0.32, BEAM_BOTTOM),
            ]
        )
        .close()
        .extrude(SHOE_DEPTH, both=True)
    )
    right_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (RAIL_OFFSET_X + SHOE_WIDTH * 0.5, 0.0),
                (RAIL_OFFSET_X - SHOE_WIDTH * 0.5, 0.0),
                (RAIL_OFFSET_X - SHOE_WIDTH * 0.5, BEAM_BOTTOM),
                (BEAM_LENGTH * 0.32, BEAM_BOTTOM),
            ]
        )
        .close()
        .extrude(SHOE_DEPTH, both=True)
    )
    return left_shoe.union(right_shoe).union(beam).union(left_web).union(right_web).cut(beam_window)


def _tool_truck_shape() -> cq.Workplane:
    carriage_block = cq.Workplane("XY").box(
        TRUCK_WIDTH,
        TRUCK_DEPTH,
        0.024,
        centered=(True, True, False),
    )
    saddle = (
        cq.Workplane("XY")
        .box(TRUCK_WIDTH * 0.78, TRUCK_DEPTH * 0.62, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, 0.024))
    )
    front_tool_plate = (
        cq.Workplane("XY")
        .box(TRUCK_WIDTH * 0.5, 0.01, 0.028, centered=(True, True, False))
        .translate((0.0, TRUCK_DEPTH * 0.5 + 0.005, 0.004))
    )
    tool_mount = (
        cq.Workplane("YZ")
        .circle(0.006)
        .extrude(0.018)
        .translate((-0.009, TRUCK_DEPTH * 0.5 + 0.014, 0.018))
    ).union(
        cq.Workplane("YZ")
        .circle(0.006)
        .extrude(0.018)
        .translate((0.009, TRUCK_DEPTH * 0.5 + 0.014, 0.018))
    )
    return carriage_block.union(saddle).union(front_tool_plate).union(tool_mount)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_gantry_axis", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.30, 0.33, 0.38, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.53, 0.57, 0.62, 1.0))
    tool_orange = model.material("tool_orange", rgba=(0.88, 0.46, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base.obj", assets=ASSETS),
        material=painted_steel,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_LENGTH, BASE_HEIGHT + 0.012)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + 0.012) * 0.5)),
    )

    rail_left = model.part("rail_left")
    rail_left.visual(
        mesh_from_cadquery(_rail_shape(), "rail_left.obj", assets=ASSETS),
        material=rail_steel,
        name="rail_shell",
    )
    rail_left.inertial = Inertial.from_geometry(
        Box((RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT * 0.5)),
    )

    rail_right = model.part("rail_right")
    rail_right.visual(
        mesh_from_cadquery(_rail_shape(), "rail_right.obj", assets=ASSETS),
        material=rail_steel,
        name="rail_shell",
    )
    rail_right.inertial = Inertial.from_geometry(
        Box((RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT * 0.5)),
    )

    gantry = model.part("gantry")
    gantry.visual(
        mesh_from_cadquery(_gantry_shape(), "gantry.obj", assets=ASSETS),
        material=carriage_gray,
        name="gantry_shell",
    )
    gantry.inertial = Inertial.from_geometry(
        Box((BEAM_LENGTH, SHOE_DEPTH, GANTRY_HEIGHT)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, GANTRY_HEIGHT * 0.5)),
    )

    tool_truck = model.part("tool_truck")
    tool_truck.visual(
        mesh_from_cadquery(_tool_truck_shape(), "tool_truck.obj", assets=ASSETS),
        material=tool_orange,
        name="truck_shell",
    )
    tool_truck.inertial = Inertial.from_geometry(
        Box((TRUCK_WIDTH, TRUCK_DEPTH + 0.028, 0.04)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.01, 0.02)),
    )

    model.articulation(
        "base_to_rail_left",
        ArticulationType.FIXED,
        parent=base,
        child=rail_left,
        origin=Origin(xyz=(-RAIL_OFFSET_X, 0.0, BASE_HEIGHT)),
    )
    model.articulation(
        "base_to_rail_right",
        ArticulationType.FIXED,
        parent=base,
        child=rail_right,
        origin=Origin(xyz=(RAIL_OFFSET_X, 0.0, BASE_HEIGHT)),
    )
    model.articulation(
        "gantry_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gantry,
        origin=Origin(xyz=(0.0, -GANTRY_TRAVEL * 0.5, BASE_HEIGHT + RAIL_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.45,
            lower=0.0,
            upper=GANTRY_TRAVEL,
        ),
    )
    model.articulation(
        "tool_slide",
        ArticulationType.PRISMATIC,
        parent=gantry,
        child=tool_truck,
        origin=Origin(xyz=(-TOOL_TRAVEL * 0.5, 0.0, GANTRY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.25,
            lower=0.0,
            upper=TOOL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    rail_left = object_model.get_part("rail_left")
    rail_right = object_model.get_part("rail_right")
    gantry = object_model.get_part("gantry")
    tool_truck = object_model.get_part("tool_truck")

    gantry_slide = object_model.get_articulation("gantry_slide")
    tool_slide = object_model.get_articulation("tool_slide")

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

    ctx.check("base_present", base is not None, "base part missing")
    ctx.check("rail_left_present", rail_left is not None, "left rail part missing")
    ctx.check("rail_right_present", rail_right is not None, "right rail part missing")
    ctx.check("gantry_present", gantry is not None, "gantry part missing")
    ctx.check("tool_truck_present", tool_truck is not None, "tool truck part missing")

    ctx.expect_gap(rail_left, base, axis="z", max_gap=0.0005, max_penetration=0.0, name="left_rail_seated")
    ctx.expect_gap(rail_right, base, axis="z", max_gap=0.0005, max_penetration=0.0, name="right_rail_seated")
    ctx.expect_overlap(rail_left, base, axes="xy", min_overlap=0.015, name="left_rail_on_base")
    ctx.expect_overlap(rail_right, base, axes="xy", min_overlap=0.015, name="right_rail_on_base")

    with ctx.pose({gantry_slide: 0.0, tool_slide: TOOL_TRAVEL * 0.5}):
        ctx.expect_contact(gantry, rail_left, name="gantry_contacts_left_rail_start")
        ctx.expect_contact(gantry, rail_right, name="gantry_contacts_right_rail_start")
        ctx.expect_within(gantry, base, axes="y", margin=0.03, name="gantry_within_base_start")
        ctx.expect_contact(tool_truck, gantry, name="tool_truck_on_crossbeam_start")
        ctx.expect_within(tool_truck, gantry, axes="x", margin=0.025, name="truck_within_beam_start")

    with ctx.pose({gantry_slide: GANTRY_TRAVEL, tool_slide: 0.0}):
        ctx.expect_contact(gantry, rail_left, name="gantry_contacts_left_rail_end")
        ctx.expect_contact(gantry, rail_right, name="gantry_contacts_right_rail_end")
        ctx.expect_within(gantry, base, axes="y", margin=0.03, name="gantry_within_base_end")
        ctx.expect_contact(tool_truck, gantry, name="tool_truck_on_crossbeam_left")
        ctx.expect_within(tool_truck, gantry, axes="x", margin=0.025, name="truck_within_beam_left")

    with ctx.pose({gantry_slide: GANTRY_TRAVEL * 0.5, tool_slide: TOOL_TRAVEL}):
        ctx.expect_contact(tool_truck, gantry, name="tool_truck_on_crossbeam_right")
        ctx.expect_within(tool_truck, gantry, axes="x", margin=0.025, name="truck_within_beam_right")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
