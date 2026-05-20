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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.16
BASE_THICKNESS = 0.02

BEARING_CENTER_X = 0.12
SHAFT_CENTER_HEIGHT = 0.085

BLOCK_LENGTH = 0.09
BLOCK_WIDTH = 0.11
BLOCK_BOTTOM_TO_CENTER = 0.065
BLOCK_BASE_THICKNESS = 0.02
BLOCK_PEDESTAL_HEIGHT = 0.03
BLOCK_HOUSING_LENGTH = 0.07
BLOCK_OUTER_RADIUS = 0.035
BLOCK_BORE_RADIUS = 0.015
BLOCK_SEAM_GAP = 0.0025

SHAFT_RADIUS = 0.015
SHAFT_LENGTH = 0.40

COLLAR_CENTER_X = -0.177
COLLAR_LENGTH = 0.014
COLLAR_RADIUS = 0.026

FLANGE_CENTER_X = 0.179
FLANGE_THICKNESS = 0.012
FLANGE_RADIUS = 0.037


def _base_plate_shape() -> cq.Workplane:
    slot_x = BASE_LENGTH * 0.34
    slot_y = BASE_WIDTH * 0.33
    return (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-slot_x, -slot_y),
                (-slot_x, slot_y),
                (slot_x, -slot_y),
                (slot_x, slot_y),
            ]
        )
        .slot2D(0.026, 0.012, angle=0.0)
        .cutThruAll()
    )


def _bearing_block_shape() -> cq.Workplane:
    bottom_z = -BLOCK_BOTTOM_TO_CENTER
    foot = (
        cq.Workplane("XY")
        .box(
            BLOCK_LENGTH,
            BLOCK_WIDTH,
            BLOCK_BASE_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, bottom_z))
    )
    pedestal = (
        cq.Workplane("XY")
        .box(
            BLOCK_HOUSING_LENGTH,
            BLOCK_WIDTH,
            BLOCK_PEDESTAL_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, bottom_z + BLOCK_BASE_THICKNESS))
    )
    housing = (
        cq.Workplane("YZ", origin=(0.0, 0.0, 0.0))
        .circle(BLOCK_OUTER_RADIUS)
        .extrude(BLOCK_HOUSING_LENGTH / 2.0, both=True)
    )
    shoulders = (
        cq.Workplane("XY")
        .box(0.076, 0.094, 0.016, centered=(True, True, False))
        .translate((0.0, 0.0, 0.008))
    )

    block = foot.union(pedestal).union(housing).union(shoulders).combine().clean()

    bore = (
        cq.Workplane("YZ", origin=(0.0, 0.0, 0.0))
        .circle(BLOCK_BORE_RADIUS)
        .extrude((BLOCK_LENGTH + 0.01) / 2.0, both=True)
    )
    split_gap = (
        cq.Workplane("XY")
        .box(0.08, BLOCK_WIDTH + 0.01, BLOCK_SEAM_GAP, centered=(True, True, True))
    )

    bolt_points = [
        (-0.022, -0.038),
        (-0.022, 0.038),
        (0.022, -0.038),
        (0.022, 0.038),
    ]
    studs = (
        cq.Workplane("XY")
        .pushPoints(bolt_points)
        .circle(0.005)
        .extrude(0.056)
        .translate((0.0, 0.0, -0.028))
    )
    top_heads = (
        cq.Workplane("XY")
        .pushPoints(bolt_points)
        .circle(0.008)
        .extrude(0.006)
        .translate((0.0, 0.0, 0.022))
    )
    bottom_heads = (
        cq.Workplane("XY")
        .pushPoints(bolt_points)
        .circle(0.008)
        .extrude(0.006)
        .translate((0.0, 0.0, -0.028))
    )

    return (
        block.cut(bore)
        .cut(split_gap)
        .union(studs)
        .union(top_heads)
        .union(bottom_heads)
        .combine()
        .clean()
    )


def _shaft_body_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(0.0, 0.0, 0.0))
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_LENGTH / 2.0, both=True)
    )


def _stop_collar_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("YZ", origin=(COLLAR_CENTER_X, 0.0, 0.0))
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_LENGTH / 2.0, both=True)
    )
    clamp_boss = (
        cq.Workplane("XY")
        .box(0.012, 0.014, 0.010, centered=(True, True, False))
        .translate((COLLAR_CENTER_X, 0.0, COLLAR_RADIUS))
    )
    set_screw_hole = (
        cq.Workplane("XZ", origin=(COLLAR_CENTER_X, 0.0, 0.0))
        .circle(0.0025)
        .extrude(0.02, both=True)
        .translate((0.0, 0.0, COLLAR_RADIUS + 0.003))
    )
    return ring.union(clamp_boss).cut(set_screw_hole)


def _flange_shape() -> cq.Workplane:
    disc = (
        cq.Workplane("YZ", origin=(FLANGE_CENTER_X, 0.0, 0.0))
        .circle(FLANGE_RADIUS)
        .extrude(FLANGE_THICKNESS / 2.0, both=True)
    )
    bolt_holes = (
        cq.Workplane("YZ", origin=(FLANGE_CENTER_X, 0.0, 0.0))
        .pushPoints([(0.024, 0.0), (-0.024, 0.0), (0.0, 0.024), (0.0, -0.024)])
        .circle(0.004)
        .extrude((FLANGE_THICKNESS + 0.002) / 2.0, both=True)
    )
    return disc.cut(bolt_holes)


def _bearing_cap_shape() -> cq.Workplane:
    outer_shell = (
        cq.Workplane("YZ")
        .circle(0.041)
        .extrude(BLOCK_HOUSING_LENGTH / 2.0, both=True)
    )
    inner_relief = (
        cq.Workplane("YZ")
        .circle(0.023)
        .extrude((BLOCK_HOUSING_LENGTH + 0.004) / 2.0, both=True)
    )
    top_crop = (
        cq.Workplane("XY")
        .box(BLOCK_HOUSING_LENGTH + 0.004, 0.100, 0.042, centered=(True, True, False))
        .translate((0.0, 0.0, 0.006))
    )
    cap = outer_shell.cut(inner_relief).intersect(top_crop)
    seam_notch = (
        cq.Workplane("XY")
        .box(BLOCK_HOUSING_LENGTH + 0.006, 0.094, 0.002, centered=(True, True, True))
        .translate((0.0, 0.0, 0.004))
    )
    return cap.cut(seam_notch).edges("|X").fillet(0.003)


def _add_bearing_block_visuals(part, cap_mesh_name: str) -> None:
    part.visual(
        Box((BLOCK_LENGTH, BLOCK_WIDTH, BLOCK_BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material="cast_housing",
        name="foot",
    )
    part.visual(
        Box((BLOCK_HOUSING_LENGTH, BLOCK_WIDTH, BLOCK_PEDESTAL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material="cast_housing",
        name="lower_body",
    )
    part.visual(
        Box((BLOCK_HOUSING_LENGTH, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, 0.034, 0.001)),
        material="cast_housing",
        name="left_cheek",
    )
    part.visual(
        Box((BLOCK_HOUSING_LENGTH, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, -0.034, 0.001)),
        material="cast_housing",
        name="right_cheek",
    )
    part.visual(
        mesh_from_cadquery(_bearing_cap_shape(), cap_mesh_name),
        material="cast_housing",
        name="top_cap",
    )

    for index, (x_pos, y_pos) in enumerate(
        [
            (-0.022, -0.038),
            (-0.022, 0.038),
            (0.022, -0.038),
            (0.022, 0.038),
        ],
        start=1,
    ):
        part.visual(
            Cylinder(radius=0.0055, length=0.056),
            origin=Origin(xyz=(x_pos, y_pos, 0.0)),
            material="machined_steel",
            name=f"tie_bolt_{index}",
        )
        part.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.025)),
            material="machined_steel",
            name=f"upper_nut_{index}",
        )
        part.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, -0.025)),
            material="machined_steel",
            name=f"lower_nut_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_bearing_shaft_module")

    model.material("painted_base", rgba=(0.20, 0.23, 0.25, 1.0))
    model.material("cast_housing", rgba=(0.39, 0.43, 0.47, 1.0))
    model.material("machined_steel", rgba=(0.73, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate"),
        material="painted_base",
        name="base_plate",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    left_block = model.part("left_bearing_block")
    _add_bearing_block_visuals(left_block, "left_bearing_cap")
    left_block.inertial = Inertial.from_geometry(
        Box((BLOCK_LENGTH, BLOCK_WIDTH, BLOCK_BOTTOM_TO_CENTER + BLOCK_OUTER_RADIUS)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
    )

    right_block = model.part("right_bearing_block")
    _add_bearing_block_visuals(right_block, "right_bearing_cap")
    right_block.inertial = Inertial.from_geometry(
        Box((BLOCK_LENGTH, BLOCK_WIDTH, BLOCK_BOTTOM_TO_CENTER + BLOCK_OUTER_RADIUS)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        mesh_from_cadquery(_shaft_body_shape(), "shaft_body"),
        material="machined_steel",
        name="shaft_body",
    )
    shaft.visual(
        mesh_from_cadquery(_stop_collar_shape(), "stop_collar"),
        material="machined_steel",
        name="stop_collar",
    )
    shaft.visual(
        mesh_from_cadquery(_flange_shape(), "shaft_flange"),
        material="machined_steel",
        name="flange",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.02, length=SHAFT_LENGTH),
        mass=3.0,
        origin=Origin(),
    )

    model.articulation(
        "base_to_left_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=left_block,
        origin=Origin(xyz=(-BEARING_CENTER_X, 0.0, SHAFT_CENTER_HEIGHT)),
    )
    model.articulation(
        "base_to_right_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=right_block,
        origin=Origin(xyz=(BEARING_CENTER_X, 0.0, SHAFT_CENTER_HEIGHT)),
    )
    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_CENTER_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
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

    base = object_model.get_part("base")
    left_block = object_model.get_part("left_bearing_block")
    right_block = object_model.get_part("right_bearing_block")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("base_to_shaft")

    ctx.allow_isolated_part(
        shaft,
        reason="The shaft is intentionally modeled with running clearance inside the split bearing blocks rather than fused contact.",
    )

    ctx.expect_gap(
        left_block,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        negative_elem="base_plate",
        name="left bearing block seats on the base",
    )
    ctx.expect_gap(
        right_block,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        negative_elem="base_plate",
        name="right bearing block seats on the base",
    )

    ctx.expect_origin_distance(
        shaft,
        left_block,
        axes="yz",
        max_dist=0.001,
        name="shaft axis aligns with left bearing centerline",
    )
    ctx.expect_origin_distance(
        shaft,
        right_block,
        axes="yz",
        max_dist=0.001,
        name="shaft axis aligns with right bearing centerline",
    )

    ctx.expect_overlap(
        shaft,
        left_block,
        axes="x",
        min_overlap=0.06,
        elem_a="shaft_body",
        name="shaft body spans through the left bearing block",
    )
    ctx.expect_overlap(
        shaft,
        right_block,
        axes="x",
        min_overlap=0.06,
        elem_a="shaft_body",
        name="shaft body spans through the right bearing block",
    )

    ctx.expect_gap(
        left_block,
        shaft,
        axis="x",
        min_gap=0.001,
        max_gap=0.008,
        negative_elem="stop_collar",
        name="stop collar sits just outboard of the left bearing",
    )
    ctx.expect_gap(
        shaft,
        right_block,
        axis="x",
        min_gap=0.003,
        max_gap=0.010,
        positive_elem="flange",
        name="flange sits just outboard of the right bearing",
    )

    ctx.check(
        "shaft articulation is continuous about the shaft axis",
        shaft_spin.joint_type == ArticulationType.CONTINUOUS and shaft_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={shaft_spin.joint_type}, axis={shaft_spin.axis}",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    collar_at_rest = ctx.part_element_world_aabb(shaft, elem="stop_collar")
    with ctx.pose({shaft_spin: 1.57079632679}):
        collar_at_quarter_turn = ctx.part_element_world_aabb(shaft, elem="stop_collar")

    rest_center = _aabb_center(collar_at_rest)
    quarter_turn_center = _aabb_center(collar_at_quarter_turn)
    ctx.check(
        "stop collar rotates with the shaft",
        rest_center is not None
        and quarter_turn_center is not None
        and abs(rest_center[2] - quarter_turn_center[2]) > 0.004,
        details=f"rest_center={rest_center}, quarter_turn_center={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
