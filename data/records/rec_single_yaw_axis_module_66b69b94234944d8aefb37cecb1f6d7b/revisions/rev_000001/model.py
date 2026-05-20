from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


BASE_X = 0.22
BASE_Y = 0.14
BASE_THICKNESS = 0.012
CHEEK_X = 0.12
CHEEK_Y = 0.014
CHEEK_HEIGHT = 0.044
CHEEK_CENTER_Y = 0.056
BEARING_OUTER_R = 0.028
BEARING_INNER_R = 0.018
BEARING_HEIGHT = 0.033
BEARING_TOP_Z = 0.044
SHAFT_R = 0.014
TOP_X = 0.12
TOP_Y = 0.075
TOP_THICKNESS = 0.012
YAW_LIMIT = 1.5708


def _base_frame_shape() -> cq.Workplane:
    """Grounded base plate with two fixed split cheeks."""
    base = (
        cq.Workplane("XY")
        .box(BASE_X, BASE_Y, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.006)
    )

    cheek_z = BASE_THICKNESS - 0.001 + CHEEK_HEIGHT / 2.0
    cheek = (
        cq.Workplane("XY")
        .box(CHEEK_X, CHEEK_Y, CHEEK_HEIGHT)
        .translate((0.0, CHEEK_CENTER_Y, cheek_z))
        .edges("|Z")
        .fillet(0.003)
    )
    opposite_cheek = cheek.mirror(mirrorPlane="XZ")

    return base.union(cheek).union(opposite_cheek)


def _bearing_sleeve_shape() -> cq.Workplane:
    """A real annular bearing sleeve, clearanced around the rotating journal."""
    center_z = BEARING_TOP_Z - BEARING_HEIGHT / 2.0
    outer = cq.Workplane("XY").cylinder(BEARING_HEIGHT, BEARING_OUTER_R).translate((0.0, 0.0, center_z))
    inner = cq.Workplane("XY").cylinder(BEARING_HEIGHT + 0.004, BEARING_INNER_R).translate((0.0, 0.0, center_z))
    return outer.cut(inner)


def _top_plate_shape() -> cq.Workplane:
    """The compact rectangular rotating table, with visible machined screw holes."""
    plate_center_z = 0.02025
    plate = (
        cq.Workplane("XY")
        .box(TOP_X, TOP_Y, TOP_THICKNESS)
        .translate((0.0, 0.0, plate_center_z))
        .edges("|Z")
        .fillet(0.005)
    )

    # Through holes in the plate make the fastener layout read as a machined stage.
    for x in (-0.040, 0.040):
        for y in (-0.022, 0.022):
            hole = cq.Workplane("XY").cylinder(TOP_THICKNESS + 0.004, 0.0034).translate((x, y, plate_center_z))
            plate = plate.cut(hole)

    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_yaw_stage")

    anodized_black = model.material("anodized_black", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    screw_black = Material("screw_black", rgba=(0.005, 0.005, 0.006, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_frame_shape(), "base_frame", tolerance=0.0008, angular_tolerance=0.08),
        material=anodized_black,
        name="base_frame",
    )
    base.visual(
        mesh_from_cadquery(_bearing_sleeve_shape(), "bearing_sleeve", tolerance=0.0007, angular_tolerance=0.06),
        material=dark_steel,
        name="bearing_sleeve",
    )

    # Fixed socket-head screws clamp the grounded plate; each head sits on the plate.
    for i, (x, y) in enumerate(((-0.085, -0.050), (-0.085, 0.050), (0.085, -0.050), (0.085, 0.050))):
        base.visual(
            Cylinder(radius=0.005, length=0.003),
            origin=Origin(xyz=(x, y, BASE_THICKNESS + 0.0015)),
            material=screw_black,
            name=f"base_screw_{i}",
        )

    top = model.part("top_plate")
    top.visual(
        mesh_from_cadquery(_top_plate_shape(), "top_plate", tolerance=0.0007, angular_tolerance=0.06),
        material=brushed_aluminum,
        name="top_plate",
    )
    for i, (x, y) in enumerate(((-0.040, -0.022), (-0.040, 0.022), (0.040, -0.022), (0.040, 0.022))):
        top.visual(
            Cylinder(radius=0.0042, length=0.0022),
            origin=Origin(xyz=(x, y, 0.0271)),
            material=screw_black,
            name=f"top_screw_{i}",
        )
    top.visual(
        Cylinder(radius=SHAFT_R, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, -0.0135)),
        material=dark_steel,
        name="journal_shaft",
    )
    top.visual(
        Cylinder(radius=0.024, length=0.0145),
        origin=Origin(xyz=(0.0, 0.0, 0.00725)),
        material=dark_steel,
        name="hub_cap",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, BEARING_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-YAW_LIMIT, upper=YAW_LIMIT, effort=12.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    top = object_model.get_part("top_plate")
    yaw = object_model.get_articulation("yaw")

    ctx.expect_within(
        top,
        base,
        axes="xy",
        inner_elem="journal_shaft",
        outer_elem="bearing_sleeve",
        margin=0.001,
        name="journal is centered in the split-cheek bearing sleeve",
    )
    ctx.expect_overlap(
        top,
        base,
        axes="z",
        elem_a="journal_shaft",
        elem_b="bearing_sleeve",
        min_overlap=0.020,
        name="journal remains inserted through the yaw bearing",
    )
    ctx.expect_gap(
        top,
        base,
        axis="z",
        positive_elem="hub_cap",
        negative_elem="bearing_sleeve",
        max_gap=0.001,
        max_penetration=0.0,
        name="hub cap seats on the bearing sleeve",
    )

    with ctx.pose({yaw: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(top, elem="top_plate")
    with ctx.pose({yaw: YAW_LIMIT}):
        turned_aabb = ctx.part_element_world_aabb(top, elem="top_plate")

    if rest_aabb is not None and turned_aabb is not None:
        rest_min, rest_max = rest_aabb
        turned_min, turned_max = turned_aabb
        rest_dx = rest_max[0] - rest_min[0]
        rest_dy = rest_max[1] - rest_min[1]
        turned_dx = turned_max[0] - turned_min[0]
        turned_dy = turned_max[1] - turned_min[1]
        ctx.check(
            "top plate yaws about the vertical hub",
            rest_dx > rest_dy and turned_dy > turned_dx,
            details=f"rest=({rest_dx:.3f}, {rest_dy:.3f}), turned=({turned_dx:.3f}, {turned_dy:.3f})",
        )
    else:
        ctx.fail("top plate yaws about the vertical hub", "could not measure top plate AABBs")

    return ctx.report()


object_model = build_object_model()
