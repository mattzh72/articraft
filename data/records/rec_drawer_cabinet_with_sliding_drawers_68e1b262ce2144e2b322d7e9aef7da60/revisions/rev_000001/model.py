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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.94
BODY_DEPTH = 0.48
BODY_BOTTOM_Z = 0.12
BODY_TOP_Z = 0.82
SIDE_THICKNESS = 0.028
BACK_THICKNESS = 0.022
BOTTOM_THICKNESS = 0.020
TRAY_FLOOR_THICKNESS = 0.018

DRAWER_FRONT_WIDTH = 0.882
DRAWER_FRONT_HEIGHT = 0.074
DRAWER_FRONT_THICKNESS = 0.018
DRAWER_BOX_WIDTH = 0.848
DRAWER_BOX_DEPTH = 0.390
DRAWER_BOX_HEIGHT = 0.064
DRAWER_BOX_THICKNESS = 0.012
DRAWER_FLOOR_THICKNESS = 0.010
DRAWER_PITCH = 0.080
DRAWER_TRAVEL = 0.31
LOWEST_DRAWER_CENTER_Z = 0.227

HALF_BODY_WIDTH = BODY_WIDTH * 0.5
HALF_BODY_DEPTH = BODY_DEPTH * 0.5


def _drawer_center_z(index: int) -> float:
    return LOWEST_DRAWER_CENTER_Z + index * DRAWER_PITCH


def _add_caster(part, *, x: float, y: float, index: int, steel, dark, rubber) -> None:
    part.visual(
        Box((0.060, 0.050, 0.016)),
        origin=Origin(xyz=(x, y, 0.112)),
        material=steel,
        name=f"caster_{index}_plate",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(x, y, 0.093)),
        material=dark,
        name=f"caster_{index}_swivel",
    )
    part.visual(
        Box((0.050, 0.032, 0.018)),
        origin=Origin(xyz=(x, y, 0.073)),
        material=dark,
        name=f"caster_{index}_bridge",
    )
    for side, x_offset in (("left", -0.018), ("right", 0.018)):
        part.visual(
            Box((0.008, 0.034, 0.042)),
            origin=Origin(xyz=(x + x_offset, y, 0.043)),
            material=dark,
            name=f"caster_{index}_{side}_fork",
        )
    part.visual(
        Cylinder(radius=0.005, length=0.044),
        origin=Origin(xyz=(x, y, 0.037), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name=f"caster_{index}_axle",
    )
    part.visual(
        Cylinder(radius=0.037, length=0.028),
        origin=Origin(xyz=(x, y, 0.037), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name=f"caster_{index}_wheel",
    )


def _build_drawer(model: ArticulatedObject, cabinet, *, index: int, front_z: float, red, black, steel) -> None:
    drawer_name = f"drawer_{index + 1}"
    drawer = model.part(drawer_name)

    drawer.visual(
        Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
        origin=Origin(),
        material=red,
        name="front_panel",
    )
    drawer.visual(
        Cylinder(radius=0.007, length=0.360),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pull_handle",
    )
    for post_index, x_offset in enumerate((-0.130, 0.130), start=1):
        drawer.visual(
            Cylinder(radius=0.005, length=0.024),
            origin=Origin(
                xyz=(x_offset, -0.018, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black,
            name=f"handle_post_{post_index}",
        )

    side_center_z = -0.002
    side_center_y = 0.204
    side_x = DRAWER_BOX_WIDTH * 0.5 - DRAWER_BOX_THICKNESS * 0.5
    drawer.visual(
        Box((DRAWER_BOX_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(xyz=(-side_x, side_center_y, side_center_z)),
        material=red,
        name="left_side",
    )
    drawer.visual(
        Box((DRAWER_BOX_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(xyz=(side_x, side_center_y, side_center_z)),
        material=red,
        name="right_side",
    )
    drawer.visual(
        Box(
            (
                DRAWER_BOX_WIDTH - 2.0 * DRAWER_BOX_THICKNESS,
                DRAWER_BOX_DEPTH,
                DRAWER_FLOOR_THICKNESS,
            )
        ),
        origin=Origin(xyz=(0.0, side_center_y, -0.029)),
        material=red,
        name="drawer_floor",
    )
    drawer.visual(
        Box((DRAWER_BOX_WIDTH, DRAWER_BOX_THICKNESS, 0.056)),
        origin=Origin(xyz=(0.0, 0.393, -0.002)),
        material=red,
        name="back_panel",
    )

    inner_slide_x = DRAWER_BOX_WIDTH * 0.5 + 0.004
    for side, sign in (("left", -1.0), ("right", 1.0)):
        drawer.visual(
            Box((0.008, 0.420, 0.018)),
            origin=Origin(xyz=(sign * inner_slide_x, 0.219, -0.004)),
            material=steel,
            name=f"{side}_slide_inner",
        )

    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FRONT_WIDTH, 0.430, DRAWER_FRONT_HEIGHT)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.170, -0.002)),
    )

    model.articulation(
        f"cabinet_to_{drawer_name}",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.250, front_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_mechanics_tool_chest")

    red = model.material("powder_red", rgba=(0.73, 0.09, 0.08, 1.0))
    black = model.material("satin_black", rgba=(0.13, 0.13, 0.14, 1.0))
    steel = model.material("zinc_steel", rgba=(0.64, 0.66, 0.69, 1.0))
    dark = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((SIDE_THICKNESS, BODY_DEPTH, BODY_TOP_Z - BODY_BOTTOM_Z)),
        origin=Origin(
            xyz=(
                -HALF_BODY_WIDTH + SIDE_THICKNESS * 0.5,
                0.0,
                (BODY_BOTTOM_Z + BODY_TOP_Z) * 0.5,
            )
        ),
        material=red,
        name="left_side_wall",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, BODY_DEPTH, BODY_TOP_Z - BODY_BOTTOM_Z)),
        origin=Origin(
            xyz=(
                HALF_BODY_WIDTH - SIDE_THICKNESS * 0.5,
                0.0,
                (BODY_BOTTOM_Z + BODY_TOP_Z) * 0.5,
            )
        ),
        material=red,
        name="right_side_wall",
    )
    cabinet.visual(
        Box((BODY_WIDTH, BACK_THICKNESS, BODY_TOP_Z - BODY_BOTTOM_Z)),
        origin=Origin(
            xyz=(0.0, HALF_BODY_DEPTH - BACK_THICKNESS * 0.5, (BODY_BOTTOM_Z + BODY_TOP_Z) * 0.5)
        ),
        material=red,
        name="back_wall",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * SIDE_THICKNESS, BODY_DEPTH - BACK_THICKNESS - 0.022, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BOTTOM_THICKNESS * 0.5)),
        material=red,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * SIDE_THICKNESS, BODY_DEPTH - BACK_THICKNESS - 0.022, TRAY_FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.699)),
        material=red,
        name="tray_floor",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * SIDE_THICKNESS, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, -0.230, 0.134)),
        material=red,
        name="lower_front_apron",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 2.0 * SIDE_THICKNESS, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.231, 0.795)),
        material=red,
        name="top_front_rim",
    )
    cabinet.visual(
        Box((0.080, 0.024, 0.010)),
        origin=Origin(xyz=(-0.290, 0.236, 0.815)),
        material=steel,
        name="left_hinge_leaf",
    )
    cabinet.visual(
        Box((0.080, 0.024, 0.010)),
        origin=Origin(xyz=(0.290, 0.236, 0.815)),
        material=steel,
        name="right_hinge_leaf",
    )

    for drawer_index in range(6):
        drawer_z = _drawer_center_z(drawer_index)
        for side, sign in (("left", -1.0), ("right", 1.0)):
            cabinet.visual(
                Box((0.010, 0.430, 0.020)),
                origin=Origin(xyz=(sign * 0.437, -0.006, drawer_z - 0.004)),
                material=steel,
                name=f"drawer_{drawer_index + 1}_{side}_slide_outer",
            )

    caster_positions = (
        (-0.365, -0.175),
        (0.365, -0.175),
        (-0.365, 0.175),
        (0.365, 0.175),
    )
    for caster_index, (x, y) in enumerate(caster_positions, start=1):
        _add_caster(cabinet, x=x, y=y, index=caster_index, steel=steel, dark=dark, rubber=rubber)

    cabinet.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_TOP_Z)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((BODY_WIDTH, BODY_DEPTH + 0.008, 0.022)),
        origin=Origin(xyz=(0.0, -0.244, 0.011)),
        material=red,
        name="lid_panel",
    )
    lid.visual(
        Box((0.018, BODY_DEPTH - 0.022, 0.032)),
        origin=Origin(xyz=(-0.479, -0.229, 0.005)),
        material=red,
        name="left_skirt",
    )
    lid.visual(
        Box((0.018, BODY_DEPTH - 0.022, 0.032)),
        origin=Origin(xyz=(0.479, -0.229, 0.005)),
        material=red,
        name="right_skirt",
    )
    lid.visual(
        Box((BODY_WIDTH - 0.040, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.494, 0.001)),
        material=red,
        name="front_skirt",
    )
    lid.visual(
        Box((0.080, 0.024, 0.010)),
        origin=Origin(xyz=(-0.290, -0.012, 0.027)),
        material=steel,
        name="left_hinge_leaf",
    )
    lid.visual(
        Box((0.080, 0.024, 0.010)),
        origin=Origin(xyz=(0.290, -0.012, 0.027)),
        material=steel,
        name="right_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.300),
        origin=Origin(xyz=(0.0, -0.507, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lid_handle",
    )
    for post_index, x_offset in enumerate((-0.105, 0.105), start=1):
        lid.visual(
            Cylinder(radius=0.0055, length=0.024),
            origin=Origin(
                xyz=(x_offset, -0.495, 0.006),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black,
            name=f"lid_handle_post_{post_index}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH + 0.020, 0.050)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -0.244, 0.014)),
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, HALF_BODY_DEPTH, BODY_TOP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=0.0,
            upper=1.55,
        ),
    )

    for drawer_index in range(6):
        _build_drawer(
            model,
            cabinet,
            index=drawer_index,
            front_z=_drawer_center_z(drawer_index),
            red=red,
            black=black,
            steel=steel,
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

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")

    cabinet.get_visual("left_hinge_leaf")
    cabinet.get_visual("right_hinge_leaf")
    lid.get_visual("left_hinge_leaf")
    lid.get_visual("right_hinge_leaf")
    ctx.check("lid has two visible hinge leaves", True)

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="xy",
            elem_a="lid_panel",
            elem_b="tray_floor",
            min_overlap=0.40,
            name="closed lid covers the full top tray",
        )
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="top_front_rim",
            max_gap=0.002,
            max_penetration=0.0005,
            name="closed lid seats on the tray rim",
        )
        closed_front_skirt = ctx.part_element_world_aabb(lid, elem="front_skirt")

    with ctx.pose({lid_hinge: 1.35}):
        open_front_skirt = ctx.part_element_world_aabb(lid, elem="front_skirt")

    ctx.check(
        "lid front edge lifts upward when opened",
        closed_front_skirt is not None
        and open_front_skirt is not None
        and open_front_skirt[1][2] > closed_front_skirt[1][2] + 0.25
        and open_front_skirt[1][1] > closed_front_skirt[1][1] + 0.12,
        details=f"closed={closed_front_skirt}, open={open_front_skirt}",
    )

    for drawer_index in range(6):
        drawer_name = f"drawer_{drawer_index + 1}"
        drawer = object_model.get_part(drawer_name)
        slide_joint = object_model.get_articulation(f"cabinet_to_{drawer_name}")

        with ctx.pose({slide_joint: 0.0}):
            closed_front = ctx.part_element_world_aabb(drawer, elem="front_panel")
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="x",
                elem_a="front_panel",
                elem_b="bottom_panel",
                min_overlap=0.84,
                name=f"{drawer_name} stays centered in the cabinet width",
            )

        with ctx.pose({slide_joint: DRAWER_TRAVEL}):
            open_front = ctx.part_element_world_aabb(drawer, elem="front_panel")
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                min_overlap=0.08,
                name=f"{drawer_name} retains insertion on its full-extension slides",
            )

        ctx.check(
            f"{drawer_name} opens forward from the cabinet",
            closed_front is not None
            and open_front is not None
            and open_front[1][1] < closed_front[1][1] - 0.20,
            details=f"closed={closed_front}, open={open_front}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
