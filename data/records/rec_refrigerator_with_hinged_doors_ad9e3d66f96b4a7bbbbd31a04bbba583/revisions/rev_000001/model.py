from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_front_slab(
    width: float,
    height: float,
    thickness: float,
    radius: float,
    mesh_name: str,
):
    """A rounded-rectangle appliance panel in local XZ, with thickness along Y."""

    flat = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=10),
        thickness,
        center=True,
    )
    slab = MeshGeometry(
        vertices=[(x, z, y) for (x, y, z) in flat.vertices],
        faces=list(flat.faces),
    )
    return mesh_from_geometry(slab, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="french_door_refrigerator")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("dark_cabinet", rgba=(0.055, 0.058, 0.060, 1.0))
    gasket = model.material("black_rubber_gasket", rgba=(0.006, 0.006, 0.007, 1.0))
    liner = model.material("white_liner", rgba=(0.91, 0.93, 0.90, 1.0))
    rail = model.material("zinc_slide_rail", rgba=(0.58, 0.60, 0.57, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.98, 0.035, 1.84)),
        origin=Origin(xyz=(0.0, 0.3525, 0.92)),
        material=dark,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.04, 0.72, 1.84)),
        origin=Origin(xyz=(-0.49, 0.0, 0.92)),
        material=dark,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((0.04, 0.72, 1.84)),
        origin=Origin(xyz=(0.49, 0.0, 0.92)),
        material=dark,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((0.98, 0.72, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 1.8125)),
        material=dark,
        name="top_cap",
    )
    cabinet.visual(
        Box((0.98, 0.72, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=dark,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.98, 0.70, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.6625)),
        material=dark,
        name="freezer_divider",
    )
    cabinet.visual(
        Box((0.045, 0.055, 1.105)),
        origin=Origin(xyz=(0.0, -0.355, 1.2325)),
        material=dark,
        name="center_mullion",
    )
    cabinet.visual(
        Box((0.94, 0.62, 0.018)),
        origin=Origin(xyz=(0.0, 0.035, 1.06)),
        material=liner,
        name="upper_shelf_0",
    )
    cabinet.visual(
        Box((0.94, 0.62, 0.018)),
        origin=Origin(xyz=(0.0, 0.035, 1.40)),
        material=liner,
        name="upper_shelf_1",
    )
    cabinet.visual(
        Box((0.054, 0.61, 0.048)),
        origin=Origin(xyz=(-0.443, -0.055, 0.350)),
        material=rail,
        name="fixed_rail_0",
    )
    cabinet.visual(
        Box((0.054, 0.61, 0.048)),
        origin=Origin(xyz=(0.443, -0.055, 0.350)),
        material=rail,
        name="fixed_rail_1",
    )
    for side_index, x in enumerate((-0.460, 0.460)):
        bracket_x = -0.493 if x < 0.0 else 0.493
        for level_index, z in enumerate((0.86, 1.24, 1.62)):
            cabinet.visual(
                Box((0.034, 0.092, 0.110)),
                origin=Origin(xyz=(bracket_x, -0.406, z)),
                material=dark,
                name=f"hinge_bracket_{side_index}_{level_index}",
            )

    upper_panel_mesh = _rounded_front_slab(0.430, 1.10, 0.080, 0.035, "upper_door_panel")

    door_0 = model.part("upper_door_0")
    door_0.visual(
        upper_panel_mesh,
        origin=Origin(xyz=(0.2400, -0.025, 0.550)),
        material=stainless,
        name="outer_panel",
    )
    door_0.visual(
        Box((0.395, 0.018, 1.020)),
        origin=Origin(xyz=(0.242, 0.022, 0.550)),
        material=gasket,
        name="inner_gasket",
    )
    door_0.visual(
        Box((0.020, 0.026, 1.070)),
        origin=Origin(xyz=(0.450, -0.067, 0.550)),
        material=gasket,
        name="center_seal",
    )
    door_0.visual(
        Cylinder(radius=0.017, length=0.760),
        origin=Origin(xyz=(0.375, -0.108, 0.565)),
        material=stainless,
        name="vertical_handle",
    )
    door_0.visual(
        Box((0.055, 0.078, 0.040)),
        origin=Origin(xyz=(0.375, -0.076, 0.255)),
        material=stainless,
        name="handle_mount_0",
    )
    door_0.visual(
        Box((0.055, 0.078, 0.040)),
        origin=Origin(xyz=(0.375, -0.076, 0.875)),
        material=stainless,
        name="handle_mount_1",
    )
    door_0.visual(
        Cylinder(radius=0.016, length=1.035),
        origin=Origin(xyz=(0.0, 0.0, 0.552)),
        material=dark,
        name="hinge_barrel",
    )
    door_0.visual(
        Box((0.030, 0.040, 1.020)),
        origin=Origin(xyz=(0.015, 0.012, 0.552)),
        material=dark,
        name="hinge_leaf",
    )

    door_1 = model.part("upper_door_1")
    door_1.visual(
        upper_panel_mesh,
        origin=Origin(xyz=(-0.2400, -0.025, 0.550)),
        material=stainless,
        name="outer_panel",
    )
    door_1.visual(
        Box((0.395, 0.018, 1.020)),
        origin=Origin(xyz=(-0.242, 0.022, 0.550)),
        material=gasket,
        name="inner_gasket",
    )
    door_1.visual(
        Box((0.020, 0.026, 1.070)),
        origin=Origin(xyz=(-0.450, -0.067, 0.550)),
        material=gasket,
        name="center_seal",
    )
    door_1.visual(
        Cylinder(radius=0.017, length=0.760),
        origin=Origin(xyz=(-0.375, -0.108, 0.565)),
        material=stainless,
        name="vertical_handle",
    )
    door_1.visual(
        Box((0.055, 0.078, 0.040)),
        origin=Origin(xyz=(-0.375, -0.076, 0.255)),
        material=stainless,
        name="handle_mount_0",
    )
    door_1.visual(
        Box((0.055, 0.078, 0.040)),
        origin=Origin(xyz=(-0.375, -0.076, 0.875)),
        material=stainless,
        name="handle_mount_1",
    )
    door_1.visual(
        Cylinder(radius=0.016, length=1.035),
        origin=Origin(xyz=(0.0, 0.0, 0.552)),
        material=dark,
        name="hinge_barrel",
    )
    door_1.visual(
        Box((0.030, 0.040, 1.020)),
        origin=Origin(xyz=(-0.015, 0.012, 0.552)),
        material=dark,
        name="hinge_leaf",
    )

    freezer_front_mesh = _rounded_front_slab(0.91, 0.535, 0.090, 0.035, "freezer_drawer_front")
    drawer = model.part("freezer_drawer")
    drawer.visual(
        freezer_front_mesh,
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=stainless,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.780, 0.560, 0.030)),
        origin=Origin(xyz=(0.0, 0.280, -0.178)),
        material=liner,
        name="basket_floor",
    )
    drawer.visual(
        Box((0.030, 0.560, 0.360)),
        origin=Origin(xyz=(-0.390, 0.280, -0.010)),
        material=liner,
        name="basket_side_0",
    )
    drawer.visual(
        Box((0.030, 0.560, 0.360)),
        origin=Origin(xyz=(0.390, 0.280, -0.010)),
        material=liner,
        name="basket_side_1",
    )
    drawer.visual(
        Box((0.780, 0.030, 0.360)),
        origin=Origin(xyz=(0.0, 0.560, -0.010)),
        material=liner,
        name="basket_back",
    )
    drawer.visual(
        Box((0.780, 0.030, 0.270)),
        origin=Origin(xyz=(0.0, 0.010, -0.045)),
        material=liner,
        name="basket_front_wall",
    )
    drawer.visual(
        Box((0.032, 0.800, 0.038)),
        origin=Origin(xyz=(-0.400, 0.180, 0.015)),
        material=rail,
        name="sliding_rail_0",
    )
    drawer.visual(
        Box((0.032, 0.800, 0.038)),
        origin=Origin(xyz=(0.400, 0.180, 0.015)),
        material=rail,
        name="sliding_rail_1",
    )
    drawer.visual(
        Cylinder(radius=0.018, length=0.740),
        origin=Origin(xyz=(0.0, -0.110, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="wide_handle",
    )
    drawer.visual(
        Box((0.045, 0.085, 0.050)),
        origin=Origin(xyz=(-0.315, -0.075, 0.080)),
        material=stainless,
        name="handle_mount_0",
    )
    drawer.visual(
        Box((0.045, 0.085, 0.050)),
        origin=Origin(xyz=(0.315, -0.075, 0.080)),
        material=stainless,
        name="handle_mount_1",
    )

    model.articulation(
        "cabinet_to_upper_door_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_0,
        origin=Origin(xyz=(-0.460, -0.425, 0.690)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cabinet_to_upper_door_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_1,
        origin=Origin(xyz=(0.460, -0.425, 0.690)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cabinet_to_freezer_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.405, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.480),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door_0 = object_model.get_part("upper_door_0")
    door_1 = object_model.get_part("upper_door_1")
    drawer = object_model.get_part("freezer_drawer")
    hinge_0 = object_model.get_articulation("cabinet_to_upper_door_0")
    hinge_1 = object_model.get_articulation("cabinet_to_upper_door_1")
    slide = object_model.get_articulation("cabinet_to_freezer_drawer")

    ctx.check(
        "upper doors use vertical revolute hinges",
        hinge_0.articulation_type == ArticulationType.REVOLUTE
        and hinge_1.articulation_type == ArticulationType.REVOLUTE
        and abs(hinge_0.axis[2]) > 0.99
        and abs(hinge_1.axis[2]) > 0.99,
        details=f"axes={hinge_0.axis}, {hinge_1.axis}",
    )
    ctx.check(
        "freezer drawer uses outward prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis[1] < -0.99
        and slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.45,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )

    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        min_gap=0.004,
        max_gap=0.030,
        positive_elem="outer_panel",
        negative_elem="outer_panel",
        name="upper door panels meet with a narrow center gap",
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="y",
        elem_a="sliding_rail_0",
        elem_b="fixed_rail_0",
        min_overlap=0.35,
        name="closed drawer rails are deeply nested",
    )

    def _aabb_center_y(part, elem: str):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return 0.5 * (lo[1] + hi[1])

    left_closed_y = _aabb_center_y(door_0, "outer_panel")
    right_closed_y = _aabb_center_y(door_1, "outer_panel")
    drawer_closed = ctx.part_world_position(drawer)
    with ctx.pose({hinge_0: 1.2, hinge_1: 1.2, slide: 0.48}):
        left_open_y = _aabb_center_y(door_0, "outer_panel")
        right_open_y = _aabb_center_y(door_1, "outer_panel")
        drawer_open = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="sliding_rail_0",
            elem_b="fixed_rail_0",
            min_overlap=0.035,
            name="extended freezer rail retains engagement",
        )

    ctx.check(
        "upper doors swing outward from the cabinet front",
        left_closed_y is not None
        and right_closed_y is not None
        and left_open_y is not None
        and right_open_y is not None
        and left_open_y < left_closed_y - 0.18
        and right_open_y < right_closed_y - 0.18,
        details=f"closed_y={left_closed_y},{right_closed_y}; open_y={left_open_y},{right_open_y}",
    )
    ctx.check(
        "freezer drawer slides outward",
        drawer_closed is not None
        and drawer_open is not None
        and drawer_open[1] < drawer_closed[1] - 0.45,
        details=f"closed={drawer_closed}, open={drawer_open}",
    )

    return ctx.report()


object_model = build_object_model()
