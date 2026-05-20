from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="height_adjustable_standing_desk")

    warm_bamboo = model.material("warm_bamboo", rgba=(0.72, 0.50, 0.28, 1.0))
    dark_edge = model.material("dark_edge_band", rgba=(0.22, 0.18, 0.14, 1.0))
    black_metal = model.material("satin_black_metal", rgba=(0.03, 0.035, 0.04, 1.0))
    inner_metal = model.material("brushed_inner_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    rail_metal = model.material("zinc_drawer_rail", rgba=(0.64, 0.66, 0.67, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.58, 0.38, 0.20, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base_frame")

    # Two long, flat feet tied by a low cross tube so the base is one real frame.
    for x in (-0.45, 0.45):
        base.visual(
            Box((0.16, 0.78, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.030)),
            material=black_metal,
            name=f"foot_{0 if x < 0 else 1}",
        )
        for y in (-0.32, 0.32):
            base.visual(
                Cylinder(radius=0.035, length=0.012),
                origin=Origin(xyz=(x, y, -0.006)),
                material=rubber,
                name=f"leveler_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    base.visual(
        Box((1.02, 0.055, 0.050)),
        origin=Origin(xyz=(0.0, 0.255, 0.055)),
        material=black_metal,
        name="rear_low_tie",
    )
    base.visual(
        Box((0.92, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, -0.170, 0.052)),
        material=black_metal,
        name="front_low_tie",
    )

    def add_lower_sleeve(x: float, suffix: int) -> None:
        """Four connected walls leave a real rectangular void for the slider."""
        outer_x = 0.130
        outer_y = 0.110
        wall = 0.014
        height = 0.600
        zc = 0.060 + height / 2.0
        base.visual(
            Box((outer_x, wall, height)),
            origin=Origin(xyz=(x, outer_y / 2.0 - wall / 2.0, zc)),
            material=black_metal,
            name=f"front_sleeve_wall_{suffix}",
        )
        base.visual(
            Box((outer_x, wall, height)),
            origin=Origin(xyz=(x, -outer_y / 2.0 + wall / 2.0, zc)),
            material=black_metal,
            name=f"rear_sleeve_wall_{suffix}",
        )
        base.visual(
            Box((wall, outer_y - 2.0 * wall, height)),
            origin=Origin(xyz=(x + outer_x / 2.0 - wall / 2.0, 0.0, zc)),
            material=black_metal,
            name=f"side_sleeve_wall_{suffix}_0",
        )
        base.visual(
            Box((wall, outer_y - 2.0 * wall, height)),
            origin=Origin(xyz=(x - outer_x / 2.0 + wall / 2.0, 0.0, zc)),
            material=black_metal,
            name=f"side_sleeve_wall_{suffix}_1",
        )
        base.visual(
            Box((0.180, 0.150, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.067)),
            material=black_metal,
            name=f"sleeve_base_plate_{suffix}",
        )

    add_lower_sleeve(-0.45, 0)
    add_lower_sleeve(0.45, 1)

    # The two moving upper tubes telescope through the hollow lower sleeves.
    for suffix, x in enumerate((-0.45, 0.45)):
        column = model.part(f"column_{suffix}")
        column.visual(
            Box((0.102, 0.082, 0.581)),
            # At full height this still leaves about 60 mm inserted in the sleeve.
            origin=Origin(xyz=(0.0, 0.0, -0.2195)),
            material=inner_metal,
            name="inner_tube",
        )
        column.visual(
            Box((0.110, 0.090, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.0765)),
            material=inner_metal,
            name="top_mount_plate",
        )
        model.articulation(
            f"base_to_column_{suffix}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=column,
            origin=Origin(xyz=(x, 0.0, 0.660)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=900.0, velocity=0.08, lower=0.0, upper=0.450),
            mimic=Mimic(joint="base_to_column_0") if suffix == 1 else None,
        )

    worktop = model.part("worktop")
    worktop.visual(
        Box((1.500, 0.750, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=warm_bamboo,
        name="desktop_slab",
    )
    # Dark edge banding makes the broad rectangular top read as a finished panel.
    worktop.visual(
        Box((1.520, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, -0.384, 0.0)),
        material=dark_edge,
        name="front_edge_band",
    )
    worktop.visual(
        Box((1.520, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, 0.384, 0.0)),
        material=dark_edge,
        name="rear_edge_band",
    )
    worktop.visual(
        Box((0.018, 0.750, 0.052)),
        origin=Origin(xyz=(-0.759, 0.0, 0.0)),
        material=dark_edge,
        name="end_edge_band_0",
    )
    worktop.visual(
        Box((0.018, 0.750, 0.052)),
        origin=Origin(xyz=(0.759, 0.0, 0.0)),
        material=dark_edge,
        name="end_edge_band_1",
    )

    # A shallow apron under the slab, interrupted at the front for the pencil drawer.
    worktop.visual(
        Box((1.400, 0.040, 0.100)),
        origin=Origin(xyz=(0.0, 0.345, -0.0725)),
        material=black_metal,
        name="rear_apron",
    )
    for x in (-0.700, 0.700):
        worktop.visual(
            Box((0.040, 0.650, 0.100)),
            origin=Origin(xyz=(x, 0.0, -0.0725)),
            material=black_metal,
            name=f"side_apron_{0 if x < 0 else 1}",
        )
    worktop.visual(
        Box((0.720, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.315, -0.0425)),
        material=black_metal,
        name="drawer_lintel",
    )
    for x in (-0.430, 0.430):
        worktop.visual(
            Box((0.160, 0.030, 0.100)),
            origin=Origin(xyz=(x, -0.315, -0.0725)),
            material=black_metal,
            name=f"drawer_jamb_{0 if x < 0 else 1}",
        )

    def add_top_collar(x: float, suffix: int) -> None:
        outer_x = 0.160
        outer_y = 0.140
        wall = 0.012
        height = 0.080
        zc = -0.0625
        worktop.visual(
            Box((outer_x, wall, height)),
            origin=Origin(xyz=(x, outer_y / 2.0 - wall / 2.0, zc)),
            material=black_metal,
            name=f"front_column_collar_{suffix}",
        )
        worktop.visual(
            Box((outer_x, wall, height)),
            origin=Origin(xyz=(x, -outer_y / 2.0 + wall / 2.0, zc)),
            material=black_metal,
            name=f"rear_column_collar_{suffix}",
        )
        worktop.visual(
            Box((wall, outer_y - 2.0 * wall, height)),
            origin=Origin(xyz=(x + outer_x / 2.0 - wall / 2.0, 0.0, zc)),
            material=black_metal,
            name=f"side_column_collar_{suffix}_0",
        )
        worktop.visual(
            Box((wall, outer_y - 2.0 * wall, height)),
            origin=Origin(xyz=(x - outer_x / 2.0 + wall / 2.0, 0.0, zc)),
            material=black_metal,
            name=f"side_column_collar_{suffix}_1",
        )

    add_top_collar(-0.45, 0)
    add_top_collar(0.45, 1)

    # Metal runners and small hanger tabs for the pencil drawer.
    for suffix, x in enumerate((-0.315, 0.315)):
        worktop.visual(
            Box((0.026, 0.460, 0.024)),
            origin=Origin(xyz=(x, -0.155, -0.085)),
            material=rail_metal,
            name=f"drawer_rail_{suffix}",
        )
        for y in (-0.320, 0.030):
            worktop.visual(
                Box((0.026, 0.030, 0.065)),
                origin=Origin(xyz=(x, y, -0.055)),
                material=rail_metal,
                name=f"rail_hanger_{suffix}_{0 if y < -0.1 else 1}",
            )

    model.articulation(
        "column_to_worktop",
        ArticulationType.FIXED,
        parent=model.get_part("column_0"),
        child=worktop,
        origin=Origin(xyz=(0.45, 0.0, 0.105)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.560, 0.380, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=drawer_wood,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.610, 0.030, 0.105)),
        origin=Origin(xyz=(0.0, -0.205, 0.0)),
        material=drawer_wood,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.540, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, 0.190, 0.005)),
        material=drawer_wood,
        name="drawer_back",
    )
    for x in (-0.280, 0.280):
        drawer.visual(
            Box((0.018, 0.380, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.005)),
            material=drawer_wood,
            name=f"drawer_side_{0 if x < 0 else 1}",
        )

    # A low, fixed divider molded into the tray signals pencil storage without
    # adding loose floating items.
    drawer.visual(
        Box((0.018, 0.300, 0.018)),
        origin=Origin(xyz=(-0.120, -0.005, -0.021)),
        material=dark_edge,
        name="pencil_divider",
    )
    for x in (-0.2955, 0.2955):
        drawer.visual(
            Box((0.013, 0.360, 0.020)),
            origin=Origin(xyz=(x, -0.005, 0.005)),
            material=rail_metal,
            name=f"drawer_slide_shoe_{0 if x < 0 else 1}",
        )

    model.articulation(
        "worktop_to_drawer",
        ArticulationType.PRISMATIC,
        parent=worktop,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.155, -0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.300),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    column_0 = object_model.get_part("column_0")
    column_1 = object_model.get_part("column_1")
    worktop = object_model.get_part("worktop")
    drawer = object_model.get_part("drawer")
    lift = object_model.get_articulation("base_to_column_0")
    drawer_slide = object_model.get_articulation("worktop_to_drawer")

    ctx.expect_overlap(
        column_0,
        base,
        axes="z",
        min_overlap=0.40,
        name="collapsed column 0 remains inserted in sleeve",
    )
    ctx.expect_overlap(
        column_1,
        base,
        axes="z",
        min_overlap=0.40,
        name="collapsed column 1 remains inserted in sleeve",
    )
    ctx.expect_origin_gap(
        worktop,
        base,
        axis="z",
        min_gap=0.70,
        name="low worktop stands above the foot frame",
    )
    ctx.expect_overlap(
        drawer,
        worktop,
        axes="y",
        min_overlap=0.30,
        name="closed drawer sits mostly inside apron",
    )

    low_top = ctx.part_world_position(worktop)
    low_col_1 = ctx.part_world_position(column_1)
    with ctx.pose({lift: 0.450}):
        high_top = ctx.part_world_position(worktop)
        high_col_1 = ctx.part_world_position(column_1)
        ctx.expect_overlap(
            column_0,
            base,
            axes="z",
            min_overlap=0.055,
            name="extended column 0 retains sleeve insertion",
        )
        ctx.expect_overlap(
            column_1,
            base,
            axes="z",
            min_overlap=0.055,
            name="extended column 1 retains sleeve insertion",
        )
    ctx.check(
        "worktop raises with telescoping columns",
        low_top is not None and high_top is not None and high_top[2] > low_top[2] + 0.40,
        details=f"low={low_top}, high={high_top}",
    )
    ctx.check(
        "second leg mimics height adjustment",
        low_col_1 is not None and high_col_1 is not None and high_col_1[2] > low_col_1[2] + 0.40,
        details=f"low={low_col_1}, high={high_col_1}",
    )

    closed_drawer = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.300}):
        extended_drawer = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            worktop,
            axes="y",
            min_overlap=0.08,
            name="extended pencil drawer remains on its guide rails",
        )
    ctx.check(
        "drawer slides out toward the user side",
        closed_drawer is not None
        and extended_drawer is not None
        and extended_drawer[1] < closed_drawer[1] - 0.25,
        details=f"closed={closed_drawer}, extended={extended_drawer}",
    )

    return ctx.report()


object_model = build_object_model()
