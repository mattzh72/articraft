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


BASE_LEN = 0.62
BASE_W = 0.26
BASE_BED_T = 0.028
BASE_RAIL_LEN = 0.56
BASE_RAIL_W = 0.045
BASE_RAIL_H = 0.018
BASE_RAIL_Y = 0.0775

CARR_LEN = 0.42
CARR_W = 0.20
CARR_BASE_H = 0.045
CARR_POCKET_LEN = 0.37
CARR_POCKET_W = 0.11
CARR_POCKET_D = 0.030
CARR_TOP_RAIL_W = 0.042
CARR_TOP_RAIL_LEN = 0.17
CARR_TOP_RAIL_H = 0.012
CARR_TOP_RAIL_X = 0.064

CROSS_X = 0.17
CROSS_Y = 0.20
CROSS_BASE_H = 0.038
CROSS_POCKET_X = 0.086
CROSS_POCKET_Y = 0.15
CROSS_POCKET_D = 0.022
Z_TOWER_X = 0.090
Z_TOWER_Y = 0.030
Z_TOWER_H = 0.195

RAM_GUIDE_X = 0.086
RAM_GUIDE_Y = 0.026
RAM_GUIDE_H = 0.130
RAM_STEM_X = 0.050
RAM_STEM_Y = 0.036
RAM_STEM_H = 0.230
RAM_HEAD_X = 0.064
RAM_HEAD_Y = 0.046
RAM_HEAD_H = 0.018

X_TRAVEL = 0.12
Y_TRAVEL = 0.075
Z_TRAVEL = 0.16


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _lower_guide_shape() -> cq.Workplane:
    bed = cq.Workplane("XY").box(
        BASE_LEN,
        BASE_W,
        BASE_BED_T,
        centered=(True, True, False),
    )
    left_rail = (
        cq.Workplane("XY")
        .box(
            BASE_RAIL_LEN,
            BASE_RAIL_W,
            BASE_RAIL_H,
            centered=(True, True, False),
        )
        .translate((0.0, BASE_RAIL_Y, BASE_BED_T))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(
            BASE_RAIL_LEN,
            BASE_RAIL_W,
            BASE_RAIL_H,
            centered=(True, True, False),
        )
        .translate((0.0, -BASE_RAIL_Y, BASE_BED_T))
    )
    center_web = (
        cq.Workplane("XY")
        .box(
            BASE_RAIL_LEN * 0.92,
            0.050,
            0.008,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_BED_T))
    )
    return bed.union(left_rail).union(right_rail).union(center_web)


def _carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        CARR_LEN,
        CARR_W,
        CARR_BASE_H,
        centered=(True, True, False),
    )
    underside_pocket = cq.Workplane("XY").box(
        CARR_POCKET_LEN,
        CARR_POCKET_W,
        CARR_POCKET_D,
        centered=(True, True, False),
    )
    body = body.cut(underside_pocket)

    left_top_rail = (
        cq.Workplane("XY")
        .box(
            CARR_TOP_RAIL_W,
            CARR_TOP_RAIL_LEN,
            CARR_TOP_RAIL_H,
            centered=(True, True, False),
        )
        .translate((CARR_TOP_RAIL_X, 0.0, CARR_BASE_H))
    )
    right_top_rail = (
        cq.Workplane("XY")
        .box(
            CARR_TOP_RAIL_W,
            CARR_TOP_RAIL_LEN,
            CARR_TOP_RAIL_H,
            centered=(True, True, False),
        )
        .translate((-CARR_TOP_RAIL_X, 0.0, CARR_BASE_H))
    )
    center_pad = (
        cq.Workplane("XY")
        .box(
            0.15,
            0.11,
            0.010,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CARR_BASE_H))
    )
    return body.union(left_top_rail).union(right_top_rail).union(center_pad)


def _cross_slide_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        CROSS_X,
        CROSS_Y,
        CROSS_BASE_H,
        centered=(True, True, False),
    )
    underside_pocket = cq.Workplane("XY").box(
        CROSS_POCKET_X,
        CROSS_POCKET_Y,
        CROSS_POCKET_D,
        centered=(True, True, False),
    )
    body = body.cut(underside_pocket)

    tower = (
        cq.Workplane("XY")
        .box(
            Z_TOWER_X,
            Z_TOWER_Y,
            Z_TOWER_H,
            centered=(True, True, False),
        )
        .translate((0.0, -0.028, CROSS_BASE_H))
    )
    left_rib = (
        cq.Workplane("XY")
        .box(
            0.018,
            0.028,
            0.080,
            centered=(True, True, False),
        )
        .translate((0.026, -0.012, CROSS_BASE_H))
    )
    right_rib = (
        cq.Workplane("XY")
        .box(
            0.018,
            0.028,
            0.080,
            centered=(True, True, False),
        )
        .translate((-0.026, -0.012, CROSS_BASE_H))
    )
    return body.union(tower).union(left_rib).union(right_rib)


def _ram_shape() -> cq.Workplane:
    guide_block = cq.Workplane("XY").box(
        RAM_GUIDE_X,
        RAM_GUIDE_Y,
        RAM_GUIDE_H,
        centered=(True, True, False),
    ).translate((0.0, RAM_GUIDE_Y / 2.0, 0.0))
    stem = (
        cq.Workplane("XY")
        .box(
            RAM_STEM_X,
            RAM_STEM_Y,
            RAM_STEM_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.022, RAM_GUIDE_H - 0.024))
    )
    head = (
        cq.Workplane("XY")
        .box(
            RAM_HEAD_X,
            RAM_HEAD_Y,
            RAM_HEAD_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.023, RAM_GUIDE_H + RAM_STEM_H - 0.008))
    )
    return guide_block.union(stem).union(head)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_slide_stack")

    model.material("base_cast", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("machined_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("slide_gray", rgba=(0.63, 0.66, 0.69, 1.0))
    model.material("ram_steel", rgba=(0.48, 0.51, 0.56, 1.0))

    lower_guide = model.part("lower_guide")
    lower_guide.visual(
        _mesh(_lower_guide_shape(), "lower_guide_body"),
        material="base_cast",
        name="body",
    )
    lower_guide.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_W, BASE_BED_T + BASE_RAIL_H)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_BED_T + BASE_RAIL_H) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        _mesh(_carriage_shape(), "x_carriage_body"),
        material="machined_gray",
        name="body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARR_LEN, CARR_W, CARR_BASE_H + CARR_TOP_RAIL_H)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (CARR_BASE_H + CARR_TOP_RAIL_H) / 2.0)),
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        Box((CROSS_X, CROSS_Y, CROSS_BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, CROSS_BASE_H / 2.0)),
        material="slide_gray",
        name="base",
    )
    cross_slide.visual(
        Box((Z_TOWER_X, Z_TOWER_Y, Z_TOWER_H)),
        origin=Origin(xyz=(0.0, -0.028, CROSS_BASE_H + Z_TOWER_H / 2.0)),
        material="slide_gray",
        name="tower",
    )
    cross_slide.inertial = Inertial.from_geometry(
        Box((CROSS_X, CROSS_Y, CROSS_BASE_H + Z_TOWER_H)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, (CROSS_BASE_H + Z_TOWER_H) / 2.0)),
    )

    ram = model.part("ram")
    ram.visual(
        Box((RAM_GUIDE_X, RAM_GUIDE_Y, RAM_GUIDE_H)),
        origin=Origin(xyz=(0.0, 0.0, RAM_GUIDE_H / 2.0)),
        material="ram_steel",
        name="guide_block",
    )
    ram.visual(
        Box((RAM_STEM_X, RAM_STEM_Y, RAM_STEM_H)),
        origin=Origin(
            xyz=(0.0, 0.005, 0.120 + RAM_STEM_H / 2.0),
        ),
        material="ram_steel",
        name="stem",
    )
    ram.visual(
        Box((RAM_HEAD_X, RAM_HEAD_Y, RAM_HEAD_H)),
        origin=Origin(
            xyz=(0.0, 0.005, 0.342 + RAM_HEAD_H / 2.0),
        ),
        material="ram_steel",
        name="head",
    )
    ram.inertial = Inertial.from_geometry(
        Box((RAM_HEAD_X, RAM_HEAD_Y, RAM_GUIDE_H + RAM_STEM_H)),
        mass=1.4,
        origin=Origin(
            xyz=(0.0, RAM_HEAD_Y / 2.0, (RAM_GUIDE_H + RAM_STEM_H) / 2.0)
        ),
    )

    model.articulation(
        "lower_to_carriage",
        ArticulationType.PRISMATIC,
        parent=lower_guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_BED_T + BASE_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.45,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, CARR_BASE_H + CARR_TOP_RAIL_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.35,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )
    model.articulation(
        "cross_slide_to_ram",
        ArticulationType.PRISMATIC,
        parent=cross_slide,
        child=ram,
        origin=Origin(xyz=(0.0, 0.0, CROSS_BASE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.25,
            lower=0.0,
            upper=Z_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_guide = object_model.get_part("lower_guide")
    carriage = object_model.get_part("carriage")
    cross_slide = object_model.get_part("cross_slide")
    ram = object_model.get_part("ram")
    x_slide = object_model.get_articulation("lower_to_carriage")
    y_slide = object_model.get_articulation("carriage_to_cross_slide")
    z_slide = object_model.get_articulation("cross_slide_to_ram")

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
        "all stage joints are prismatic and oriented",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(y_slide.axis) == (0.0, 1.0, 0.0)
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(z_slide.axis) == (0.0, 0.0, 1.0),
        details="Expected orthogonal X, Y, and Z prismatic stages.",
    )

    with ctx.pose({x_slide: 0.0, y_slide: 0.0, z_slide: 0.0}):
        ctx.expect_contact(
            carriage,
            lower_guide,
            name="carriage is supported by lower guide",
        )
        ctx.expect_contact(
            cross_slide,
            carriage,
            name="cross slide is supported by carriage",
        )
        ctx.expect_contact(
            ram,
            cross_slide,
            name="ram is supported by upper slide",
        )
        ctx.expect_within(
            carriage,
            lower_guide,
            axes="xy",
            margin=0.0,
            name="carriage footprint stays within lower guide envelope",
        )
        ctx.expect_within(
            cross_slide,
            carriage,
            axes="xy",
            margin=0.0,
            name="cross slide footprint stays within carriage envelope",
        )

    with ctx.pose({x_slide: X_TRAVEL}):
        ctx.expect_origin_gap(
            carriage,
            lower_guide,
            axis="x",
            min_gap=X_TRAVEL - 0.005,
            name="x stage moves carriage in +x",
        )
        ctx.expect_contact(
            carriage,
            lower_guide,
            name="x stage remains guided at positive travel",
        )
        ctx.expect_overlap(
            carriage,
            lower_guide,
            axes="x",
            min_overlap=0.20,
            name="x carriage still overlaps lower guide rails at positive travel",
        )

    with ctx.pose({y_slide: Y_TRAVEL}):
        ctx.expect_origin_gap(
            cross_slide,
            carriage,
            axis="y",
            min_gap=Y_TRAVEL - 0.005,
            name="y stage moves cross slide in +y",
        )
        ctx.expect_contact(
            cross_slide,
            carriage,
            name="y stage remains guided at positive travel",
        )
        ctx.expect_overlap(
            cross_slide,
            carriage,
            axes="y",
            min_overlap=0.08,
            name="cross slide still overlaps carriage rails at positive travel",
        )

    with ctx.pose({z_slide: Z_TRAVEL}):
        ctx.expect_origin_gap(
            ram,
            cross_slide,
            axis="z",
            min_gap=Z_TRAVEL - 0.005,
            name="z stage lifts ram upward",
        )
        ctx.expect_contact(
            ram,
            cross_slide,
            name="ram remains guided at positive lift",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
