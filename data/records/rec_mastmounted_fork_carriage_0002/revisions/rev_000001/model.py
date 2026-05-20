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

OUTER_MAST_HEIGHT = 1.95
OUTER_RAIL_CENTER_X = 0.33
OUTER_RAIL_WIDTH = 0.14
OUTER_RAIL_DEPTH = 0.16
OUTER_RAIL_WALL = 0.012

INNER_STAGE_HEIGHT = 1.56
INNER_RAIL_CENTER_X = 0.324
INNER_RAIL_WIDTH = 0.10
INNER_RAIL_DEPTH = 0.12
INNER_RAIL_WALL = 0.010

INNER_STAGE_BASE_Z = 0.19

CARRIAGE_SIDE_X = 0.335
CARRIAGE_PLATE_Y = 0.105
CARRIAGE_GUIDE_Y = 0.070
CARRIAGE_LOWER_RAIL_Z = 0.26
CARRIAGE_TOP_RAIL_Z = 0.72

FORK_X_OFFSET = 0.18


def _compound(*objs: object) -> cq.Workplane:
    shapes = []
    for obj in objs:
        if obj is None:
            continue
        if hasattr(obj, "vals"):
            shapes.extend(obj.vals())
        else:
            shapes.append(obj)
    return cq.Workplane(obj=cq.Compound.makeCompound(shapes))


def _box_tube(length: float, depth: float, height: float, wall: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(length, depth, height)
    inner = cq.Workplane("XY").box(length - 2 * wall, depth - 2 * wall, height + 0.004)
    return outer.cut(inner)


def _u_channel(
    height: float,
    width: float,
    depth: float,
    wall: float,
    *,
    open_to_right: bool,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height)
    slot = cq.Workplane("XY").box(width - wall, depth - 2 * wall, height + 0.004)
    shift_x = wall / 2 if open_to_right else -wall / 2
    return outer.cut(slot.translate((shift_x, 0.0, 0.0)))


def _bolt_head(radius: float, length: float, pos: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate(pos)
    )


def _lug_pair(
    *,
    span_x: float,
    plate_w: float,
    plate_t: float,
    plate_h: float,
    hole_r: float,
    y: float,
    z_base: float,
) -> cq.Workplane:
    parts = []
    for x in (-span_x / 2, span_x / 2):
        plate = (
            cq.Workplane("XY")
            .box(plate_w, plate_t, plate_h)
            .translate((x, y, z_base + plate_h / 2))
        )
        hole = (
            cq.Workplane("XY")
            .cylinder(plate_t + 0.006, hole_r)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((x, y, z_base + plate_h * 0.62))
        )
        parts.append(plate.cut(hole))
    bridge = (
        cq.Workplane("XY")
        .box(span_x + plate_w, plate_t, plate_t * 1.5)
        .translate((0.0, y, z_base + plate_t * 0.75))
    )
    return _compound(*parts, bridge)


def make_outer_mast_rails() -> cq.Workplane:
    left_rail = _u_channel(
        OUTER_MAST_HEIGHT,
        OUTER_RAIL_WIDTH,
        OUTER_RAIL_DEPTH,
        OUTER_RAIL_WALL,
        open_to_right=True,
    ).translate((-OUTER_RAIL_CENTER_X, 0.0, OUTER_MAST_HEIGHT / 2))
    right_rail = _u_channel(
        OUTER_MAST_HEIGHT,
        OUTER_RAIL_WIDTH,
        OUTER_RAIL_DEPTH,
        OUTER_RAIL_WALL,
        open_to_right=False,
    ).translate((OUTER_RAIL_CENTER_X, 0.0, OUTER_MAST_HEIGHT / 2))

    mid_tie = cq.Workplane("XY").box(0.58, 0.05, 0.05).translate((0.0, -0.04, 0.96))
    upper_stiffener = cq.Workplane("XY").box(0.62, 0.03, 0.04).translate((0.0, 0.04, 1.63))

    top_bracket_left = cq.Workplane("XY").box(0.08, 0.12, 0.18).translate((-0.15, 0.03, 1.80))
    top_bracket_right = cq.Workplane("XY").box(0.08, 0.12, 0.18).translate((0.15, 0.03, 1.80))
    sheave_guard = cq.Workplane("XY").box(0.34, 0.03, 0.11).translate((0.0, 0.07, 1.89))

    left_cover = cq.Workplane("XY").box(0.09, 0.006, 0.18).translate((-0.41, 0.083, 0.74))
    right_cover = cq.Workplane("XY").box(0.09, 0.006, 0.18).translate((0.41, 0.083, 0.74))
    cover_bolts = _compound(
        _bolt_head(0.006, 0.009, (-0.41, 0.089, 0.80)),
        _bolt_head(0.006, 0.009, (-0.41, 0.089, 0.68)),
        _bolt_head(0.006, 0.009, (0.41, 0.089, 0.80)),
        _bolt_head(0.006, 0.009, (0.41, 0.089, 0.68)),
    )

    return _compound(
        left_rail,
        right_rail,
        mid_tie,
        upper_stiffener,
        top_bracket_left,
        top_bracket_right,
        sheave_guard,
        left_cover,
        right_cover,
        cover_bolts,
    )


def make_inner_stage_structure() -> cq.Workplane:
    left_rail = _u_channel(
        INNER_STAGE_HEIGHT,
        INNER_RAIL_WIDTH,
        INNER_RAIL_DEPTH,
        INNER_RAIL_WALL,
        open_to_right=False,
    ).translate((-INNER_RAIL_CENTER_X, 0.0, INNER_STAGE_HEIGHT / 2))
    right_rail = _u_channel(
        INNER_STAGE_HEIGHT,
        INNER_RAIL_WIDTH,
        INNER_RAIL_DEPTH,
        INNER_RAIL_WALL,
        open_to_right=True,
    ).translate((INNER_RAIL_CENTER_X, 0.0, INNER_STAGE_HEIGHT / 2))

    guide_pads = []
    for z in (0.24, 1.22):
        guide_pads.extend(
            [
                cq.Workplane("XY").box(0.014, 0.08, 0.12).translate((-0.381, 0.0, z)),
                cq.Workplane("XY").box(0.014, 0.08, 0.12).translate((-0.267, 0.0, z)),
                cq.Workplane("XY").box(0.014, 0.08, 0.12).translate((0.381, 0.0, z)),
                cq.Workplane("XY").box(0.014, 0.08, 0.12).translate((0.267, 0.0, z)),
            ]
        )

    lower_chain_lugs = _lug_pair(
        span_x=0.11,
        plate_w=0.022,
        plate_t=0.018,
        plate_h=0.13,
        hole_r=0.011,
        y=0.045,
        z_base=0.17,
    )
    lower_chain_pad = cq.Workplane("XY").box(0.18, 0.03, 0.06).translate((0.0, 0.03, 0.16))

    return _compound(left_rail, right_rail, *guide_pads, lower_chain_lugs, lower_chain_pad)


def make_carriage_structure() -> cq.Workplane:
    left_plate = cq.Workplane("XY").box(0.022, 0.040, 0.82).translate((-CARRIAGE_SIDE_X, CARRIAGE_PLATE_Y, 0.53))
    right_plate = cq.Workplane("XY").box(0.022, 0.040, 0.82).translate((CARRIAGE_SIDE_X, CARRIAGE_PLATE_Y, 0.53))
    left_window = cq.Workplane("XY").box(0.030, 0.028, 0.42).translate((-CARRIAGE_SIDE_X, CARRIAGE_PLATE_Y, 0.51))
    right_window = cq.Workplane("XY").box(0.030, 0.028, 0.42).translate((CARRIAGE_SIDE_X, CARRIAGE_PLATE_Y, 0.51))
    left_plate = left_plate.cut(left_window)
    right_plate = right_plate.cut(right_window)

    guide_groups = []
    for x in (-INNER_RAIL_CENTER_X, INNER_RAIL_CENTER_X):
        for z in (0.22, 0.66):
            guide_groups.append(cq.Workplane("XY").box(0.10, 0.020, 0.10).translate((x, CARRIAGE_GUIDE_Y, z)))
            guide_groups.append(cq.Workplane("XY").box(0.10, 0.020, 0.10).translate((x, 0.09, z)))
            guide_groups.append(_bolt_head(0.0065, 0.010, (x - 0.028, 0.101, z + 0.026)))
            guide_groups.append(_bolt_head(0.0065, 0.010, (x + 0.028, 0.101, z - 0.026)))

    chain_lugs = _lug_pair(
        span_x=0.12,
        plate_w=0.024,
        plate_t=0.018,
        plate_h=0.16,
        hole_r=0.010,
        y=0.11,
        z_base=0.75,
    )
    chain_brace = cq.Workplane("XY").box(0.20, 0.03, 0.06).translate((0.0, 0.10, 0.74))

    top_cover = cq.Workplane("XY").box(0.20, 0.006, 0.14).translate((0.0, 0.133, 0.72))
    lower_cover = cq.Workplane("XY").box(0.20, 0.006, 0.14).translate((0.0, 0.133, 0.26))
    cover_bolts = _compound(
        _bolt_head(0.006, 0.009, (-0.07, 0.139, 0.77)),
        _bolt_head(0.006, 0.009, (0.07, 0.139, 0.77)),
        _bolt_head(0.006, 0.009, (-0.07, 0.139, 0.67)),
        _bolt_head(0.006, 0.009, (0.07, 0.139, 0.67)),
        _bolt_head(0.006, 0.009, (-0.07, 0.139, 0.31)),
        _bolt_head(0.006, 0.009, (0.07, 0.139, 0.31)),
        _bolt_head(0.006, 0.009, (-0.07, 0.139, 0.21)),
        _bolt_head(0.006, 0.009, (0.07, 0.139, 0.21)),
    )

    heel_stiffener = cq.Workplane("XY").box(0.62, 0.022, 0.06).translate((0.0, 0.09, 0.12))

    return _compound(
        left_plate,
        right_plate,
        *guide_groups,
        chain_lugs,
        chain_brace,
        top_cover,
        lower_cover,
        cover_bolts,
        heel_stiffener,
    )


def make_backrest() -> cq.Workplane:
    left_tab = cq.Workplane("XY").box(0.08, 0.04, 0.08).translate((-0.18, CARRIAGE_PLATE_Y, 0.04))
    right_tab = cq.Workplane("XY").box(0.08, 0.04, 0.08).translate((0.18, CARRIAGE_PLATE_Y, 0.04))
    left_upright = cq.Workplane("XY").box(0.04, 0.04, 0.56).translate((-0.28, CARRIAGE_PLATE_Y, 0.36))
    right_upright = cq.Workplane("XY").box(0.04, 0.04, 0.56).translate((0.28, CARRIAGE_PLATE_Y, 0.36))
    top_rail = cq.Workplane("XY").box(0.62, 0.04, 0.05).translate((0.0, CARRIAGE_PLATE_Y, 0.62))
    center_rail = cq.Workplane("XY").box(0.56, 0.03, 0.04).translate((0.0, CARRIAGE_PLATE_Y, 0.38))

    slats = []
    for x in (-0.18, -0.06, 0.06, 0.18):
        slats.append(cq.Workplane("XY").box(0.028, 0.028, 0.46).translate((x, CARRIAGE_PLATE_Y, 0.34)))

    return _compound(left_tab, right_tab, left_upright, right_upright, top_rail, center_rail, *slats)


def make_fork() -> cq.Workplane:
    shank = cq.Workplane("XY").box(0.11, 0.022, 0.82).translate((0.0, 0.141, 0.41))
    tine = cq.Workplane("XY").box(0.11, 0.52, 0.045).translate((0.0, 0.39, 0.0225))
    hook_pad = cq.Workplane("XY").box(0.11, 0.04, 0.06).translate((0.0, 0.121, 0.79))
    lower_keeper = cq.Workplane("XY").box(0.11, 0.035, 0.04).translate((0.0, 0.121, 0.29))
    heel_gusset = (
        cq.Workplane("YZ")
        .polyline([(0.13, 0.045), (0.28, 0.045), (0.13, 0.23)])
        .close()
        .extrude(0.11)
        .translate((-0.055, 0.0, 0.0))
    )
    tip_bevel = cq.Workplane("YZ").polyline([(0.60, 0.045), (0.65, 0.045), (0.65, 0.0)]).close().extrude(0.11).translate((-0.055, 0.0, 0.0))
    return _compound(shank, tine, hook_pad, lower_keeper, heel_gusset, tip_bevel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_mounted_fork_carriage", assets=ASSETS)

    steel = model.material("steel", rgba=(0.46, 0.48, 0.50, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    blackened = model.material("blackened", rgba=(0.16, 0.17, 0.18, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.58, 0.60, 0.62, 1.0))

    outer_mast = model.part("outer_mast")
    outer_mast.visual(
        mesh_from_cadquery(make_outer_mast_rails(), "outer_mast_rails.obj", assets=ASSETS),
        material=steel,
        name="mast_rails",
    )
    outer_mast.visual(
        mesh_from_cadquery(_box_tube(0.56, 0.10, 0.09, 0.010), "outer_mast_base_tie.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, -0.01, 0.16)),
        material=dark_steel,
        name="base_tie",
    )
    outer_mast.visual(
        mesh_from_cadquery(_box_tube(0.56, 0.10, 0.08, 0.010), "outer_mast_top_tie.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, -0.01, 1.82)),
        material=dark_steel,
        name="top_tie",
    )
    outer_mast.inertial = Inertial.from_geometry(
        Box((0.86, 0.18, OUTER_MAST_HEIGHT)),
        mass=210.0,
        origin=Origin(xyz=(0.0, 0.0, OUTER_MAST_HEIGHT / 2)),
    )

    inner_stage = model.part("inner_stage")
    inner_stage.visual(
        mesh_from_cadquery(make_inner_stage_structure(), "inner_stage_structure.obj", assets=ASSETS),
        material=steel,
        name="stage_structure",
    )
    inner_stage.visual(
        mesh_from_cadquery(_box_tube(0.50, 0.08, 0.08, 0.009), "inner_stage_bottom_bar.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, -0.01, 0.10)),
        material=dark_steel,
        name="stage_bottom_bar",
    )
    inner_stage.visual(
        mesh_from_cadquery(_box_tube(0.50, 0.08, 0.07, 0.009), "inner_stage_top_bar.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, -0.01, 1.44)),
        material=dark_steel,
        name="stage_top_bar",
    )
    inner_stage.inertial = Inertial.from_geometry(
        Box((0.80, 0.16, INNER_STAGE_HEIGHT)),
        mass=122.0,
        origin=Origin(xyz=(0.0, 0.0, INNER_STAGE_HEIGHT / 2)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_structure(), "carriage_structure.obj", assets=ASSETS),
        material=steel,
        name="carriage_structure",
    )
    carriage.visual(
        Box((0.72, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, CARRIAGE_PLATE_Y, CARRIAGE_TOP_RAIL_Z)),
        material=blackened,
        name="top_rail",
    )
    carriage.visual(
        Box((0.72, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, CARRIAGE_PLATE_Y, CARRIAGE_LOWER_RAIL_Z)),
        material=blackened,
        name="lower_rail",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.76, 0.16, 0.96)),
        mass=74.0,
        origin=Origin(xyz=(0.0, 0.09, 0.48)),
    )

    backrest = model.part("load_backrest")
    backrest.visual(
        mesh_from_cadquery(make_backrest(), "load_backrest.obj", assets=ASSETS),
        material=cover_gray,
        name="body",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.66, 0.05, 0.64)),
        mass=18.0,
        origin=Origin(xyz=(0.0, CARRIAGE_PLATE_Y, 0.32)),
    )

    fork_left = model.part("fork_left")
    fork_left.visual(
        mesh_from_cadquery(make_fork(), "fork_left.obj", assets=ASSETS),
        material=blackened,
        name="body",
    )
    fork_left.inertial = Inertial.from_geometry(
        Box((0.11, 0.65, 0.82)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.33, 0.41)),
    )

    fork_right = model.part("fork_right")
    fork_right.visual(
        mesh_from_cadquery(make_fork(), "fork_right.obj", assets=ASSETS),
        material=blackened,
        name="body",
    )
    fork_right.inertial = Inertial.from_geometry(
        Box((0.11, 0.65, 0.82)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.33, 0.41)),
    )

    model.articulation(
        "mast_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent=outer_mast,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, INNER_STAGE_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16000.0, velocity=0.40, lower=0.0, upper=0.30),
    )
    model.articulation(
        "inner_stage_to_carriage",
        ArticulationType.PRISMATIC,
        parent=inner_stage,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.55, lower=0.0, upper=0.42),
    )
    model.articulation(
        "carriage_to_backrest",
        ArticulationType.FIXED,
        parent=carriage,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOP_RAIL_Z + 0.03)),
    )
    model.articulation(
        "carriage_to_fork_left",
        ArticulationType.FIXED,
        parent=carriage,
        child=fork_left,
        origin=Origin(xyz=(-FORK_X_OFFSET, 0.0, 0.0)),
    )
    model.articulation(
        "carriage_to_fork_right",
        ArticulationType.FIXED,
        parent=carriage,
        child=fork_right,
        origin=Origin(xyz=(FORK_X_OFFSET, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer_mast = object_model.get_part("outer_mast")
    inner_stage = object_model.get_part("inner_stage")
    carriage = object_model.get_part("carriage")
    backrest = object_model.get_part("load_backrest")
    fork_left = object_model.get_part("fork_left")
    fork_right = object_model.get_part("fork_right")

    mast_to_stage = object_model.get_articulation("mast_to_inner_stage")
    stage_to_carriage = object_model.get_articulation("inner_stage_to_carriage")

    base_tie = outer_mast.get_visual("base_tie")
    top_rail = carriage.get_visual("top_rail")
    lower_rail = carriage.get_visual("lower_rail")

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

    ctx.check(
        "all_key_parts_present",
        all(part is not None for part in (outer_mast, inner_stage, carriage, backrest, fork_left, fork_right)),
        "One or more required mechanical study parts could not be resolved.",
    )
    ctx.check(
        "lift_axes_vertical",
        tuple(mast_to_stage.axis) == (0.0, 0.0, 1.0) and tuple(stage_to_carriage.axis) == (0.0, 0.0, 1.0),
        f"Expected both lift axes to be vertical, got {mast_to_stage.axis} and {stage_to_carriage.axis}.",
    )
    ctx.check(
        "nested_lift_strokes_present",
        mast_to_stage.motion_limits.upper >= 0.25 and stage_to_carriage.motion_limits.upper >= 0.35,
        "Lift strokes are too short to read as a nested mast and carriage study.",
    )

    ctx.expect_contact(inner_stage, outer_mast, name="inner_stage_guided_in_outer_mast")
    ctx.expect_contact(carriage, inner_stage, name="carriage_guided_on_stage")
    ctx.expect_contact(backrest, carriage, name="backrest_seated_on_carriage")
    ctx.expect_contact(fork_left, carriage, name="left_fork_seated_on_carriage")
    ctx.expect_contact(fork_right, carriage, name="right_fork_seated_on_carriage")

    ctx.expect_overlap(inner_stage, outer_mast, axes="xy", min_overlap=0.08, name="stage_overlaps_mast_footprint")
    ctx.expect_overlap(carriage, inner_stage, axes="xy", min_overlap=0.03, name="carriage_overlaps_stage_footprint")
    ctx.expect_within(inner_stage, outer_mast, axes="x", margin=0.03, name="stage_kept_within_outer_rails")
    ctx.expect_within(carriage, outer_mast, axes="x", margin=0.05, name="carriage_kept_between_mast_uprights")

    ctx.expect_gap(
        carriage,
        outer_mast,
        axis="z",
        positive_elem=lower_rail,
        negative_elem=base_tie,
        min_gap=0.18,
        max_gap=0.28,
        name="carriage_clear_of_base_tie_at_rest",
    )
    ctx.expect_origin_distance(
        fork_left,
        fork_right,
        axes="x",
        min_dist=0.34,
        max_dist=0.38,
        name="fork_spacing_realistic",
    )
    ctx.expect_origin_distance(
        fork_left,
        fork_right,
        axes="yz",
        max_dist=0.001,
        name="forks_level_and_coplanar",
    )

    with ctx.pose({mast_to_stage: 0.20, stage_to_carriage: 0.34}):
        ctx.expect_overlap(carriage, outer_mast, axes="x", min_overlap=0.55, name="raised_carriage_stays_laterally_captured")
        ctx.expect_within(carriage, outer_mast, axes="x", margin=0.07, name="raised_carriage_still_between_uprights")
        ctx.expect_gap(
            carriage,
            outer_mast,
            axis="z",
            positive_elem=lower_rail,
            negative_elem=base_tie,
            min_gap=0.70,
            name="raised_carriage_has_clear_lift_path",
        )
        ctx.expect_contact(carriage, inner_stage, name="raised_carriage_remains_guided")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
