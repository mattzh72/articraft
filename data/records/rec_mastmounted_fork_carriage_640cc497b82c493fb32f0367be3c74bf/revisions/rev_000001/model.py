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


MAST_WIDTH = 1.64
MAST_DEPTH = 0.30
BASE_HEIGHT = 0.14
UPRIGHT_WIDTH = 0.14
UPRIGHT_DEPTH = 0.18
UPRIGHT_HEIGHT = 2.04
UPRIGHT_CENTER_X = 0.64
TOP_BEAM_HEIGHT = 0.12
MID_BEAM_HEIGHT = 0.10

CARRIAGE_WIDTH = 1.50
CARRIAGE_LOWER_Z = 0.16
GUIDE_HEIGHT = 0.48
GUIDE_OUTER_WIDTH = 0.22
GUIDE_OUTER_DEPTH = 0.24
GUIDE_INNER_WIDTH = UPRIGHT_WIDTH + 0.004
GUIDE_INNER_DEPTH = UPRIGHT_DEPTH
RAISE_TRAVEL = 0.78
BOX_WALL = 0.018


def _box(
    size: tuple[float, float, float],
    *,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    centered: tuple[bool, bool, bool] = (True, True, False),
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size, centered=centered).translate(xyz)


def _guide_channel(height: float) -> cq.Workplane:
    outer = _box((GUIDE_OUTER_WIDTH, GUIDE_OUTER_DEPTH, height))
    cavity = _box(
        (GUIDE_INNER_WIDTH, GUIDE_INNER_DEPTH, height + 0.02),
        xyz=(0.0, 0.0, -0.01),
    )
    front_opening = _box(
        (GUIDE_INNER_WIDTH + 0.02, 0.08, height + 0.02),
        xyz=(0.0, 0.08, -0.01),
    )
    return outer.cut(cavity).cut(front_opening)


def _box_tube(
    size: tuple[float, float, float],
    *,
    wall: float = BOX_WALL,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    sx, sy, sz = size
    outer = _box(size, xyz=xyz)
    inner = _box(
        (sx - 2.0 * wall, sy - 2.0 * wall, sz - 2.0 * wall),
        xyz=(xyz[0], xyz[1], xyz[2] + wall),
    )
    return outer.cut(inner)


def _make_mast_components() -> tuple[
    cq.Workplane, cq.Workplane, cq.Workplane, cq.Workplane, cq.Workplane, cq.Workplane
]:
    base = _box((MAST_WIDTH, MAST_DEPTH, BASE_HEIGHT))
    left_upright = _box_tube(
        (UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT),
        xyz=(-UPRIGHT_CENTER_X, 0.0, BASE_HEIGHT),
    )
    right_upright = _box_tube(
        (UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT),
        xyz=(UPRIGHT_CENTER_X, 0.0, BASE_HEIGHT),
    )
    top_beam = _box_tube(
        (MAST_WIDTH - 0.10, 0.22, TOP_BEAM_HEIGHT),
        xyz=(0.0, 0.0, BASE_HEIGHT + UPRIGHT_HEIGHT),
    )
    left_rail = _box((0.07, 0.03, 1.74), xyz=(-UPRIGHT_CENTER_X, 0.105, 0.22))
    right_rail = _box((0.07, 0.03, 1.74), xyz=(UPRIGHT_CENTER_X, 0.105, 0.22))
    return base, left_upright, right_upright, top_beam, left_rail, right_rail


def _make_slider_side(x_center: float, mesh_side: float) -> cq.Workplane:
    lower_shoe = _box((0.11, 0.03, 0.22), xyz=(x_center, 0.135, 0.22))
    upper_shoe = _box((0.11, 0.03, 0.22), xyz=(x_center, 0.135, 0.68))
    lower_standoff = _box((0.15, 0.07, 0.12), xyz=(x_center + mesh_side * 0.03, 0.185, 0.27))
    upper_standoff = _box((0.15, 0.07, 0.12), xyz=(x_center + mesh_side * 0.03, 0.185, 0.73))
    side_spine = _box_tube((0.12, 0.08, 0.88), xyz=(x_center + mesh_side * 0.06, 0.21, 0.22))
    return lower_shoe.union(upper_shoe).union(lower_standoff).union(upper_standoff).union(side_spine)


def _make_carriage_components() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    left_slider = _make_slider_side(-UPRIGHT_CENTER_X, 1.0)
    right_slider = _make_slider_side(UPRIGHT_CENTER_X, -1.0)

    lower_crosshead = _box_tube((1.44, 0.10, 0.20), xyz=(0.0, 0.24, 0.20))
    faceplate_bottom_beam = _box_tube((1.46, 0.10, 0.16), xyz=(0.0, 0.24, 0.24))
    faceplate_panel = _box((1.48, 0.03, 0.74), xyz=(0.0, 0.285, 0.30))
    faceplate_top_beam = _box_tube((1.46, 0.09, 0.10), xyz=(0.0, 0.24, 0.96))

    left_rib = _box((0.10, 0.07, 0.60), xyz=(-0.44, 0.24, 0.34))
    center_rib = _box((0.12, 0.07, 0.60), xyz=(0.0, 0.24, 0.34))
    right_rib = _box((0.10, 0.07, 0.60), xyz=(0.44, 0.24, 0.34))

    left_arm_mount = _box((0.18, 0.16, 0.16), xyz=(-0.38, 0.24, CARRIAGE_LOWER_Z))
    right_arm_mount = _box((0.18, 0.16, 0.16), xyz=(0.38, 0.24, CARRIAGE_LOWER_Z))
    left_arm = _box(
        (0.30, 0.40, 0.11),
        xyz=(-0.38, 0.26, CARRIAGE_LOWER_Z),
        centered=(True, False, False),
    )
    right_arm = _box(
        (0.30, 0.40, 0.11),
        xyz=(0.38, 0.26, CARRIAGE_LOWER_Z),
        centered=(True, False, False),
    )

    left_post = _box_tube((0.10, 0.08, 0.76), xyz=(-0.56, 0.24, 1.00))
    right_post = _box_tube((0.10, 0.08, 0.76), xyz=(0.56, 0.24, 1.00))
    top_back_bar = _box_tube((1.24, 0.08, 0.08), xyz=(0.0, 0.24, 1.68))
    mid_back_bar = _box_tube((1.18, 0.07, 0.06), xyz=(0.0, 0.24, 1.38))
    left_slat = _box((0.05, 0.04, 0.64), xyz=(-0.24, 0.25, 1.00))
    center_slat = _box((0.05, 0.04, 0.64), xyz=(0.0, 0.25, 1.00))
    right_slat = _box((0.05, 0.04, 0.64), xyz=(0.24, 0.25, 1.00))

    body = (
        lower_crosshead.union(faceplate_bottom_beam)
        .union(faceplate_panel)
        .union(faceplate_top_beam)
        .union(left_rib)
        .union(center_rib)
        .union(right_rib)
        .union(left_arm_mount)
        .union(right_arm_mount)
        .union(left_arm)
        .union(right_arm)
        .union(left_post)
        .union(right_post)
        .union(top_back_bar)
        .union(mid_back_bar)
        .union(left_slat)
        .union(center_slat)
        .union(right_slat)
    )
    return left_slider, right_slider, body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="die_handling_lift_mast")

    mast_paint = model.material("mast_paint", rgba=(0.22, 0.24, 0.27, 1.0))
    carriage_paint = model.material("carriage_paint", rgba=(0.73, 0.64, 0.18, 1.0))

    (
        mast_base_shape,
        mast_left_upright_shape,
        mast_right_upright_shape,
        mast_top_beam_shape,
        mast_left_rail_shape,
        mast_right_rail_shape,
    ) = _make_mast_components()
    carriage_left_slider_shape, carriage_right_slider_shape, carriage_body_shape = (
        _make_carriage_components()
    )

    mast = model.part(
        "mast",
        inertial=Inertial.from_geometry(
            Box((MAST_WIDTH, MAST_DEPTH, BASE_HEIGHT + UPRIGHT_HEIGHT + TOP_BEAM_HEIGHT)),
            mass=420.0,
            origin=Origin(xyz=(0.0, 0.0, 1.15)),
        ),
    )
    mast.visual(
        mesh_from_cadquery(mast_base_shape, "mast_base"),
        material=mast_paint,
        name="mast_base",
    )
    mast.visual(
        mesh_from_cadquery(mast_left_upright_shape, "mast_left_upright"),
        material=mast_paint,
        name="mast_left_upright",
    )
    mast.visual(
        mesh_from_cadquery(mast_right_upright_shape, "mast_right_upright"),
        material=mast_paint,
        name="mast_right_upright",
    )
    mast.visual(
        mesh_from_cadquery(mast_top_beam_shape, "mast_top_beam"),
        material=mast_paint,
        name="mast_top_beam",
    )
    mast.visual(
        mesh_from_cadquery(mast_left_rail_shape, "mast_left_rail"),
        material=mast_paint,
        name="mast_left_rail",
    )
    mast.visual(
        mesh_from_cadquery(mast_right_rail_shape, "mast_right_rail"),
        material=mast_paint,
        name="mast_right_rail",
    )

    carriage = model.part(
        "carriage",
        inertial=Inertial.from_geometry(
            Box((CARRIAGE_WIDTH, 0.72, 1.62)),
            mass=255.0,
            origin=Origin(xyz=(0.0, 0.24, 0.91)),
        ),
    )
    carriage.visual(
        mesh_from_cadquery(carriage_left_slider_shape, "carriage_left_slider"),
        material=carriage_paint,
        name="carriage_left_slider",
    )
    carriage.visual(
        mesh_from_cadquery(carriage_right_slider_shape, "carriage_right_slider"),
        material=carriage_paint,
        name="carriage_right_slider",
    )
    carriage.visual(
        mesh_from_cadquery(carriage_body_shape, "carriage_body"),
        material=carriage_paint,
        name="carriage_body",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=0.25,
            lower=0.0,
            upper=RAISE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")
    mast_left_rail = mast.get_visual("mast_left_rail")
    mast_right_rail = mast.get_visual("mast_right_rail")
    carriage_left_slider = carriage.get_visual("carriage_left_slider")
    carriage_right_slider = carriage.get_visual("carriage_right_slider")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        carriage,
        mast,
        elem_a=carriage_left_slider,
        elem_b=mast_left_rail,
        reason="Left carriage wear pad rides the left mast guide rail with mesh-contact at the sliding face.",
    )
    ctx.allow_overlap(
        carriage,
        mast,
        elem_a=carriage_right_slider,
        elem_b=mast_right_rail,
        reason="Right carriage wear pad rides the right mast guide rail with mesh-contact at the sliding face.",
    )

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

    ctx.check("mast_exists", mast is not None, "missing mast part")
    ctx.check("carriage_exists", carriage is not None, "missing carriage part")
    ctx.check(
        "carriage_joint_is_vertical_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in lift.axis) == (0.0, 0.0, 1.0),
        f"expected vertical prismatic joint, got {lift.articulation_type} axis={lift.axis}",
    )

    mast_aabb = ctx.part_world_aabb(mast)
    carriage_aabb = ctx.part_world_aabb(carriage)
    if mast_aabb is None or carriage_aabb is None:
        ctx.fail("part_bounds_available", "mast or carriage world bounds are unavailable")
    else:
        mast_height = mast_aabb[1][2] - mast_aabb[0][2]
        carriage_width = carriage_aabb[1][0] - carriage_aabb[0][0]
        ctx.check(
            "mast_reads_full_height",
            mast_height >= 2.20,
            f"mast height {mast_height:.3f} m is too short",
        )
        ctx.check(
            "carriage_reads_broad_and_heavy",
            carriage_width >= 1.45,
            f"carriage width {carriage_width:.3f} m is too narrow",
        )

    ctx.expect_contact(
        carriage,
        mast,
        elem_a=carriage_left_slider,
        elem_b=mast_left_rail,
        name="left_slider_contacts_left_rail_lowered",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=carriage_right_slider,
        elem_b=mast_right_rail,
        name="right_slider_contacts_right_rail_lowered",
    )
    ctx.expect_origin_distance(
        carriage,
        mast,
        axes="x",
        max_dist=0.001,
        name="carriage_centered_between_uprights",
    )

    with ctx.pose({lift: 0.72}):
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=carriage_left_slider,
            elem_b=mast_left_rail,
            name="left_slider_contacts_left_rail_raised",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=carriage_right_slider,
            elem_b=mast_right_rail,
            name="right_slider_contacts_right_rail_raised",
        )
        ctx.expect_origin_gap(
            carriage,
            mast,
            axis="z",
            min_gap=0.71,
            max_gap=0.73,
            name="carriage_moves_up_mast",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
