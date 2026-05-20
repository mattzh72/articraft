from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


WIDTH = 1.62
DEPTH = 0.62
HEIGHT = 0.84
SIDE_THICK = 0.03
TOP_THICK = 0.032
BOTTOM_THICK = 0.026
BACK_THICK = 0.018
PLINTH_HEIGHT = 0.09

DRAWER_WIDTH = 0.734
DRAWER_DEPTH = 0.46
DRAWER_HEIGHT = 0.175
DRAWER_TRAVEL = 0.21
OUTER_SLIDE_GAP = 0.0095
INNER_SLIDE_GAP = 0.0015

TAMBOUR_WIDTH = 1.30
TAMBOUR_THICK = 0.014
TAMBOUR_CURTAIN_HEIGHT = 0.38
TAMBOUR_FRONT_Y = -0.302
TAMBOUR_ENTRY_Z = 0.515
TAMBOUR_TRAVEL = 0.26


def _drawer_part(
    model: ArticulatedObject,
    name: str,
    wood,
    runner,
    liner,
    *,
    left_slide_gap: float,
    right_slide_gap: float,
):
    part = model.part(name)

    side_thick = 0.012
    bottom_thick = 0.010
    back_thick = 0.012
    front_thick = 0.018
    side_height = DRAWER_HEIGHT - 0.022
    interior_depth = DRAWER_DEPTH - front_thick - back_thick
    slide_length = DRAWER_DEPTH - 0.060
    slide_height = 0.032
    slide_z = 0.082

    part.visual(
        Box((DRAWER_WIDTH, front_thick, DRAWER_HEIGHT)),
        origin=Origin(xyz=(0.0, -DRAWER_DEPTH / 2.0 + front_thick / 2.0, DRAWER_HEIGHT / 2.0)),
        material=wood,
        name="front_panel",
    )
    part.visual(
        Box((DRAWER_WIDTH - 0.024, DRAWER_DEPTH - 0.032, bottom_thick)),
        origin=Origin(xyz=(0.0, 0.004, bottom_thick / 2.0)),
        material=liner,
        name="drawer_bottom",
    )
    part.visual(
        Box((side_thick, interior_depth, side_height)),
        origin=Origin(
            xyz=(
                -DRAWER_WIDTH / 2.0 + side_thick / 2.0,
                (front_thick - back_thick) / 2.0,
                bottom_thick + side_height / 2.0,
            )
        ),
        material=wood,
        name="left_side",
    )
    part.visual(
        Box((side_thick, interior_depth, side_height)),
        origin=Origin(
            xyz=(
                DRAWER_WIDTH / 2.0 - side_thick / 2.0,
                (front_thick - back_thick) / 2.0,
                bottom_thick + side_height / 2.0,
            )
        ),
        material=wood,
        name="right_side",
    )
    part.visual(
        Box((DRAWER_WIDTH - 0.024, back_thick, side_height)),
        origin=Origin(
            xyz=(0.0, DRAWER_DEPTH / 2.0 - back_thick / 2.0, bottom_thick + side_height / 2.0)
        ),
        material=wood,
        name="back_panel",
    )
    part.visual(
        Box((left_slide_gap, slide_length, slide_height)),
        origin=Origin(
            xyz=(
                -DRAWER_WIDTH / 2.0 - left_slide_gap / 2.0,
                0.012,
                slide_z,
            )
        ),
        material=runner,
        name="left_slide_member",
    )
    part.visual(
        Box((right_slide_gap, slide_length, slide_height)),
        origin=Origin(
            xyz=(
                DRAWER_WIDTH / 2.0 + right_slide_gap / 2.0,
                0.012,
                slide_z,
            )
        ),
        material=runner,
        name="right_slide_member",
    )
    part.visual(
        Box((0.18, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, -DRAWER_DEPTH / 2.0 + 0.010, 0.104)),
        material=runner,
        name="pull_recess",
    )
    part.inertial = Inertial.from_geometry(
        Box((DRAWER_WIDTH, DRAWER_DEPTH, DRAWER_HEIGHT)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, DRAWER_HEIGHT / 2.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roll_front_credenza_desk")

    walnut = model.material("walnut", rgba=(0.45, 0.29, 0.18, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.28, 0.19, 0.12, 1.0))
    drawer_liner = model.material("drawer_liner", rgba=(0.63, 0.57, 0.48, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.75, 0.67, 0.42, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.16, 0.17, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((SIDE_THICK, DEPTH, HEIGHT - PLINTH_HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + SIDE_THICK / 2.0, 0.0, PLINTH_HEIGHT + (HEIGHT - PLINTH_HEIGHT) / 2.0)),
        material=walnut,
        name="left_side_panel",
    )
    carcass.visual(
        Box((SIDE_THICK, DEPTH, HEIGHT - PLINTH_HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - SIDE_THICK / 2.0, 0.0, PLINTH_HEIGHT + (HEIGHT - PLINTH_HEIGHT) / 2.0)),
        material=walnut,
        name="right_side_panel",
    )
    carcass.visual(
        Box((WIDTH, DEPTH, TOP_THICK)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - TOP_THICK / 2.0)),
        material=walnut,
        name="top_panel",
    )
    carcass.visual(
        Box((WIDTH - 0.02, DEPTH - 0.02, BOTTOM_THICK)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + BOTTOM_THICK / 2.0)),
        material=walnut_dark,
        name="bottom_deck",
    )
    carcass.visual(
        Box((WIDTH - 2.0 * SIDE_THICK + 0.01, BACK_THICK, HEIGHT - PLINTH_HEIGHT - 0.026)),
        origin=Origin(
            xyz=(
                0.0,
                DEPTH / 2.0 - BACK_THICK / 2.0,
                PLINTH_HEIGHT + 0.010 + (HEIGHT - PLINTH_HEIGHT - 0.026) / 2.0,
            )
        ),
        material=walnut_dark,
        name="back_panel",
    )
    carcass.visual(
        Box((WIDTH - 0.22, 0.036, 0.100)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + 0.040, HEIGHT - 0.048)),
        material=walnut,
        name="front_header",
    )
    carcass.visual(
        Box((0.026, DEPTH - 0.08, 0.29)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + BOTTOM_THICK + 0.145)),
        material=walnut_dark,
        name="center_divider",
    )
    carcass.visual(
        Box((WIDTH - 0.20, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + 0.021, TAMBOUR_ENTRY_Z + 0.003)),
        material=walnut_dark,
        name="lintel_stop",
    )
    carcass.visual(
        Box((WIDTH - 0.06, 0.28, 0.022)),
        origin=Origin(xyz=(0.0, 0.14, 0.645)),
        material=walnut_dark,
        name="tambour_well_floor",
    )
    carcass.visual(
        Box((WIDTH - 0.22, 0.18, 0.018)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - 0.11, 0.675)),
        material=walnut_dark,
        name="tambour_well_backer",
    )
    carcass.visual(
        Box((WIDTH - 0.10, DEPTH - 0.16, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.02, PLINTH_HEIGHT / 2.0)),
        material=walnut_dark,
        name="plinth_block",
    )
    carcass.visual(
        Box((WIDTH - 0.28, 0.036, 0.030)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + 0.030, 0.120)),
        material=walnut_dark,
        name="drawer_sill",
    )

    rail_x = WIDTH / 2.0 - 0.135
    rail_points = [
        (0.0, TAMBOUR_FRONT_Y + 0.008, 0.145),
        (0.0, TAMBOUR_FRONT_Y + 0.008, 0.320),
        (0.0, TAMBOUR_FRONT_Y + 0.008, TAMBOUR_ENTRY_Z),
        (0.0, -0.255, 0.605),
        (0.0, -0.145, 0.668),
        (0.0, 0.015, 0.690),
        (0.0, 0.195, 0.690),
    ]
    left_rail = tube_from_spline_points(
        [(-rail_x, y, z) for _, y, z in rail_points],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    right_rail = tube_from_spline_points(
        [(rail_x, y, z) for _, y, z in rail_points],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    carcass.visual(mesh_from_geometry(left_rail, "left_tambour_guide"), material=satin_brass, name="left_tambour_guide")
    carcass.visual(mesh_from_geometry(right_rail, "right_tambour_guide"), material=satin_brass, name="right_tambour_guide")

    for sign, prefix in ((-1.0, "left"), (1.0, "right")):
        x = sign * rail_x
        carcass.visual(
            Box((0.024, 0.060, 0.080)),
            origin=Origin(xyz=(x, TAMBOUR_FRONT_Y + 0.014, 0.170)),
            material=walnut_dark,
            name=f"{prefix}_guide_front_block",
        )
        carcass.visual(
            Box((0.024, 0.140, 0.040)),
            origin=Origin(xyz=(x, 0.155, 0.690)),
            material=walnut_dark,
            name=f"{prefix}_guide_rear_block",
        )
        carcass.visual(
            Box((0.024, 0.080, 0.055)),
            origin=Origin(xyz=(x, -0.180, 0.655)),
            material=walnut_dark,
            name=f"{prefix}_guide_mid_block",
        )

    track_length = DRAWER_DEPTH - 0.060
    for x, prefix in (
        (-0.779, "left_outer"),
        (-0.022, "left_inner"),
        (0.022, "right_inner"),
        (0.779, "right_outer"),
    ):
        carcass.visual(
            Box((0.012, track_length, 0.040)),
            origin=Origin(xyz=(x, 0.012, 0.202)),
            material=charcoal,
            name=f"{prefix}_track",
        )
    carcass.visual(
        Box((0.024, 0.090, 0.040)),
        origin=Origin(xyz=(-0.013, 0.165, 0.202)),
        material=charcoal,
        name="left_inner_track_bridge",
    )
    carcass.visual(
        Box((0.024, 0.090, 0.040)),
        origin=Origin(xyz=(0.013, 0.165, 0.202)),
        material=charcoal,
        name="right_inner_track_bridge",
    )

    carcass.inertial = Inertial.from_geometry(
        Box((WIDTH, DEPTH, HEIGHT)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, HEIGHT / 2.0)),
    )

    tambour = model.part("tambour")
    tambour.visual(
        Box((TAMBOUR_WIDTH, TAMBOUR_THICK, TAMBOUR_CURTAIN_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -TAMBOUR_CURTAIN_HEIGHT / 2.0)),
        material=walnut,
        name="front_curtain",
    )
    tambour.visual(
        Box((TAMBOUR_WIDTH, TAMBOUR_THICK, 0.120)),
        origin=Origin(xyz=(0.0, 0.018, 0.052), rpy=(-0.88, 0.0, 0.0)),
        material=walnut,
        name="lower_crown",
    )
    tambour.visual(
        Box((TAMBOUR_WIDTH, TAMBOUR_THICK, 0.120)),
        origin=Origin(xyz=(0.0, 0.078, 0.118), rpy=(-0.52, 0.0, 0.0)),
        material=walnut,
        name="mid_crown",
    )
    tambour.visual(
        Box((TAMBOUR_WIDTH, TAMBOUR_THICK, 0.105)),
        origin=Origin(xyz=(0.0, 0.156, 0.162), rpy=(-0.20, 0.0, 0.0)),
        material=walnut,
        name="upper_crown",
    )
    tambour.visual(
        Box((TAMBOUR_WIDTH, 0.150, TAMBOUR_THICK)),
        origin=Origin(xyz=(0.0, 0.160, 0.175)),
        material=walnut,
        name="top_tail",
    )
    tambour.visual(
        Box((TAMBOUR_WIDTH, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, 0.110, 0.170)),
        material=walnut_dark,
        name="top_ribbon",
    )
    strap_x = TAMBOUR_WIDTH / 2.0 - 0.008
    strap_points = [
        (0.0, 0.0, -0.190),
        (0.0, 0.0, -0.030),
        (0.0, 0.034, -0.008),
        (0.0, 0.040, 0.050),
        (0.0, 0.075, 0.105),
        (0.0, 0.110, 0.145),
        (0.0, 0.140, 0.168),
    ]
    left_strap = tube_from_spline_points(
        [(-strap_x, y, z) for _, y, z in strap_points],
        radius=0.007,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    right_strap = tube_from_spline_points(
        [(strap_x, y, z) for _, y, z in strap_points],
        radius=0.007,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    tambour.visual(mesh_from_geometry(left_strap, "left_tambour_side_strap"), material=walnut_dark, name="left_side_strap")
    tambour.visual(mesh_from_geometry(right_strap, "right_tambour_side_strap"), material=walnut_dark, name="right_side_strap")
    for index, z in enumerate((-0.345, -0.303, -0.261, -0.219, -0.177, -0.135, -0.093, -0.051)):
        tambour.visual(
            Box((TAMBOUR_WIDTH - 0.030, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, -0.002, z)),
            material=walnut_dark,
            name=f"front_slat_{index}",
        )
    for index, y in enumerate((0.028, 0.068, 0.108, 0.148, 0.188)):
        tambour.visual(
            Box((TAMBOUR_WIDTH - 0.030, 0.026, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.170)),
            material=walnut_dark,
            name=f"top_slat_{index}",
        )
    tambour.inertial = Inertial.from_geometry(
        Box((TAMBOUR_WIDTH, 0.26, 0.56)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.09, -0.08)),
    )

    left_drawer = _drawer_part(
        model,
        "left_drawer",
        walnut,
        charcoal,
        drawer_liner,
        left_slide_gap=OUTER_SLIDE_GAP,
        right_slide_gap=INNER_SLIDE_GAP,
    )
    right_drawer = _drawer_part(
        model,
        "right_drawer",
        walnut,
        charcoal,
        drawer_liner,
        left_slide_gap=INNER_SLIDE_GAP,
        right_slide_gap=OUTER_SLIDE_GAP,
    )

    model.articulation(
        "carcass_to_tambour",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=tambour,
        origin=Origin(xyz=(0.0, TAMBOUR_FRONT_Y, TAMBOUR_ENTRY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.16, lower=0.0, upper=TAMBOUR_TRAVEL),
    )
    model.articulation(
        "carcass_to_left_drawer",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=left_drawer,
        origin=Origin(xyz=(-0.3965, -0.020, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "carcass_to_right_drawer",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=right_drawer,
        origin=Origin(xyz=(0.3965, -0.020, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=DRAWER_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    tambour = object_model.get_part("tambour")
    left_drawer = object_model.get_part("left_drawer")
    right_drawer = object_model.get_part("right_drawer")
    tambour_joint = object_model.get_articulation("carcass_to_tambour")
    left_joint = object_model.get_articulation("carcass_to_left_drawer")
    right_joint = object_model.get_articulation("carcass_to_right_drawer")

    with ctx.pose({tambour_joint: 0.0, left_joint: 0.0, right_joint: 0.0}):
        ctx.expect_gap(
            carcass,
            tambour,
            axis="y",
            positive_elem="lintel_stop",
            negative_elem="front_curtain",
            max_gap=0.008,
            max_penetration=0.001,
            name="closed tambour sits at the front opening",
        )
        ctx.expect_gap(
            left_drawer,
            tambour,
            axis="y",
            positive_elem="front_panel",
            negative_elem="front_curtain",
            min_gap=0.025,
            name="left drawer sits behind closed tambour",
        )
        ctx.expect_gap(
            right_drawer,
            tambour,
            axis="y",
            positive_elem="front_panel",
            negative_elem="front_curtain",
            min_gap=0.025,
            name="right drawer sits behind closed tambour",
        )
        closed_tambour_pos = ctx.part_world_position(tambour)
        closed_left_pos = ctx.part_world_position(left_drawer)
        closed_right_pos = ctx.part_world_position(right_drawer)

    with ctx.pose({tambour_joint: TAMBOUR_TRAVEL, left_joint: DRAWER_TRAVEL, right_joint: DRAWER_TRAVEL}):
        ctx.expect_gap(
            tambour,
            left_drawer,
            axis="y",
            positive_elem="front_curtain",
            negative_elem="front_panel",
            min_gap=0.12,
            name="opened tambour retracts behind left drawer front",
        )
        ctx.expect_gap(
            tambour,
            right_drawer,
            axis="y",
            positive_elem="front_curtain",
            negative_elem="front_panel",
            min_gap=0.12,
            name="opened tambour retracts behind right drawer front",
        )
        open_tambour_pos = ctx.part_world_position(tambour)
        open_left_pos = ctx.part_world_position(left_drawer)
        open_right_pos = ctx.part_world_position(right_drawer)

    ctx.check(
        "tambour retracts rearward into the top well",
        closed_tambour_pos is not None
        and open_tambour_pos is not None
        and open_tambour_pos[1] > closed_tambour_pos[1] + 0.20,
        details=f"closed={closed_tambour_pos}, open={open_tambour_pos}",
    )
    ctx.check(
        "left drawer opens forward",
        closed_left_pos is not None and open_left_pos is not None and open_left_pos[1] < closed_left_pos[1] - 0.15,
        details=f"closed={closed_left_pos}, open={open_left_pos}",
    )
    ctx.check(
        "right drawer opens forward",
        closed_right_pos is not None and open_right_pos is not None and open_right_pos[1] < closed_right_pos[1] - 0.15,
        details=f"closed={closed_right_pos}, open={open_right_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
