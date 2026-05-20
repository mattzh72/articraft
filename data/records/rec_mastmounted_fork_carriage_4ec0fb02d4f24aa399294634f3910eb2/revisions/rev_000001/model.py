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


MAST_HEIGHT = 0.92
MAST_DEPTH = 0.08
UPRIGHT_WIDTH = 0.05
UPRIGHT_CENTER_X = 0.125
TOP_CROSS_Z = 0.83
TOP_CROSS_HEIGHT = 0.055
BOTTOM_CROSS_HEIGHT = 0.10
BOTTOM_CROSS_DEPTH = 0.10
GUIDE_STRIP_HEIGHT = 0.66
GUIDE_STRIP_Z = 0.46
GUIDE_STRIP_WIDTH = 0.024
GUIDE_STRIP_DEPTH = 0.008

PLATEN_WIDTH = 0.24
PLATEN_PLATE_HEIGHT = 0.16
PLATEN_PLATE_DEPTH = 0.018
PLATEN_FRAME_Y = 0.074
PLATEN_LOWER_Z = 0.12
PLATEN_TRAVEL = 0.34

FORK_CENTER_X = 0.095
FORK_WIDTH = 0.065
FORK_LENGTH = 0.56
FORK_THICKNESS = 0.032


def _mast_frame_shape() -> cq.Workplane:
    left_upright = (
        cq.Workplane("XY")
        .box(UPRIGHT_WIDTH, MAST_DEPTH, MAST_HEIGHT, centered=(True, True, False))
        .translate((-UPRIGHT_CENTER_X, 0.0, 0.0))
    )
    right_upright = (
        cq.Workplane("XY")
        .box(UPRIGHT_WIDTH, MAST_DEPTH, MAST_HEIGHT, centered=(True, True, False))
        .translate((UPRIGHT_CENTER_X, 0.0, 0.0))
    )
    bottom_cross = (
        cq.Workplane("XY")
        .box(0.24, BOTTOM_CROSS_DEPTH, BOTTOM_CROSS_HEIGHT, centered=(True, True, False))
        .translate((0.0, -0.002, 0.0))
    )
    top_cross = (
        cq.Workplane("XY")
        .box(0.28, 0.06, TOP_CROSS_HEIGHT, centered=(True, True, True))
        .translate((0.0, -0.006, TOP_CROSS_Z))
    )
    center_guide = (
        cq.Workplane("XY")
        .box(0.070, 0.028, 0.71, centered=(True, True, True))
        .translate((0.0, -0.016, 0.455))
    )
    left_strip = (
        cq.Workplane("XY")
        .box(
            GUIDE_STRIP_WIDTH,
            GUIDE_STRIP_DEPTH,
            GUIDE_STRIP_HEIGHT,
            centered=(True, True, True),
        )
        .translate((-UPRIGHT_CENTER_X, 0.044, GUIDE_STRIP_Z))
    )
    right_strip = (
        cq.Workplane("XY")
        .box(
            GUIDE_STRIP_WIDTH,
            GUIDE_STRIP_DEPTH,
            GUIDE_STRIP_HEIGHT,
            centered=(True, True, True),
        )
        .translate((UPRIGHT_CENTER_X, 0.044, GUIDE_STRIP_Z))
    )
    left_gusset = (
        cq.Workplane("YZ")
        .polyline([(0.06, 0.0), (0.06, 0.20), (-0.012, 0.10), (-0.012, 0.0)])
        .close()
        .extrude(0.012)
        .translate((-0.098, -0.028, 0.0))
    )
    right_gusset = (
        cq.Workplane("YZ")
        .polyline([(0.06, 0.0), (0.06, 0.20), (-0.012, 0.10), (-0.012, 0.0)])
        .close()
        .extrude(-0.012)
        .translate((0.098, -0.028, 0.0))
    )

    return (
        left_upright.union(right_upright)
        .union(bottom_cross)
        .union(top_cross)
        .union(center_guide)
        .union(left_strip)
        .union(right_strip)
        .union(left_gusset)
        .union(right_gusset)
    )


def _platen_weldment_shape() -> cq.Workplane:
    back_plate = cq.Workplane("XY").box(
        PLATEN_WIDTH,
        PLATEN_PLATE_DEPTH,
        PLATEN_PLATE_HEIGHT,
        centered=True,
    )
    top_rail = (
        cq.Workplane("XY")
        .box(0.26, 0.024, 0.028, centered=True)
        .translate((0.0, 0.0, 0.078))
    )
    bottom_rail = (
        cq.Workplane("XY")
        .box(0.26, 0.028, 0.032, centered=True)
        .translate((0.0, 0.002, -0.068))
    )
    left_shoe = (
        cq.Workplane("XY")
        .box(0.038, 0.028, 0.13, centered=True)
        .translate((-0.108, -0.012, 0.0))
    )
    right_shoe = (
        cq.Workplane("XY")
        .box(0.038, 0.028, 0.13, centered=True)
        .translate((0.108, -0.012, 0.0))
    )
    left_post = (
        cq.Workplane("XY")
        .box(0.030, 0.018, 0.30, centered=True)
        .translate((-0.095, 0.0, 0.22))
    )
    right_post = (
        cq.Workplane("XY")
        .box(0.030, 0.018, 0.30, centered=True)
        .translate((0.095, 0.0, 0.22))
    )
    top_guard = (
        cq.Workplane("XY")
        .box(0.22, 0.018, 0.022, centered=True)
        .translate((0.0, 0.0, 0.37))
    )
    mid_guard = (
        cq.Workplane("XY")
        .box(0.20, 0.014, 0.018, centered=True)
        .translate((0.0, 0.0, 0.28))
    )
    low_guard = (
        cq.Workplane("XY")
        .box(0.20, 0.012, 0.016, centered=True)
        .translate((0.0, 0.0, 0.19))
    )

    return (
        back_plate.union(top_rail)
        .union(bottom_rail)
        .union(left_shoe)
        .union(right_shoe)
        .union(left_post)
        .union(right_post)
        .union(top_guard)
        .union(mid_guard)
        .union(low_guard)
    )


def _fork_shape() -> cq.Workplane:
    heel = (
        cq.Workplane("YZ")
        .polyline([(-0.010, 0.066), (0.065, 0.066), (0.158, -0.040), (-0.010, -0.040)])
        .close()
        .extrude(FORK_WIDTH, both=True)
    )
    shank = (
        cq.Workplane("XY")
        .box(0.056, 0.032, 0.14, centered=True)
        .translate((0.0, 0.012, 0.0))
    )
    tine = (
        cq.Workplane("XY")
        .box(FORK_WIDTH, FORK_LENGTH, FORK_THICKNESS, centered=(True, False, True))
        .translate((0.0, 0.0, -0.056))
    )
    toe_cut = (
        cq.Workplane("YZ")
        .polyline([(0.47, -0.080), (0.56, -0.080), (0.56, -0.040)])
        .close()
        .extrude(FORK_WIDTH + 0.010, both=True)
    )

    return heel.union(shank).union(tine).cut(toe_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_fork_carriage")

    model.material("mast_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("carriage_black", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("fork_steel", rgba=(0.32, 0.33, 0.35, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_mast_frame_shape(), "mast_frame"),
        material="mast_gray",
        name="mast_frame",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.32, 0.10, MAST_HEIGHT)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT / 2.0)),
    )

    platen = model.part("platen")
    platen.visual(
        mesh_from_cadquery(_platen_weldment_shape(), "platen_weldment"),
        material="carriage_black",
        name="platen_weldment",
    )
    platen.visual(
        mesh_from_cadquery(_fork_shape(), "left_fork"),
        origin=Origin(xyz=(-FORK_CENTER_X, 0.0, 0.0)),
        material="fork_steel",
        name="left_fork",
    )
    platen.visual(
        mesh_from_cadquery(_fork_shape(), "right_fork"),
        origin=Origin(xyz=(FORK_CENTER_X, 0.0, 0.0)),
        material="fork_steel",
        name="right_fork",
    )
    platen.inertial = Inertial.from_geometry(
        Box((0.32, 0.58, 0.46)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.27, 0.15)),
    )

    model.articulation(
        "mast_to_platen",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=platen,
        origin=Origin(xyz=(0.0, PLATEN_FRAME_Y, PLATEN_LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PLATEN_TRAVEL,
            effort=1400.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    mast = object_model.get_part("mast")
    platen = object_model.get_part("platen")
    lift = object_model.get_articulation("mast_to_platen")
    mast_frame = mast.get_visual("mast_frame")
    weldment = platen.get_visual("platen_weldment")
    left_fork = platen.get_visual("left_fork")
    right_fork = platen.get_visual("right_fork")

    ctx.check(
        "named carriage visuals exist",
        mast_frame is not None
        and weldment is not None
        and left_fork is not None
        and right_fork is not None,
        details="Expected mast frame, weldment, and both fork visuals.",
    )
    ctx.expect_contact(
        platen,
        mast,
        elem_a=weldment,
        elem_b=mast_frame,
        name="platen is supported against the mast guide faces",
    )
    ctx.expect_within(
        platen,
        mast,
        axes="x",
        margin=0.035,
        inner_elem=weldment,
        outer_elem=mast_frame,
        name="platen weldment stays within mast width",
    )
    ctx.expect_overlap(
        platen,
        mast,
        axes="z",
        min_overlap=0.12,
        elem_a=weldment,
        elem_b=mast_frame,
        name="lowered platen remains engaged along the mast",
    )

    mast_box = ctx.part_element_world_aabb(mast, elem=mast_frame)
    left_box = ctx.part_element_world_aabb(platen, elem=left_fork)
    right_box = ctx.part_element_world_aabb(platen, elem=right_fork)
    forks_project = (
        mast_box is not None
        and left_box is not None
        and right_box is not None
        and left_box[1][1] > mast_box[1][1] + 0.45
        and right_box[1][1] > mast_box[1][1] + 0.45
    )
    forks_level = (
        left_box is not None
        and right_box is not None
        and abs(left_box[0][2] - right_box[0][2]) < 1e-4
        and abs(left_box[1][1] - right_box[1][1]) < 1e-4
    )
    ctx.check(
        "paired forks project forward evenly",
        forks_project and forks_level,
        details=f"mast={mast_box}, left={left_box}, right={right_box}",
    )

    rest_pos = ctx.part_world_position(platen)
    raised_pos = None
    with ctx.pose({lift: PLATEN_TRAVEL}):
        ctx.expect_gap(
            platen,
            mast,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem=weldment,
            negative_elem=mast_frame,
            name="raised platen stays aligned with the mast guide plane",
        )
        ctx.expect_contact(
            platen,
            mast,
            elem_a=weldment,
            elem_b=mast_frame,
            name="raised platen remains supported on the mast guides",
        )
        ctx.expect_overlap(
            platen,
            mast,
            axes="z",
            min_overlap=0.12,
            elem_a=weldment,
            elem_b=mast_frame,
            name="raised platen retains vertical mast engagement",
        )
        raised_pos = ctx.part_world_position(platen)
    ctx.check(
        "platen lifts upward",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
