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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="murphy_fold_down_wall_desk")

    walnut = model.material("walnut", rgba=(0.37, 0.25, 0.17, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.28, 0.18, 0.12, 1.0))
    interior_oak = model.material("interior_oak", rgba=(0.62, 0.50, 0.34, 1.0))
    hardware = model.material("hardware", rgba=(0.69, 0.70, 0.73, 1.0))
    felt = model.material("felt", rgba=(0.17, 0.21, 0.19, 1.0))

    cabinet_width = 0.82
    carcass_depth = 0.222
    face_frame_depth = 0.018
    cabinet_depth = carcass_depth + face_frame_depth
    cabinet_height = 0.78

    side_thickness = 0.020
    back_thickness = 0.018
    top_thickness = 0.020
    bottom_thickness = 0.024
    stile_width = 0.040

    drawer_height = 0.110
    drawer_front_width = 0.730
    drawer_front_thickness = 0.018
    drawer_box_depth = 0.164
    drawer_box_width = 0.692
    drawer_box_height = 0.080
    drawer_slide_upper = 0.100

    leaf_width = 0.736
    leaf_height = 0.532
    leaf_thickness = 0.022

    cabinet = model.part("cabinet_body")
    cabinet.visual(
        Box((back_thickness, cabinet_width, cabinet_height)),
        origin=Origin(xyz=(back_thickness / 2.0, 0.0, cabinet_height / 2.0)),
        material=walnut,
        name="back_panel",
    )
    cabinet.visual(
        Box((carcass_depth, cabinet_width, top_thickness)),
        origin=Origin(xyz=(carcass_depth / 2.0, 0.0, cabinet_height - top_thickness / 2.0)),
        material=walnut,
        name="carcass_top",
    )
    cabinet.visual(
        Box((carcass_depth, cabinet_width, bottom_thickness)),
        origin=Origin(xyz=(carcass_depth / 2.0, 0.0, bottom_thickness / 2.0)),
        material=walnut_dark,
        name="carcass_bottom",
    )
    cabinet.visual(
        Box((carcass_depth, side_thickness, cabinet_height)),
        origin=Origin(xyz=(carcass_depth / 2.0, -cabinet_width / 2.0 + side_thickness / 2.0, cabinet_height / 2.0)),
        material=walnut,
        name="left_side",
    )
    cabinet.visual(
        Box((carcass_depth, side_thickness, cabinet_height)),
        origin=Origin(xyz=(carcass_depth / 2.0, cabinet_width / 2.0 - side_thickness / 2.0, cabinet_height / 2.0)),
        material=walnut,
        name="right_side",
    )
    cabinet.visual(
        Box((face_frame_depth, cabinet_width - 0.040, 0.048)),
        origin=Origin(xyz=(carcass_depth + face_frame_depth / 2.0, 0.0, cabinet_height - 0.024)),
        material=walnut_dark,
        name="top_face_rail",
    )
    cabinet.visual(
        Box((face_frame_depth, stile_width, 0.540)),
        origin=Origin(xyz=(carcass_depth + face_frame_depth / 2.0, -cabinet_width / 2.0 + stile_width / 2.0, 0.270)),
        material=walnut_dark,
        name="left_leaf_stile",
    )
    cabinet.visual(
        Box((face_frame_depth, stile_width, 0.540)),
        origin=Origin(xyz=(carcass_depth + face_frame_depth / 2.0, cabinet_width / 2.0 - stile_width / 2.0, 0.270)),
        material=walnut_dark,
        name="right_leaf_stile",
    )
    cabinet.visual(
        Box((face_frame_depth, cabinet_width - 0.080, 0.036)),
        origin=Origin(xyz=(carcass_depth + face_frame_depth / 2.0, 0.0, 0.550)),
        material=walnut_dark,
        name="drawer_separator_rail",
    )
    cabinet.visual(
        Box((face_frame_depth, stile_width, 0.124)),
        origin=Origin(xyz=(carcass_depth + face_frame_depth / 2.0, -cabinet_width / 2.0 + stile_width / 2.0, 0.622)),
        material=walnut_dark,
        name="left_drawer_stile",
    )
    cabinet.visual(
        Box((face_frame_depth, stile_width, 0.124)),
        origin=Origin(xyz=(carcass_depth + face_frame_depth / 2.0, cabinet_width / 2.0 - stile_width / 2.0, 0.622)),
        material=walnut_dark,
        name="right_drawer_stile",
    )
    cabinet.visual(
        Box((face_frame_depth, cabinet_width - 0.080, 0.032)),
        origin=Origin(xyz=(carcass_depth + face_frame_depth / 2.0, 0.0, 0.698)),
        material=walnut_dark,
        name="drawer_top_rail",
    )
    cabinet.visual(
        Box((0.186, cabinet_width - 0.080, 0.018)),
        origin=Origin(xyz=(0.111, 0.0, 0.568)),
        material=interior_oak,
        name="drawer_bay_floor",
    )
    cabinet.visual(
        Box((0.150, cabinet_width - 0.100, 0.014)),
        origin=Origin(xyz=(0.089, 0.0, 0.318)),
        material=interior_oak,
        name="organizer_shelf",
    )
    cabinet.visual(
        Box((0.150, 0.016, 0.220)),
        origin=Origin(xyz=(0.089, -0.185, 0.425)),
        material=interior_oak,
        name="left_divider",
    )
    cabinet.visual(
        Box((0.150, 0.016, 0.220)),
        origin=Origin(xyz=(0.089, 0.185, 0.425)),
        material=interior_oak,
        name="right_divider",
    )
    cabinet.visual(
        Box((0.014, 0.460, 0.170)),
        origin=Origin(xyz=(0.079, 0.0, 0.433)),
        material=interior_oak,
        name="center_divider",
    )
    cabinet.visual(
        Box((0.120, 0.720, 0.008)),
        origin=Origin(xyz=(0.078, 0.0, 0.445)),
        material=felt,
        name="writing_nook_backer",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_depth, cabinet_width, cabinet_height)),
        mass=24.0,
        origin=Origin(xyz=(cabinet_depth / 2.0, 0.0, cabinet_height / 2.0)),
    )

    leaf = model.part("work_leaf")
    leaf.visual(
        Box((leaf_thickness, leaf_width, leaf_height)),
        origin=Origin(xyz=(leaf_thickness / 2.0, 0.0, leaf_height / 2.0)),
        material=walnut,
        name="leaf_panel",
    )
    leaf.visual(
        Box((0.042, leaf_width - 0.060, 0.040)),
        origin=Origin(xyz=(0.032, 0.0, leaf_height - 0.026)),
        material=walnut_dark,
        name="leaf_front_apron",
    )
    leaf.visual(
        Box((0.046, 0.034, leaf_height - 0.090)),
        origin=Origin(xyz=(0.033, -leaf_width / 2.0 + 0.017, (leaf_height - 0.090) / 2.0 + 0.020)),
        material=walnut_dark,
        name="left_leaf_batten",
    )
    leaf.visual(
        Box((0.046, 0.034, leaf_height - 0.090)),
        origin=Origin(xyz=(0.033, leaf_width / 2.0 - 0.017, (leaf_height - 0.090) / 2.0 + 0.020)),
        material=walnut_dark,
        name="right_leaf_batten",
    )
    leaf.visual(
        Cylinder(radius=0.006, length=0.170),
        origin=Origin(xyz=(0.039, 0.0, 0.438), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="leaf_pull_bar",
    )
    leaf.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(xyz=(0.028, -0.055, 0.438), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="leaf_pull_left_post",
    )
    leaf.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(xyz=(0.028, 0.055, 0.438), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="leaf_pull_right_post",
    )
    leaf.inertial = Inertial.from_geometry(
        Box((0.060, leaf_width, leaf_height)),
        mass=8.5,
        origin=Origin(xyz=(0.030, 0.0, leaf_height / 2.0)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((drawer_front_thickness, drawer_front_width, drawer_height)),
        origin=Origin(xyz=(drawer_front_thickness / 2.0, 0.0, drawer_height / 2.0)),
        material=walnut,
        name="drawer_front",
    )
    drawer.visual(
        Cylinder(radius=0.006, length=0.160),
        origin=Origin(xyz=(0.036, 0.0, drawer_height / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="drawer_pull_bar",
    )
    drawer.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.026, -0.050, drawer_height / 2.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="drawer_pull_left_post",
    )
    drawer.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.026, 0.050, drawer_height / 2.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="drawer_pull_right_post",
    )
    drawer.visual(
        Box((drawer_box_depth, drawer_box_width, 0.006)),
        origin=Origin(xyz=(-drawer_box_depth / 2.0, 0.0, 0.006)),
        material=interior_oak,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((drawer_box_depth, 0.012, drawer_box_height)),
        origin=Origin(xyz=(-drawer_box_depth / 2.0, -drawer_box_width / 2.0 + 0.006, drawer_box_height / 2.0)),
        material=interior_oak,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((drawer_box_depth, 0.012, drawer_box_height)),
        origin=Origin(xyz=(-drawer_box_depth / 2.0, drawer_box_width / 2.0 - 0.006, drawer_box_height / 2.0)),
        material=interior_oak,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((0.012, drawer_box_width, drawer_box_height)),
        origin=Origin(xyz=(-drawer_box_depth + 0.006, 0.0, drawer_box_height / 2.0)),
        material=interior_oak,
        name="drawer_back",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_box_depth + drawer_front_thickness, drawer_front_width, drawer_height)),
        mass=3.2,
        origin=Origin(xyz=(-0.073, 0.0, drawer_height / 2.0)),
    )

    model.articulation(
        "cabinet_to_leaf",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=leaf,
        origin=Origin(xyz=(cabinet_depth, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(cabinet_depth, 0.0, 0.588)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=0.0,
            upper=drawer_slide_upper,
        ),
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

    cabinet = object_model.get_part("cabinet_body")
    leaf = object_model.get_part("work_leaf")
    drawer = object_model.get_part("drawer")
    leaf_hinge = object_model.get_articulation("cabinet_to_leaf")
    drawer_slide = object_model.get_articulation("cabinet_to_drawer")
    drawer_slide_upper = 0.0
    if drawer_slide.motion_limits is not None and drawer_slide.motion_limits.upper is not None:
        drawer_slide_upper = drawer_slide.motion_limits.upper

    ctx.expect_gap(
        leaf,
        cabinet,
        axis="x",
        positive_elem="leaf_panel",
        max_gap=0.003,
        max_penetration=1e-6,
        name="closed leaf seats against cabinet front",
    )
    ctx.expect_overlap(
        leaf,
        cabinet,
        axes="yz",
        elem_a="leaf_panel",
        min_overlap=0.50,
        name="closed leaf covers cabinet opening",
    )
    ctx.expect_gap(
        cabinet,
        leaf,
        axis="z",
        positive_elem="drawer_separator_rail",
        negative_elem="leaf_panel",
        min_gap=0.0,
        max_gap=0.025,
        name="drawer rail clears top of closed leaf",
    )
    ctx.expect_within(
        drawer,
        cabinet,
        axes="yz",
        inner_elem="drawer_front",
        margin=0.050,
        name="drawer front stays centered in cabinet face",
    )

    drawer_rest = ctx.part_world_position(drawer)
    cabinet_aabb = ctx.part_world_aabb(cabinet)
    leaf_closed_aabb = ctx.part_world_aabb(leaf)

    with ctx.pose({drawer_slide: drawer_slide_upper}):
        ctx.expect_within(
            drawer,
            cabinet,
            axes="yz",
            inner_elem="drawer_front",
            margin=0.050,
            name="extended drawer stays centered in cabinet face",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="x",
            elem_a="drawer_bottom",
            min_overlap=0.040,
            name="drawer retains insertion at full extension",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer extends outward from cabinet",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.080,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    with ctx.pose({leaf_hinge: math.pi / 2.0, drawer_slide: drawer_slide_upper}):
        leaf_open_aabb = ctx.part_world_aabb(leaf)
        ctx.expect_gap(
            drawer,
            leaf,
            axis="z",
            positive_elem="drawer_bottom",
            negative_elem="leaf_panel",
            min_gap=0.45,
            name="open drawer remains above horizontal work surface",
        )
    ctx.check(
        "leaf folds down into a desk surface",
        cabinet_aabb is not None
        and leaf_closed_aabb is not None
        and leaf_open_aabb is not None
        and leaf_open_aabb[1][0] > cabinet_aabb[1][0] + 0.40
        and leaf_open_aabb[1][2] < leaf_closed_aabb[1][2] - 0.45,
        details=f"cabinet={cabinet_aabb}, leaf_closed={leaf_closed_aabb}, leaf_open={leaf_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
