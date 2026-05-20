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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rect_loop(x0: float, y0: float, x1: float, y1: float) -> list[tuple[float, float]]:
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


DRAWER_TRAVEL = 0.22
TAMBOUR_TRAVEL = 0.19


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roll_top_desk")

    walnut = model.material("walnut", rgba=(0.41, 0.27, 0.16, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.27, 0.17, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.70, 0.58, 0.30, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.32, 0.29, 0.24, 1.0))
    liner = model.material("felt_liner", rgba=(0.33, 0.19, 0.14, 1.0))

    desk_w = 1.34
    desk_d = 0.68
    desk_h = 1.22
    pedestal_open_w = 0.26
    drawer_open_h = 0.30
    knee_open_w = 0.62
    knee_open_h = 0.67
    knee_bottom_z = 0.10
    knee_top_z = knee_bottom_z + knee_open_h
    tambour_open_w = 0.58
    tambour_open_bottom = 0.81
    tambour_open_top = 1.11
    drawer_depth = 0.56
    drawer_body_w = 0.224
    drawer_body_h = 0.255
    drawer_face_w = 0.284
    drawer_face_h = 0.316
    drawer_face_t = 0.018
    drawer_travel = DRAWER_TRAVEL
    tambour_travel = TAMBOUR_TRAVEL

    half_w = desk_w * 0.5
    half_d = desk_d * 0.5

    body = model.part("body")

    front_frame_t = 0.024
    wall_t = 0.030
    roof_t = 0.022
    pedestal_span_w = half_w - knee_open_w * 0.5

    body_shell = ExtrudeWithHolesGeometry(
        _rect_loop(-half_w, 0.0, half_w, desk_h),
        [
            _rect_loop(-0.63, knee_bottom_z, -0.37, knee_bottom_z + drawer_open_h),
            _rect_loop(-knee_open_w * 0.5, knee_bottom_z, knee_open_w * 0.5, knee_top_z),
            _rect_loop(0.37, knee_bottom_z, 0.63, knee_bottom_z + drawer_open_h),
            _rect_loop(
                -tambour_open_w * 0.5,
                tambour_open_bottom,
                tambour_open_w * 0.5,
                tambour_open_top,
            ),
        ],
        height=front_frame_t,
        center=True,
    ).rotate_x(math.pi / 2.0)
    body.visual(
        _mesh("roll_top_desk_case_shell", body_shell),
        origin=Origin(xyz=(0.0, half_d - front_frame_t * 0.5, 0.0)),
        material=walnut,
        name="case_shell",
    )

    body.visual(
        Box((desk_w, desk_d, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_walnut,
        name="base_plinth",
    )
    body.visual(
        Box((wall_t, desk_d, desk_h)),
        origin=Origin(xyz=(-half_w + wall_t * 0.5, 0.0, desk_h * 0.5)),
        material=dark_walnut,
        name="left_outer_wall",
    )
    body.visual(
        Box((wall_t, desk_d, desk_h)),
        origin=Origin(xyz=(half_w - wall_t * 0.5, 0.0, desk_h * 0.5)),
        material=dark_walnut,
        name="right_outer_wall",
    )
    body.visual(
        Box((wall_t, desk_d, desk_h)),
        origin=Origin(xyz=(-knee_open_w * 0.5 - wall_t * 0.5, 0.0, desk_h * 0.5)),
        material=dark_walnut,
        name="left_inner_divider",
    )
    body.visual(
        Box((wall_t, desk_d, desk_h)),
        origin=Origin(xyz=(knee_open_w * 0.5 + wall_t * 0.5, 0.0, desk_h * 0.5)),
        material=dark_walnut,
        name="right_inner_divider",
    )
    body.visual(
        Box((pedestal_span_w, desk_d, 0.030)),
        origin=Origin(
            xyz=(-knee_open_w * 0.5 - pedestal_span_w * 0.5, 0.0, knee_top_z + 0.015),
        ),
        material=walnut,
        name="left_pedestal_cap",
    )
    body.visual(
        Box((pedestal_span_w, desk_d, 0.030)),
        origin=Origin(
            xyz=(knee_open_w * 0.5 + pedestal_span_w * 0.5, 0.0, knee_top_z + 0.015),
        ),
        material=walnut,
        name="right_pedestal_cap",
    )
    body.visual(
        Box((knee_open_w, 0.30, roof_t)),
        origin=Origin(xyz=(0.0, 0.020, tambour_open_top + roof_t * 0.5)),
        material=dark_walnut,
        name="tambour_roof",
    )

    body.visual(
        Box((desk_w, 0.020, desk_h)),
        origin=Origin(xyz=(0.0, -half_d + 0.010, desk_h * 0.5)),
        material=dark_walnut,
        name="rear_panel",
    )
    body.visual(
        Box((knee_open_w, 0.48, 0.030)),
        origin=Origin(xyz=(0.0, -0.02, 0.745)),
        material=dark_walnut,
        name="writing_surface",
    )
    body.visual(
        Box((knee_open_w, 0.040, 0.060)),
        origin=Origin(xyz=(0.0, -0.24, 0.855)),
        material=dark_walnut,
        name="rear_tambour_pocket_bridge",
    )

    for prefix, sign in (("left", -1.0), ("right", 1.0)):
        opening_center_x = sign * 0.50
        outer_runner_x = sign * 0.626
        inner_runner_x = sign * 0.380
        for runner_name, runner_x in (
            (f"{prefix}_outer_runner", outer_runner_x),
            (f"{prefix}_inner_runner", inner_runner_x),
        ):
            body.visual(
                Box((0.016, 0.48, 0.022)),
                origin=Origin(xyz=(runner_x, -0.04, 0.265)),
                material=rail_metal,
                name=runner_name,
            )
            body.visual(
                Box((0.016, 0.042, knee_top_z - 0.276)),
                origin=Origin(
                    xyz=(runner_x, -0.20, (0.276 + knee_top_z) * 0.5),
                ),
                material=dark_walnut,
                name=f"{runner_name}_hanger",
            )
        body.visual(
            Box((0.270, 0.018, 0.040)),
            origin=Origin(xyz=(opening_center_x, -0.26, 0.255)),
            material=liner,
            name=f"{prefix}_drawer_stop",
        )

    rail_path_yz = [
        (0.170, 0.818),
        (0.145, 0.930),
        (0.085, 1.030),
        (0.010, 1.082),
        (-0.060, 1.040),
        (-0.105, 0.935),
        (-0.105, 0.825),
    ]
    for index, rail_x in enumerate((-0.299, 0.299)):
        rail_geom = tube_from_spline_points(
            [(rail_x, y, z) for y, z in rail_path_yz],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        )
        body.visual(
            _mesh(f"roll_top_desk_guide_rail_{index}", rail_geom),
            material=rail_metal,
            name=f"guide_rail_{index}",
        )
    for prefix, sign in (("left", -1.0), ("right", 1.0)):
        body.visual(
            Box((0.030, 0.310, 0.290)),
            origin=Origin(xyz=(sign * 0.296, 0.035, 0.955)),
            material=dark_walnut,
            name=f"{prefix}_tambour_channel",
        )

    body.inertial = Inertial.from_geometry(
        Box((desk_w, desk_d, desk_h)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, desk_h * 0.5)),
    )

    tambour = model.part("tambour")
    tambour.visual(
        Box((tambour_open_w - 0.018, 0.018, 0.235)),
        origin=Origin(xyz=(0.0, 0.172, 0.9275)),
        material=dark_walnut,
        name="tambour_shell",
    )
    tambour.visual(
        Box((tambour_open_w - 0.018, 0.305, 0.020)),
        origin=Origin(xyz=(0.0, 0.018, 1.040)),
        material=dark_walnut,
        name="tambour_top",
    )
    tambour.visual(
        Cylinder(radius=0.020, length=tambour_open_w - 0.026),
        origin=Origin(
            xyz=(0.0, 0.159, 1.031),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_walnut,
        name="tambour_corner",
    )
    slat_yz = [
        (0.172, 0.836),
        (0.150, 0.900),
        (0.118, 0.968),
        (0.074, 1.025),
        (0.018, 1.062),
        (-0.038, 1.042),
        (-0.082, 0.990),
        (-0.104, 0.920),
    ]
    for index, (slat_y, slat_z) in enumerate(slat_yz):
        tambour.visual(
            Box((tambour_open_w - 0.030, 0.010, 0.020)),
            origin=Origin(xyz=(0.0, slat_y, slat_z)),
            material=walnut,
            name=f"slat_{index}",
        )
    for index, strap_x in enumerate((-0.266, 0.266)):
        strap_geom = tube_from_spline_points(
            [(strap_x, y, z) for y, z in rail_path_yz],
            radius=0.007,
            samples_per_segment=14,
            radial_segments=12,
            cap_ends=True,
        )
        tambour.visual(
            _mesh(f"roll_top_desk_tambour_strap_{index}", strap_geom),
            material=dark_walnut,
            name=f"backing_strap_{index}",
        )
    tambour.visual(
        Cylinder(radius=0.012, length=tambour_open_w - 0.020),
        origin=Origin(
            xyz=(0.0, 0.176, 0.829),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="pull_bar",
    )
    tambour.inertial = Inertial.from_geometry(
        Box((tambour_open_w, 0.34, 0.28)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.02, 0.96)),
    )

    left_drawer = model.part("left_drawer")
    left_drawer.visual(
        Box((drawer_body_w, drawer_depth, drawer_body_h)),
        origin=Origin(xyz=(0.0, -drawer_depth * 0.5 + 0.004, 0.0)),
        material=dark_walnut,
        name="drawer_box",
    )
    left_drawer.visual(
        Box((drawer_face_w, drawer_face_t, drawer_face_h)),
        origin=Origin(xyz=(0.0, drawer_face_t * 0.5 + 0.004, 0.0)),
        material=walnut,
        name="drawer_face",
    )
    left_drawer.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(
            xyz=(0.0, drawer_face_t + 0.012, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="drawer_pull",
    )
    left_drawer.inertial = Inertial.from_geometry(
        Box((drawer_face_w, drawer_depth, drawer_face_h)),
        mass=7.0,
        origin=Origin(xyz=(0.0, -0.20, 0.0)),
    )

    right_drawer = model.part("right_drawer")
    right_drawer.visual(
        Box((drawer_body_w, drawer_depth, drawer_body_h)),
        origin=Origin(xyz=(0.0, -drawer_depth * 0.5 + 0.004, 0.0)),
        material=dark_walnut,
        name="drawer_box",
    )
    right_drawer.visual(
        Box((drawer_face_w, drawer_face_t, drawer_face_h)),
        origin=Origin(xyz=(0.0, drawer_face_t * 0.5 + 0.004, 0.0)),
        material=walnut,
        name="drawer_face",
    )
    right_drawer.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(
            xyz=(0.0, drawer_face_t + 0.012, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="drawer_pull",
    )
    right_drawer.inertial = Inertial.from_geometry(
        Box((drawer_face_w, drawer_depth, drawer_face_h)),
        mass=7.0,
        origin=Origin(xyz=(0.0, -0.20, 0.0)),
    )

    model.articulation(
        "body_to_tambour",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tambour,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.18,
            lower=0.0,
            upper=tambour_travel,
        ),
    )
    model.articulation(
        "body_to_left_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_drawer,
        origin=Origin(xyz=(-0.50, half_d, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=drawer_travel,
        ),
    )
    model.articulation(
        "body_to_right_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_drawer,
        origin=Origin(xyz=(0.50, half_d, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=drawer_travel,
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

    body = object_model.get_part("body")
    tambour = object_model.get_part("tambour")
    left_drawer = object_model.get_part("left_drawer")
    right_drawer = object_model.get_part("right_drawer")
    tambour_slide = object_model.get_articulation("body_to_tambour")
    left_slide = object_model.get_articulation("body_to_left_drawer")
    right_slide = object_model.get_articulation("body_to_right_drawer")

    ctx.expect_gap(
        tambour,
        body,
        axis="z",
        positive_elem="tambour_shell",
        negative_elem="writing_surface",
        min_gap=0.05,
        max_gap=0.11,
        name="tambour clears the writing surface",
    )
    ctx.expect_gap(
        left_drawer,
        body,
        axis="y",
        positive_elem="drawer_face",
        negative_elem="case_shell",
        min_gap=0.002,
        max_gap=0.012,
        name="left drawer face sits just proud of the case front",
    )
    ctx.expect_gap(
        right_drawer,
        body,
        axis="y",
        positive_elem="drawer_face",
        negative_elem="case_shell",
        min_gap=0.002,
        max_gap=0.012,
        name="right drawer face sits just proud of the case front",
    )
    ctx.expect_within(
        left_drawer,
        body,
        axes="xz",
        inner_elem="drawer_box",
        outer_elem="case_shell",
        margin=0.06,
        name="left drawer stays aligned within the left pedestal footprint",
    )
    ctx.expect_within(
        right_drawer,
        body,
        axes="xz",
        inner_elem="drawer_box",
        outer_elem="case_shell",
        margin=0.06,
        name="right drawer stays aligned within the right pedestal footprint",
    )

    left_closed = ctx.part_world_position(left_drawer)
    right_closed = ctx.part_world_position(right_drawer)
    tambour_closed = ctx.part_world_position(tambour)

    with ctx.pose({left_slide: DRAWER_TRAVEL, right_slide: DRAWER_TRAVEL, tambour_slide: TAMBOUR_TRAVEL}):
        ctx.expect_overlap(
            left_drawer,
            body,
            axes="y",
            elem_a="drawer_box",
            elem_b="left_pedestal_cap",
            min_overlap=0.30,
            name="left drawer retains insertion when extended",
        )
        ctx.expect_overlap(
            right_drawer,
            body,
            axes="y",
            elem_a="drawer_box",
            elem_b="right_pedestal_cap",
            min_overlap=0.30,
            name="right drawer retains insertion when extended",
        )
        left_open = ctx.part_world_position(left_drawer)
        right_open = ctx.part_world_position(right_drawer)
        tambour_open = ctx.part_world_position(tambour)

    ctx.check(
        "left drawer opens forward",
        left_closed is not None and left_open is not None and left_open[1] > left_closed[1] + 0.18,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right drawer opens forward",
        right_closed is not None and right_open is not None and right_open[1] > right_closed[1] + 0.18,
        details=f"closed={right_closed}, open={right_open}",
    )
    ctx.check(
        "tambour retracts rearward",
        tambour_closed is not None and tambour_open is not None and tambour_open[1] < tambour_closed[1] - 0.15,
        details=f"closed={tambour_closed}, open={tambour_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
