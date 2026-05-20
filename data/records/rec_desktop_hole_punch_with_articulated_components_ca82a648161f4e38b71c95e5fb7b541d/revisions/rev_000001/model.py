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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_hole_office_punch")

    body_metal = model.material("body_metal", rgba=(0.24, 0.27, 0.31, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.56, 0.61, 0.67, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    tray_plastic = model.material("tray_plastic", rgba=(0.15, 0.15, 0.17, 1.0))
    guide_accent = model.material("guide_accent", rgba=(0.78, 0.12, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.305, 0.105, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=body_metal,
        name="deck",
    )
    body.visual(
        Box((0.020, 0.095, 0.030)),
        origin=Origin(xyz=(-0.1425, 0.0, 0.015)),
        material=body_metal,
        name="side_wall_0",
    )
    body.visual(
        Box((0.020, 0.095, 0.016)),
        origin=Origin(xyz=(0.1425, 0.0, 0.022)),
        material=body_metal,
        name="side_wall_1",
    )
    body.visual(
        Box((0.265, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.0435, 0.015)),
        material=body_metal,
        name="rear_bridge",
    )
    body.visual(
        Box((0.255, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.041, 0.020)),
        material=black_plastic,
        name="front_fence",
    )
    body.visual(
        Box((0.238, 0.015, 0.010)),
        origin=Origin(xyz=(0.0, -0.011, 0.028)),
        material=black_plastic,
        name="punch_bar",
    )
    body.visual(
        Box((0.248, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.046, 0.006)),
        material=black_plastic,
        name="tray_rail",
    )
    body.visual(
        Box((0.020, 0.016, 0.014)),
        origin=Origin(xyz=(0.1425, -0.0445, 0.007)),
        material=body_metal,
        name="foot_0",
    )
    body.visual(
        Box((0.020, 0.016, 0.014)),
        origin=Origin(xyz=(0.1425, 0.0445, 0.007)),
        material=body_metal,
        name="foot_1",
    )
    for idx, x_pos in enumerate((-0.095, 0.0, 0.095)):
        body.visual(
            Cylinder(radius=0.010, length=0.014),
            origin=Origin(xyz=(x_pos, -0.011, 0.016)),
            material=black_plastic,
            name=f"die_{idx}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.305, 0.105, 0.050)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    handle = model.part("handle")

    def xz_section(width: float, height: float, radius: float, y: float, z_center: float) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_center) for x, z in rounded_rect_profile(width, height, radius)]

    handle_geom = section_loft(
        [
            xz_section(0.280, 0.018, 0.007, -0.002, 0.002),
            xz_section(0.292, 0.030, 0.012, -0.050, 0.012),
            xz_section(0.258, 0.022, 0.009, -0.108, 0.006),
        ]
    )
    handle.visual(
        mesh_from_geometry(handle_geom, "handle_shell"),
        material=handle_metal,
        name="handle_shell",
    )
    handle.visual(
        Box((0.240, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.014, -0.004)),
        material=black_plastic,
        name="handle_rib",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.292, 0.112, 0.036)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.056, 0.012)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.043, 0.058)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.258, 0.069, 0.004)),
        origin=Origin(xyz=(0.129, 0.0, 0.002)),
        material=tray_plastic,
        name="tray_floor",
    )
    tray.visual(
        Box((0.258, 0.004, 0.009)),
        origin=Origin(xyz=(0.129, -0.0325, 0.0045)),
        material=tray_plastic,
        name="tray_front",
    )
    tray.visual(
        Box((0.258, 0.004, 0.009)),
        origin=Origin(xyz=(0.129, 0.0325, 0.0045)),
        material=tray_plastic,
        name="tray_rear",
    )
    tray.visual(
        Box((0.004, 0.061, 0.009)),
        origin=Origin(xyz=(0.002, 0.0, 0.0045)),
        material=tray_plastic,
        name="tray_inner_side",
    )
    tray.visual(
        Box((0.004, 0.061, 0.009)),
        origin=Origin(xyz=(0.256, 0.0, 0.0045)),
        material=tray_plastic,
        name="tray_outer_side",
    )
    tray.visual(
        Box((0.014, 0.032, 0.008)),
        origin=Origin(xyz=(0.265, 0.0, 0.004)),
        material=tray_plastic,
        name="tray_grip",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.272, 0.068, 0.010)),
        mass=0.18,
        origin=Origin(xyz=(0.136, 0.0, 0.005)),
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(-0.129, 0.0, 0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
    )

    tab = model.part("tab")
    tab.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="tab_hub",
    )
    tab.visual(
        Box((0.004, 0.014, 0.008)),
        origin=Origin(xyz=(0.006, 0.007, 0.0)),
        material=guide_accent,
        name="tab_paddle",
    )
    tab.inertial = Inertial.from_geometry(
        Box((0.010, 0.016, 0.010)),
        mass=0.02,
        origin=Origin(xyz=(0.005, 0.006, 0.0)),
    )

    model.articulation(
        "tray_to_tab",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=tab,
        origin=Origin(xyz=(0.258, -0.018, 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=3.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.028, 0.010, 0.018)),
        origin=Origin(xyz=(0.014, 0.0, 0.009)),
        material=black_plastic,
        name="guide_body",
    )
    guide.visual(
        Box((0.006, 0.014, 0.010)),
        origin=Origin(xyz=(0.024, -0.002, 0.017)),
        material=guide_accent,
        name="guide_flag",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.030, 0.014, 0.022)),
        mass=0.04,
        origin=Origin(xyz=(0.015, -0.001, 0.010)),
    )

    model.articulation(
        "body_to_guide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=guide,
        origin=Origin(xyz=(-0.112, -0.050, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.194,
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
    handle = object_model.get_part("handle")
    tray = object_model.get_part("tray")
    tab = object_model.get_part("tab")
    guide = object_model.get_part("guide")
    handle_joint = object_model.get_articulation("body_to_handle")
    tray_joint = object_model.get_articulation("body_to_tray")
    tab_joint = object_model.get_articulation("tray_to_tab")
    guide_joint = object_model.get_articulation("body_to_guide")

    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem="handle_shell",
            negative_elem="deck",
            min_gap=0.0005,
            max_gap=0.012,
            name="handle rests just above the deck",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="x",
            elem_a="handle_shell",
            elem_b="deck",
            min_overlap=0.24,
            name="handle spans the body width",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="z",
            positive_elem="deck",
            negative_elem="tray_floor",
            min_gap=0.020,
            max_gap=0.030,
            name="tray sits below the punching deck",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="xy",
            elem_a="tray_floor",
            elem_b="deck",
            min_overlap=0.060,
            name="tray nests under the punch body",
        )
        ctx.expect_gap(
            body,
            guide,
            axis="y",
            positive_elem="front_fence",
            negative_elem="guide_body",
            min_gap=0.0,
            max_gap=0.001,
            name="guide bears against the front fence",
        )

    rest_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: math.radians(50.0)}):
        open_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle opens upward",
        rest_aabb is not None and open_aabb is not None and open_aabb[1][2] > rest_aabb[1][2] + 0.03,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    tray_rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: 0.055}):
        tray_open_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="tray_floor",
            elem_b="deck",
            min_overlap=0.18,
            name="tray remains retained when extended",
        )

    ctx.check(
        "tray slides out to the side",
        tray_rest_pos is not None and tray_open_pos is not None and tray_open_pos[0] > tray_rest_pos[0] + 0.045,
        details=f"rest={tray_rest_pos}, open={tray_open_pos}",
    )

    guide_rest_pos = ctx.part_world_position(guide)
    with ctx.pose({guide_joint: 0.194}):
        guide_open_pos = ctx.part_world_position(guide)
        ctx.expect_gap(
            body,
            guide,
            axis="y",
            positive_elem="front_fence",
            negative_elem="guide_body",
            min_gap=0.0,
            max_gap=0.001,
            name="guide stays on the front fence through travel",
        )

    ctx.check(
        "guide slides across the front fence",
        guide_rest_pos is not None and guide_open_pos is not None and guide_open_pos[0] > guide_rest_pos[0] + 0.18,
        details=f"rest={guide_rest_pos}, open={guide_open_pos}",
    )

    with ctx.pose({tray_joint: 0.055, tab_joint: -0.30}):
        tab_low = ctx.part_element_world_aabb(tab, elem="tab_paddle")
    with ctx.pose({tray_joint: 0.055, tab_joint: 0.30}):
        tab_high = ctx.part_element_world_aabb(tab, elem="tab_paddle")

    ctx.check(
        "tab pivots about its side-facing axis",
        tab_low is not None and tab_high is not None and tab_high[1][2] > tab_low[1][2] + 0.003,
        details=f"low={tab_low}, high={tab_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
