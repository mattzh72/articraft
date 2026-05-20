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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_slab(width: float, depth: float, height: float, radius: float, name: str):
    profile = rounded_rect_profile(width, depth, radius)
    return mesh_from_geometry(
        ExtrudeGeometry.centered(profile, height, cap=True, closed=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_hole_punch")

    body_dark = model.material("body_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    accent = model.material("accent", rgba=(0.86, 0.28, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_slab(0.146, 0.248, 0.008, 0.010, "base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=body_dark,
        name="base_plate",
    )
    body.visual(
        _rounded_slab(0.122, 0.064, 0.018, 0.006, "rear_deck"),
        origin=Origin(xyz=(0.0, 0.094, 0.017)),
        material=body_dark,
        name="rear_deck",
    )
    body.visual(
        Box((0.046, 0.058, 0.024)),
        origin=Origin(xyz=(0.0, 0.030, 0.020)),
        material=steel,
        name="punch_pillar",
    )
    body.visual(
        _rounded_slab(0.096, 0.098, 0.026, 0.008, "punch_housing"),
        origin=Origin(xyz=(0.0, 0.002, 0.034)),
        material=body_dark,
        name="punch_housing",
    )
    body.visual(
        Box((0.036, 0.034, 0.017)),
        origin=Origin(xyz=(0.0, -0.057, 0.0165)),
        material=steel,
        name="die_block",
    )
    body.visual(
        Box((0.090, 0.068, 0.006)),
        origin=Origin(xyz=(0.060, 0.000, 0.024)),
        material=steel,
        name="drawer_guide",
    )
    body.visual(
        Box((0.026, 0.036, 0.018)),
        origin=Origin(xyz=(0.060, 0.056, 0.017)),
        material=steel,
        name="button_guide",
    )
    for index, x_pos in enumerate((-0.045, 0.045)):
        body.visual(
            Box((0.016, 0.018, 0.022)),
            origin=Origin(xyz=(x_pos, 0.120, 0.035)),
            material=body_dark,
            name=f"hinge_cheek_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.146, 0.248, 0.050)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.015, 0.025)),
    )

    handle = model.part("handle")
    handle.visual(
        _rounded_slab(0.104, 0.210, 0.010, 0.008, "handle_pad"),
        origin=Origin(xyz=(0.0, -0.118, 0.008)),
        material=handle_dark,
        name="top_pad",
    )
    handle.visual(
        Box((0.044, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, -0.020, 0.004)),
        material=handle_dark,
        name="center_web",
    )
    for index, x_pos in enumerate((-0.028, 0.028)):
        handle.visual(
            Box((0.014, 0.026, 0.016)),
            origin=Origin(xyz=(x_pos, -0.012, 0.002)),
            material=handle_dark,
            name=f"arm_{index}",
        )
        handle.visual(
            Cylinder(radius=0.0055, length=0.018),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"hinge_barrel_{index}",
        )
    handle.inertial = Inertial.from_geometry(
        Box((0.104, 0.210, 0.028)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.102, 0.008)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.110, 0.045)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=0.70,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.060, 0.078, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=steel,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.056, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, -0.038, 0.007)),
        material=steel,
        name="front_wall",
    )
    drawer.visual(
        Box((0.060, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, 0.038, 0.007)),
        material=steel,
        name="rear_wall",
    )
    drawer.visual(
        Box((0.002, 0.074, 0.010)),
        origin=Origin(xyz=(-0.029, 0.0, 0.007)),
        material=steel,
        name="left_wall",
    )
    drawer.visual(
        Box((0.002, 0.074, 0.010)),
        origin=Origin(xyz=(0.029, 0.0, 0.007)),
        material=steel,
        name="right_wall",
    )
    drawer.visual(
        Box((0.006, 0.074, 0.006)),
        origin=Origin(xyz=(0.033, 0.0, 0.008)),
        material=steel,
        name="guide_lip",
    )
    drawer.visual(
        _rounded_slab(0.010, 0.032, 0.010, 0.003, "drawer_pull"),
        origin=Origin(xyz=(0.038, 0.0, 0.006)),
        material=accent,
        name="pull_tab",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.070, 0.078, 0.014)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.014, 0.000, 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.15,
            lower=0.0,
            upper=0.055,
        ),
    )

    button = model.part("button")
    button.visual(
        _rounded_slab(0.012, 0.020, 0.008, 0.0025, "release_button"),
        origin=Origin(xyz=(-0.006, 0.0, 0.004)),
        material=accent,
        name="button_cap",
    )
    button.inertial = Inertial.from_geometry(
        Box((0.012, 0.020, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(-0.006, 0.0, 0.004)),
    )

    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.085, 0.056, 0.013)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=0.004,
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
    drawer = object_model.get_part("drawer")
    button = object_model.get_part("button")
    handle_joint = object_model.get_articulation("body_to_handle")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    button_joint = object_model.get_articulation("body_to_button")

    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem="top_pad",
            negative_elem="punch_housing",
            min_gap=0.001,
            max_gap=0.006,
            name="handle rests just above the punch housing",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="xy",
            elem_a="top_pad",
            elem_b="punch_housing",
            min_overlap=0.070,
            name="handle covers the punch housing footprint",
        )
        ctx.expect_gap(
            body,
            drawer,
            axis="z",
            positive_elem="punch_housing",
            min_gap=0.0005,
            max_gap=0.0035,
            name="chip drawer sits directly below the punch housing",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="xy",
            elem_b="punch_housing",
            min_overlap=0.055,
            name="drawer nests under the punch head at rest",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="yz",
            elem_a="button_cap",
            elem_b="button_guide",
            min_overlap=0.008,
            name="side button stays aligned in its guide pocket",
        )
        ctx.expect_gap(
            body,
            drawer,
            axis="z",
            positive_elem="drawer_guide",
            negative_elem="guide_lip",
            min_gap=0.001,
            max_gap=0.004,
            name="drawer lip rides just under the lateral guide",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="guide_lip",
            elem_b="drawer_guide",
            min_overlap=0.055,
            name="drawer lip stays captured along the guide depth",
        )

    rest_pad = ctx.part_element_world_aabb(handle, elem="top_pad")
    rest_drawer = ctx.part_world_position(drawer)
    rest_button = ctx.part_world_position(button)
    with ctx.pose({handle_joint: 0.70}):
        open_pad = ctx.part_element_world_aabb(handle, elem="top_pad")
    with ctx.pose({drawer_joint: 0.055}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="guide_lip",
            elem_b="drawer_guide",
            min_overlap=0.055,
            name="drawer stays guided when extended",
        )
        ctx.expect_gap(
            body,
            drawer,
            axis="z",
            positive_elem="drawer_guide",
            negative_elem="guide_lip",
            min_gap=0.001,
            max_gap=0.004,
            name="drawer lip remains just under the guide when extended",
        )
        extended_drawer = ctx.part_world_position(drawer)
    with ctx.pose({button_joint: 0.004}):
        pressed_button = ctx.part_world_position(button)
    ctx.check(
        "handle opens upward from the rear hinge",
        rest_pad is not None
        and open_pad is not None
        and open_pad[1][2] > rest_pad[1][2] + 0.050
        and open_pad[0][1] > rest_pad[0][1] + 0.040,
        details=f"rest={rest_pad}, open={open_pad}",
    )
    ctx.check(
        "drawer slides laterally outward",
        rest_drawer is not None
        and extended_drawer is not None
        and extended_drawer[0] > rest_drawer[0] + 0.045,
        details=f"rest={rest_drawer}, extended={extended_drawer}",
    )
    ctx.check(
        "button presses inward laterally",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[0] < rest_button[0] - 0.003,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
