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


def _rounded_block(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, min(radius, width * 0.25, depth * 0.25)),
            height,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_hole_punch")

    base_black = model.material("base_black", rgba=(0.15, 0.15, 0.16, 1.0))
    handle_grey = model.material("handle_grey", rgba=(0.23, 0.24, 0.26, 1.0))
    guide_grey = model.material("guide_grey", rgba=(0.55, 0.57, 0.60, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    accent = model.material("accent", rgba=(0.92, 0.56, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        _rounded_block(0.132, 0.112, 0.008, 0.010, "base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=base_black,
        name="base_plate",
    )
    base.visual(
        _rounded_block(0.118, 0.074, 0.022, 0.008, "base_body"),
        origin=Origin(xyz=(0.0, -0.004, 0.019)),
        material=base_black,
        name="base_body",
    )
    base.visual(
        _rounded_block(0.088, 0.042, 0.014, 0.008, "top_deck"),
        origin=Origin(xyz=(0.0, -0.004, 0.030)),
        material=base_black,
        name="top_deck",
    )
    base.visual(
        Box((0.110, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.026, 0.024)),
        material=steel,
        name="paper_lip",
    )
    base.visual(
        _rounded_block(0.110, 0.016, 0.012, 0.003, "guide_housing"),
        origin=Origin(xyz=(0.0, 0.049, 0.014)),
        material=guide_grey,
        name="guide_housing",
    )
    base.visual(
        Box((0.074, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -0.040, 0.016)),
        material=base_black,
        name="rear_rib",
    )
    for index, x_pos in enumerate((-0.036, 0.036)):
        base.visual(
            Box((0.014, 0.018, 0.032)),
            origin=Origin(xyz=(x_pos, -0.040, 0.024)),
            material=base_black,
            name=f"hinge_cheek_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.132, 0.112, 0.050)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.056),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    handle.visual(
        Box((0.040, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.012, 0.004)),
        material=handle_grey,
        name="rear_gusset",
    )
    handle.visual(
        _rounded_block(0.116, 0.088, 0.010, 0.010, "lever_shell"),
        origin=Origin(xyz=(0.0, 0.044, 0.009)),
        material=handle_grey,
        name="lever_shell",
    )
    handle.visual(
        _rounded_block(0.084, 0.018, 0.008, 0.004, "press_pad"),
        origin=Origin(xyz=(0.0, 0.080, 0.002)),
        material=handle_grey,
        name="press_pad",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.120, 0.092, 0.020)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.045, 0.008)),
    )

    guide = model.part("guide")
    guide.visual(
        _rounded_block(0.020, 0.024, 0.006, 0.003, "slider_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=guide_grey,
        name="slider_cap",
    )
    guide.visual(
        Box((0.020, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -0.014, -0.006)),
        material=guide_grey,
        name="rear_cheek",
    )
    guide.visual(
        Box((0.020, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.014, -0.006)),
        material=guide_grey,
        name="front_cheek",
    )
    guide.visual(
        Box((0.004, 0.020, 0.020)),
        origin=Origin(xyz=(0.006, 0.006, 0.016)),
        material=guide_grey,
        name="stop_fence",
    )
    guide.visual(
        Box((0.010, 0.024, 0.012)),
        origin=Origin(xyz=(-0.009, 0.012, 0.009)),
        material=guide_grey,
        name="disc_arm",
    )
    guide.visual(
        Cylinder(radius=0.0035, length=0.006),
        origin=Origin(xyz=(-0.010, 0.027, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="disc_boss",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.032, 0.032, 0.030)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    disc = model.part("disc")
    disc.visual(
        Cylinder(radius=0.012, length=0.005),
        origin=Origin(xyz=(-0.0025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="indicator_disc",
    )
    disc.visual(
        Cylinder(radius=0.002, length=0.002),
        origin=Origin(xyz=(0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_stub",
    )
    disc.visual(
        Box((0.002, 0.005, 0.008)),
        origin=Origin(xyz=(-0.003, 0.0, 0.008)),
        material=steel,
        name="index_tab",
    )
    disc.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.007),
        mass=0.02,
        origin=Origin(),
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(0.0, -0.040, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "base_to_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(-0.036, 0.049, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.15,
            lower=0.0,
            upper=0.072,
        ),
    )
    model.articulation(
        "guide_to_disc",
        ArticulationType.REVOLUTE,
        parent=guide,
        child=disc,
        origin=Origin(xyz=(-0.015, 0.027, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=3.0,
            lower=-1.2,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    guide = object_model.get_part("guide")
    disc = object_model.get_part("disc")
    handle_hinge = object_model.get_articulation("base_to_handle")
    guide_slide = object_model.get_articulation("base_to_guide")
    disc_spin = object_model.get_articulation("guide_to_disc")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="press_pad",
        negative_elem="top_deck",
        max_gap=0.004,
        max_penetration=0.0,
        name="closed handle seats just above the body",
    )
    ctx.expect_overlap(
        handle,
        base,
        axes="xy",
        elem_a="lever_shell",
        elem_b="top_deck",
        min_overlap=0.035,
        name="closed handle covers the punch body",
    )
    ctx.expect_gap(
        guide,
        base,
        axis="z",
        positive_elem="slider_cap",
        negative_elem="guide_housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="guide rides on the front housing",
    )
    ctx.expect_gap(
        disc,
        guide,
        axis="y",
        positive_elem="indicator_disc",
        negative_elem="slider_cap",
        min_gap=0.003,
        name="indicator disc sits proud of the guide housing",
    )

    rest_guide_pos = ctx.part_world_position(guide)
    rest_tab_aabb = ctx.part_element_world_aabb(disc, elem="index_tab")

    with ctx.pose({handle_hinge: 1.0}):
        ctx.expect_gap(
            handle,
            base,
            axis="z",
            positive_elem="press_pad",
            negative_elem="top_deck",
            min_gap=0.030,
            name="opened handle lifts clear of the body",
        )

    with ctx.pose({guide_slide: 0.072, disc_spin: math.pi / 2.0}):
        ctx.expect_gap(
            guide,
            base,
            axis="z",
            positive_elem="slider_cap",
            negative_elem="guide_housing",
            max_gap=0.001,
            max_penetration=0.0,
            name="guide stays seated at full travel",
        )
        shifted_guide_pos = ctx.part_world_position(guide)
        rotated_tab_aabb = ctx.part_element_world_aabb(disc, elem="index_tab")

    ctx.check(
        "guide slides across the front edge",
        rest_guide_pos is not None
        and shifted_guide_pos is not None
        and shifted_guide_pos[0] > rest_guide_pos[0] + 0.06,
        details=f"rest={rest_guide_pos}, shifted={shifted_guide_pos}",
    )
    ctx.check(
        "indicator disc visibly rotates on its axle",
        rest_tab_aabb is not None
        and rotated_tab_aabb is not None
        and rotated_tab_aabb[1][2] < rest_tab_aabb[1][2] - 0.004,
        details=f"rest={rest_tab_aabb}, rotated={rotated_tab_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
