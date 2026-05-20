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


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + z_center)
        for z_pos, y_pos in rounded_rect_profile(height, width, radius)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_hole_punch")

    body_paint = model.material("body_paint", rgba=(0.22, 0.25, 0.28, 1.0))
    handle_paint = model.material("handle_paint", rgba=(0.14, 0.16, 0.18, 1.0))
    guide_plastic = model.material("guide_plastic", rgba=(0.71, 0.74, 0.77, 1.0))
    guide_mark = model.material("guide_mark", rgba=(0.90, 0.92, 0.94, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.58, 0.61, 1.0))

    base = model.part("base")
    base_shell = section_loft(
        [
            _yz_section(-0.058, 0.060, 0.018, 0.007, 0.009),
            _yz_section(-0.026, 0.064, 0.024, 0.009, 0.012),
            _yz_section(-0.004, 0.062, 0.020, 0.008, 0.010),
            _yz_section(0.014, 0.058, 0.016, 0.007, 0.008),
        ]
    )
    base.visual(
        mesh_from_geometry(base_shell, "base_shell"),
        material=body_paint,
        name="base_shell",
    )
    base.visual(
        Box((0.088, 0.058, 0.004)),
        origin=Origin(xyz=(-0.012, 0.0, 0.002)),
        material=body_paint,
        name="base_floor",
    )
    base.visual(
        Box((0.056, 0.062, 0.004)),
        origin=Origin(xyz=(0.032, 0.0, 0.016)),
        material=body_paint,
        name="upper_bridge",
    )
    base.visual(
        Box((0.042, 0.034, 0.006)),
        origin=Origin(xyz=(0.000, 0.0, 0.023)),
        material=steel,
        name="die_deck",
    )
    base.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(
            xyz=(-0.048, 0.0, 0.027),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="hinge_knuckle",
    )
    base.visual(
        Box((0.012, 0.024, 0.008)),
        origin=Origin(xyz=(-0.048, 0.0, 0.021)),
        material=steel,
        name="hinge_saddle",
    )
    base.visual(
        Box((0.010, 0.026, 0.004)),
        origin=Origin(xyz=(0.015, 0.0, 0.026)),
        material=steel,
        name="die_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.132, 0.068, 0.034)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    handle = model.part("handle")
    handle_shell = section_loft(
        [
            _yz_section(0.004, 0.032, 0.010, 0.004, 0.007),
            _yz_section(0.028, 0.054, 0.016, 0.006, 0.011),
            _yz_section(0.062, 0.050, 0.014, 0.006, 0.010),
            _yz_section(0.090, 0.040, 0.010, 0.004, 0.008),
        ]
    )
    handle.visual(
        mesh_from_geometry(handle_shell, "handle_shell"),
        material=handle_paint,
        name="handle_shell",
    )
    for index, y_pos in enumerate((-0.020, 0.020)):
        handle.visual(
            Cylinder(radius=0.005, length=0.016),
            origin=Origin(
                xyz=(0.0, y_pos, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"barrel_{index}",
        )
        handle.visual(
            Box((0.010, 0.010, 0.008)),
            origin=Origin(xyz=(0.006, y_pos, 0.006)),
            material=steel,
            name=f"hinge_arm_{index}",
        )
    handle.visual(
        Box((0.018, 0.018, 0.004)),
        origin=Origin(xyz=(0.052, 0.0, 0.004)),
        material=steel,
        name="punch_pad",
    )
    handle.visual(
        Box((0.024, 0.040, 0.004)),
        origin=Origin(xyz=(0.081, 0.0, 0.013)),
        material=guide_mark,
        name="front_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.100, 0.056, 0.020)),
        mass=0.18,
        origin=Origin(xyz=(0.048, 0.0, 0.010)),
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.014, 0.024, 0.005)),
        origin=Origin(xyz=(-0.003, 0.0, 0.0)),
        material=guide_plastic,
        name="carriage",
    )
    guide.visual(
        Box((0.006, 0.010, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, -0.005)),
        material=guide_plastic,
        name="guide_fin",
    )
    guide.visual(
        Box((0.006, 0.110, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, -0.006)),
        material=guide_plastic,
        name="guide_bar",
    )
    guide.visual(
        Box((0.010, 0.014, 0.010)),
        origin=Origin(xyz=(0.010, 0.058, -0.004)),
        material=guide_mark,
        name="thumb_tab",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.026, 0.118, 0.016)),
        mass=0.05,
        origin=Origin(xyz=(0.006, 0.0, -0.004)),
    )

    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.048, 0.0, 0.027)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=0.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(0.044, 0.0, 0.0115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.15,
            lower=-0.020,
            upper=0.020,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    guide = object_model.get_part("guide")
    handle_hinge = object_model.get_articulation("handle_hinge")
    guide_slide = object_model.get_articulation("guide_slide")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="punch_pad",
        negative_elem="die_deck",
        min_gap=0.001,
        max_gap=0.004,
        name="closed punch pad hovers just above the die deck",
    )
    ctx.expect_gap(
        base,
        guide,
        axis="z",
        positive_elem="upper_bridge",
        negative_elem="guide_bar",
        min_gap=0.003,
        max_gap=0.008,
        name="guide bar sits below the front bridge",
    )
    ctx.expect_overlap(
        guide,
        base,
        axes="y",
        elem_a="carriage",
        elem_b="upper_bridge",
        min_overlap=0.020,
        name="guide carriage stays captured at rest",
    )

    closed_grip_center = _aabb_center(
        ctx.part_element_world_aabb(handle, elem="front_grip")
    )
    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        ctx.expect_gap(
            handle,
            base,
            axis="z",
            positive_elem="punch_pad",
            negative_elem="die_deck",
            min_gap=0.022,
            name="opened handle lifts the punch pad clear of the deck",
        )
        open_grip_center = _aabb_center(
            ctx.part_element_world_aabb(handle, elem="front_grip")
        )

    ctx.check(
        "handle rotates upward from the rear hinge",
        closed_grip_center is not None
        and open_grip_center is not None
        and open_grip_center[2] > closed_grip_center[2] + 0.025,
        details=f"closed={closed_grip_center}, open={open_grip_center}",
    )

    guide_rest = ctx.part_world_position(guide)
    with ctx.pose({guide_slide: guide_slide.motion_limits.upper}):
        ctx.expect_overlap(
            guide,
            base,
            axes="y",
            elem_a="carriage",
            elem_b="upper_bridge",
            min_overlap=0.010,
            name="guide carriage keeps insertion at full travel",
        )
        guide_open = ctx.part_world_position(guide)

    ctx.check(
        "guide slides laterally across the front slot",
        guide_rest is not None
        and guide_open is not None
        and guide_open[1] > guide_rest[1] + 0.015,
        details=f"rest={guide_rest}, extended={guide_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
