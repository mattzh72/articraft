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


def _xz_section(
    y: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    radius = min(radius, width * 0.45, height * 0.45)
    return [
        (x, y, z_center + z)
        for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _build_body_shell():
    return section_loft(
        [
            _xz_section(-0.024, width=0.246, height=0.020, radius=0.008, z_center=0.042),
            _xz_section(-0.004, width=0.252, height=0.024, radius=0.009, z_center=0.045),
            _xz_section(0.018, width=0.238, height=0.018, radius=0.008, z_center=0.040),
        ]
    )


def _build_handle_shell():
    return section_loft(
        [
            _xz_section(0.004, width=0.258, height=0.018, radius=0.007, z_center=0.013),
            _xz_section(0.046, width=0.286, height=0.017, radius=0.007, z_center=0.014),
            _xz_section(0.090, width=0.302, height=0.014, radius=0.006, z_center=0.013),
            _xz_section(0.120, width=0.292, height=0.010, radius=0.004, z_center=0.011),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_hole_desktop_punch")

    body_metal = model.material("body_metal", rgba=(0.18, 0.20, 0.22, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.56, 0.60, 0.64, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    body_shell_mesh = mesh_from_geometry(_build_body_shell(), "body_shell")
    handle_shell_mesh = mesh_from_geometry(_build_handle_shell(), "handle_shell")

    base = model.part("base")
    base.visual(
        Box((0.312, 0.122, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.248, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.018, 0.012)),
        material=steel,
        name="die_plate",
    )
    base.visual(
        Box((0.276, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, 0.053, 0.016)),
        material=steel,
        name="fence_rail",
    )
    base.visual(
        body_shell_mesh,
        material=body_metal,
        name="body_shell",
    )
    base.visual(
        Box((0.020, 0.060, 0.034)),
        origin=Origin(xyz=(-0.118, -0.002, 0.025)),
        material=body_metal,
        name="left_cheek",
    )
    base.visual(
        Box((0.020, 0.060, 0.022)),
        origin=Origin(xyz=(0.118, -0.002, 0.041)),
        material=body_metal,
        name="right_cheek",
    )
    base.visual(
        Box((0.250, 0.014, 0.038)),
        origin=Origin(xyz=(0.0, -0.028, 0.027)),
        material=body_metal,
        name="rear_wall",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.246),
        origin=Origin(xyz=(0.0, -0.028, 0.054), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_rod",
    )
    base.visual(
        Box((0.018, 0.016, 0.018)),
        origin=Origin(xyz=(-0.116, -0.028, 0.051)),
        material=body_metal,
        name="left_hinge_block",
    )
    base.visual(
        Box((0.018, 0.016, 0.018)),
        origin=Origin(xyz=(0.116, -0.028, 0.051)),
        material=body_metal,
        name="right_hinge_block",
    )
    base.visual(
        Box((0.066, 0.044, 0.002)),
        origin=Origin(xyz=(0.098, -0.002, 0.013)),
        material=steel,
        name="drawer_guide",
    )
    base.visual(
        Box((0.066, 0.044, 0.002)),
        origin=Origin(xyz=(0.098, -0.002, 0.028)),
        material=steel,
        name="drawer_roof",
    )
    base.visual(
        Box((0.004, 0.044, 0.020)),
        origin=Origin(xyz=(0.065, -0.002, 0.021)),
        material=body_metal,
        name="drawer_wall",
    )
    for hole_x in (-0.108, 0.0, 0.108):
        base.visual(
            Cylinder(radius=0.0095, length=0.010),
            origin=Origin(xyz=(hole_x, 0.008, 0.029)),
            material=steel,
            name=f"punch_collar_{int(round((hole_x + 0.108) / 0.108))}",
        )
        base.visual(
            Cylinder(radius=0.0055, length=0.002),
            origin=Origin(xyz=(hole_x, 0.024, 0.009)),
            material=rubber,
            name=f"die_hole_{int(round((hole_x + 0.108) / 0.108))}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.312, 0.122, 0.060)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    handle = model.part("handle")
    handle.visual(
        handle_shell_mesh,
        material=handle_metal,
        name="handle_shell",
    )
    handle.visual(
        Cylinder(radius=0.005, length=0.262),
        origin=Origin(xyz=(0.0, 0.010, 0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="handle_spine",
    )
    handle.visual(
        Box((0.176, 0.042, 0.005)),
        origin=Origin(xyz=(0.0, 0.086, 0.012)),
        material=rubber,
        name="grip_pad",
    )
    handle.visual(
        Box((0.020, 0.080, 0.006)),
        origin=Origin(xyz=(0.0, 0.048, 0.013)),
        material=rubber,
        name="grip_strip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.304, 0.124, 0.022)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.062, 0.011)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.066, 0.038, 0.002)),
        origin=Origin(xyz=(-0.034, 0.0, 0.001)),
        material=steel,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.064, 0.002, 0.010)),
        origin=Origin(xyz=(-0.034, -0.018, 0.006)),
        material=steel,
        name="drawer_back_wall",
    )
    drawer.visual(
        Box((0.064, 0.002, 0.010)),
        origin=Origin(xyz=(-0.034, 0.018, 0.006)),
        material=steel,
        name="drawer_front_wall",
    )
    drawer.visual(
        Box((0.002, 0.038, 0.010)),
        origin=Origin(xyz=(-0.066, 0.0, 0.006)),
        material=steel,
        name="drawer_inner_wall",
    )
    drawer.visual(
        Box((0.003, 0.038, 0.014)),
        origin=Origin(xyz=(0.0005, 0.0, 0.007)),
        material=body_metal,
        name="drawer_face",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.069, 0.038, 0.014)),
        mass=0.12,
        origin=Origin(xyz=(-0.032, 0.0, 0.007)),
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.026, 0.011, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=steel,
        name="guide_clamp",
    )
    guide.visual(
        Box((0.026, 0.0025, 0.014)),
        origin=Origin(xyz=(0.0, -0.00475, 0.0)),
        material=steel,
        name="guide_rear_cheek",
    )
    guide.visual(
        Box((0.026, 0.0025, 0.018)),
        origin=Origin(xyz=(0.0, 0.00475, 0.0)),
        material=steel,
        name="guide_front_cheek",
    )
    guide.visual(
        Box((0.018, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="guide_lower_clip",
    )
    guide.visual(
        Box((0.003, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, -0.010, 0.020)),
        material=body_metal,
        name="guide_stop",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.028, 0.022, 0.030)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.004, 0.015)),
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(0.0, -0.028, 0.054)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "base_to_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(0.133, -0.002, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.12,
            lower=0.0,
            upper=0.040,
        ),
    )
    model.articulation(
        "base_to_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(-0.112, 0.053, 0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.12,
            lower=0.0,
            upper=0.224,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    drawer = object_model.get_part("drawer")
    guide = object_model.get_part("guide")

    handle_joint = object_model.get_articulation("base_to_handle")
    drawer_joint = object_model.get_articulation("base_to_drawer")
    guide_joint = object_model.get_articulation("base_to_guide")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="handle_shell",
        negative_elem="body_shell",
        max_gap=0.008,
        max_penetration=0.0,
        name="handle rests close over the punch body",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        open_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")
    ctx.check(
        "handle opens upward",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.05,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    ctx.expect_overlap(
        drawer,
        base,
        axes="x",
        elem_a="drawer_tray",
        elem_b="drawer_guide",
        min_overlap=0.060,
        name="drawer starts inserted in the side guide",
    )
    drawer_closed_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
        ctx.expect_overlap(
            drawer,
            base,
            axes="x",
            elem_a="drawer_tray",
            elem_b="drawer_guide",
            min_overlap=0.024,
            name="drawer keeps retained insertion when opened",
        )
        drawer_open_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides outward from the body side",
        drawer_closed_pos is not None
        and drawer_open_pos is not None
        and drawer_open_pos[0] > drawer_closed_pos[0] + 0.035,
        details=f"closed={drawer_closed_pos}, open={drawer_open_pos}",
    )

    ctx.expect_overlap(
        guide,
        base,
        axes="x",
        elem_a="guide_clamp",
        elem_b="fence_rail",
        min_overlap=0.020,
        name="paper guide starts captured on the front fence",
    )
    guide_closed_pos = ctx.part_world_position(guide)
    with ctx.pose({guide_joint: guide_joint.motion_limits.upper}):
        ctx.expect_overlap(
            guide,
            base,
            axes="x",
            elem_a="guide_clamp",
            elem_b="fence_rail",
            min_overlap=0.020,
            name="paper guide stays captured at far travel",
        )
        guide_open_pos = ctx.part_world_position(guide)
    ctx.check(
        "paper guide slides across the front fence",
        guide_closed_pos is not None
        and guide_open_pos is not None
        and guide_open_pos[0] > guide_closed_pos[0] + 0.20,
        details=f"closed={guide_closed_pos}, open={guide_open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
