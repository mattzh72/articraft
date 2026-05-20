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
    ExtrudeWithHolesGeometry,
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
    model = ArticulatedObject(name="desktop_two_hole_punch")

    enamel = model.material("enamel", rgba=(0.24, 0.26, 0.30, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.13, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    guide_gray = model.material("guide_gray", rgba=(0.68, 0.69, 0.72, 1.0))

    base = model.part("base")

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.150, 0.118, 0.010), 0.005),
        "base_plate",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=enamel,
        name="base_plate",
    )

    base.visual(
        Box((0.074, 0.086, 0.022)),
        origin=Origin(xyz=(-0.010, 0.0, 0.016)),
        material=enamel,
        name="housing_block",
    )

    def yz_section(
        width_y: float,
        height_z: float,
        radius: float,
        x_pos: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y, z + z_center)
            for z, y in rounded_rect_profile(height_z, width_y, radius)
        ]

    housing_cap_mesh = mesh_from_geometry(
        section_loft(
            [
                yz_section(0.074, 0.018, 0.008, -0.041, 0.010),
                yz_section(0.088, 0.024, 0.010, -0.008, 0.012),
                yz_section(0.076, 0.019, 0.008, 0.024, 0.0105),
            ]
        ),
        "housing_cap",
    )
    base.visual(
        housing_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=enamel,
        name="housing_cap",
    )

    base.visual(
        Cylinder(radius=0.0065, length=0.003),
        origin=Origin(xyz=(0.011, -0.0165, 0.0065)),
        material=steel,
        name="die_left",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.003),
        origin=Origin(xyz=(0.011, 0.0165, 0.0065)),
        material=steel,
        name="die_right",
    )

    fence_outer = [
        (-0.008, -0.053),
        (0.008, -0.053),
        (0.008, 0.053),
        (-0.008, 0.053),
    ]
    fence_slot = rounded_rect_profile(0.0042, 0.082, 0.0016)
    front_fence_geom = ExtrudeWithHolesGeometry(
        fence_outer,
        [fence_slot],
        0.004,
    ).rotate_y(math.pi / 2.0)
    front_fence_mesh = mesh_from_geometry(front_fence_geom, "front_fence")
    base.visual(
        front_fence_mesh,
        origin=Origin(xyz=(0.073, 0.0, 0.013)),
        material=charcoal,
        name="front_fence",
    )

    base.visual(
        Box((0.016, 0.012, 0.037)),
        origin=Origin(xyz=(-0.060, -0.040, 0.0235)),
        material=charcoal,
        name="hinge_saddle_0",
    )
    base.visual(
        Box((0.016, 0.012, 0.037)),
        origin=Origin(xyz=(-0.060, 0.040, 0.0235)),
        material=charcoal,
        name="hinge_saddle_1",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.150, 0.118, 0.052)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    handle = model.part("handle")
    bridge_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.126, 0.112, 0.014), 0.012),
        "handle_bridge",
    )
    handle.visual(
        bridge_mesh,
        origin=Origin(xyz=(0.063, 0.0, 0.012)),
        material=enamel,
        name="bridge",
    )
    handle.visual(
        Box((0.024, 0.090, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, 0.003)),
        material=enamel,
        name="rear_rib",
    )
    handle.visual(
        Box((0.020, 0.050, 0.008)),
        origin=Origin(xyz=(0.112, 0.0, 0.002)),
        material=enamel,
        name="grip_lip",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.102),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.126, 0.112, 0.024)),
        mass=0.42,
        origin=Origin(xyz=(0.063, 0.0, 0.012)),
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.004, 0.018, 0.0032)),
        material=guide_gray,
        name="slider_bar",
    )
    guide.visual(
        Box((0.024, 0.006, 0.012)),
        origin=Origin(xyz=(-0.014, 0.0, -0.001)),
        material=guide_gray,
        name="paper_stop",
    )
    guide.visual(
        Box((0.012, 0.020, 0.008)),
        origin=Origin(xyz=(0.008, 0.0, 0.001)),
        material=guide_gray,
        name="thumb_tab",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.036, 0.020, 0.012)),
        mass=0.04,
        origin=Origin(xyz=(-0.004, 0.0, -0.001)),
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.060, 0.0, 0.048)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )

    model.articulation(
        "base_to_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(0.073, 0.0, 0.013)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=-0.034,
            upper=0.034,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    guide = object_model.get_part("guide")
    handle_hinge = object_model.get_articulation("base_to_handle")
    guide_slide = object_model.get_articulation("base_to_guide")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        positive_elem="bridge",
        negative_elem="housing_cap",
        min_gap=0.001,
        max_gap=0.008,
        name="closed handle sits just above housing",
    )
    ctx.expect_overlap(
        handle,
        base,
        axes="xy",
        elem_a="bridge",
        elem_b="housing_cap",
        min_overlap=0.05,
        name="handle spans over punch head housing",
    )

    closed_bridge_aabb = ctx.part_element_world_aabb(handle, elem="bridge")
    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        opened_bridge_aabb = ctx.part_element_world_aabb(handle, elem="bridge")
        ctx.expect_gap(
            handle,
            base,
            axis="z",
            positive_elem="bridge",
            negative_elem="housing_cap",
            max_penetration=0.0,
            name="opened handle stays out of the housing",
        )
    ctx.check(
        "handle opens upward",
        closed_bridge_aabb is not None
        and opened_bridge_aabb is not None
        and opened_bridge_aabb[1][2] > closed_bridge_aabb[1][2] + 0.035,
        details=f"closed={closed_bridge_aabb}, opened={opened_bridge_aabb}",
    )

    guide_rest = ctx.part_world_position(guide)
    with ctx.pose({guide_slide: guide_slide.motion_limits.upper}):
        guide_upper = ctx.part_world_position(guide)
    with ctx.pose({guide_slide: guide_slide.motion_limits.lower}):
        guide_lower = ctx.part_world_position(guide)
    ctx.check(
        "paper guide slides transversely in front slot",
        guide_rest is not None
        and guide_upper is not None
        and guide_lower is not None
        and abs(guide_upper[1] - guide_lower[1]) > 0.06
        and abs(guide_upper[0] - guide_rest[0]) < 1e-4
        and abs(guide_upper[2] - guide_rest[2]) < 1e-4
        and abs(guide_lower[0] - guide_rest[0]) < 1e-4
        and abs(guide_lower[2] - guide_rest[2]) < 1e-4,
        details=f"rest={guide_rest}, upper={guide_upper}, lower={guide_lower}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
