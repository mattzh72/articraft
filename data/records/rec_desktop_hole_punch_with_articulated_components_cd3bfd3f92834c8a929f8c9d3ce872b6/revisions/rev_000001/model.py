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


def yz_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="personal_hole_punch")

    body_color = model.material("body_color", rgba=(0.15, 0.16, 0.18, 1.0))
    handle_color = model.material("handle_color", rgba=(0.22, 0.23, 0.26, 1.0))
    guide_color = model.material("guide_color", rgba=(0.70, 0.72, 0.76, 1.0))
    clear_cover = model.material("clear_cover", rgba=(0.78, 0.90, 0.98, 0.35))

    body = model.part("body")

    body_shell = section_loft(
        [
            yz_section(-0.056, 0.030, 0.014, 0.006, 0.010),
            yz_section(-0.020, 0.032, 0.016, 0.007, 0.011),
            yz_section(0.020, 0.032, 0.016, 0.007, 0.011),
            yz_section(0.056, 0.026, 0.013, 0.005, 0.0095),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        material=body_color,
        name="base_shell",
    )
    body.visual(
        Box((0.036, 0.030, 0.012)),
        origin=Origin(xyz=(-0.004, 0.0, 0.025)),
        material=body_color,
        name="chamber_shell",
    )
    body.visual(
        Box((0.022, 0.030, 0.004)),
        origin=Origin(xyz=(0.038, 0.0, 0.018)),
        material=body_color,
        name="lip_track",
    )
    body.visual(
        Box((0.012, 0.020, 0.006)),
        origin=Origin(xyz=(-0.046, 0.0, 0.022)),
        material=body_color,
        name="hinge_bridge",
    )
    body.visual(
        Box((0.016, 0.028, 0.007)),
        origin=Origin(xyz=(-0.046, 0.0, 0.018)),
        material=body_color,
        name="rear_pedestal",
    )
    body.visual(
        Box((0.010, 0.004, 0.012)),
        origin=Origin(xyz=(-0.048, -0.013, 0.027)),
        material=body_color,
        name="hinge_lug_0",
    )
    body.visual(
        Box((0.010, 0.004, 0.012)),
        origin=Origin(xyz=(-0.048, 0.013, 0.027)),
        material=body_color,
        name="hinge_lug_1",
    )
    body.visual(
        Box((0.008, 0.024, 0.008)),
        origin=Origin(xyz=(-0.010, 0.0, 0.018)),
        material=body_color,
        name="cover_web",
    )
    body.visual(
        Box((0.010, 0.024, 0.004)),
        origin=Origin(xyz=(-0.010, 0.0, 0.013)),
        material=body_color,
        name="cover_mount",
    )
    body.visual(
        Box((0.004, 0.004, 0.008)),
        origin=Origin(xyz=(-0.015, -0.010, 0.014)),
        material=body_color,
        name="cover_ear_0",
    )
    body.visual(
        Box((0.004, 0.004, 0.008)),
        origin=Origin(xyz=(-0.015, 0.010, 0.014)),
        material=body_color,
        name="cover_ear_1",
    )

    for idx, (x, y) in enumerate(((-0.040, -0.011), (-0.040, 0.011), (0.036, -0.011), (0.036, 0.011))):
        body.visual(
            Box((0.010, 0.008, 0.003)),
            origin=Origin(xyz=(x, y, 0.0015)),
            material=body_color,
            name=f"foot_{idx}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.112, 0.034, 0.031)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_color,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.012, 0.020, 0.006)),
        origin=Origin(xyz=(0.006, 0.0, 0.001)),
        material=handle_color,
        name="rear_bridge",
    )
    lever_shell = section_loft(
        [
            yz_section(0.012, 0.028, 0.010, 0.004, 0.007),
            yz_section(0.046, 0.030, 0.012, 0.005, 0.008),
            yz_section(0.082, 0.028, 0.010, 0.004, 0.007),
            yz_section(0.104, 0.020, 0.007, 0.003, 0.005),
        ]
    )
    handle.visual(
        mesh_from_geometry(lever_shell, "lever_shell"),
        material=handle_color,
        name="lever_shell",
    )
    handle.visual(
        Box((0.014, 0.018, 0.004)),
        origin=Origin(xyz=(0.032, 0.0, 0.0015)),
        material=handle_color,
        name="press_pad",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.106, 0.030, 0.014)),
        mass=0.12,
        origin=Origin(xyz=(0.052, 0.0, 0.007)),
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.014, 0.020, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=guide_color,
        name="guide_cap",
    )
    guide.visual(
        Box((0.004, 0.020, 0.012)),
        origin=Origin(xyz=(0.009, 0.0, -0.0045)),
        material=guide_color,
        name="guide_fence",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.018, 0.020, 0.012)),
        mass=0.03,
        origin=Origin(xyz=(0.004, 0.0, -0.003)),
    )

    cover = model.part("cover")
    cover.visual(
        Cylinder(radius=0.002, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.001), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clear_cover,
        name="cover_barrel",
    )
    cover.visual(
        Box((0.024, 0.022, 0.002)),
        origin=Origin(xyz=(0.012, 0.0, -0.003)),
        material=clear_cover,
        name="cover_top",
    )
    cover.visual(
        Box((0.002, 0.022, 0.007)),
        origin=Origin(xyz=(0.025, 0.0, -0.0045)),
        material=clear_cover,
        name="cover_front",
    )
    cover.visual(
        Box((0.024, 0.002, 0.007)),
        origin=Origin(xyz=(0.012, -0.010, -0.0045)),
        material=clear_cover,
        name="cover_side_0",
    )
    cover.visual(
        Box((0.024, 0.002, 0.007)),
        origin=Origin(xyz=(0.012, 0.010, -0.0045)),
        material=clear_cover,
        name="cover_side_1",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.026, 0.022, 0.008)),
        mass=0.02,
        origin=Origin(xyz=(0.013, 0.0, -0.004)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.046, 0.0, 0.033)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "body_to_guide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=guide,
        origin=Origin(xyz=(0.046, 0.0, 0.0202)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.10,
            lower=-0.008,
            upper=0.008,
        ),
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.015, 0.0, 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    guide = object_model.get_part("guide")
    cover = object_model.get_part("cover")

    handle_hinge = object_model.get_articulation("body_to_handle")
    guide_slide = object_model.get_articulation("body_to_guide")
    cover_hinge = object_model.get_articulation("body_to_cover")

    with ctx.pose({handle_hinge: 0.0, guide_slide: 0.0, cover_hinge: 0.0}):
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem="lever_shell",
            negative_elem="chamber_shell",
            min_gap=0.001,
            max_gap=0.008,
            name="closed handle clears chamber",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="xy",
            elem_a="lever_shell",
            elem_b="chamber_shell",
            min_overlap=0.020,
            name="handle covers punch chamber",
        )
        ctx.expect_gap(
            guide,
            body,
            axis="z",
            positive_elem="guide_cap",
            negative_elem="lip_track",
            max_gap=0.0015,
            max_penetration=0.0,
            name="guide rests on front lip",
        )
        ctx.expect_overlap(
            guide,
            body,
            axes="xy",
            elem_a="guide_cap",
            elem_b="lip_track",
            min_overlap=0.009,
            name="guide overlaps lip track",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="z",
            positive_elem="cover_mount",
            negative_elem="cover_top",
            min_gap=0.0,
            max_gap=0.003,
            name="cover tucks below chamber",
        )

    rest_guide = ctx.part_world_position(guide)
    with ctx.pose({guide_slide: 0.008}):
        moved_guide = ctx.part_world_position(guide)
        ctx.expect_gap(
            guide,
            body,
            axis="z",
            positive_elem="guide_cap",
            negative_elem="lip_track",
            max_gap=0.0015,
            max_penetration=0.0,
            name="guide stays seated while sliding",
        )
    ctx.check(
        "guide slides across the front lip",
        rest_guide is not None
        and moved_guide is not None
        and moved_guide[1] > rest_guide[1] + 0.006
        and abs(moved_guide[0] - rest_guide[0]) < 0.001,
        details=f"rest={rest_guide}, moved={moved_guide}",
    )

    rest_handle = ctx.part_element_world_aabb(handle, elem="lever_shell")
    with ctx.pose({handle_hinge: handle_hinge.motion_limits.upper}):
        open_handle = ctx.part_element_world_aabb(handle, elem="lever_shell")
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem="lever_shell",
            negative_elem="chamber_shell",
            min_gap=0.0125,
            name="opened handle lifts clear of the body",
        )
    ctx.check(
        "handle opens upward",
        rest_handle is not None
        and open_handle is not None
        and open_handle[1][2] > rest_handle[1][2] + 0.020,
        details=f"rest={rest_handle}, open={open_handle}",
    )

    rest_cover = ctx.part_element_world_aabb(cover, elem="cover_top")
    with ctx.pose({cover_hinge: cover_hinge.motion_limits.upper}):
        open_cover = ctx.part_element_world_aabb(cover, elem="cover_top")
    ctx.check(
        "cover swings downward",
        rest_cover is not None
        and open_cover is not None
        and open_cover[0][2] < rest_cover[0][2] - 0.012,
        details=f"rest={rest_cover}, open={open_cover}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
