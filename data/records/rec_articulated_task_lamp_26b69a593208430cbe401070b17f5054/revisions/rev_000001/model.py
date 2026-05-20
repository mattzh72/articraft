from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="portable_uv_inspection_wand")

    housing_dark = model.material("housing_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    fin_silver = model.material("fin_silver", rgba=(0.62, 0.65, 0.69, 1.0))
    lens_uv = model.material("lens_uv", rgba=(0.47, 0.31, 0.88, 0.58))
    accent_blue = model.material("accent_blue", rgba=(0.17, 0.40, 0.63, 1.0))

    handle_body = model.part("handle_body")
    handle_body.inertial = Inertial.from_geometry(
        Box((0.10, 0.22, 0.28)),
        mass=0.70,
        origin=Origin(xyz=(0.0, -0.01, 0.12)),
    )
    handle_body.visual(
        Box((0.006, 0.068, 0.156)),
        origin=Origin(xyz=(0.020, -0.028, 0.090), rpy=(0.47, 0.0, 0.0)),
        material=grip_rubber,
        name="grip_left_shell",
    )
    handle_body.visual(
        Box((0.006, 0.068, 0.156)),
        origin=Origin(xyz=(-0.020, -0.028, 0.090), rpy=(0.47, 0.0, 0.0)),
        material=grip_rubber,
        name="grip_right_shell",
    )
    handle_body.visual(
        Box((0.030, 0.028, 0.102)),
        origin=Origin(xyz=(0.0, -0.008, 0.112), rpy=(0.47, 0.0, 0.0)),
        material=grip_rubber,
        name="grip_top_bridge",
    )
    handle_body.visual(
        Cylinder(radius=0.024, length=0.044),
        origin=Origin(xyz=(0.0, -0.074, 0.029), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="grip_butt",
    )
    handle_body.visual(
        Cylinder(radius=0.020, length=0.046),
        origin=Origin(xyz=(0.0, 0.003, 0.093), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="front_finger_swell",
    )
    handle_body.visual(
        Box((0.056, 0.102, 0.060)),
        origin=Origin(xyz=(0.0, 0.010, 0.164)),
        material=housing_dark,
        name="upper_body",
    )
    handle_body.visual(
        Box((0.036, 0.028, 0.046)),
        origin=Origin(xyz=(0.0, 0.047, 0.188)),
        material=housing_dark,
        name="head_bridge",
    )
    handle_body.visual(
        Cylinder(radius=0.015, length=0.036),
        origin=Origin(xyz=(0.0, 0.046, 0.188), rpy=(0.0, pi / 2.0, 0.0)),
        material=accent_blue,
        name="knuckle_pod",
    )
    handle_body.visual(
        Box((0.012, 0.030, 0.060)),
        origin=Origin(xyz=(0.024, 0.076, 0.188)),
        material=housing_dark,
        name="head_left_ear",
    )
    handle_body.visual(
        Box((0.012, 0.030, 0.060)),
        origin=Origin(xyz=(-0.024, 0.076, 0.188)),
        material=housing_dark,
        name="head_right_ear",
    )
    handle_body.visual(
        Box((0.010, 0.012, 0.012)),
        origin=Origin(xyz=(0.013, -0.006, 0.030)),
        material=housing_dark,
        name="cover_left_lug",
    )
    handle_body.visual(
        Box((0.010, 0.012, 0.012)),
        origin=Origin(xyz=(-0.013, -0.006, 0.030)),
        material=housing_dark,
        name="cover_right_lug",
    )
    handle_body.visual(
        Box((0.004, 0.080, 0.012)),
        origin=Origin(xyz=(0.021, -0.042, 0.026), rpy=(0.47, 0.0, 0.0)),
        material=housing_dark,
        name="compartment_left_guide",
    )
    handle_body.visual(
        Box((0.004, 0.080, 0.012)),
        origin=Origin(xyz=(-0.021, -0.042, 0.026), rpy=(0.47, 0.0, 0.0)),
        material=housing_dark,
        name="compartment_right_guide",
    )
    handle_body.visual(
        Box((0.018, 0.034, 0.030)),
        origin=Origin(xyz=(0.0, 0.025, 0.132), rpy=(0.15, 0.0, 0.0)),
        material=accent_blue,
        name="trigger",
    )

    uv_head = model.part("uv_head")
    uv_head.inertial = Inertial.from_geometry(
        Box((0.16, 0.12, 0.06)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.07, 0.02)),
    )
    uv_head.visual(
        Cylinder(radius=0.011, length=0.036),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=accent_blue,
        name="head_pivot_barrel",
    )
    uv_head.visual(
        Box((0.030, 0.046, 0.032)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=housing_dark,
        name="head_neck",
    )
    uv_head.visual(
        Box((0.148, 0.074, 0.050)),
        origin=Origin(xyz=(0.0, 0.074, 0.020)),
        material=housing_dark,
        name="head_shell",
    )
    uv_head.visual(
        Box((0.136, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, 0.109, 0.020)),
        material=lens_uv,
        name="uv_window",
    )
    uv_head.visual(
        Box((0.126, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.086, -0.012)),
        material=accent_blue,
        name="emitter_bezel",
    )
    for fin_index, fin_y in enumerate((0.054, 0.068, 0.082, 0.096), start=1):
        uv_head.visual(
            Box((0.122, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, fin_y, 0.050)),
            material=fin_silver,
            name=f"heat_fin_{fin_index}",
        )

    battery_cover = model.part("battery_cover")
    battery_cover.inertial = Inertial.from_geometry(
        Box((0.05, 0.10, 0.02)),
        mass=0.06,
        origin=Origin(xyz=(0.0, -0.05, -0.02)),
    )
    battery_cover.visual(
        Cylinder(radius=0.0055, length=0.016),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=accent_blue,
        name="cover_hinge_barrel",
    )
    battery_cover.visual(
        Box((0.010, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.014, -0.006)),
        material=housing_dark,
        name="cover_spine",
    )
    battery_cover.visual(
        Box((0.024, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.017, -0.013), rpy=(0.47, 0.0, 0.0)),
        material=housing_dark,
        name="cover_front_web",
    )
    battery_cover.visual(
        Box((0.032, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, -0.040, -0.030), rpy=(0.47, 0.0, 0.0)),
        material=housing_dark,
        name="cover_panel",
    )
    battery_cover.visual(
        Box((0.018, 0.040, 0.008)),
        origin=Origin(xyz=(0.0, -0.043, -0.030), rpy=(0.47, 0.0, 0.0)),
        material=grip_rubber,
        name="cover_rib",
    )
    battery_cover.visual(
        Box((0.018, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.066, -0.039), rpy=(0.47, 0.0, 0.0)),
        material=accent_blue,
        name="cover_latch",
    )

    model.articulation(
        "head_pivot",
        ArticulationType.REVOLUTE,
        parent=handle_body,
        child=uv_head,
        origin=Origin(xyz=(0.0, 0.072, 0.188)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.60, upper=0.95),
    )
    model.articulation(
        "battery_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=handle_body,
        child=battery_cover,
        origin=Origin(xyz=(0.0, -0.006, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle_body = object_model.get_part("handle_body")
    uv_head = object_model.get_part("uv_head")
    battery_cover = object_model.get_part("battery_cover")
    head_pivot = object_model.get_articulation("head_pivot")
    cover_hinge = object_model.get_articulation("battery_cover_hinge")

    ctx.check("handle body exists", handle_body is not None)
    ctx.check("uv head exists", uv_head is not None)
    ctx.check("battery cover exists", battery_cover is not None)
    ctx.check(
        "head pivot axis is horizontal",
        tuple(round(v, 6) for v in head_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"axis={head_pivot.axis}",
    )
    ctx.check(
        "battery cover hinge axis is horizontal",
        tuple(round(v, 6) for v in cover_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={cover_hinge.axis}",
    )

    with ctx.pose({head_pivot: 0.0, cover_hinge: 0.0}):
        ctx.expect_contact(
            uv_head,
            handle_body,
            elem_a="head_pivot_barrel",
            elem_b="head_left_ear",
            contact_tol=0.0002,
            name="head pivot barrel seats on left knuckle ear",
        )
        ctx.expect_contact(
            uv_head,
            handle_body,
            elem_a="head_pivot_barrel",
            elem_b="head_right_ear",
            contact_tol=0.0002,
            name="head pivot barrel seats on right knuckle ear",
        )
        ctx.expect_contact(
            battery_cover,
            handle_body,
            elem_a="cover_hinge_barrel",
            elem_b="cover_left_lug",
            contact_tol=0.0002,
            name="battery cover hinge barrel seats on left lug",
        )
        ctx.expect_contact(
            battery_cover,
            handle_body,
            elem_a="cover_hinge_barrel",
            elem_b="cover_right_lug",
            contact_tol=0.0002,
            name="battery cover hinge barrel seats on right lug",
        )

        head_rest_aabb = ctx.part_element_world_aabb(uv_head, elem="head_shell")
        cover_rest_aabb = ctx.part_element_world_aabb(battery_cover, elem="cover_panel")

    with ctx.pose({head_pivot: 0.85}):
        head_open_aabb = ctx.part_element_world_aabb(uv_head, elem="head_shell")

    with ctx.pose({cover_hinge: 1.45}):
        cover_open_aabb = ctx.part_element_world_aabb(battery_cover, elem="cover_panel")

    if head_rest_aabb is not None and head_open_aabb is not None:
        head_rest_center_z = 0.5 * (head_rest_aabb[0][2] + head_rest_aabb[1][2])
        head_open_center_z = 0.5 * (head_open_aabb[0][2] + head_open_aabb[1][2])
        ctx.check(
            "uv head tilts upward when opened",
            head_open_center_z > head_rest_center_z + 0.015,
            details=f"rest_z={head_rest_center_z}, open_z={head_open_center_z}",
        )
    else:
        ctx.fail("uv head tilt measurement available", "could not resolve head shell AABBs")

    if cover_rest_aabb is not None and cover_open_aabb is not None:
        cover_rest_center_z = 0.5 * (cover_rest_aabb[0][2] + cover_rest_aabb[1][2])
        cover_open_center_z = 0.5 * (cover_open_aabb[0][2] + cover_open_aabb[1][2])
        ctx.check(
            "battery cover swings downward when opened",
            cover_open_center_z < cover_rest_center_z - 0.010,
            details=f"rest_z={cover_rest_center_z}, open_z={cover_open_center_z}",
        )
    else:
        ctx.fail("battery cover motion measurement available", "could not resolve cover panel AABBs")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
