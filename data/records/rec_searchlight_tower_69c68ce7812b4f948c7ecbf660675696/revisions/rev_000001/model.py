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
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    base_gray = model.material("base_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.45, 0.48, 0.52, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.74, 0.86, 0.94, 0.55))
    safety_yellow = model.material("safety_yellow", rgba=(0.76, 0.71, 0.30, 1.0))

    support_base = model.part("support_base")

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(1.10, 0.82, 0.08, corner_segments=10),
            0.06,
            center=True,
        ),
        "support_base_plate",
    )
    support_base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=base_gray,
        name="base_plate",
    )
    support_base.visual(
        Box((1.18, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.29, 0.05)),
        material=dark_metal,
        name="front_skid",
    )
    support_base.visual(
        Box((1.18, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, -0.29, 0.05)),
        material=dark_metal,
        name="rear_skid",
    )
    support_base.visual(
        Box((0.34, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=steel_gray,
        name="pedestal_plinth",
    )
    support_base.visual(
        Cylinder(radius=0.24, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_metal,
        name="bearing_housing",
    )
    support_base.inertial = Inertial.from_geometry(
        Box((1.18, 0.82, 0.18)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="turntable",
    )
    pan_stage.visual(
        Cylinder(radius=0.14, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=steel_gray,
        name="mast_socket",
    )
    pan_stage.visual(
        Box((0.10, 0.22, 0.12)),
        origin=Origin(xyz=(-0.10, 0.0, 0.13)),
        material=steel_gray,
        name="mast_core",
    )
    pan_stage.visual(
        Box((0.12, 0.60, 0.08)),
        origin=Origin(xyz=(-0.04, 0.0, 0.17)),
        material=steel_gray,
        name="yoke_bridge",
    )
    pan_stage.visual(
        Box((0.08, 0.06, 0.24)),
        origin=Origin(xyz=(0.02, 0.275, 0.33)),
        material=steel_gray,
        name="left_yoke_arm",
    )
    pan_stage.visual(
        Box((0.08, 0.06, 0.24)),
        origin=Origin(xyz=(0.02, -0.275, 0.33)),
        material=steel_gray,
        name="right_yoke_arm",
    )
    pan_stage.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(0.04, 0.275, 0.41), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion_bearing",
    )
    pan_stage.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(0.04, -0.275, 0.41), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion_bearing",
    )
    pan_stage.inertial = Inertial.from_geometry(
        Box((0.32, 0.66, 0.56)),
        mass=45.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    lamp_head = model.part("lamp_head")

    def yz_section(
        x: float,
        width: float,
        height: float,
        radius: float,
    ) -> tuple[tuple[float, float, float], ...]:
        return tuple(
            (x, y, z)
            for y, z in rounded_rect_profile(width, height, radius, corner_segments=10)
        )

    lamp_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                yz_section(-0.14, 0.22, 0.22, 0.04),
                yz_section(-0.02, 0.30, 0.28, 0.05),
                yz_section(0.18, 0.34, 0.34, 0.06),
                yz_section(0.35, 0.38, 0.38, 0.06),
            ]
        ),
        "lamp_shell",
    )
    lamp_head.visual(
        lamp_shell_mesh,
        material=safety_yellow,
        name="lamp_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.152, length=0.14),
        origin=Origin(xyz=(0.29, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_yellow,
        name="front_barrel",
    )
    lamp_head.visual(
        Cylinder(radius=0.165, length=0.03),
        origin=Origin(xyz=(0.369, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.150, length=0.008),
        origin=Origin(xyz=(0.386, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Box((0.08, 0.20, 0.18)),
        origin=Origin(xyz=(-0.18, 0.0, 0.0)),
        material=dark_metal,
        name="rear_ballast_box",
    )
    lamp_head.visual(
        Cylinder(radius=0.045, length=0.34),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_tube",
    )
    lamp_head.visual(
        Cylinder(radius=0.04, length=0.09),
        origin=Origin(xyz=(0.02, 0.20, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion_boss",
    )
    lamp_head.visual(
        Cylinder(radius=0.04, length=0.09),
        origin=Origin(xyz=(0.02, -0.20, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion_boss",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.54, 0.40, 0.40)),
        mass=28.0,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    model.articulation(
        "base_pan",
        ArticulationType.REVOLUTE,
        parent=support_base,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.8,
            lower=-2.7,
            upper=2.7,
        ),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.04, 0.0, 0.41)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.8,
            lower=-0.35,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support_base = object_model.get_part("support_base")
    pan_stage = object_model.get_part("pan_stage")
    lamp_head = object_model.get_part("lamp_head")
    base_pan = object_model.get_articulation("base_pan")
    yoke_tilt = object_model.get_articulation("yoke_tilt")

    ctx.expect_gap(
        pan_stage,
        support_base,
        axis="z",
        positive_elem="turntable",
        negative_elem="bearing_housing",
        max_gap=0.005,
        max_penetration=0.0,
        name="turntable seats on the bearing housing",
    )
    ctx.expect_contact(
        lamp_head,
        pan_stage,
        elem_a="left_trunnion_boss",
        elem_b="left_trunnion_bearing",
        contact_tol=1e-5,
        name="left trunnion stays seated in the yoke bearing",
    )
    ctx.expect_contact(
        lamp_head,
        pan_stage,
        elem_a="right_trunnion_boss",
        elem_b="right_trunnion_bearing",
        contact_tol=1e-5,
        name="right trunnion stays seated in the yoke bearing",
    )
    ctx.expect_overlap(
        lamp_head,
        support_base,
        axes="xy",
        min_overlap=0.18,
        name="lamp assembly stays over the wide support footprint",
    )

    support_aabb = ctx.part_world_aabb(support_base)
    if support_aabb is None:
        ctx.fail("support base resolves to geometry", "support_base world AABB was unavailable")
    else:
        base_size = tuple(
            support_aabb[1][i] - support_aabb[0][i]
            for i in range(3)
        )
        ctx.check(
            "support stays low and wide",
            base_size[0] >= 1.0 and base_size[1] >= 0.75 and base_size[2] <= 0.20,
            details=f"support_base dims={base_size}",
        )

    ctx.check(
        "pan axis stays near the mounting plane",
        base_pan.origin.xyz[2] <= 0.22,
        details=f"pan joint origin z={base_pan.origin.xyz[2]}",
    )

    rest_bezel_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_bezel")
    with ctx.pose({yoke_tilt: yoke_tilt.motion_limits.upper}):
        raised_bezel_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_bezel")
    ctx.check(
        "positive tilt raises the lamp nose",
        rest_bezel_aabb is not None
        and raised_bezel_aabb is not None
        and (raised_bezel_aabb[0][2] + raised_bezel_aabb[1][2]) / 2.0
        > (rest_bezel_aabb[0][2] + rest_bezel_aabb[1][2]) / 2.0 + 0.12,
        details=f"rest_bezel={rest_bezel_aabb}, raised_bezel={raised_bezel_aabb}",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    with ctx.pose({base_pan: math.pi / 2.0}):
        panned_lens_aabb = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    ctx.check(
        "positive pan swings the lamp sideways",
        rest_lens_aabb is not None
        and panned_lens_aabb is not None
        and abs((panned_lens_aabb[0][1] + panned_lens_aabb[1][1]) / 2.0) > 0.25
        and abs((panned_lens_aabb[0][0] + panned_lens_aabb[1][0]) / 2.0) < 0.12,
        details=f"rest_lens={rest_lens_aabb}, panned_lens={panned_lens_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
