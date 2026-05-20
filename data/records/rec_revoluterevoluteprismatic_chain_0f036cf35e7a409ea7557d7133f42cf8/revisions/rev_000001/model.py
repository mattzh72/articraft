from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


L1 = 0.70
L2 = 0.62
RAM_TRAVEL = 0.20


def _cyl_y() -> Origin:
    """URDF cylinders are local-Z; this turns the visible barrel across width."""

    return Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _cyl_x(xyz=(0.0, 0.0, 0.0)) -> Origin:
    """URDF cylinders are local-Z; this turns the visible barrel along the arm."""

    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_two_link_arm")

    dark_steel = model.material("dark_blued_steel", rgba=(0.10, 0.12, 0.14, 1.0))
    painted_frame = model.material("painted_bridge_frame", rgba=(0.20, 0.23, 0.25, 1.0))
    safety_yellow = model.material("safety_yellow_links", rgba=(0.95, 0.63, 0.10, 1.0))
    pivot_black = model.material("black_pivot_bushings", rgba=(0.04, 0.04, 0.04, 1.0))
    chrome = model.material("polished_ram_rod", rgba=(0.82, 0.86, 0.88, 1.0))
    nose_blue = model.material("blue_output_nose", rgba=(0.08, 0.25, 0.62, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.62, 0.42, 0.07)),
        origin=Origin(xyz=(-0.08, 0.0, 0.035)),
        material=dark_steel,
        name="floor_foot",
    )
    rear_support.visual(
        Box((0.14, 0.16, 0.72)),
        origin=Origin(xyz=(-0.18, 0.0, 0.40)),
        material=painted_frame,
        name="rear_mast",
    )
    rear_support.visual(
        Box((0.08, 0.28, 0.20)),
        origin=Origin(xyz=(-0.16, 0.0, 0.78)),
        material=painted_frame,
        name="bridge_back",
    )
    for side, yoke_name, bushing_name, y in (
        (0, "shoulder_yoke_0", "shoulder_bushing_0", -0.115),
        (1, "shoulder_yoke_1", "shoulder_bushing_1", 0.115),
    ):
        rear_support.visual(
            Box((0.26, 0.04, 0.20)),
            origin=Origin(xyz=(-0.01, y, 0.78)),
            material=painted_frame,
            name=yoke_name,
        )
        rear_support.visual(
            Cylinder(radius=0.060, length=0.018),
            origin=Origin(xyz=(0.0, y * 1.04, 0.80), rpy=(pi / 2.0, 0.0, 0.0)),
            material=pivot_black,
            name=bushing_name,
        )
    rear_support.visual(
        Cylinder(radius=0.030, length=0.290),
        origin=Origin(xyz=(0.0, 0.0, 0.80), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_pin",
    )
    rear_support.visual(
        Box((0.20, 0.20, 0.06)),
        origin=Origin(xyz=(-0.13, 0.0, 0.90)),
        material=painted_frame,
        name="bridge_cap",
    )
    for side, y in enumerate((-0.145, 0.145)):
        rear_support.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (-0.29, y, 0.08),
                        (-0.27, y, 0.34),
                        (-0.22, y, 0.58),
                        (-0.16, y, 0.80),
                    ],
                    radius=0.018,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
                f"arched_bridge_{side}",
            ),
            material=painted_frame,
            name=f"arched_bridge_{side}",
        )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.075, length=0.080),
        origin=_cyl_y(),
        material=safety_yellow,
        name="shoulder_boss",
    )
    upper_link.visual(
        Box((0.46, 0.070, 0.055)),
        origin=Origin(xyz=(0.260, 0.0, 0.0)),
        material=safety_yellow,
        name="upper_web",
    )
    upper_link.visual(
        Box((0.070, 0.210, 0.055)),
        origin=Origin(xyz=(0.510, 0.0, 0.0)),
        material=safety_yellow,
        name="elbow_bridge",
    )
    for side, ear_name, boss_name, bushing_name, y in (
        (0, "elbow_ear_0", "elbow_boss_0", "elbow_bushing_0", -0.085),
        (1, "elbow_ear_1", "elbow_boss_1", "elbow_bushing_1", 0.085),
    ):
        upper_link.visual(
            Box((0.250, 0.035, 0.090)),
            origin=Origin(xyz=(0.625, y, 0.0)),
            material=safety_yellow,
            name=ear_name,
        )
        upper_link.visual(
            Cylinder(radius=0.066, length=0.035),
            origin=Origin(xyz=(L1, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=safety_yellow,
            name=boss_name,
        )
        upper_link.visual(
            Cylinder(radius=0.040, length=0.006),
            origin=Origin(xyz=(L1, y * 1.08, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=pivot_black,
            name=bushing_name,
        )
    upper_link.visual(
        Cylinder(radius=0.028, length=0.260),
        origin=Origin(xyz=(L1, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_pin",
    )

    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.060, length=0.080),
        origin=_cyl_y(),
        material=safety_yellow,
        name="rear_lug",
    )
    forelink.visual(
        Box((0.410, 0.075, 0.055)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=safety_yellow,
        name="fore_web",
    )
    for side, y in enumerate((-0.055, 0.055)):
        forelink.visual(
            Box((0.120, 0.035, 0.078)),
            origin=Origin(xyz=(0.475, y, 0.0)),
            material=safety_yellow,
            name=f"sleeve_side_strap_{side}",
        )
    forelink.visual(
        Box((0.240, 0.130, 0.130)),
        origin=Origin(xyz=(L2, 0.0, 0.0)),
        material=painted_frame,
        name="ram_sleeve",
    )
    forelink.visual(
        Box((0.035, 0.160, 0.160)),
        origin=Origin(xyz=(L2 + 0.125, 0.0, 0.0)),
        material=dark_steel,
        name="sleeve_front_band",
    )

    output_ram = model.part("output_ram")
    output_ram.visual(
        Cylinder(radius=0.026, length=0.420),
        origin=_cyl_x((0.050, 0.0, 0.0)),
        material=chrome,
        name="ram_rod",
    )
    output_ram.visual(
        Cylinder(radius=0.050, length=0.080),
        origin=_cyl_x((0.300, 0.0, 0.0)),
        material=nose_blue,
        name="nose_plunger",
    )
    output_ram.visual(
        Box((0.025, 0.090, 0.060)),
        origin=Origin(xyz=(0.350, 0.0, 0.0)),
        material=nose_blue,
        name="output_pad",
    )

    model.articulation(
        "support_to_upper",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=-0.65, upper=0.95),
    )
    model.articulation(
        "upper_to_fore",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(L1, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.8, lower=-1.10, upper=1.15),
    )
    model.articulation(
        "fore_to_ram",
        ArticulationType.PRISMATIC,
        parent=forelink,
        child=output_ram,
        origin=Origin(xyz=(L2, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=RAM_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    output_ram = object_model.get_part("output_ram")
    shoulder = object_model.get_articulation("support_to_upper")
    elbow = object_model.get_articulation("upper_to_fore")
    ram_slide = object_model.get_articulation("fore_to_ram")

    ctx.allow_overlap(
        forelink,
        output_ram,
        elem_a="ram_sleeve",
        elem_b="ram_rod",
        reason=(
            "The compact output ram is intentionally represented as a polished rod "
            "sliding through a simplified solid sleeve proxy on the forelink."
        ),
    )
    ctx.allow_overlap(
        forelink,
        output_ram,
        elem_a="sleeve_front_band",
        elem_b="ram_rod",
        reason="The front gland band is represented as a solid proxy around the sliding ram rod.",
    )
    ctx.allow_overlap(
        rear_support,
        upper_link,
        elem_a="shoulder_pin",
        elem_b="shoulder_boss",
        reason="The shoulder shaft is intentionally captured through the rotating upper-link boss.",
    )
    ctx.allow_overlap(
        upper_link,
        forelink,
        elem_a="elbow_pin",
        elem_b="rear_lug",
        reason="The elbow shaft is intentionally captured through the forelink rear lug.",
    )

    ctx.check(
        "primary joints are articulated",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and ram_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"types={(shoulder.articulation_type, elbow.articulation_type, ram_slide.articulation_type)!r}",
    )
    ctx.check(
        "ram has compact usable travel",
        ram_slide.motion_limits is not None
        and ram_slide.motion_limits.lower == 0.0
        and 0.18 <= float(ram_slide.motion_limits.upper) <= 0.23,
        details=f"limits={ram_slide.motion_limits!r}",
    )

    ctx.expect_overlap(
        output_ram,
        forelink,
        axes="yz",
        elem_a="ram_rod",
        elem_b="ram_sleeve",
        min_overlap=0.045,
        name="ram rod is centered in sleeve",
    )
    ctx.expect_overlap(
        output_ram,
        forelink,
        axes="x",
        elem_a="ram_rod",
        elem_b="ram_sleeve",
        min_overlap=0.20,
        name="collapsed ram remains deeply inserted",
    )
    ctx.expect_overlap(
        output_ram,
        forelink,
        axes="x",
        elem_a="ram_rod",
        elem_b="sleeve_front_band",
        min_overlap=0.025,
        name="ram rod passes through front gland band",
    )
    rest_aabb = ctx.part_world_aabb(output_ram)
    with ctx.pose({ram_slide: RAM_TRAVEL}):
        ctx.expect_overlap(
            output_ram,
            forelink,
            axes="yz",
            elem_a="ram_rod",
            elem_b="ram_sleeve",
            min_overlap=0.045,
            name="extended ram stays centered in sleeve",
        )
        ctx.expect_overlap(
            output_ram,
            forelink,
            axes="x",
            elem_a="ram_rod",
            elem_b="ram_sleeve",
            min_overlap=0.070,
            name="extended ram retains sleeve insertion",
        )
        extended_aabb = ctx.part_world_aabb(output_ram)

    if rest_aabb is not None and extended_aabb is not None:
        ctx.check(
            "ram nose extends forward",
            float(extended_aabb[1][0] - rest_aabb[1][0]) > RAM_TRAVEL * 0.85,
            details=f"rest={rest_aabb}, extended={extended_aabb}",
        )
    else:
        ctx.fail("ram nose extends forward", "Could not compute ram AABBs.")

    ctx.expect_overlap(
        upper_link,
        rear_support,
        axes="xz",
        elem_a="shoulder_boss",
        elem_b="shoulder_yoke_0",
        min_overlap=0.06,
        name="upper link sits between rear yoke plates",
    )
    ctx.expect_overlap(
        rear_support,
        upper_link,
        axes="xyz",
        elem_a="shoulder_pin",
        elem_b="shoulder_boss",
        min_overlap=0.040,
        name="shoulder pin captures upper boss",
    )
    ctx.expect_overlap(
        forelink,
        upper_link,
        axes="xz",
        elem_a="rear_lug",
        elem_b="elbow_ear_0",
        min_overlap=0.055,
        name="forelink lug sits between upper clevis ears",
    )
    ctx.expect_overlap(
        upper_link,
        forelink,
        axes="xyz",
        elem_a="elbow_pin",
        elem_b="rear_lug",
        min_overlap=0.040,
        name="elbow pin captures forelink lug",
    )

    return ctx.report()


object_model = build_object_model()
