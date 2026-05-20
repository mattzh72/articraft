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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_music_stand")

    powder_black = model.material("powder_black", rgba=(0.16, 0.16, 0.17, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=powder_black,
        name="tripod_hub",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=powder_black,
        name="lower_collar",
    )
    base.visual(
        Cylinder(radius=0.0145, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=powder_black,
        name="outer_tube",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        material=powder_black,
        name="top_socket",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(
            xyz=(0.030, 0.0, 0.550),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="clamp_stem",
    )
    base.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.048, 0.0, 0.550)),
        material=rubber,
        name="clamp_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.024 * c, 0.024 * s, 0.112),
                (0.170 * c, 0.170 * s, 0.070),
                (0.360 * c, 0.360 * s, 0.015),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(leg_mesh, f"music_stand_leg_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        base.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.360 * c, 0.360 * s, 0.015)),
            material=rubber,
            name=f"foot_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.76, 0.76, 0.72)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )

    upper_pole = model.part("upper_pole")
    upper_pole.visual(
        # Keep a real telescoping overlap so the pole stays engaged when extended.
        Cylinder(radius=0.0105, length=1.020),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=graphite,
        name="inner_tube",
    )
    upper_pole.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
        material=powder_black,
        name="top_cap",
    )
    upper_pole.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 1.050)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    model.articulation(
        "base_to_upper_pole",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_pole,
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.15,
            lower=0.0,
            upper=0.300,
        ),
    )

    desk = model.part("desk")
    desk.visual(
        Box((0.064, 0.030, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=powder_black,
        name="receiver_block",
    )
    desk.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.008),
                    (0.0, -0.025, 0.055),
                    (0.0, -0.075, 0.135),
                    (0.0, -0.125, 0.205),
                ],
                radius=0.010,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "music_stand_support_arm",
        ),
        material=powder_black,
        name="support_arm",
    )
    desk_angle = 0.33
    desk.visual(
        Box((0.540, 0.006, 0.350)),
        origin=Origin(xyz=(0.0, -0.145, 0.230), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="panel",
    )
    desk.visual(
        Box((0.100, 0.018, 0.220)),
        origin=Origin(xyz=(0.0, -0.141, 0.205), rpy=(desk_angle, 0.0, 0.0)),
        material=powder_black,
        name="center_reinforcement",
    )
    desk.visual(
        Box((0.014, 0.028, 0.350)),
        origin=Origin(xyz=(-0.263, -0.140, 0.230), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="left_flange",
    )
    desk.visual(
        Box((0.014, 0.028, 0.350)),
        origin=Origin(xyz=(0.263, -0.140, 0.230), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="right_flange",
    )
    desk.visual(
        Box((0.500, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, -0.096, 0.076), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="lower_back_rail",
    )
    desk.visual(
        Box((0.500, 0.055, 0.012)),
        origin=Origin(xyz=(0.0, -0.012, 0.062), rpy=(0.08, 0.0, 0.0)),
        material=graphite,
        name="retaining_shelf",
    )
    desk.visual(
        Box((0.500, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.014, 0.068), rpy=(0.08, 0.0, 0.0)),
        material=graphite,
        name="retaining_fence",
    )
    desk.inertial = Inertial.from_geometry(
        Box((0.560, 0.220, 0.430)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.090, 0.205)),
    )

    model.articulation(
        "upper_pole_to_desk",
        ArticulationType.FIXED,
        parent=upper_pole,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_pole = object_model.get_part("upper_pole")
    desk = object_model.get_part("desk")
    height_slide = object_model.get_articulation("base_to_upper_pole")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        base,
        upper_pole,
        elem_a="lower_collar",
        elem_b="inner_tube",
        reason="The telescoping pole intentionally passes through the lower collar sleeve.",
    )
    ctx.allow_overlap(
        base,
        upper_pole,
        elem_a="outer_tube",
        elem_b="inner_tube",
        reason="The inner mast intentionally remains overlapped with the outer tube.",
    )
    ctx.allow_overlap(
        base,
        upper_pole,
        elem_a="top_socket",
        elem_b="inner_tube",
        reason="The inner mast intentionally remains captured within the top socket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        upper_pole,
        desk,
        elem_a="top_cap",
        elem_b="receiver_block",
        name="desk support is seated on the pole top",
    )
    ctx.expect_within(
        upper_pole,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="top_socket",
        margin=0.004,
        name="collapsed pole remains centered in the socket",
    )
    ctx.expect_overlap(
        upper_pole,
        base,
        axes="z",
        min_overlap=0.300,
        elem_a="inner_tube",
        elem_b="outer_tube",
        name="collapsed pole remains deeply inserted in the outer tube",
    )

    low_desk_pos = ctx.part_world_position(desk)
    with ctx.pose({height_slide: 0.300}):
        ctx.expect_within(
            upper_pole,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_tube",
            margin=0.004,
            name="extended pole remains centered in the outer tube",
        )
        ctx.expect_overlap(
            upper_pole,
            base,
            axes="z",
            min_overlap=0.045,
            elem_a="inner_tube",
            elem_b="top_socket",
            name="extended pole retains telescoping overlap inside the top socket",
        )
        high_desk_pos = ctx.part_world_position(desk)

    desk_rises = (
        low_desk_pos is not None
        and high_desk_pos is not None
        and high_desk_pos[2] > low_desk_pos[2] + 0.25
    )
    ctx.check(
        "desk rises when the telescoping pole extends",
        desk_rises,
        details=f"collapsed={low_desk_pos}, extended={high_desk_pos}",
    )

    base_aabb = ctx.part_world_aabb(base)
    tripod_spread_ok = (
        base_aabb is not None
        and (base_aabb[1][0] - base_aabb[0][0]) > 0.53
        and (base_aabb[1][1] - base_aabb[0][1]) > 0.55
    )
    ctx.check(
        "tripod base has a broad footprint",
        tripod_spread_ok,
        details=f"base_aabb={base_aabb}",
    )

    panel_aabb = ctx.part_element_world_aabb(desk, elem="panel")
    lip_aabb = ctx.part_element_world_aabb(desk, elem="retaining_shelf")
    desk_shape_ok = (
        panel_aabb is not None
        and lip_aabb is not None
        and (panel_aabb[1][0] - panel_aabb[0][0]) > 0.50
        and (panel_aabb[1][2] - panel_aabb[0][2]) > 0.30
        and lip_aabb[1][2] < panel_aabb[1][2]
        and lip_aabb[0][1] > panel_aabb[0][1]
    )
    ctx.check(
        "desk is broad and includes a forward retaining lip",
        desk_shape_ok,
        details=f"panel_aabb={panel_aabb}, lip_aabb={lip_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
