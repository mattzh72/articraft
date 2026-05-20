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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)]


def _build_base_shell():
    return section_loft(
        [
            _rounded_section(0.148, 0.148, 0.022, 0.000),
            _rounded_section(0.156, 0.156, 0.030, 0.036),
            _rounded_section(0.144, 0.144, 0.026, 0.074),
            _rounded_section(0.112, 0.112, 0.016, 0.097),
        ]
    )


def _build_cup_shell():
    outer_profile = [
        (0.052, 0.000),
        (0.051, 0.010),
        (0.050, 0.028),
        (0.044, 0.050),
        (0.046, 0.122),
        (0.0475, 0.198),
        (0.0475, 0.206),
    ]
    inner_profile = [
        (0.0365, 0.000),
        (0.042, 0.010),
        (0.044, 0.028),
        (0.039, 0.050),
        (0.041, 0.122),
        (0.042, 0.198),
    ]
    return LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=72)


def _build_lid_cap():
    return ExtrudeGeometry(
        rounded_rect_profile(0.096, 0.086, 0.016, corner_segments=8),
        0.008,
        cap=True,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="personal_single_serve_blender")

    base_body = model.material("base_body", rgba=(0.30, 0.32, 0.35, 1.0))
    base_trim = model.material("base_trim", rgba=(0.10, 0.10, 0.11, 1.0))
    accent = model.material("accent", rgba=(0.18, 0.62, 0.70, 1.0))
    cup_clear = model.material("cup_clear", rgba=(0.80, 0.92, 0.97, 0.30))
    cup_collar = model.material("cup_collar", rgba=(0.16, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    lid_smoke = model.material("lid_smoke", rgba=(0.16, 0.18, 0.19, 0.92))

    base = model.part("motor_base")
    base.visual(
        mesh_from_geometry(_build_base_shell(), "motor_base_shell"),
        material=base_body,
        name="housing_shell",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=base_trim,
        name="top_mount",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=base_trim,
        name="drive_gasket",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=steel,
        name="drive_coupling",
    )
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.028, 0.014, 0.004, corner_segments=6),
                0.004,
                cap=True,
                center=True,
            ),
            "power_button",
        ),
        origin=Origin(xyz=(0.0, 0.071, 0.046), rpy=(math.radians(18.0), 0.0, 0.0)),
        material=accent,
        name="power_button",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.044, -0.044),
            (0.044, -0.044),
            (-0.044, 0.044),
            (0.044, 0.044),
        )
    ):
        base.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(x_pos, y_pos, 0.004)),
            material=base_trim,
            name=f"foot_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.160, 0.160, 0.100)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    cup = model.part("blending_cup")
    cup.visual(
        mesh_from_geometry(_build_cup_shell(), "blending_cup_shell"),
        material=cup_clear,
        name="cup_shell",
    )
    cup.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.0495, tube=0.0025, radial_segments=16, tubular_segments=52),
            "thread_ridge_0",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=base_trim,
        name="thread_ridge_0",
    )
    cup.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.0488, tube=0.0025, radial_segments=16, tubular_segments=52),
            "thread_ridge_1",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0170)),
        material=base_trim,
        name="thread_ridge_1",
    )
    cup.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.0482, tube=0.0025, radial_segments=16, tubular_segments=52),
            "thread_ridge_2",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0235)),
        material=base_trim,
        name="thread_ridge_2",
    )
    cup.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.0455, tube=0.0018, radial_segments=16, tubular_segments=56),
            "rim_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.203)),
        material=lid_smoke,
        name="rim_ring",
    )
    cup.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(-0.016, -0.045, 0.206), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cup_collar,
        name="hinge_knuckle_left",
    )
    cup.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.016, -0.045, 0.206), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cup_collar,
        name="hinge_knuckle_right",
    )
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.206),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
    )

    model.articulation(
        "base_to_cup",
        ArticulationType.FIXED,
        parent=base,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
    )

    blade = model.part("blade_assembly")
    blade.visual(
        Cylinder(radius=0.045, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0015)),
        material=base_trim,
        name="mount_flange",
    )
    blade.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=steel,
        name="blade_shaft",
    )
    blade.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=base_trim,
        name="blade_hub",
    )
    blade.visual(
        Box((0.050, 0.008, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.016), rpy=(math.radians(18.0), math.radians(-12.0), math.radians(12.0))),
        material=steel,
        name="blade_upper_a",
    )
    blade.visual(
        Box((0.050, 0.008, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.016), rpy=(math.radians(-18.0), math.radians(12.0), math.pi + math.radians(12.0))),
        material=steel,
        name="blade_upper_b",
    )
    blade.visual(
        Box((0.044, 0.008, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(math.radians(-14.0), math.radians(10.0), math.pi / 2.0 + math.radians(10.0))),
        material=steel,
        name="blade_lower_a",
    )
    blade.visual(
        Box((0.044, 0.008, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(math.radians(14.0), math.radians(-10.0), 3.0 * math.pi / 2.0 + math.radians(10.0))),
        material=steel,
        name="blade_lower_b",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.026),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "cup_to_blade",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=20.0,
            lower=-math.tau,
            upper=math.tau,
        ),
    )

    lid = model.part("flip_spout_lid")
    lid.visual(
        mesh_from_geometry(_build_lid_cap(), "flip_lid_cap"),
        origin=Origin(xyz=(0.0, 0.043, 0.004)),
        material=lid_smoke,
        name="cap_shell",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.036),
        origin=Origin(xyz=(0.0, 0.078, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lid_smoke,
        name="spout_cover",
    )
    lid.visual(
        Box((0.018, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.087, 0.006)),
        material=lid_smoke,
        name="thumb_tab",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lid_smoke,
        name="hinge_knuckle",
    )
    lid.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(-0.010, 0.006, 0.004)),
        material=lid_smoke,
        name="hinge_web_left",
    )
    lid.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.010, 0.006, 0.004)),
        material=lid_smoke,
        name="hinge_web_right",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.100, 0.092, 0.020)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.044, 0.008)),
    )

    model.articulation(
        "cup_to_lid",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=lid,
        origin=Origin(xyz=(0.0, -0.045, 0.206)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=5.0,
            lower=0.0,
            upper=math.radians(118.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("motor_base")
    cup = object_model.get_part("blending_cup")
    blade = object_model.get_part("blade_assembly")
    lid = object_model.get_part("flip_spout_lid")
    cap_shell = lid.get_visual("cap_shell")
    rim_ring = cup.get_visual("rim_ring")

    base_to_cup = object_model.get_articulation("base_to_cup")
    cup_to_blade = object_model.get_articulation("cup_to_blade")
    cup_to_lid = object_model.get_articulation("cup_to_lid")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    assert base_to_cup.articulation_type == ArticulationType.FIXED
    assert cup_to_blade.axis == (0.0, 0.0, 1.0)
    assert cup_to_lid.axis == (1.0, 0.0, 0.0)

    ctx.expect_gap(
        cup,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        max_penetration=0.0,
        name="cup_seats_on_motor_base",
    )
    ctx.expect_overlap(
        cup,
        base,
        axes="xy",
        min_overlap=0.070,
        name="cup_centered_over_base",
    )
    ctx.expect_contact(
        blade,
        cup,
        name="blade_assembly_mounted_to_cup",
    )
    ctx.expect_within(
        blade,
        cup,
        axes="xy",
        margin=0.010,
        name="blade_stays_inside_cup_profile",
    )

    with ctx.pose({cup_to_lid: 0.0}):
        ctx.expect_gap(
            lid,
            cup,
            axis="z",
            min_gap=0.0,
            max_gap=0.0015,
            max_penetration=0.0,
            name="lid_sits_flush_on_rim",
            positive_elem=cap_shell,
            negative_elem=rim_ring,
        )
        ctx.expect_overlap(
            lid,
            cup,
            axes="xy",
            min_overlap=0.075,
            name="lid_covers_cup_opening",
            elem_a=cap_shell,
        )

    closed_cap_aabb = ctx.part_element_world_aabb(lid, elem="cap_shell")
    assert closed_cap_aabb is not None
    with ctx.pose({cup_to_lid: math.radians(110.0)}):
        open_cap_aabb = ctx.part_element_world_aabb(lid, elem="cap_shell")
        assert open_cap_aabb is not None
        assert open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.030
        assert open_cap_aabb[0][1] < closed_cap_aabb[0][1] - 0.020

    with ctx.pose({cup_to_blade: math.radians(85.0)}):
        ctx.expect_contact(
            blade,
            cup,
            name="blade_stays_seated_while_rotating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
