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
    tube_from_spline_points,
)


REST_BOOM_ANGLE = math.radians(38.0)
BOOM_TIP_ATTACH = (4.98, 0.0, 0.46)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _boom_section(
    *,
    x: float,
    width: float,
    height: float,
    z_center: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        height,
        radius=min(radius, width * 0.45, height * 0.45),
        corner_segments=8,
    )
    return [(x, y, z + z_center) for y, z in profile]


def _hang_from_boom_tip(distance_down: float) -> tuple[float, float, float]:
    return (
        BOOM_TIP_ATTACH[0] - math.sin(REST_BOOM_ANGLE) * distance_down,
        BOOM_TIP_ATTACH[1],
        BOOM_TIP_ATTACH[2] - math.cos(REST_BOOM_ANGLE) * distance_down,
    )


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.018, 0.0, -0.125),
            (0.034, 0.0, -0.155),
            (0.036, 0.0, -0.192),
            (0.018, 0.0, -0.226),
            (-0.012, 0.0, -0.236),
            (-0.034, 0.0, -0.212),
            (-0.028, 0.0, -0.176),
        ],
        radius=0.0065,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return _save_mesh("offshore_crane_hook", hook_geom)


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((low + high) * 0.5 for low, high in zip(aabb[0], aabb[1]))


def _axis_matches(
    actual: tuple[float, float, float],
    expected: tuple[float, float, float],
    tol: float = 1e-6,
) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offshore_pedestal_crane")

    safety_yellow = model.material("safety_yellow", rgba=(0.90, 0.75, 0.15, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.70, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.60, 0.76, 0.84, 0.35))
    cable_black = model.material("cable_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hook_red = model.material("hook_red", rgba=(0.73, 0.14, 0.10, 1.0))

    pedestal = model.part("pedestal_base")
    pedestal.visual(
        Cylinder(radius=1.15, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="flange_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.82, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=steel,
        name="mounting_doubler",
    )
    for bolt_index in range(12):
        angle = math.tau * bolt_index / 12.0
        pedestal.visual(
            Cylinder(radius=0.05, length=0.11),
            origin=Origin(
                xyz=(0.92 * math.cos(angle), 0.92 * math.sin(angle), 0.055)
            ),
            material=steel,
            name=f"anchor_bolt_{bolt_index:02d}",
        )
    pedestal.visual(
        Cylinder(radius=0.42, length=2.66),
        origin=Origin(xyz=(0.0, 0.0, 1.49)),
        material=safety_yellow,
        name="pedestal_shaft",
    )
    pedestal.visual(
        Cylinder(radius=0.56, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 2.94)),
        material=safety_yellow,
        name="service_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.78, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 3.08)),
        material=dark_steel,
        name="bearing_seat",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((2.30, 2.30, 3.10)),
        mass=6500.0,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
    )

    upperworks = model.part("slewing_superstructure")
    upperworks.visual(
        Cylinder(radius=0.80, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_steel,
        name="slew_ring",
    )
    upperworks.visual(
        Box((0.96, 1.16, 0.10)),
        origin=Origin(xyz=(-0.56, 0.0, 0.29)),
        material=machinery_gray,
        name="turntable_deck",
    )
    upperworks.visual(
        Box((0.34, 0.26, 0.10)),
        origin=Origin(xyz=(0.12, 0.46, 0.29)),
        material=machinery_gray,
        name="port_deck_wing",
    )
    upperworks.visual(
        Box((0.34, 0.26, 0.10)),
        origin=Origin(xyz=(0.12, -0.46, 0.29)),
        material=machinery_gray,
        name="starboard_deck_wing",
    )
    upperworks.visual(
        Box((0.32, 0.72, 0.05)),
        origin=Origin(xyz=(0.98, 0.0, 0.62)),
        material=steel,
        name="front_service_platform",
    )
    upperworks.visual(
        Box((0.84, 1.20, 1.02)),
        origin=Origin(xyz=(-0.98, 0.0, 0.85)),
        material=machinery_gray,
        name="machinery_house",
    )
    upperworks.visual(
        Box((0.94, 1.04, 0.08)),
        origin=Origin(xyz=(-0.98, 0.0, 1.40)),
        material=dark_steel,
        name="house_roof",
    )
    upperworks.visual(
        Box((0.62, 0.98, 0.60)),
        origin=Origin(xyz=(-1.10, 0.0, 0.64)),
        material=dark_steel,
        name="counterweight_box",
    )
    upperworks.visual(
        Box((0.76, 0.70, 0.70)),
        origin=Origin(xyz=(0.56, -0.50, 0.69)),
        material=safety_yellow,
        name="cab_shell",
    )
    upperworks.visual(
        Box((0.70, 0.66, 0.05)),
        origin=Origin(xyz=(0.56, -0.50, 1.065)),
        material=dark_steel,
        name="cab_roof",
    )
    upperworks.visual(
        Box((0.54, 0.46, 0.46)),
        origin=Origin(xyz=(0.72, -0.64, 0.74)),
        material=cab_glass,
        name="cab_glass",
    )
    upperworks.visual(
        Box((0.24, 0.06, 0.36)),
        origin=Origin(xyz=(0.46, 0.15, 1.16)),
        material=safety_yellow,
        name="port_boom_cheek",
    )
    upperworks.visual(
        Box((0.24, 0.06, 0.36)),
        origin=Origin(xyz=(0.46, -0.15, 1.16)),
        material=safety_yellow,
        name="starboard_boom_cheek",
    )
    upperworks.visual(
        Box((0.14, 0.34, 0.92)),
        origin=Origin(xyz=(0.27, 0.0, 0.80)),
        material=safety_yellow,
        name="boom_support_column",
    )
    upperworks.visual(
        Box((0.16, 0.06, 0.08)),
        origin=Origin(xyz=(0.28, 0.15, 1.32)),
        material=safety_yellow,
        name="port_tie_brace",
    )
    upperworks.visual(
        Box((0.16, 0.06, 0.08)),
        origin=Origin(xyz=(0.28, -0.15, 1.32)),
        material=safety_yellow,
        name="starboard_tie_brace",
    )
    upperworks.visual(
        Cylinder(radius=0.18, length=0.48),
        origin=Origin(xyz=(-0.78, 0.0, 0.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="winch_drum",
    )
    upperworks.visual(
        Cylinder(radius=0.07, length=0.48),
        origin=Origin(xyz=(-0.98, 0.38, 1.68)),
        material=dark_steel,
        name="exhaust_stack",
    )
    upperworks.inertial = Inertial.from_geometry(
        Box((2.40, 1.60, 1.95)),
        mass=4200.0,
        origin=Origin(xyz=(-0.22, 0.0, 0.98)),
    )

    boom = model.part("boom_assembly")
    boom.visual(
        Cylinder(radius=0.055, length=0.24),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="heel_trunnion",
    )
    boom.visual(
        Box((0.18, 0.18, 0.16)),
        origin=Origin(xyz=(0.08, 0.0, 0.04)),
        material=safety_yellow,
        name="heel_block",
    )
    boom_shell = section_loft(
        [
            _boom_section(x=0.06, width=0.18, height=0.16, z_center=0.08, radius=0.024),
            _boom_section(x=1.90, width=0.22, height=0.16, z_center=0.14, radius=0.022),
            _boom_section(x=4.00, width=0.16, height=0.12, z_center=0.24, radius=0.018),
            _boom_section(x=4.96, width=0.12, height=0.10, z_center=0.34, radius=0.014),
        ]
    )
    boom.visual(
        _save_mesh("offshore_boom_shell", boom_shell),
        material=safety_yellow,
        name="boom_shell",
    )
    boom.visual(
        Box((0.96, 0.10, 0.05)),
        origin=Origin(xyz=(0.82, 0.0, 0.03)),
        material=machinery_gray,
        name="underside_stiffener",
    )
    boom.visual(
        Box((0.22, 0.22, 0.18)),
        origin=Origin(xyz=(4.94, 0.0, 0.36)),
        material=safety_yellow,
        name="boom_tip_head",
    )
    boom.visual(
        Cylinder(radius=0.075, length=0.18),
        origin=Origin(
            xyz=(4.98, 0.0, 0.46),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="tip_sheave",
    )

    cable_length = 1.25
    hook_block_height = 0.28
    hook_stem_length = 0.11
    hook_tip_length = 0.12
    cable_center = _hang_from_boom_tip(cable_length * 0.5)
    hook_block_center = _hang_from_boom_tip(cable_length + hook_block_height * 0.5)
    hook_stem_center = _hang_from_boom_tip(
        cable_length + hook_block_height + hook_stem_length * 0.5
    )
    hook_tip_center = _hang_from_boom_tip(
        cable_length + hook_block_height + hook_stem_length + hook_tip_length * 0.5
    )

    boom.visual(
        Cylinder(radius=0.012, length=cable_length),
        origin=Origin(xyz=cable_center, rpy=(0.0, REST_BOOM_ANGLE, 0.0)),
        material=cable_black,
        name="pendant_cable",
    )
    boom.visual(
        Box((0.16, 0.12, hook_block_height)),
        origin=Origin(xyz=hook_block_center, rpy=(0.0, REST_BOOM_ANGLE, 0.0)),
        material=safety_yellow,
        name="hook_block",
    )
    boom.visual(
        Cylinder(radius=0.012, length=hook_stem_length),
        origin=Origin(xyz=hook_stem_center, rpy=(0.0, REST_BOOM_ANGLE, 0.0)),
        material=hook_red,
        name="hook_stem",
    )
    boom.visual(
        Cylinder(radius=0.014, length=hook_tip_length),
        origin=Origin(xyz=hook_tip_center, rpy=(0.0, REST_BOOM_ANGLE, 0.0)),
        material=hook_red,
        name="hook_tip",
    )
    boom.inertial = Inertial.from_geometry(
        Box((5.90, 0.45, 1.90)),
        mass=2100.0,
        origin=Origin(xyz=(2.95, 0.0, 0.28)),
    )

    model.articulation(
        "pedestal_slew",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 3.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160000.0, velocity=0.45),
    )
    model.articulation(
        "boom_luff",
        ArticulationType.REVOLUTE,
        parent=upperworks,
        child=boom,
        origin=Origin(xyz=(0.46, 0.0, 1.16), rpy=(0.0, -REST_BOOM_ANGLE, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=95000.0,
            velocity=0.40,
            lower=math.radians(-25.0),
            upper=math.radians(45.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    pedestal = object_model.get_part("pedestal_base")
    upperworks = object_model.get_part("slewing_superstructure")
    boom = object_model.get_part("boom_assembly")
    slew = object_model.get_articulation("pedestal_slew")
    luff = object_model.get_articulation("boom_luff")

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

    ctx.expect_contact(
        upperworks,
        pedestal,
        elem_a="slew_ring",
        elem_b="bearing_seat",
        name="slew ring seated on pedestal bearing seat",
    )
    ctx.expect_overlap(
        upperworks,
        pedestal,
        axes="xy",
        elem_a="slew_ring",
        elem_b="bearing_seat",
        min_overlap=1.50,
        name="slew ring centered over pedestal",
    )
    ctx.expect_contact(
        boom,
        upperworks,
        elem_a="heel_trunnion",
        elem_b="port_boom_cheek",
        name="boom trunnion contacts port cheek",
    )
    ctx.expect_contact(
        boom,
        upperworks,
        elem_a="heel_trunnion",
        elem_b="starboard_boom_cheek",
        name="boom trunnion contacts starboard cheek",
    )

    ctx.check(
        "slew articulation axis is vertical",
        _axis_matches(slew.axis, (0.0, 0.0, 1.0)),
        f"axis={slew.axis}",
    )
    ctx.check(
        "boom articulation axis is transverse",
        _axis_matches(luff.axis, (0.0, -1.0, 0.0)),
        f"axis={luff.axis}",
    )

    tip_aabb = ctx.part_element_world_aabb(boom, elem="boom_tip_head")
    hook_aabb = ctx.part_element_world_aabb(boom, elem="hook_block")
    cab_aabb = ctx.part_element_world_aabb(upperworks, elem="cab_glass")

    ctx.check("boom tip bounds available", tip_aabb is not None, "missing boom_tip_head bounds")
    ctx.check("hook block bounds available", hook_aabb is not None, "missing hook_block bounds")
    ctx.check("cab bounds available", cab_aabb is not None, "missing cab_glass bounds")

    tip_center = _aabb_center(tip_aabb)
    hook_center = _aabb_center(hook_aabb)
    cab_center = _aabb_center(cab_aabb)

    if tip_center is not None and hook_center is not None:
        ctx.check(
            "hook block hangs below boom tip",
            hook_center[2] < tip_center[2] - 1.0,
            f"tip_center={tip_center} hook_center={hook_center}",
        )
        ctx.check(
            "hook block stays nearly under boom tip in plan",
            abs(hook_center[0] - tip_center[0]) < 0.30
            and abs(hook_center[1] - tip_center[1]) < 0.22,
            f"tip_center={tip_center} hook_center={hook_center}",
        )

    low_tip_center = None
    with ctx.pose({luff: math.radians(-20.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps with boom lowered")
        ctx.expect_contact(
            boom,
            upperworks,
            elem_a="heel_trunnion",
            elem_b="port_boom_cheek",
            name="boom stays pinned to port cheek when lowered",
        )
        low_tip_center = _aabb_center(ctx.part_element_world_aabb(boom, elem="boom_tip_head"))

    high_tip_center = None
    with ctx.pose({luff: math.radians(30.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps with boom raised")
        ctx.expect_contact(
            boom,
            upperworks,
            elem_a="heel_trunnion",
            elem_b="starboard_boom_cheek",
            name="boom stays pinned to starboard cheek when raised",
        )
        high_tip_center = _aabb_center(ctx.part_element_world_aabb(boom, elem="boom_tip_head"))

    if low_tip_center is not None and high_tip_center is not None:
        ctx.check(
            "boom tip rises through luff range",
            high_tip_center[2] > low_tip_center[2] + 1.2
            and high_tip_center[0] < low_tip_center[0] - 0.60,
            f"low_tip_center={low_tip_center} high_tip_center={high_tip_center}",
        )

    with ctx.pose({slew: math.pi / 2.0}):
        ctx.expect_contact(
            upperworks,
            pedestal,
            elem_a="slew_ring",
            elem_b="bearing_seat",
            name="slew ring stays seated while rotated",
        )
        slewed_cab_center = _aabb_center(
            ctx.part_element_world_aabb(upperworks, elem="cab_glass")
        )

    if cab_center is not None and slewed_cab_center is not None:
        ctx.check(
            "cab moves around pedestal when slewing",
            abs(abs(slewed_cab_center[0]) - abs(cab_center[1])) < 0.10
            and abs(abs(slewed_cab_center[1]) - abs(cab_center[0])) < 0.10,
            f"cab_center={cab_center} slewed_cab_center={slewed_cab_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
