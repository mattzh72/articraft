from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _bar_visual(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width: float,
    thickness: float,
    material: Material | str,
    name: str,
) -> None:
    """Add a flat rectangular link between two local-space points."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, 0.0, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_wiper_assembly")

    powder_coat = model.material("black_powder_coated_aluminum", rgba=(0.02, 0.025, 0.025, 1.0))
    satin_black = model.material("satin_black_weather_cover", rgba=(0.005, 0.006, 0.006, 1.0))
    rubber = model.material("black_epdm_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    stainless = model.material("brushed_stainless_hardware", rgba=(0.68, 0.70, 0.68, 1.0))
    zinc = model.material("zinc_plated_linkage", rgba=(0.50, 0.52, 0.50, 1.0))
    glass_tint = model.material("pale_wet_windscreen_area", rgba=(0.20, 0.30, 0.35, 0.38))

    cowl = model.part("cowl")
    # One continuous stationary weatherproof tray with overhangs, gutter lips,
    # spindle towers, sealed motor cover, and stainless fasteners.
    cowl.visual(
        Box((1.10, 0.22, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=powder_coat,
        name="cowl_plate",
    )
    cowl.visual(
        Box((1.16, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, -0.130, 0.040)),
        material=powder_coat,
        name="front_drip_overhang",
    )
    cowl.visual(
        Box((1.10, 0.026, 0.025)),
        origin=Origin(xyz=(0.0, 0.118, 0.0375)),
        material=rubber,
        name="rear_gasket_lip",
    )
    cowl.visual(
        Box((0.96, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.092, 0.035)),
        material=satin_black,
        name="drain_gutter",
    )
    cowl.visual(
        Box((0.22, 0.13, 0.080)),
        origin=Origin(xyz=(0.0, -0.055, 0.065)),
        material=satin_black,
        name="motor_housing",
    )
    cowl.visual(
        Box((0.26, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, -0.055, 0.111)),
        material=powder_coat,
        name="motor_cover_overhang",
    )
    cowl.visual(
        Box((0.24, 0.145, 0.008)),
        origin=Origin(xyz=(0.0, -0.055, 0.032)),
        material=rubber,
        name="motor_base_gasket",
    )
    cowl.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, -0.055, 0.120)),
        material=stainless,
        name="motor_shaft_seal",
    )
    cowl.visual(
        Box((1.04, 0.13, 0.006)),
        origin=Origin(xyz=(0.0, 0.178, 0.052)),
        material=glass_tint,
        name="windscreen_reference",
    )

    for idx, x in enumerate((-0.35, 0.35)):
        cowl.visual(
            Cylinder(radius=0.058, length=0.060),
            origin=Origin(xyz=(x, 0.020, 0.055)),
            material=powder_coat,
            name=f"pivot_tower_{idx}",
        )
        cowl.visual(
            Cylinder(radius=0.071, length=0.010),
            origin=Origin(xyz=(x, 0.020, 0.030)),
            material=powder_coat,
            name=f"tower_flange_{idx}",
        )
        for bolt_idx, (bx, by) in enumerate(((0.048, 0.000), (-0.024, 0.041), (-0.024, -0.041))):
            cowl.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(xyz=(x + bx, 0.020 + by, 0.038)),
                material=stainless,
                name=f"flange_bolt_{idx}_{bolt_idx}",
            )

    cowl.visual(
        Cylinder(radius=0.045, length=0.030),
        origin=Origin(xyz=(-0.35, 0.020, 0.100)),
        material=rubber,
        name="pivot_boot_0",
    )
    cowl.visual(
        Cylinder(radius=0.045, length=0.030),
        origin=Origin(xyz=(0.35, 0.020, 0.100)),
        material=rubber,
        name="pivot_boot_1",
    )

    for idx, (x, y) in enumerate(((-0.082, -0.102), (0.082, -0.102), (-0.082, -0.008), (0.082, -0.008))):
        cowl.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(x, y, 0.120)),
            material=stainless,
            name=f"cover_screw_{idx}",
        )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=stainless,
        name="crank_shaft",
    )
    crank.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=zinc,
        name="crank_hub",
    )
    _bar_visual(
        crank,
        (0.0, 0.0, 0.020),
        (0.130, 0.0, 0.020),
        width=0.030,
        thickness=0.008,
        material=zinc,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.130, 0.0, 0.030)),
        material=zinc,
        name="crank_eye",
    )

    link_rod = model.part("link_rod")
    link_rod.visual(
        Box((0.0738, 0.018, 0.010)),
        origin=Origin(xyz=(-0.030, 0.0215, 0.0), rpy=(0.0, 0.0, 2.519)),
        material=zinc,
        name="rod_crank_stub",
    )
    link_rod.visual(
        Box((0.260, 0.018, 0.010)),
        origin=Origin(xyz=(-0.190, 0.043, 0.0)),
        material=zinc,
        name="left_rod_bar",
    )
    link_rod.visual(
        Box((0.0738, 0.018, 0.010)),
        origin=Origin(xyz=(-0.350, 0.0215, 0.0), rpy=(0.0, 0.0, -2.519)),
        material=zinc,
        name="rod_eye_stub_0",
    )
    link_rod.visual(
        Box((0.120, 0.018, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=zinc,
        name="rod_eye_stub_1",
    )
    link_rod.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=zinc,
        name="rod_crank_eye",
    )
    link_rod.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(-0.380, 0.0, 0.0)),
        material=zinc,
        name="rod_eye_0",
    )
    link_rod.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=zinc,
        name="rod_eye_1",
    )

    wiper_specs = (
        ("wiper_0", "blade_carrier_0", -0.35, 0.13, 0.10, -0.55),
        ("wiper_1", "blade_carrier_1", 0.35, -0.13, -0.10, -1.73),
    )
    for idx, (wiper_name, blade_name, x, arm_tip_x, rocker_x, wiper_multiplier) in enumerate(wiper_specs):
        wiper = model.part(wiper_name)
        wiper.visual(
            Cylinder(radius=0.020, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, 0.0125)),
            material=stainless,
            name="shaft",
        )
        wiper.visual(
            Cylinder(radius=0.034, length=0.013),
            origin=Origin(xyz=(0.0, 0.0, 0.0315)),
            material=rubber,
            name="sealed_cap",
        )
        wiper.visual(
            Cylinder(radius=0.020, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, 0.051)),
            material=stainless,
            name="arm_mount",
        )
        _bar_visual(
            wiper,
            (0.0, 0.0, 0.030),
            (rocker_x, -0.075, 0.030),
            width=0.024,
            thickness=0.008,
            material=zinc,
            name="rocker_crank",
        )
        wiper.visual(
            Cylinder(radius=0.019, length=0.012),
            origin=Origin(xyz=(rocker_x, -0.075, 0.040)),
            material=zinc,
            name="rocker_eye",
        )
        _bar_visual(
            wiper,
            (0.0, 0.020, 0.058),
            (arm_tip_x, 0.445, 0.058),
            width=0.020,
            thickness=0.010,
            material=stainless,
            name="wiper_arm",
        )
        _bar_visual(
            wiper,
            (0.0, 0.030, 0.066),
            (arm_tip_x * 0.70, 0.345, 0.066),
            width=0.008,
            thickness=0.006,
            material=stainless,
            name="spring_spine",
        )
        wiper.visual(
            Cylinder(radius=0.015, length=0.035),
            origin=Origin(xyz=(arm_tip_x, 0.460, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="blade_pin",
        )

        blade = model.part(blade_name)
        blade.visual(
            Cylinder(radius=0.014, length=0.044),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="hinge_socket",
        )
        blade.visual(
            Box((0.060, 0.018, 0.014)),
            origin=Origin(xyz=(0.0, 0.006, 0.0)),
            material=stainless,
            name="yoke_block",
        )
        blade.visual(
            Box((0.030, 0.104, 0.010)),
            origin=Origin(xyz=(0.0, 0.052, -0.004)),
            material=stainless,
            name="yoke_bridge",
        )
        blade.visual(
            Box((0.380, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, 0.100, -0.006)),
            material=satin_black,
            name="blade_spine",
        )
        blade.visual(
            Box((0.360, 0.012, 0.038)),
            origin=Origin(xyz=(0.0, 0.100, -0.034)),
            material=rubber,
            name="rubber_squeegee",
        )
        blade.visual(
            Box((0.022, 0.024, 0.022)),
            origin=Origin(xyz=(-0.201, 0.100, -0.010)),
            material=satin_black,
            name="end_clip_0",
        )
        blade.visual(
            Box((0.022, 0.024, 0.022)),
            origin=Origin(xyz=(0.201, 0.100, -0.010)),
            material=satin_black,
            name="end_clip_1",
        )

        model.articulation(
            f"cowl_to_{wiper_name}",
            ArticulationType.REVOLUTE,
            parent=cowl,
            child=wiper,
            origin=Origin(xyz=(x, 0.020, 0.115)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-1.10, upper=1.10),
            mimic=Mimic(joint="cowl_to_crank", multiplier=wiper_multiplier, offset=0.0),
        )
        model.articulation(
            f"{wiper_name}_to_blade_carrier",
            ArticulationType.REVOLUTE,
            parent=wiper,
            child=blade,
            origin=Origin(xyz=(arm_tip_x, 0.460, 0.058)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=-0.18, upper=0.18),
        )

    model.articulation(
        "cowl_to_crank",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=crank,
        origin=Origin(xyz=(0.0, -0.055, 0.129)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.5, lower=-0.60, upper=0.60),
    )
    model.articulation(
        "crank_to_link_rod",
        ArticulationType.REVOLUTE,
        parent=crank,
        child=link_rod,
        origin=Origin(xyz=(0.130, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.25, upper=0.25),
        mimic=Mimic(joint="cowl_to_crank", multiplier=-0.35, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crank = object_model.get_part("crank")
    link_rod = object_model.get_part("link_rod")
    wiper_0 = object_model.get_part("wiper_0")
    wiper_1 = object_model.get_part("wiper_1")
    cowl = object_model.get_part("cowl")
    drive = object_model.get_articulation("cowl_to_crank")

    ctx.allow_overlap(
        crank,
        link_rod,
        elem_a="crank_eye",
        elem_b="rod_crank_eye",
        reason="The rod eye is intentionally captured on the corrosion-resistant crank pin.",
    )
    ctx.allow_overlap(
        crank,
        link_rod,
        elem_a="crank_eye",
        elem_b="rod_crank_stub",
        reason="The flat rod end locally tucks under the crank eye around the captured pin stack.",
    )
    ctx.allow_overlap(
        crank,
        link_rod,
        elem_a="crank_eye",
        elem_b="rod_eye_stub_1",
        reason="The second rod takeoff shares the same protected crank pin stack.",
    )
    ctx.allow_overlap(
        wiper_0,
        link_rod,
        elem_a="rocker_eye",
        elem_b="rod_eye_0",
        reason="The linkage eye is intentionally captured on the supported wiper rocker pin.",
    )
    ctx.allow_overlap(
        wiper_0,
        link_rod,
        elem_a="rocker_eye",
        elem_b="rod_eye_stub_0",
        reason="The plated rod end locally seats under the rocker eye as part of the retained pin stack.",
    )
    ctx.allow_overlap(
        wiper_1,
        link_rod,
        elem_a="rocker_eye",
        elem_b="rod_eye_1",
        reason="The linkage eye is intentionally captured on the supported wiper rocker pin.",
    )
    ctx.allow_overlap(
        wiper_1,
        link_rod,
        elem_a="rocker_eye",
        elem_b="rod_eye_stub_1",
        reason="The plated rod end locally seats under the rocker eye as part of the retained pin stack.",
    )
    ctx.allow_overlap(
        wiper_0,
        "blade_carrier_0",
        elem_a="blade_pin",
        elem_b="hinge_socket",
        reason="The blade carrier socket is intentionally retained on the stainless arm pin.",
    )
    ctx.allow_overlap(
        wiper_0,
        "blade_carrier_0",
        elem_a="blade_pin",
        elem_b="yoke_block",
        reason="The clevis yoke is intentionally pierced by the blade hinge pin.",
    )
    ctx.allow_overlap(
        wiper_0,
        "blade_carrier_0",
        elem_a="blade_pin",
        elem_b="yoke_bridge",
        reason="The central blade yoke bridge is intentionally pierced by the hinge pin.",
    )
    ctx.allow_overlap(
        wiper_1,
        "blade_carrier_1",
        elem_a="blade_pin",
        elem_b="hinge_socket",
        reason="The blade carrier socket is intentionally retained on the stainless arm pin.",
    )
    ctx.allow_overlap(
        wiper_1,
        "blade_carrier_1",
        elem_a="blade_pin",
        elem_b="yoke_block",
        reason="The clevis yoke is intentionally pierced by the blade hinge pin.",
    )
    ctx.allow_overlap(
        wiper_1,
        "blade_carrier_1",
        elem_a="blade_pin",
        elem_b="yoke_bridge",
        reason="The central blade yoke bridge is intentionally pierced by the hinge pin.",
    )

    ctx.expect_contact(
        crank,
        cowl,
        elem_a="crank_shaft",
        elem_b="motor_shaft_seal",
        contact_tol=0.001,
        name="crank shaft seats on sealed motor spindle",
    )
    ctx.expect_contact(
        wiper_0,
        cowl,
        elem_a="shaft",
        elem_b="pivot_boot_0",
        contact_tol=0.001,
        name="wiper 0 spindle is supported by sealed boot",
    )
    ctx.expect_contact(
        wiper_1,
        cowl,
        elem_a="shaft",
        elem_b="pivot_boot_1",
        contact_tol=0.001,
        name="wiper 1 spindle is supported by sealed boot",
    )
    ctx.expect_overlap(
        crank,
        link_rod,
        axes="xy",
        elem_a="crank_eye",
        elem_b="rod_crank_eye",
        min_overlap=0.025,
        name="crank pin remains inside rod eye footprint",
    )
    ctx.expect_overlap(
        wiper_0,
        link_rod,
        axes="xy",
        elem_a="rocker_eye",
        elem_b="rod_eye_0",
        min_overlap=0.018,
        name="link rod reaches wiper 0 rocker eye",
    )
    ctx.expect_overlap(
        wiper_1,
        link_rod,
        axes="xy",
        elem_a="rocker_eye",
        elem_b="rod_eye_1",
        min_overlap=0.018,
        name="link rod reaches wiper 1 rocker eye",
    )
    ctx.expect_overlap(
        crank,
        link_rod,
        axes="xy",
        elem_a="crank_eye",
        elem_b="rod_crank_stub",
        min_overlap=0.018,
        name="rod end is retained under crank eye",
    )
    ctx.expect_overlap(
        crank,
        link_rod,
        axes="xy",
        elem_a="crank_eye",
        elem_b="rod_eye_stub_1",
        min_overlap=0.018,
        name="second rod takeoff is retained at crank eye",
    )
    ctx.expect_overlap(
        wiper_0,
        link_rod,
        axes="xy",
        elem_a="rocker_eye",
        elem_b="rod_eye_stub_0",
        min_overlap=0.018,
        name="rod bar is retained at wiper 0 rocker",
    )
    ctx.expect_overlap(
        wiper_1,
        link_rod,
        axes="xy",
        elem_a="rocker_eye",
        elem_b="rod_eye_stub_1",
        min_overlap=0.018,
        name="rod bar is retained at wiper 1 rocker",
    )
    ctx.expect_overlap(
        wiper_0,
        "blade_carrier_0",
        axes="xy",
        elem_a="blade_pin",
        elem_b="hinge_socket",
        min_overlap=0.025,
        name="wiper 0 blade socket captures arm pin",
    )
    ctx.expect_overlap(
        wiper_1,
        "blade_carrier_1",
        axes="xy",
        elem_a="blade_pin",
        elem_b="hinge_socket",
        min_overlap=0.025,
        name="wiper 1 blade socket captures arm pin",
    )
    ctx.expect_overlap(
        wiper_0,
        "blade_carrier_0",
        axes="xy",
        elem_a="blade_pin",
        elem_b="yoke_block",
        min_overlap=0.018,
        name="wiper 0 yoke surrounds blade pin",
    )
    ctx.expect_overlap(
        wiper_1,
        "blade_carrier_1",
        axes="xy",
        elem_a="blade_pin",
        elem_b="yoke_block",
        min_overlap=0.018,
        name="wiper 1 yoke surrounds blade pin",
    )
    ctx.expect_overlap(
        wiper_0,
        "blade_carrier_0",
        axes="xy",
        elem_a="blade_pin",
        elem_b="yoke_bridge",
        min_overlap=0.014,
        name="wiper 0 yoke bridge is pinned",
    )
    ctx.expect_overlap(
        wiper_1,
        "blade_carrier_1",
        axes="xy",
        elem_a="blade_pin",
        elem_b="yoke_bridge",
        min_overlap=0.014,
        name="wiper 1 yoke bridge is pinned",
    )
    ctx.check(
        "crank sweep is mechanically limited",
        drive.motion_limits is not None
        and drive.motion_limits.lower is not None
        and drive.motion_limits.upper is not None
        and -0.7 <= drive.motion_limits.lower < 0.0 < drive.motion_limits.upper <= 0.7,
        details=f"limits={drive.motion_limits}",
    )

    rest_aabb = ctx.part_element_world_aabb("blade_carrier_0", elem="rubber_squeegee")
    with ctx.pose({drive: 0.45}):
        ctx.expect_overlap(
            wiper_0,
            link_rod,
            axes="xy",
            elem_a="rocker_eye",
            elem_b="rod_eye_0",
            min_overlap=0.006,
            name="swept linkage stays on wiper 0 rocker",
        )
        ctx.expect_overlap(
            wiper_1,
            link_rod,
            axes="xy",
            elem_a="rocker_eye",
            elem_b="rod_eye_1",
            min_overlap=0.020,
            name="swept linkage stays on wiper 1 rocker",
        )
        swept_aabb = ctx.part_element_world_aabb("blade_carrier_0", elem="rubber_squeegee")

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[0] + hi[0]) * 0.5

    rest_x = _aabb_center_x(rest_aabb)
    swept_x = _aabb_center_x(swept_aabb)
    ctx.check(
        "crank drive sweeps blade carrier",
        rest_x is not None and swept_x is not None and abs(swept_x - rest_x) > 0.025,
        details=f"rest_x={rest_x}, swept_x={swept_x}",
    )

    return ctx.report()


object_model = build_object_model()
