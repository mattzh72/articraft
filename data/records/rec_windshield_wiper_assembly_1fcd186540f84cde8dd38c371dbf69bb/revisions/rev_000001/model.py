from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


DRIVE_PIVOT = (-0.36, 0.035, 0.092)
FOLLOWER_PIVOT = (0.36, 0.035, 0.092)
MOTOR_PIVOT = (0.0, -0.045, 0.092)

CRANK_PIN = (-0.105, 0.0, 0.015)
DRIVE_LINK_PIN = (0.080, -0.055, 0.015)
DRIVE_TIE_PIN = (0.110, 0.025, 0.010)
FOLLOWER_TIE_PIN = (-0.110, 0.025, 0.010)

DRIVE_BLADE_HINGE = (0.110, 0.530, 0.075)
FOLLOWER_BLADE_HINGE = (-0.110, 0.490, 0.075)


def _matte(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba=rgba)


def _rounded_slab(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius, corner_segments=12),
            height,
            cap=True,
            center=True,
        ),
        name,
    )


def _swept_bar(points, width: float, height: float, name: str):
    return mesh_from_geometry(
        sweep_profile_along_spline(
            points,
            profile=rounded_rect_profile(width, height, radius=min(width, height) * 0.45, corner_segments=8),
            samples_per_segment=14,
            cap_profile=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
        name,
    )


def _tube(points, radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=16,
            radial_segments=20,
            cap_ends=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
        name,
    )


def _add_blade_carrier(part, *, length: float, metal: Material, polymer: Material, rubber: Material, prefix: str) -> None:
    part.visual(
        Box((0.060, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=polymer,
        name="blade_adapter",
    )
    part.visual(
        Box((length, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.000, -0.032)),
        material=metal,
        name="blade_spine",
    )
    part.visual(
        Box((length * 0.96, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.000, -0.060)),
        material=rubber,
        name="rubber",
    )
    part.visual(
        Box((0.028, 0.026, 0.038)),
        origin=Origin(xyz=(-length * 0.44, 0.0, -0.042)),
        material=polymer,
        name=f"{prefix}_end_cap_0",
    )
    part.visual(
        Box((0.028, 0.026, 0.038)),
        origin=Origin(xyz=(length * 0.44, 0.0, -0.042)),
        material=polymer,
        name=f"{prefix}_end_cap_1",
    )
    for idx, x in enumerate((-0.17, 0.0, 0.17)):
        part.visual(
            Box((0.034, 0.028, 0.046)),
            origin=Origin(xyz=(x, 0.0, -0.024)),
            material=polymer,
            name=f"{prefix}_claw_{idx}",
        )


def _add_wiper_part(
    part,
    *,
    side: int,
    arm_end: tuple[float, float, float],
    drive_pin: tuple[float, float, float] | None,
    tie_pin: tuple[float, float, float],
    painted: Material,
    polymer: Material,
    bearing_steel: Material,
    prefix: str,
) -> None:
    part.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=bearing_steel,
        name=f"{prefix}_spindle",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=polymer,
        name=f"{prefix}_hub_cap",
    )
    part.visual(
        _swept_bar(
            [
                (0.0, 0.0, 0.048),
                (side * 0.018, 0.170, 0.060),
                (arm_end[0] * 0.82, arm_end[1] * 0.76, arm_end[2] - 0.004),
                arm_end,
            ],
            0.026,
            0.008,
            f"{prefix}_main_arm_mesh",
        ),
        material=painted,
        name=f"{prefix}_main_arm",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=arm_end, rpy=(pi / 2.0, 0.0, 0.0)),
        material=polymer,
        name=f"{prefix}_tip_hinge",
    )
    if drive_pin is not None:
        part.visual(
            _swept_bar(
                [
                    (0.0, 0.0, -0.022),
                    (drive_pin[0] * 0.48, drive_pin[1] * 0.44, -0.004),
                    drive_pin,
                ],
                0.024,
                0.009,
                f"{prefix}_drive_lever_mesh",
            ),
            material=painted,
            name=f"{prefix}_drive_lever",
        )
        part.visual(
            Cylinder(radius=0.011, length=0.030),
            origin=Origin(xyz=drive_pin),
            material=bearing_steel,
            name=f"{prefix}_link_pin",
        )
    part.visual(
        _swept_bar(
            [
                (0.0, 0.0, -0.014),
                (tie_pin[0] * 0.58, tie_pin[1] * 0.56, -0.002),
                tie_pin,
            ],
            0.021,
            0.008,
            f"{prefix}_tie_lever_mesh",
        ),
        material=painted,
        name=f"{prefix}_tie_lever",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=tie_pin),
        material=bearing_steel,
        name=f"{prefix}_tie_pin",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_windshield_wiper_assembly")

    painted = model.material("satin_black_painted_metal", rgba=(0.015, 0.017, 0.018, 1.0))
    warm_metal = model.material("dark_anodized_linkage", rgba=(0.18, 0.18, 0.17, 1.0))
    bearing_steel = model.material("brushed_bearing_steel", rgba=(0.46, 0.45, 0.42, 1.0))
    polymer = model.material("fine_grain_black_polymer", rgba=(0.035, 0.038, 0.040, 1.0))
    rubber = model.material("deep_matte_elastomer", rgba=(0.004, 0.004, 0.004, 1.0))
    seam = model.material("soft_shadow_seam", rgba=(0.0, 0.0, 0.0, 1.0))

    cowl = model.part("cowl")
    cowl.visual(
        _rounded_slab(1.16, 0.155, 0.032, 0.044, "cowl_skin_mesh"),
        origin=Origin(xyz=(0.0, 0.000, 0.026)),
        material=polymer,
        name="cowl_skin",
    )
    cowl.visual(
        Cylinder(radius=0.018, length=1.08),
        origin=Origin(xyz=(0.0, -0.070, 0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=painted,
        name="rear_torsion_tube",
    )
    cowl.visual(
        Cylinder(radius=0.064, length=0.040),
        origin=Origin(xyz=(MOTOR_PIVOT[0], MOTOR_PIVOT[1], 0.060)),
        material=polymer,
        name="motor_bearing",
    )
    cowl.visual(
        Cylinder(radius=0.042, length=0.052),
        origin=Origin(xyz=(DRIVE_PIVOT[0], DRIVE_PIVOT[1], 0.036)),
        material=polymer,
        name="drive_bearing",
    )
    cowl.visual(
        Cylinder(radius=0.042, length=0.052),
        origin=Origin(xyz=(FOLLOWER_PIVOT[0], FOLLOWER_PIVOT[1], 0.036)),
        material=polymer,
        name="follower_bearing",
    )
    for prefix, x in (("drive", DRIVE_PIVOT[0]), ("follower", FOLLOWER_PIVOT[0])):
        cowl.visual(
            Box((0.110, 0.020, 0.030)),
            origin=Origin(xyz=(x, -0.010, 0.050)),
            material=painted,
            name=f"{prefix}_bearing_gusset",
        )
        cowl.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(xyz=(x, 0.035, 0.056)),
            material=bearing_steel,
            name=f"{prefix}_bearing_sleeve",
        )
    cowl.visual(
        Box((1.03, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.061, 0.046)),
        material=seam,
        name="front_seam",
    )
    cowl.visual(
        Box((0.006, 0.122, 0.006)),
        origin=Origin(xyz=(-0.480, -0.004, 0.043)),
        material=seam,
        name="end_seam_0",
    )
    cowl.visual(
        Box((0.006, 0.122, 0.006)),
        origin=Origin(xyz=(0.480, -0.004, 0.043)),
        material=seam,
        name="end_seam_1",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=bearing_steel,
        name="motor_shaft",
    )
    crank.visual(
        Cylinder(radius=0.053, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=warm_metal,
        name="crank_disk",
    )
    crank.visual(
        _swept_bar(
            [(0.0, 0.0, 0.020), (-0.040, 0.002, 0.021), (-0.086, 0.0, 0.018)],
            0.030,
            0.010,
            "crank_arm_mesh",
        ),
        material=warm_metal,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(CRANK_PIN[0], CRANK_PIN[1], 0.012)),
        material=warm_metal,
        name="crank_pin_boss",
    )
    crank.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=CRANK_PIN),
        material=bearing_steel,
        name="crank_pin",
    )
    crank.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.038, 0.010, 0.031)),
        material=warm_metal,
        name="counterweight",
    )

    drive_link = model.part("drive_link")
    drive_link.visual(
        _tube([(0.0, 0.0, 0.024), (-0.060, 0.010, 0.025), (-0.125, 0.020, 0.025), (-0.175, 0.025, 0.024)], 0.0065, "drive_link_rod_mesh"),
        material=warm_metal,
        name="drive_link_rod",
    )
    drive_link.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=bearing_steel,
        name="link_socket_0",
    )
    drive_link.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(-0.175, 0.025, 0.024)),
        material=bearing_steel,
        name="link_socket_1",
    )

    drive_wiper = model.part("drive_wiper")
    _add_wiper_part(
        drive_wiper,
        side=1,
        arm_end=DRIVE_BLADE_HINGE,
        drive_pin=DRIVE_LINK_PIN,
        tie_pin=DRIVE_TIE_PIN,
        painted=painted,
        polymer=polymer,
        bearing_steel=bearing_steel,
        prefix="drive",
    )

    follower_wiper = model.part("follower_wiper")
    _add_wiper_part(
        follower_wiper,
        side=-1,
        arm_end=FOLLOWER_BLADE_HINGE,
        drive_pin=None,
        tie_pin=FOLLOWER_TIE_PIN,
        painted=painted,
        polymer=polymer,
        bearing_steel=bearing_steel,
        prefix="follower",
    )

    tie_link = model.part("tie_link")
    tie_link.visual(
        _tube([(0.0, 0.0, 0.022), (0.165, 0.000, 0.022), (0.335, 0.000, 0.022), (0.500, 0.000, 0.022)], 0.006, "tie_link_rod_mesh"),
        material=warm_metal,
        name="tie_link_rod",
    )
    tie_link.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=bearing_steel,
        name="tie_socket_0",
    )
    tie_link.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.500, 0.0, 0.022)),
        material=bearing_steel,
        name="tie_socket_1",
    )

    drive_blade = model.part("drive_blade")
    _add_blade_carrier(
        drive_blade,
        length=0.500,
        metal=warm_metal,
        polymer=polymer,
        rubber=rubber,
        prefix="drive",
    )

    follower_blade = model.part("follower_blade")
    _add_blade_carrier(
        follower_blade,
        length=0.460,
        metal=warm_metal,
        polymer=polymer,
        rubber=rubber,
        prefix="follower",
    )

    model.articulation(
        "motor_crank",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=crank,
        origin=Origin(xyz=MOTOR_PIVOT),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0, lower=-0.16, upper=0.16),
    )
    model.articulation(
        "crank_link_pin",
        ArticulationType.REVOLUTE,
        parent=crank,
        child=drive_link,
        origin=Origin(xyz=CRANK_PIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.65, upper=0.65),
        mimic=Mimic(joint="motor_crank", multiplier=-2.92, offset=0.0),
    )
    model.articulation(
        "drive_pivot",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=drive_wiper,
        origin=Origin(xyz=DRIVE_PIVOT),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=3.5, lower=-0.68, upper=0.68),
        mimic=Mimic(joint="motor_crank", multiplier=2.50, offset=0.0),
    )
    model.articulation(
        "follower_pivot",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=follower_wiper,
        origin=Origin(xyz=FOLLOWER_PIVOT),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.5, lower=-0.66, upper=0.66),
        mimic=Mimic(joint="motor_crank", multiplier=1.50, offset=0.0),
    )
    model.articulation(
        "tie_pin",
        ArticulationType.REVOLUTE,
        parent=drive_wiper,
        child=tie_link,
        origin=Origin(xyz=DRIVE_TIE_PIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0, lower=-1.00, upper=1.00),
        mimic=Mimic(joint="motor_crank", multiplier=-3.33, offset=0.0),
    )
    model.articulation(
        "drive_blade_hinge",
        ArticulationType.REVOLUTE,
        parent=drive_wiper,
        child=drive_blade,
        origin=Origin(xyz=DRIVE_BLADE_HINGE),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "follower_blade_hinge",
        ArticulationType.REVOLUTE,
        parent=follower_wiper,
        child=follower_blade,
        origin=Origin(xyz=FOLLOWER_BLADE_HINGE),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=-0.18, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cowl = object_model.get_part("cowl")
    crank = object_model.get_part("crank")
    drive_link = object_model.get_part("drive_link")
    drive_wiper = object_model.get_part("drive_wiper")
    follower_wiper = object_model.get_part("follower_wiper")
    tie_link = object_model.get_part("tie_link")
    drive_blade = object_model.get_part("drive_blade")
    follower_blade = object_model.get_part("follower_blade")
    motor_joint = object_model.get_articulation("motor_crank")

    ctx.allow_overlap(
        cowl,
        crank,
        elem_a="motor_bearing",
        elem_b="motor_shaft",
        reason="The motor shaft is intentionally captured inside the supported motor bearing.",
    )
    ctx.expect_within(
        crank,
        cowl,
        axes="xy",
        inner_elem="motor_shaft",
        outer_elem="motor_bearing",
        margin=0.002,
        name="motor shaft is centered in motor bearing",
    )
    ctx.expect_overlap(
        crank,
        cowl,
        axes="z",
        elem_a="motor_shaft",
        elem_b="motor_bearing",
        min_overlap=0.025,
        name="motor shaft has retained bearing engagement",
    )

    for wiper, bearing, spindle, label in (
        (drive_wiper, "drive_bearing", "drive_spindle", "drive"),
        (follower_wiper, "follower_bearing", "follower_spindle", "follower"),
    ):
        ctx.allow_overlap(
            cowl,
            wiper,
            elem_a=bearing,
            elem_b=spindle,
            reason=f"The {label} spindle is intentionally seated in a supported cowl bearing.",
        )
        ctx.expect_within(
            wiper,
            cowl,
            axes="xy",
            inner_elem=spindle,
            outer_elem=bearing,
            margin=0.002,
            name=f"{label} spindle is constrained by its bearing",
        )
        ctx.expect_overlap(
            wiper,
            cowl,
            axes="z",
            elem_a=spindle,
            elem_b=bearing,
            min_overlap=0.018,
            name=f"{label} spindle has bearing insertion",
        )
        ctx.allow_overlap(
            cowl,
            wiper,
            elem_a=f"{label}_bearing_sleeve",
            elem_b=spindle,
            reason=f"The exposed {label} sleeve is a simplified bushing wrapped around the rotating spindle.",
        )
        ctx.expect_overlap(
            cowl,
            wiper,
            axes="z",
            elem_a=f"{label}_bearing_sleeve",
            elem_b=spindle,
            min_overlap=0.020,
            name=f"{label} spindle remains inside exposed sleeve",
        )

    ctx.allow_overlap(
        crank,
        drive_link,
        elem_a="crank_pin",
        elem_b="link_socket_0",
        reason="The drive link spherical socket wraps around the crank pin.",
    )
    ctx.expect_overlap(
        crank,
        drive_link,
        axes="xyz",
        elem_a="crank_pin",
        elem_b="link_socket_0",
        min_overlap=0.006,
        name="crank pin is captured by drive link socket",
    )

    ctx.allow_overlap(
        drive_wiper,
        drive_link,
        elem_a="drive_link_pin",
        elem_b="link_socket_1",
        reason="The drive link far socket is seated over the wiper bellcrank pin.",
    )
    ctx.expect_overlap(
        drive_wiper,
        drive_link,
        axes="xyz",
        elem_a="drive_link_pin",
        elem_b="link_socket_1",
        min_overlap=0.006,
        name="drive link reaches the supported drive bellcrank",
    )

    ctx.allow_overlap(
        drive_wiper,
        tie_link,
        elem_a="drive_tie_pin",
        elem_b="tie_socket_0",
        reason="The synchronizing tie link socket is captured over the drive pivot pin.",
    )
    ctx.allow_overlap(
        follower_wiper,
        tie_link,
        elem_a="follower_tie_pin",
        elem_b="tie_socket_1",
        reason="The synchronizing tie link socket is captured over the follower pivot pin.",
    )
    ctx.expect_overlap(
        drive_wiper,
        tie_link,
        axes="xyz",
        elem_a="drive_tie_pin",
        elem_b="tie_socket_0",
        min_overlap=0.006,
        name="tie link is retained on drive pin",
    )
    ctx.expect_overlap(
        follower_wiper,
        tie_link,
        axes="xyz",
        elem_a="follower_tie_pin",
        elem_b="tie_socket_1",
        min_overlap=0.006,
        name="tie link reaches follower pin",
    )

    for parent, blade, hinge, label in (
        (drive_wiper, drive_blade, "drive_tip_hinge", "drive"),
        (follower_wiper, follower_blade, "follower_tip_hinge", "follower"),
    ):
        ctx.allow_overlap(
            parent,
            blade,
            elem_a=hinge,
            elem_b="blade_adapter",
            reason=f"The {label} blade adapter is locally captured by the arm hinge clevis.",
        )
        ctx.expect_overlap(
            parent,
            blade,
            axes="xyz",
            elem_a=hinge,
            elem_b="blade_adapter",
            min_overlap=0.010,
            name=f"{label} blade adapter is captured at the arm tip",
        )
        ctx.allow_overlap(
            parent,
            blade,
            elem_a=hinge,
            elem_b=f"{label}_claw_1",
            reason=f"The central {label} blade claw wraps locally around the tip hinge knuckle.",
        )
        ctx.expect_overlap(
            parent,
            blade,
            axes="xyz",
            elem_a=hinge,
            elem_b=f"{label}_claw_1",
            min_overlap=0.008,
            name=f"{label} center claw wraps the hinge",
        )
        ctx.allow_overlap(
            parent,
            blade,
            elem_a=f"{label}_main_arm",
            elem_b="blade_adapter",
            reason=f"The {label} blade adapter nests into the formed arm-tip fork.",
        )
        ctx.expect_overlap(
            parent,
            blade,
            axes="xyz",
            elem_a=f"{label}_main_arm",
            elem_b="blade_adapter",
            min_overlap=0.006,
            name=f"{label} adapter is nested in arm fork",
        )
        ctx.allow_overlap(
            parent,
            blade,
            elem_a=f"{label}_main_arm",
            elem_b=f"{label}_claw_1",
            reason=f"The central {label} claw sits under the arm-tip fork as part of the captured blade hinge.",
        )
        ctx.expect_overlap(
            parent,
            blade,
            axes="xyz",
            elem_a=f"{label}_main_arm",
            elem_b=f"{label}_claw_1",
            min_overlap=0.006,
            name=f"{label} central claw is captured by arm fork",
        )

    ctx.check(
        "crank sweep is limited",
        motor_joint.motion_limits is not None
        and motor_joint.motion_limits.lower is not None
        and motor_joint.motion_limits.upper is not None
        and -0.20 <= motor_joint.motion_limits.lower < 0.0
        and 0.0 < motor_joint.motion_limits.upper <= 0.20,
        details=f"limits={motor_joint.motion_limits}",
    )

    def coord(vec, axis_index: int) -> float:
        try:
            return float(vec[axis_index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[axis_index])

    rest_aabb = ctx.part_element_world_aabb(drive_blade, elem="rubber")
    with ctx.pose({motor_joint: 0.14}):
        swept_aabb = ctx.part_element_world_aabb(drive_blade, elem="rubber")
        ctx.expect_overlap(
            crank,
            drive_link,
            axes="xyz",
            elem_a="crank_pin",
            elem_b="link_socket_0",
            min_overlap=0.006,
            name="swept crank pin stays in drive link socket",
        )
        ctx.expect_overlap(
            drive_wiper,
            drive_link,
            axes="xyz",
            elem_a="drive_link_pin",
            elem_b="link_socket_1",
            min_overlap=0.006,
            name="swept drive link remains on bellcrank pin",
        )
        ctx.expect_overlap(
            drive_wiper,
            tie_link,
            axes="xyz",
            elem_a="drive_tie_pin",
            elem_b="tie_socket_0",
            min_overlap=0.006,
            name="swept tie link remains on drive pin",
        )
        ctx.expect_overlap(
            follower_wiper,
            tie_link,
            axes="xyz",
            elem_a="follower_tie_pin",
            elem_b="tie_socket_1",
            min_overlap=0.006,
            name="swept tie link remains on follower pin",
        )

    if rest_aabb is None or swept_aabb is None:
        ctx.fail("drive blade sweeps with crank", "drive blade rubber AABB unavailable")
    else:
        rest_center = (
            (coord(rest_aabb[0], 0) + coord(rest_aabb[1], 0)) * 0.5,
            (coord(rest_aabb[0], 1) + coord(rest_aabb[1], 1)) * 0.5,
        )
        swept_center = (
            (coord(swept_aabb[0], 0) + coord(swept_aabb[1], 0)) * 0.5,
            (coord(swept_aabb[0], 1) + coord(swept_aabb[1], 1)) * 0.5,
        )
        travel = ((swept_center[0] - rest_center[0]) ** 2 + (swept_center[1] - rest_center[1]) ** 2) ** 0.5
        ctx.check(
            "drive blade sweeps with crank",
            travel > 0.10,
            details=f"rest={rest_center}, swept={swept_center}, travel={travel:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
