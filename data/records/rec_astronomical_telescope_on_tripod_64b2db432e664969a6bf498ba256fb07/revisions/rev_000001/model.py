from __future__ import annotations

from math import acos, atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _origin_from_z_axis(
    direction: tuple[float, float, float],
    *,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    """Orient local +Z onto a world/local direction using Rz(yaw)*Ry(pitch)."""

    vx, vy, vz = direction
    length = sqrt(vx * vx + vy * vy + vz * vz)
    ux, uy, uz = vx / length, vy / length, vz / length
    return Origin(xyz=xyz, rpy=(0.0, acos(max(-1.0, min(1.0, uz))), atan2(uy, ux)))


def _point_along(
    start: tuple[float, float, float],
    direction: tuple[float, float, float],
    distance: float,
) -> tuple[float, float, float]:
    return (
        start[0] + direction[0] * distance,
        start[1] + direction[1] * distance,
        start[2] + direction[2] * distance,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="astrograph_refractor_equatorial_mount")

    anodized_black = Material("anodized_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = Material("satin_black", rgba=(0.02, 0.022, 0.025, 1.0))
    machined_gray = Material("machined_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    dark_gray = Material("dark_gray", rgba=(0.12, 0.13, 0.14, 1.0))
    white_tube = Material("white_powdercoat", rgba=(0.92, 0.93, 0.90, 1.0))
    blue_glass = Material("blue_coated_glass", rgba=(0.18, 0.36, 0.58, 0.62))
    rubber = Material("matte_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    brass = Material("brass_bushings", rgba=(0.77, 0.55, 0.23, 1.0))
    red_anodized = Material("red_anodized", rgba=(0.65, 0.07, 0.04, 1.0))
    for mat in (
        anodized_black,
        satin_black,
        machined_gray,
        dark_gray,
        white_tube,
        blue_glass,
        rubber,
        brass,
        red_anodized,
    ):
        model.materials.append(mat)

    # Root: tripod crown, fixed upper leg sleeves, and the stationary wedge yoke.
    base = model.part("tripod_base")
    base.visual(
        Cylinder(radius=0.13, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=machined_gray,
        name="tripod_crown",
    )
    base.visual(
        Cylinder(radius=0.17, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.752)),
        material=anodized_black,
        name="top_plate",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        material=machined_gray,
        name="wedge_post",
    )
    base.visual(
        Box((0.22, 0.36, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.805)),
        material=anodized_black,
        name="latitude_yoke_base",
    )
    for side, y in enumerate((-0.17, 0.17)):
        base.visual(
            Box((0.13, 0.030, 0.22)),
            origin=Origin(xyz=(0.0, y, 0.905)),
            material=anodized_black,
            name=f"latitude_cheek_{side}",
        )

    leg_dirs: list[tuple[float, float, float]] = []
    leg_joints: list[str] = []
    for i, yaw in enumerate((pi / 2.0, pi / 2.0 + 2.0 * pi / 3.0, pi / 2.0 + 4.0 * pi / 3.0)):
        raw = (0.50 * cos(yaw), 0.50 * sin(yaw), -0.72)
        norm = sqrt(raw[0] ** 2 + raw[1] ** 2 + raw[2] ** 2)
        direction = (raw[0] / norm, raw[1] / norm, raw[2] / norm)
        leg_dirs.append(direction)
        anchor = (0.105 * cos(yaw), 0.105 * sin(yaw), 0.695)
        sleeve_origin = _origin_from_z_axis(
            direction,
            xyz=_point_along(anchor, direction, 0.18),
        )
        base.visual(
            Cylinder(radius=0.028, length=0.38),
            origin=sleeve_origin,
            material=dark_gray,
            name=f"upper_sleeve_{i}",
        )
        # A short cast lug makes each sleeve visibly grow out of the crown.
        base.visual(
            Box((0.050, 0.050, 0.060)),
            origin=Origin(xyz=anchor),
            material=machined_gray,
            name=f"leg_socket_{i}",
        )

        lower_leg = model.part(f"lower_leg_{i}")
        lower_leg.visual(
            Cylinder(radius=0.018, length=0.78),
            origin=Origin(xyz=(0.0, 0.0, 0.31)),
            material=machined_gray,
            name="lower_tube",
        )
        lower_leg.visual(
            Cylinder(radius=0.050, length=0.035),
            origin=Origin(xyz=(0.0, 0.0, 0.70)),
            material=rubber,
            name="rubber_foot",
        )
        joint_origin = _origin_from_z_axis(
            direction,
            xyz=_point_along(anchor, direction, 0.13),
        )
        leg_joint = model.articulation(
            f"leg_extension_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=lower_leg,
            origin=joint_origin,
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=120.0, velocity=0.15, lower=0.0, upper=0.25),
        )
        leg_joints.append(leg_joint.name)

    # Wedge head: an inclined plate that tilts about the latitude pin.
    wedge_head = model.part("wedge_head")
    wedge_head.visual(
        Cylinder(radius=0.019, length=0.42),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="latitude_pin",
    )
    wedge_head.visual(
        Box((0.18, 0.20, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=anodized_black,
        name="pivot_boss",
    )
    wedge_head.visual(
        Box((0.28, 0.24, 0.050)),
        origin=Origin(xyz=(0.05, 0.0, 0.085)),
        material=anodized_black,
        name="tilt_plate",
    )
    wedge_head.visual(
        Cylinder(radius=0.090, length=0.060),
        origin=Origin(xyz=(0.10, 0.0, 0.140)),
        material=machined_gray,
        name="ra_bearing_housing",
    )
    wedge_head.visual(
        Box((0.080, 0.060, 0.055)),
        origin=Origin(xyz=(-0.035, 0.105, 0.105)),
        material=satin_black,
        name="drive_motor",
    )
    wedge_head.visual(
        Box((0.180, 0.014, 0.026)),
        origin=Origin(xyz=(0.030, 0.132, 0.125)),
        material=rubber,
        name="belt_run_upper",
    )
    wedge_head.visual(
        Box((0.180, 0.014, 0.026)),
        origin=Origin(xyz=(0.030, 0.132, 0.070)),
        material=rubber,
        name="belt_run_lower",
    )

    latitude_joint = model.articulation(
        "latitude_adjust",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wedge_head,
        origin=Origin(xyz=(0.0, 0.0, 0.905), rpy=(0.0, -0.58, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=-0.35, upper=0.35),
    )

    # The RA axis spins continuously about the inclined polar axis.
    ra_axis = model.part("ra_axis")
    ra_axis.visual(
        Cylinder(radius=0.052, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=machined_gray,
        name="polar_shaft",
    )
    ra_axis.visual(
        Cylinder(radius=0.105, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=red_anodized,
        name="ra_pulley",
    )
    ra_axis.visual(
        Box((0.075, 0.070, 0.130)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=machined_gray,
        name="saddle_stalk",
    )
    ra_axis.visual(
        Box((0.64, 0.105, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=anodized_black,
        name="dovetail_saddle_plate",
    )
    for side, y in enumerate((-0.060, 0.060)):
        ra_axis.visual(
            Box((0.62, 0.018, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.260)),
            material=machined_gray,
            name=f"saddle_jaw_{side}",
        )
    ra_axis.visual(
        Box((0.090, 0.030, 0.050)),
        origin=Origin(xyz=(-0.20, -0.083, 0.258)),
        material=machined_gray,
        name="knob_boss",
    )

    model.articulation(
        "ra_rotation",
        ArticulationType.CONTINUOUS,
        parent=wedge_head,
        child=ra_axis,
        origin=Origin(xyz=(0.10, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.08),
    )

    # The imaging refractor is clipped into the saddle by a Vixen-style dovetail rail.
    refractor = model.part("refractor")
    refractor.visual(
        Box((0.58, 0.046, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=red_anodized,
        name="dovetail_rail",
    )
    for x in (-0.19, 0.19):
        refractor.visual(
            Box((0.055, 0.090, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=machined_gray,
            name=f"tube_saddle_{0 if x < 0 else 1}",
        )
        refractor.visual(
            Cylinder(radius=0.079, length=0.040),
            origin=Origin(xyz=(x, 0.0, 0.132), rpy=(0.0, pi / 2.0, 0.0)),
            material=anodized_black,
            name=f"tube_ring_{0 if x < 0 else 1}",
        )
    refractor.visual(
        Cylinder(radius=0.064, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.132), rpy=(0.0, pi / 2.0, 0.0)),
        material=white_tube,
        name="optical_tube",
    )
    refractor.visual(
        Cylinder(radius=0.078, length=0.185),
        origin=Origin(xyz=(0.405, 0.0, 0.132), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="dew_shield",
    )
    refractor.visual(
        Cylinder(radius=0.058, length=0.010),
        origin=Origin(xyz=(0.498, 0.0, 0.132), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue_glass,
        name="front_lens",
    )
    refractor.visual(
        Cylinder(radius=0.047, length=0.105),
        origin=Origin(xyz=(-0.410, 0.0, 0.132), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="focuser_drawtube",
    )
    refractor.visual(
        Box((0.085, 0.105, 0.080)),
        origin=Origin(xyz=(-0.500, 0.0, 0.132)),
        material=dark_gray,
        name="imaging_camera",
    )
    refractor.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(-0.445, 0.060, 0.132)),
        material=machined_gray,
        name="focus_knob_0",
    )
    refractor.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(-0.445, -0.060, 0.132)),
        material=machined_gray,
        name="focus_knob_1",
    )

    model.articulation(
        "saddle_to_tube",
        ArticulationType.FIXED,
        parent=ra_axis,
        child=refractor,
        origin=Origin(xyz=(0.0, 0.0, 0.2525)),
    )

    # Revolute locking knob on the saddle clamp screw.
    saddle_knob = model.part("saddle_knob")
    saddle_knob.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=machined_gray,
        name="clamp_screw",
    )
    saddle_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.070,
                0.040,
                body_style="lobed",
                base_diameter=0.050,
                top_diameter=0.066,
                crown_radius=0.002,
            ),
            "saddle_lock_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=anodized_black,
        name="lobed_knob",
    )
    model.articulation(
        "saddle_lock",
        ArticulationType.REVOLUTE,
        parent=ra_axis,
        child=saddle_knob,
        origin=Origin(xyz=(-0.20, -0.098, 0.258), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=2.0 * pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("tripod_base")
    wedge = object_model.get_part("wedge_head")
    ra_axis = object_model.get_part("ra_axis")

    ctx.check(
        "primary articulations present",
        all(
            object_model.get_articulation(name) is not None
            for name in (
                "latitude_adjust",
                "ra_rotation",
                "saddle_lock",
                "leg_extension_0",
                "leg_extension_1",
                "leg_extension_2",
            )
        ),
        details="expected latitude, RA, saddle lock, and three leg-extension joints",
    )

    # Captured pins/shafts and telescoping legs intentionally use local hidden
    # insertion, rather than a visual gap, so the mechanisms read mechanically real.
    for i in range(3):
        leg = object_model.get_part(f"lower_leg_{i}")
        joint = object_model.get_articulation(f"leg_extension_{i}")
        ctx.allow_overlap(
            base,
            leg,
            elem_a=f"upper_sleeve_{i}",
            elem_b="lower_tube",
            reason="The lower tripod tube is intentionally retained inside its upper sleeve.",
        )
        ctx.expect_overlap(
            leg,
            base,
            axes="xyz",
            elem_a="lower_tube",
            elem_b=f"upper_sleeve_{i}",
            min_overlap=0.02,
            name=f"leg_{i} is inserted in sleeve at rest",
        )
        with ctx.pose({joint: 0.25}):
            ctx.expect_overlap(
                leg,
                base,
                axes="xyz",
                elem_a="lower_tube",
                elem_b=f"upper_sleeve_{i}",
                min_overlap=0.015,
                name=f"leg_{i} retains insertion at full extension",
            )

    for cheek in ("latitude_cheek_0", "latitude_cheek_1"):
        ctx.allow_overlap(
            base,
            wedge,
            elem_a=cheek,
            elem_b="latitude_pin",
            reason="The latitude hinge pin is captured through the wedge-yoke cheeks.",
        )
        ctx.expect_overlap(
            wedge,
            base,
            axes="xyz",
            elem_a="latitude_pin",
            elem_b=cheek,
            min_overlap=0.006,
            name=f"latitude pin passes through {cheek}",
        )

    ctx.allow_overlap(
        wedge,
        ra_axis,
        elem_a="ra_bearing_housing",
        elem_b="polar_shaft",
        reason="The RA shaft is intentionally captured inside the polar bearing housing.",
    )
    ctx.expect_overlap(
        ra_axis,
        wedge,
        axes="xyz",
        elem_a="polar_shaft",
        elem_b="ra_bearing_housing",
        min_overlap=0.02,
        name="RA shaft is seated in polar bearing",
    )

    refractor = object_model.get_part("refractor")
    ctx.allow_overlap(
        ra_axis,
        refractor,
        elem_a="dovetail_saddle_plate",
        elem_b="dovetail_rail",
        reason="The dovetail rail is represented as seated in the saddle plate's clamp channel.",
    )
    ctx.expect_within(
        refractor,
        ra_axis,
        axes="xy",
        inner_elem="dovetail_rail",
        outer_elem="dovetail_saddle_plate",
        margin=0.02,
        name="dovetail rail is captured inside saddle footprint",
    )
    ctx.expect_overlap(
        refractor,
        ra_axis,
        axes="xy",
        elem_a="dovetail_rail",
        elem_b="dovetail_saddle_plate",
        min_overlap=0.04,
        name="dovetail rail bears along saddle length",
    )

    for i in range(3):
        leg = object_model.get_part(f"lower_leg_{i}")
        joint = object_model.get_articulation(f"leg_extension_{i}")
        rest = ctx.part_world_position(leg)
        with ctx.pose({joint: 0.25}):
            extended = ctx.part_world_position(leg)
        ctx.check(
            f"leg_{i} extends downward",
            rest is not None and extended is not None and extended[2] < rest[2] - 0.15,
            details=f"rest={rest}, extended={extended}",
        )

    return ctx.report()


object_model = build_object_model()
