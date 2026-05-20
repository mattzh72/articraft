from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_profile(radius: float, center: tuple[float, float], segments: int = 40) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (cx + radius * cos(2.0 * pi * i / segments), cy + radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_studio_spotlight_yoke")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    satin_black = model.material("satin_black", rgba=(0.055, 0.058, 0.060, 1.0))
    warm_graphite = model.material("warm_graphite", rgba=(0.12, 0.115, 0.105, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.56, 0.55, 0.50, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.205, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    lens_glass = model.material("smoked_glass", rgba=(0.55, 0.72, 0.86, 0.42))
    reflector = model.material("soft_reflector", rgba=(0.83, 0.80, 0.70, 1.0))

    # Root: heavy floor stand with a real pan bearing stack.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.225, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_black,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.180, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=matte_black,
        name="rubber_underlay",
    )
    for i, (x, y) in enumerate(((0.155, 0.112), (-0.155, 0.112), (0.155, -0.112), (-0.155, -0.112))):
        base.visual(
            Box((0.070, 0.040, 0.010)),
            origin=Origin(xyz=(x, y, 0.008)),
            material=rubber,
            name=f"foot_{i}",
        )
    base.visual(
        Cylinder(radius=0.036, length=0.455),
        origin=Origin(xyz=(0.0, 0.0, 0.2665)),
        material=matte_black,
        name="stand_tube",
    )
    base.visual(
        Cylinder(radius=0.065, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.5185)),
        material=dark_steel,
        name="pan_bearing",
    )
    base.visual(
        Cylinder(radius=0.092, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.553)),
        material=brushed_steel,
        name="pan_race",
    )

    # Pan child: one-piece yoke casting with cheek bores and an exposed turntable.
    yoke = model.part("yoke")
    yoke.visual(
        Box((0.100, 0.340, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=matte_black,
        name="lower_bridge",
    )
    plate_outer = [(-0.044, 0.0), (0.044, 0.0), (0.044, 0.310), (-0.044, 0.310)]
    plate_hole = _circle_profile(0.028, (0.0, 0.225), segments=48)
    for side, y, plate_name in ((0, 0.150, "side_plate_0"), (1, -0.150, "side_plate_1")):
        sign = 1.0 if y > 0 else -1.0
        yoke.visual(
            mesh_from_geometry(
                ExtrudeWithHolesGeometry(plate_outer, [plate_hole], 0.030, center=True),
                f"side_plate_{side}",
            ),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=matte_black,
            name=plate_name,
        )
    yoke.visual(
        Cylinder(radius=0.100, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=satin_black,
        name="turntable",
    )
    yoke.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=brushed_steel,
        name="pan_spindle",
    )
    for side, y in enumerate((0.1695, -0.1695)):
        sign = 1.0 if y > 0 else -1.0
        yoke.visual(
            mesh_from_geometry(TorusGeometry(radius=0.033, tube=0.0045, radial_segments=16, tubular_segments=48), f"bearing_trim_{side}"),
            origin=Origin(xyz=(0.0, y, 0.225), rpy=(sign * pi / 2, 0.0, 0.0)),
            material=brushed_steel,
            name=f"bearing_trim_{side}",
        )
        for j, (x, z) in enumerate(((-0.027, 0.186), (0.027, 0.186), (-0.027, 0.264), (0.027, 0.264))):
            yoke.visual(
                Cylinder(radius=0.006, length=0.005),
                origin=Origin(xyz=(x, sign * 0.1675, z), rpy=(-sign * pi / 2, 0.0, 0.0)),
                material=dark_steel,
                name=f"cheek_screw_{side}_{j}",
            )

    # Tilt child: a cylindrical spotlight can with explicit shell, rings, glass,
    # internal reflector, trunnion spindle and top service handle.
    can = model.part("can")
    can.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile=[
                    (0.090, -0.205),
                    (0.100, -0.175),
                    (0.108, 0.070),
                    (0.118, 0.235),
                    (0.121, 0.260),
                ],
                inner_profile=[
                    (0.082, -0.197),
                    (0.092, -0.167),
                    (0.100, 0.070),
                    (0.110, 0.232),
                    (0.113, 0.253),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            "can_shell",
        ),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=warm_graphite,
        name="can_shell",
    )
    can.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.188, 0.188),
                (0.250, 0.250),
                0.024,
                opening_shape="circle",
                outer_shape="circle",
                center=True,
            ),
            "front_bezel",
        ),
        origin=Origin(xyz=(0.258, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=satin_black,
        name="front_bezel",
    )
    can.visual(
        Cylinder(radius=0.096, length=0.010),
        origin=Origin(xyz=(0.249, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=lens_glass,
        name="lens",
    )
    for i, r in enumerate((0.030, 0.055, 0.078)):
        can.visual(
            mesh_from_geometry(TorusGeometry(radius=r, tube=0.0009, radial_segments=8, tubular_segments=48), f"fresnel_ring_{i}"),
            origin=Origin(xyz=(0.251, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
            material=Material("fresnel_highlight", rgba=(0.80, 0.92, 1.0, 0.35)),
            name=f"fresnel_ring_{i}",
        )
    can.visual(
        Cylinder(radius=0.091, length=0.012),
        origin=Origin(xyz=(-0.210, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=satin_black,
        name="rear_cap",
    )
    for i, z in enumerate((-0.040, -0.020, 0.000, 0.020, 0.040)):
        can.visual(
            Box((0.006, 0.105, 0.006)),
            origin=Origin(xyz=(-0.217, 0.0, z)),
            material=matte_black,
            name=f"rear_slot_{i}",
        )
    can.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile=[
                    (0.028, -0.145),
                    (0.046, -0.070),
                    (0.074, 0.085),
                    (0.089, 0.205),
                ],
                inner_profile=[
                    (0.018, -0.138),
                    (0.033, -0.065),
                    (0.060, 0.085),
                    (0.077, 0.198),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            "reflector_bowl",
        ),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=reflector,
        name="reflector_bowl",
    )
    can.visual(
        Cylinder(radius=0.022, length=0.076),
        origin=Origin(xyz=(-0.177, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=dark_steel,
        name="lamp_socket",
    )
    for i, (x, r) in enumerate(((-0.190, 0.096), (0.238, 0.119))):
        can.visual(
            mesh_from_geometry(TorusGeometry(radius=r, tube=0.0028, radial_segments=10, tubular_segments=72), f"seam_band_{i}"),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
            material=brushed_steel,
            name=f"seam_band_{i}",
        )
    can.visual(
        Cylinder(radius=0.0223, length=0.326),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=brushed_steel,
        name="tilt_spindle",
    )
    for side, y, hub_name in ((0, 0.127, "side_hub_0"), (1, -0.127, "side_hub_1")):
        sign = 1.0 if y > 0 else -1.0
        can.visual(
            Cylinder(radius=0.037, length=0.016),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-sign * pi / 2, 0.0, 0.0)),
            material=satin_black,
            name=hub_name,
        )
    for i, x in enumerate((-0.075, 0.105)):
        can.visual(
            Box((0.026, 0.040, 0.076)),
            origin=Origin(xyz=(x, 0.0, 0.143)),
            material=satin_black,
            name=f"handle_post_{i}",
        )
    can.visual(
        Cylinder(radius=0.014, length=0.205),
        origin=Origin(xyz=(0.015, 0.0, 0.180), rpy=(0.0, pi / 2, 0.0)),
        material=dark_steel,
        name="top_handle",
    )

    # User-facing yoke clamp knobs rotate independently on the cheek collars.
    for side, y in enumerate((0.166, -0.166)):
        sign = 1.0 if y > 0 else -1.0
        knob = model.part(f"clamp_knob_{side}")
        knob.visual(
            mesh_from_geometry(
                KnobGeometry(
                    0.052,
                    0.026,
                    body_style="lobed",
                    top_diameter=0.045,
                    crown_radius=0.0015,
                    grip=KnobGrip(style="ribbed", count=12, depth=0.0010, width=0.0020),
                    center=False,
                ),
                f"clamp_knob_{side}",
            ),
            origin=Origin(rpy=(-sign * pi / 2, 0.0, 0.0)),
            material=satin_black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.0, sign * 0.001, 0.0), rpy=(-sign * pi / 2, 0.0, 0.0)),
            material=brushed_steel,
            name="knob_stem",
        )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-pi, upper=pi),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.0, lower=-0.70, upper=0.95),
    )
    for side, y in enumerate((0.166, -0.166)):
        sign = 1.0 if y > 0 else -1.0
        model.articulation(
            f"clamp_spin_{side}",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=f"clamp_knob_{side}",
            origin=Origin(xyz=(0.0, y, 0.225)),
            axis=(0.0, sign, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=3.5, lower=-pi, upper=pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.allow_overlap(
        base,
        yoke,
        elem_a="pan_bearing",
        elem_b="pan_spindle",
        reason="The yoke's pan spindle is intentionally captured inside the stand bearing collar.",
    )
    ctx.allow_overlap(
        base,
        yoke,
        elem_a="pan_race",
        elem_b="pan_spindle",
        reason="The spindle passes through the upper pan race as part of the rotary bearing stack.",
    )
    for side, plate_name in ((0, "side_plate_0"), (1, "side_plate_1")):
        knob = object_model.get_part(f"clamp_knob_{side}")
        ctx.allow_overlap(
            can,
            yoke,
            elem_a="tilt_spindle",
            elem_b=plate_name,
            reason="The tilt spindle intentionally passes through the yoke cheek bore.",
        )
        ctx.allow_overlap(
            knob,
            yoke,
            elem_a="knob_stem",
            elem_b=plate_name,
            reason="The clamp screw stem passes through the cheek bore to lock the tilt spindle.",
        )
        ctx.allow_overlap(
            can,
            knob,
            elem_a="tilt_spindle",
            elem_b="knob_stem",
            reason="The clamp knob stem threads into the end of the shared tilt spindle.",
        )
        ctx.expect_overlap(
            knob,
            can,
            axes="xz",
            elem_a="knob_stem",
            elem_b="tilt_spindle",
            min_overlap=0.015,
            name=f"clamp knob {side} stem aligns with tilt spindle",
        )
        ctx.expect_overlap(
            knob,
            can,
            axes="y",
            elem_a="knob_stem",
            elem_b="tilt_spindle",
            min_overlap=0.004,
            name=f"clamp knob {side} stem is threaded into spindle",
        )
        ctx.expect_overlap(
            knob,
            yoke,
            axes="y",
            elem_a="knob_stem",
            elem_b=plate_name,
            min_overlap=0.006,
            name=f"clamp knob {side} stem passes through cheek",
        )
    ctx.expect_within(
        yoke,
        base,
        axes="xy",
        inner_elem="pan_spindle",
        outer_elem="pan_bearing",
        margin=0.002,
        name="pan spindle is centered in bearing",
    )
    ctx.expect_overlap(
        yoke,
        base,
        axes="z",
        elem_a="pan_spindle",
        elem_b="pan_bearing",
        min_overlap=0.020,
        name="pan spindle remains seated in bearing",
    )

    ctx.expect_overlap(
        can,
        yoke,
        axes="y",
        elem_a="tilt_spindle",
        elem_b="side_plate_0",
        min_overlap=0.025,
        name="tilt spindle passes through side bracket 0",
    )
    ctx.expect_within(
        can,
        yoke,
        axes="xz",
        inner_elem="tilt_spindle",
        outer_elem="side_plate_0",
        margin=0.060,
        name="tilt spindle aligns with bracket bore 0",
    )
    ctx.expect_overlap(
        can,
        yoke,
        axes="y",
        elem_a="tilt_spindle",
        elem_b="side_plate_1",
        min_overlap=0.025,
        name="tilt spindle passes through side bracket 1",
    )
    ctx.expect_within(
        can,
        yoke,
        axes="xz",
        inner_elem="tilt_spindle",
        outer_elem="side_plate_1",
        margin=0.060,
        name="tilt spindle aligns with bracket bore 1",
    )
    ctx.expect_contact(
        can,
        yoke,
        elem_a="side_hub_0",
        elem_b="side_plate_0",
        contact_tol=0.001,
        name="positive side hub seats against yoke cheek",
    )
    ctx.expect_contact(
        can,
        yoke,
        elem_a="side_hub_1",
        elem_b="side_plate_1",
        contact_tol=0.001,
        name="negative side hub seats against yoke cheek",
    )

    rest_front = ctx.part_element_world_aabb(can, elem="front_bezel")
    with ctx.pose({tilt: 0.55}):
        raised_front = ctx.part_element_world_aabb(can, elem="front_bezel")
    ctx.check(
        "positive tilt raises the spotlight face",
        rest_front is not None
        and raised_front is not None
        and raised_front[0][2] > rest_front[0][2] + 0.020,
        details=f"rest={rest_front}, raised={raised_front}",
    )

    rest_yoke = ctx.part_world_position(yoke)
    with ctx.pose({pan: 0.75}):
        panned_yoke = ctx.part_world_position(yoke)
    ctx.check(
        "pan stage rotates on fixed stand hub",
        rest_yoke is not None
        and panned_yoke is not None
        and abs(panned_yoke[2] - rest_yoke[2]) < 0.001,
        details=f"rest={rest_yoke}, panned={panned_yoke}",
    )

    return ctx.report()


object_model = build_object_model()
