from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _x_axis_shell(outer_profile, inner_profile, name: str, *, segments: int = 96):
    """Build a thin revolved shell whose longitudinal axis is world/local +X."""
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        lip_samples=8,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _x_axis_lathe(profile, name: str, *, segments: int = 80):
    geom = LatheGeometry(profile, segments=segments, closed=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _x_axis_torus(major_radius: float, tube_radius: float, name: str):
    geom = TorusGeometry(major_radius, tube_radius, radial_segments=18, tubular_segments=96)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_turbofan_nacelle")

    nacelle_white = model.material("warm_white_composite", rgba=(0.86, 0.88, 0.86, 1.0))
    graphite = model.material("dark_graphite_liner", rgba=(0.025, 0.027, 0.030, 1.0))
    titanium = model.material("dark_titanium_fan", rgba=(0.18, 0.19, 0.20, 1.0))
    polished = model.material("brushed_spinner", rgba=(0.72, 0.74, 0.73, 1.0))
    stand_paint = model.material("black_steel_stand", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("matte_rubber_pads", rgba=(0.005, 0.005, 0.006, 1.0))
    sleeve_gray = model.material("sliding_sleeve_gray", rgba=(0.64, 0.67, 0.68, 1.0))
    seam_dark = model.material("dark_panel_seams", rgba=(0.035, 0.037, 0.04, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((2.20, 0.92, 0.060)),
        origin=Origin(xyz=(0.30, 0.0, -0.965)),
        material=stand_paint,
        name="base_plate",
    )
    for y in (-0.38, 0.38):
        stand.visual(
            Box((2.05, 0.075, 0.065)),
            origin=Origin(xyz=(0.30, y, -0.915)),
            material=stand_paint,
            name=f"side_rail_{0 if y < 0 else 1}",
        )
    for x in (-0.38, 0.55, 1.42):
        stand.visual(
            Box((0.095, 0.86, 0.055)),
            origin=Origin(xyz=(x, 0.0, -0.875)),
            material=stand_paint,
            name=f"cross_tie_{x:.1f}",
        )
    for label, x, top_z, length in [("front", -0.38, -0.575, 0.405), ("rear", 1.42, -0.375, 0.565)]:
        for y in (-0.26, 0.26):
            stand.visual(
                Cylinder(radius=0.032, length=length),
                origin=Origin(xyz=(x, y, top_z - length / 2.0)),
                material=stand_paint,
                name=f"{label}_post_{0 if y < 0 else 1}",
            )
        stand.visual(
            Box((0.15, 0.62 if x < 0.0 else 0.48, 0.050)),
            origin=Origin(xyz=(x, 0.0, top_z + 0.010)),
            material=stand_paint,
            name=f"{label}_saddle_beam",
        )
        pad_height = 0.070 if x < 0.0 else 0.030
        pad_center_z = top_z + (0.063 if x < 0.0 else 0.018)
        stand.visual(
            Box((0.18, 0.38 if x < 0.0 else 0.28, pad_height)),
            origin=Origin(xyz=(x, 0.0, pad_center_z)),
            material=rubber,
            name="front_rubber_pad" if label == "front" else "rear_rubber_pad",
        )

    nacelle = model.part("nacelle")
    nacelle.visual(
        _x_axis_shell(
            [
                (0.480, -0.780),
                (0.520, -0.650),
                (0.505, -0.430),
                (0.455, -0.100),
                (0.372, 0.250),
                (0.340, 1.050),
                (0.305, 1.350),
                (0.245, 1.580),
            ],
            [
                (0.360, -0.780),
                (0.347, -0.560),
                (0.318, -0.160),
                (0.292, 0.520),
                (0.258, 1.180),
                (0.193, 1.580),
            ],
            "nacelle_shell",
        ),
        material=nacelle_white,
        name="intake_shell",
    )
    nacelle.visual(
        _x_axis_shell(
            [
                (0.365, -0.750),
                (0.350, -0.540),
                (0.322, -0.150),
                (0.295, 0.520),
                (0.262, 1.180),
                (0.198, 1.575),
            ],
            [
                (0.325, -0.750),
                (0.300, -0.200),
                (0.270, 0.650),
                (0.230, 1.260),
                (0.150, 1.575),
            ],
            "dark_inner_duct",
        ),
        material=graphite,
        name="inner_duct",
    )
    nacelle.visual(
        _x_axis_shell(
            [(0.386, 0.115), (0.389, 0.350)],
            [(0.372, 0.115), (0.374, 0.350)],
            "cascade_ring",
            segments=80,
        ),
        material=graphite,
        name="cascade_ring",
    )
    nacelle.visual(
        _x_axis_shell(
            [(0.268, 1.075), (0.242, 1.340), (0.202, 1.610)],
            [(0.190, 1.075), (0.165, 1.340), (0.130, 1.610)],
            "exhaust_nozzle",
            segments=96,
        ),
        material=graphite,
        name="exhaust_nozzle",
    )
    nacelle.visual(
        Cylinder(radius=0.048, length=0.120),
        origin=Origin(xyz=(-0.430, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="bearing_boss",
    )
    nacelle.visual(
        Box((0.040, 0.720, 0.020)),
        origin=Origin(xyz=(-0.355, 0.0, 0.0)),
        material=graphite,
        name="stator_strut_y",
    )
    nacelle.visual(
        Box((0.040, 0.020, 0.720)),
        origin=Origin(xyz=(-0.355, 0.0, 0.0)),
        material=graphite,
        name="stator_strut_z",
    )
    nacelle.visual(
        Box((0.960, 0.075, 0.030)),
        origin=Origin(xyz=(0.570, 0.0, -0.485)),
        material=graphite,
        name="slider_track",
    )
    nacelle.visual(
        Box((0.095, 0.060, 0.170)),
        origin=Origin(xyz=(0.130, 0.0, -0.425)),
        material=graphite,
        name="track_pylon",
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.312,
                0.090,
                18,
                thickness=0.050,
                blade_pitch_deg=34.0,
                blade_sweep_deg=32.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=18.0, camber=0.18, tip_clearance=0.012),
                hub=FanRotorHub(style="flat", rear_collar_height=0.018, rear_collar_radius=0.085, bore_diameter=0.022),
            ),
            "fan_stage",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="fan_stage",
    )
    fan_rotor.visual(
        _x_axis_lathe(
            [
                (0.000, -0.105),
                (0.040, -0.083),
                (0.088, -0.018),
                (0.082, 0.040),
                (0.000, 0.040),
            ],
            "spinner_cone",
        ),
        material=polished,
        name="spinner_cone",
    )
    fan_rotor.visual(
        Cylinder(radius=0.021, length=0.150),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="rotor_shaft",
    )

    sleeve = model.part("reverser_sleeve")
    sleeve.visual(
        _x_axis_shell(
            [(0.430, 0.175), (0.438, 0.320), (0.424, 0.980), (0.394, 1.100)],
            [(0.405, 0.175), (0.407, 0.320), (0.382, 0.980), (0.352, 1.100)],
            "reverser_sleeve_shell",
            segments=96,
        ),
        material=sleeve_gray,
        name="sleeve_shell",
    )
    for x, radius in ((0.185, 0.430), (0.335, 0.436), (0.965, 0.424), (1.090, 0.394)):
        sleeve.visual(
            _x_axis_torus(radius, 0.0060, f"sleeve_seam_{x:.2f}"),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=seam_dark,
            name=f"sleeve_seam_{x:.2f}",
        )
    for y in (-0.300, 0.300):
        sleeve.visual(
            Box((0.72, 0.018, 0.026)),
            origin=Origin(xyz=(0.610, y, -0.322)),
            material=seam_dark,
            name=f"actuator_fairing_{0 if y < 0 else 1}",
        )
    sleeve.visual(
        Box((0.760, 0.130, 0.060)),
        origin=Origin(xyz=(0.620, 0.0, -0.510)),
        material=seam_dark,
        name="slider_shoe",
    )
    for y in (-0.058, 0.058):
        sleeve.visual(
            Box((0.700, 0.028, 0.115)),
            origin=Origin(xyz=(0.620, y, -0.462)),
            material=seam_dark,
            name=f"shoe_hanger_{0 if y < 0 else 1}",
        )

    model.articulation(
        "stand_to_nacelle",
        ArticulationType.FIXED,
        parent=stand,
        child=nacelle,
        origin=Origin(),
    )
    model.articulation(
        "nacelle_to_fan",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan_rotor,
        origin=Origin(xyz=(-0.570, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=120.0),
    )
    model.articulation(
        "nacelle_to_sleeve",
        ArticulationType.PRISMATIC,
        parent=nacelle,
        child=sleeve,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.300),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    fan = object_model.get_part("fan_rotor")
    sleeve = object_model.get_part("reverser_sleeve")
    stand = object_model.get_part("stand")
    fan_joint = object_model.get_articulation("nacelle_to_fan")
    sleeve_joint = object_model.get_articulation("nacelle_to_sleeve")

    ctx.allow_overlap(
        stand,
        nacelle,
        elem_a="front_rubber_pad",
        elem_b="intake_shell",
        reason="The front rubber saddle pad is modeled with slight local compression against the nacelle underside.",
    )
    ctx.allow_overlap(
        nacelle,
        fan,
        elem_a="bearing_boss",
        elem_b="rotor_shaft",
        reason="The rotating fan shaft is intentionally captured inside the stationary front bearing boss.",
    )
    ctx.allow_overlap(
        nacelle,
        sleeve,
        elem_a="slider_track",
        elem_b="slider_shoe",
        reason="The thrust-reverser sleeve shoe is intentionally represented as sliding on the fixed nacelle track.",
    )
    ctx.expect_gap(
        nacelle,
        stand,
        axis="z",
        positive_elem="intake_shell",
        negative_elem="front_rubber_pad",
        max_penetration=0.050,
        name="front rubber saddle has only local compression",
    )
    ctx.expect_overlap(
        stand,
        nacelle,
        axes="xy",
        elem_a="front_rubber_pad",
        elem_b="intake_shell",
        min_overlap=0.050,
        name="front saddle is directly under nacelle",
    )
    ctx.expect_within(
        fan,
        nacelle,
        axes="yz",
        inner_elem="rotor_shaft",
        outer_elem="bearing_boss",
        margin=0.0,
        name="rotor shaft is concentric in bearing boss",
    )
    ctx.expect_overlap(
        fan,
        nacelle,
        axes="x",
        elem_a="rotor_shaft",
        elem_b="bearing_boss",
        min_overlap=0.060,
        name="rotor shaft remains captured by bearing boss",
    )
    ctx.expect_overlap(
        sleeve,
        nacelle,
        axes="x",
        elem_a="slider_shoe",
        elem_b="slider_track",
        min_overlap=0.60,
        name="closed sleeve shoe is retained on track",
    )
    ctx.check(
        "front fan uses continuous spin joint",
        fan_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(fan_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={fan_joint.articulation_type}, axis={fan_joint.axis}",
    )
    ctx.check(
        "reverser sleeve uses aft prismatic travel",
        sleeve_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(sleeve_joint.axis) == (1.0, 0.0, 0.0)
        and sleeve_joint.motion_limits is not None
        and sleeve_joint.motion_limits.upper >= 0.28,
        details=f"type={sleeve_joint.articulation_type}, axis={sleeve_joint.axis}, limits={sleeve_joint.motion_limits}",
    )
    fan_pos = ctx.part_world_position(fan)
    ctx.check(
        "fan rotor centered on nacelle axis",
        fan_pos is not None and abs(fan_pos[1]) < 1e-6 and abs(fan_pos[2]) < 1e-6,
        details=f"fan_pos={fan_pos}",
    )
    ctx.expect_within(
        fan,
        nacelle,
        axes="yz",
        inner_elem="fan_stage",
        outer_elem="intake_shell",
        margin=0.0,
        name="fan disk fits within intake annulus",
    )
    ctx.expect_overlap(
        fan,
        nacelle,
        axes="x",
        elem_a="fan_stage",
        elem_b="intake_shell",
        min_overlap=0.040,
        name="fan stage sits inside intake length",
    )
    ctx.expect_overlap(
        sleeve,
        nacelle,
        axes="x",
        elem_a="sleeve_shell",
        elem_b="intake_shell",
        min_overlap=0.80,
        name="closed sleeve surrounds aft nacelle body",
    )

    rest_pos = ctx.part_world_position(sleeve)
    with ctx.pose({sleeve_joint: 0.300}):
        extended_pos = ctx.part_world_position(sleeve)
        ctx.expect_overlap(
            sleeve,
            nacelle,
            axes="x",
            elem_a="sleeve_shell",
            elem_b="intake_shell",
            min_overlap=0.70,
            name="extended sleeve remains retained on nacelle",
        )
        ctx.expect_overlap(
            sleeve,
            nacelle,
            axes="x",
            elem_a="slider_shoe",
            elem_b="slider_track",
            min_overlap=0.45,
            name="extended sleeve shoe remains on track",
        )
    ctx.check(
        "sleeve translates rearward along engine axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
