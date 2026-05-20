from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeWithHolesGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_points(radius: float, segments: int, *, start: float = 0.0, end: float | None = None):
    if end is None:
        end = start + math.tau
    return [
        (radius * math.cos(start + (end - start) * i / segments),
         radius * math.sin(start + (end - start) * i / segments))
        for i in range(segments)
    ]


def _sector_hole(inner_radius: float, outer_radius: float, start_angle: float, end_angle: float):
    """Closed annular-sector loop used as one airflow opening in the front frame."""
    steps = 9
    outer = _circle_points(outer_radius, steps, start=start_angle, end=end_angle)
    inner = list(reversed(_circle_points(inner_radius, steps, start=start_angle, end=end_angle)))
    return outer + inner


def _front_strut_frame_geometry():
    """A single connected perforated disk: outer ring, six radial struts, and center hub."""
    outer_radius = 0.207
    hub_radius = 0.055
    flow_radius = 0.178
    strut_width_angle = math.radians(12.0)
    holes = []
    for index in range(6):
        center = index * math.tau / 6.0
        start = center + strut_width_angle * 0.5
        end = center + math.tau / 6.0 - strut_width_angle * 0.5
        holes.append(_sector_hole(hub_radius, flow_radius, start, end))

    return ExtrudeWithHolesGeometry(
        _circle_points(outer_radius, 96),
        holes,
        0.055,
        cap=True,
        center=True,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_turbojet_module")

    brushed_titanium = model.material("brushed_titanium", rgba=(0.64, 0.67, 0.70, 1.0))
    dark_stator = model.material("dark_stator", rgba=(0.10, 0.11, 0.12, 1.0))
    compressor_metal = model.material("compressor_metal", rgba=(0.72, 0.74, 0.77, 1.0))

    casing_shell = LatheGeometry.from_shell_profiles(
        [
            (0.292, -0.550),
            (0.304, -0.520),
            (0.286, -0.475),
            (0.252, -0.365),
            (0.250, 0.350),
            (0.232, 0.455),
            (0.158, 0.755),
        ],
        [
            (0.209, -0.550),
            (0.205, -0.430),
            (0.205, 0.330),
            (0.168, 0.500),
            (0.112, 0.755),
        ],
        segments=96,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )

    casing = model.part("casing")
    casing.visual(
        mesh_from_geometry(casing_shell, "casing_shell"),
        material=brushed_titanium,
        name="casing_shell",
    )

    front_frame = model.part("front_frame")
    front_frame.visual(
        mesh_from_geometry(_front_strut_frame_geometry(), "front_strut_frame"),
        origin=Origin(xyz=(0.0, 0.0, -0.470)),
        material=dark_stator,
        name="strut_frame",
    )

    rotor = model.part("compressor_rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.176,
                0.052,
                12,
                thickness=0.052,
                blade_pitch_deg=34.0,
                blade_sweep_deg=28.0,
                blade=FanRotorBlade(
                    shape="scimitar",
                    tip_pitch_deg=15.0,
                    camber=0.18,
                    tip_clearance=0.003,
                ),
                hub=FanRotorHub(
                    style="spinner",
                    rear_collar_height=0.016,
                    rear_collar_radius=0.050,
                    bore_diameter=0.014,
                ),
            ),
            "compressor_rotor",
        ),
        material=compressor_metal,
        name="blade_disk",
    )
    rotor.visual(
        Cylinder(radius=0.026, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=dark_stator,
        name="bearing_shaft",
    )

    model.articulation(
        "casing_to_front_frame",
        ArticulationType.FIXED,
        parent=casing,
        child=front_frame,
        origin=Origin(),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=casing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.370)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=420.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    casing = object_model.get_part("casing")
    front_frame = object_model.get_part("front_frame")
    rotor = object_model.get_part("compressor_rotor")
    spin = object_model.get_articulation("rotor_spin")

    ctx.allow_overlap(
        front_frame,
        casing,
        elem_a="strut_frame",
        elem_b="casing_shell",
        reason="The welded outer ring of the intake strut frame is intentionally seated into the lip of the casing.",
    )
    ctx.allow_overlap(
        rotor,
        front_frame,
        elem_a="bearing_shaft",
        elem_b="strut_frame",
        reason="The rotor bearing shaft is intentionally captured in the stationary front hub.",
    )

    ctx.check(
        "rotor is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type={spin.articulation_type}",
    )
    ctx.check(
        "rotor axis follows engine centerline",
        tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_within(
        rotor,
        casing,
        axes="xy",
        margin=0.0,
        name="rotor fits inside casing diameter",
    )
    ctx.expect_overlap(
        front_frame,
        casing,
        axes="xy",
        min_overlap=0.18,
        name="front struts span the intake",
    )
    ctx.expect_overlap(
        front_frame,
        casing,
        axes="z",
        elem_a="strut_frame",
        elem_b="casing_shell",
        min_overlap=0.030,
        name="front strut ring is seated in intake lip depth",
    )
    ctx.expect_overlap(
        rotor,
        front_frame,
        axes="z",
        elem_a="bearing_shaft",
        elem_b="strut_frame",
        min_overlap=0.045,
        name="bearing shaft remains captured in front hub",
    )
    ctx.expect_gap(
        rotor,
        front_frame,
        axis="z",
        positive_elem="blade_disk",
        negative_elem="strut_frame",
        min_gap=0.015,
        name="rotor blades sit behind the intake struts",
    )

    rest_position = ctx.part_world_position(rotor)
    with ctx.pose({spin: 1.35}):
        spun_position = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            casing,
            axes="xy",
            margin=0.0,
            name="spun rotor stays concentric in casing",
        )
    ctx.check(
        "continuous spin does not translate rotor",
        rest_position is not None
        and spun_position is not None
        and max(abs(a - b) for a, b in zip(rest_position, spun_position)) < 1e-6,
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
