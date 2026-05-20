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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


AXIS_TO_X = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _merge(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _thin_shell(
    outer: list[tuple[float, float]],
    inner: list[tuple[float, float]],
    *,
    segments: int = 96,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _stationary_shell() -> MeshGeometry:
    # Profiles are authored around local Z and then rotated so Z becomes the
    # longitudinal engine X axis.  The mid body stays under the translating
    # sleeve, leaving a clear annular gap for the moving cowl.
    return _thin_shell(
        outer=[
            (1.06, -1.72),
            (1.20, -1.60),
            (1.26, -1.20),
            (1.18, -0.48),
            (0.95, -0.34),
            (0.92, 1.08),
            (1.04, 1.24),
            (1.02, 1.72),
            (0.84, 1.92),
        ],
        inner=[
            (0.82, -1.74),
            (1.00, -1.58),
            (1.03, -1.18),
            (0.91, -0.48),
            (0.72, -0.33),
            (0.70, 1.08),
            (0.74, 1.24),
            (0.68, 1.94),
        ],
        segments=112,
    )


def _core_body() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, -0.72),
            (0.18, -0.68),
            (0.34, -0.45),
            (0.44, 0.10),
            (0.43, 1.12),
            (0.32, 1.62),
            (0.12, 1.92),
            (0.0, 2.04),
        ],
        segments=80,
    )


def _cascade_band() -> MeshGeometry:
    # A louver/cascade-looking band mounted under the translating sleeve.
    base = _thin_shell(
        outer=[(1.02, -0.08), (1.04, 0.16), (1.04, 0.84), (1.00, 1.06)],
        inner=[(0.96, -0.10), (0.97, 0.16), (0.97, 0.84), (0.94, 1.08)],
        segments=80,
    )
    return base


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="airliner_turbofan_reverser")

    nacelle_white = model.material("nacelle_white", rgba=(0.86, 0.88, 0.90, 1.0))
    sleeve_white = model.material("sleeve_white", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.56, 0.60, 0.66, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    cascade_metal = model.material("cascade_metal", rgba=(0.38, 0.40, 0.42, 1.0))

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_stationary_shell(), "stationary_nacelle_shell"),
        origin=AXIS_TO_X,
        material=nacelle_white,
        name="stationary_shell",
    )
    nacelle.visual(
        mesh_from_geometry(_core_body(), "centered_core_body"),
        origin=AXIS_TO_X,
        material=dark_metal,
        name="core_body",
    )
    nacelle.visual(
        Cylinder(radius=0.082, length=1.40),
        origin=Origin(xyz=(-1.00, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="bearing_shaft",
    )
    nacelle.visual(
        mesh_from_geometry(_cascade_band(), "reverser_cascade_band"),
        origin=AXIS_TO_X,
        material=cascade_metal,
        name="cascade_band",
    )

    # Four broad outlet guide vanes visually tie the central core to the duct.
    nacelle.visual(
        Box((0.10, 2.02, 0.055)),
        origin=Origin(xyz=(-0.58, 0.0, 0.0)),
        material=dark_metal,
        name="horizontal_stator",
    )
    nacelle.visual(
        Box((0.10, 0.055, 2.02)),
        origin=Origin(xyz=(-0.58, 0.0, 0.0)),
        material=dark_metal,
        name="vertical_stator",
    )
    for name, x, y, z, size in (
        ("cascade_frame_0", 0.48, 0.96, 0.0, (1.02, 0.16, 0.16)),
        ("cascade_frame_1", 0.48, -0.96, 0.0, (1.02, 0.16, 0.16)),
        ("cascade_frame_2", 0.48, 0.0, 0.96, (1.02, 0.16, 0.16)),
        ("cascade_frame_3", 0.48, 0.0, -0.96, (1.02, 0.16, 0.16)),
    ):
        nacelle.visual(
            Box(size),
            origin=Origin(xyz=(x, y, z)),
            material=cascade_metal,
            name=name,
        )

    # External guide tracks and their fixed bridge lugs are part of the nacelle.
    nacelle.visual(
        Box((2.18, 0.075, 0.085)),
        origin=Origin(xyz=(0.50, 1.37, 0.0)),
        material=rail_metal,
        name="guide_rail_0",
    )
    nacelle.visual(
        Box((2.18, 0.075, 0.085)),
        origin=Origin(xyz=(0.50, -1.37, 0.0)),
        material=rail_metal,
        name="guide_rail_1",
    )
    for name, x, y in (
        ("rail_mount_0_0", -0.42, 1.225),
        ("rail_mount_0_1", 1.22, 1.225),
        ("rail_mount_1_0", -0.42, -1.225),
        ("rail_mount_1_1", 1.22, -1.225),
    ):
        nacelle.visual(
            Box((0.14, 0.30, 0.12)),
            origin=Origin(xyz=(x, y, 0.0)),
            material=rail_metal,
            name=name,
        )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.91,
                0.23,
                22,
                thickness=0.14,
                blade_pitch_deg=32.0,
                blade_sweep_deg=28.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.18),
                hub=FanRotorHub(
                    style="spinner",
                    rear_collar_height=0.040,
                    rear_collar_radius=0.21,
                    bore_diameter=0.16,
                ),
            ),
            "front_fan_rotor",
        ),
        origin=AXIS_TO_X,
        material=blade_metal,
        name="fan_stage",
    )

    reverser_sleeve = model.part("reverser_sleeve")
    reverser_sleeve.visual(
        mesh_from_geometry(
            _merge(
                [
                    _thin_shell(
                        outer=[
                            (1.17, -0.31),
                            (1.25, -0.20),
                            (1.27, 0.28),
                            (1.23, 0.82),
                            (1.18, 1.08),
                        ],
                        inner=[
                            (1.11, -0.33),
                            (1.13, -0.06),
                            (1.13, 0.82),
                            (1.10, 1.10),
                        ],
                        segments=112,
                    ),
                    _thin_shell(
                        outer=[(1.285, -0.24), (1.292, -0.17), (1.285, -0.10)],
                        inner=[(1.244, -0.25), (1.252, -0.17), (1.244, -0.09)],
                        segments=96,
                    ),
                    _thin_shell(
                        outer=[(1.255, 0.86), (1.264, 0.94), (1.255, 1.02)],
                        inner=[(1.214, 0.85), (1.222, 0.94), (1.214, 1.03)],
                        segments=96,
                    ),
                ]
            ),
            "translating_reverser_sleeve",
        ),
        origin=AXIS_TO_X,
        material=sleeve_white,
        name="outer_cowl",
    )
    reverser_sleeve.visual(
        Box((1.18, 0.13, 0.18)),
        origin=Origin(xyz=(0.38, 1.2675, 0.0)),
        material=rail_metal,
        name="slider_lug_0",
    )
    reverser_sleeve.visual(
        Box((1.18, 0.13, 0.18)),
        origin=Origin(xyz=(0.38, -1.2675, 0.0)),
        material=rail_metal,
        name="slider_lug_1",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan_rotor,
        origin=Origin(xyz=(-1.23, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=70.0),
    )
    model.articulation(
        "sleeve_slide",
        ArticulationType.PRISMATIC,
        parent=nacelle,
        child=reverser_sleeve,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8500.0, velocity=0.35, lower=0.0, upper=0.46),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    fan_rotor = object_model.get_part("fan_rotor")
    sleeve = object_model.get_part("reverser_sleeve")
    fan_spin = object_model.get_articulation("fan_spin")
    sleeve_slide = object_model.get_articulation("sleeve_slide")

    ctx.allow_overlap(
        nacelle,
        fan_rotor,
        elem_a="bearing_shaft",
        elem_b="fan_stage",
        reason="The stationary bearing shaft is intentionally captured inside the fan hub bore.",
    )
    ctx.expect_overlap(
        nacelle,
        fan_rotor,
        axes="x",
        elem_a="bearing_shaft",
        elem_b="fan_stage",
        min_overlap=0.12,
        name="fan hub remains on the bearing shaft",
    )
    ctx.expect_within(
        nacelle,
        fan_rotor,
        axes="yz",
        elem_a="bearing_shaft",
        elem_b="fan_stage",
        margin=0.0,
        name="bearing shaft is centered in the fan hub bore",
    )

    ctx.check(
        "fan is a continuous longitudinal rotor",
        fan_spin is not None
        and fan_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(fan_spin.axis) == (1.0, 0.0, 0.0),
        details=f"fan_spin={fan_spin!r}",
    )
    ctx.check(
        "reverser sleeve slides on engine axis",
        sleeve_slide is not None
        and sleeve_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(sleeve_slide.axis) == (1.0, 0.0, 0.0)
        and sleeve_slide.motion_limits is not None
        and sleeve_slide.motion_limits.upper >= 0.40,
        details=f"sleeve_slide={sleeve_slide!r}",
    )

    ctx.expect_within(
        fan_rotor,
        nacelle,
        axes="yz",
        margin=0.02,
        name="fan stage is centered inside nacelle inlet",
    )
    ctx.expect_within(
        sleeve,
        nacelle,
        axes="yz",
        elem_a="outer_cowl",
        elem_b="stationary_shell",
        margin=0.26,
        name="reverser sleeve stays concentric around nacelle body",
    )
    ctx.expect_overlap(
        sleeve,
        nacelle,
        axes="x",
        elem_a="slider_lug_0",
        elem_b="guide_rail_0",
        min_overlap=0.90,
        name="closed sleeve lugs are carried by guide rails",
    )

    rest_pos = ctx.part_world_position(sleeve)
    with ctx.pose({sleeve_slide: 0.46, fan_spin: 1.2}):
        extended_pos = ctx.part_world_position(sleeve)
        ctx.expect_overlap(
            sleeve,
            nacelle,
            axes="x",
            elem_a="slider_lug_0",
            elem_b="guide_rail_0",
            min_overlap=0.80,
            name="extended sleeve remains on guide rails",
        )
        ctx.expect_within(
            fan_rotor,
            nacelle,
            axes="yz",
            margin=0.02,
            name="spinning fan remains on longitudinal centerline",
        )

    ctx.check(
        "sleeve translates aft",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.40,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
