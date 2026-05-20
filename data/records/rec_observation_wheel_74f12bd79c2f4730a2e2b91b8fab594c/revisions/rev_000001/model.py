from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


WHEEL_CENTER_Z = 72.0
RIM_RADIUS = 60.0
PIVOT_RADIUS = 64.0
CABIN_COUNT = 12


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _cyl_y(radius: float, length: float, *, segments: int = 32) -> MeshGeometry:
    """CylinderGeometry helper whose axis is local Y rather than local Z."""
    return CylinderGeometry(radius, length, radial_segments=segments).rotate_x(math.pi / 2.0)


def _support_structure_geometry() -> MeshGeometry:
    """One connected mesh for the plaza plinth, broad A-frames, and bearing towers."""
    geom = BoxGeometry((78.0, 22.0, 0.45)).translate(0.0, 0.0, 0.225)

    # Heavy concrete foot blocks under each leg, overlapping the plaza slab so the
    # root part is one supported assembly.
    for x in (-30.0, 30.0):
        for y in (-6.8, 6.8):
            geom.merge(BoxGeometry((12.5, 5.5, 1.15)).translate(x, y, 0.85))

    # Rear A-frame tube legs. The matching front A-frame is added as sloped box
    # members below so the authored support stays exact-connectivity clean.
    for y in (-5.8,):
        for x in (-30.0, 30.0):
            geom.merge(
                tube_from_spline_points(
                    [(x, y, 1.0), (x * 0.62, y, 35.0), (0.0, y, WHEEL_CENTER_Z)],
                    radius=1.15,
                    samples_per_segment=16,
                    radial_segments=22,
                    cap_ends=True,
                )
            )

    return geom


def _wheel_geometry() -> MeshGeometry:
    """Double-sided observation wheel rim with spokes, hub, and cabin hanger arms."""
    geom = MeshGeometry()

    for y in (-3.6, 3.6):
        geom.merge(
            TorusGeometry(
                radius=RIM_RADIUS,
                tube=0.78,
                radial_segments=18,
                tubular_segments=144,
            )
            .rotate_x(math.pi / 2.0)
            .translate(0.0, y, 0.0)
        )
        geom.merge(
            TorusGeometry(
                radius=RIM_RADIUS - 4.0,
                tube=0.34,
                radial_segments=14,
                tubular_segments=144,
            )
            .rotate_x(math.pi / 2.0)
            .translate(0.0, y, 0.0)
        )

    # Hub and main axle barrel.
    geom.merge(_cyl_y(4.3, 7.3, segments=48))
    geom.merge(_cyl_y(2.5, 7.4, segments=40))

    # Radial spokes, paired on the two wheel faces, overlap the hub and rims.
    for idx in range(CABIN_COUNT * 2):
        angle = 2.0 * math.pi * idx / (CABIN_COUNT * 2)
        c = math.cos(angle)
        s = math.sin(angle)
        for y in (-3.6, 3.6):
            geom.merge(
                tube_from_spline_points(
                    [(2.7 * c, y, 2.7 * s), ((RIM_RADIUS - 1.2) * c, y, (RIM_RADIUS - 1.2) * s)],
                    radius=0.24 if idx % 2 else 0.34,
                    samples_per_segment=2,
                    radial_segments=14,
                    cap_ends=True,
                )
            )

    # Short radial leveling arms and cross-shafts where the separate cabins hang.
    for idx in range(CABIN_COUNT):
        angle = math.pi / 2.0 - 2.0 * math.pi * idx / CABIN_COUNT
        c = math.cos(angle)
        s = math.sin(angle)
        rim = RIM_RADIUS - 2.0
        pivot = PIVOT_RADIUS
        for y in (-3.6, 3.6):
            geom.merge(
                tube_from_spline_points(
                    [(rim * c, y, rim * s), (pivot * c, y, pivot * s)],
                    radius=0.32,
                    samples_per_segment=2,
                    radial_segments=16,
                    cap_ends=True,
                )
            )
        geom.merge(_cyl_y(0.43, 7.6, segments=24).translate(pivot * c, 0.0, pivot * s))
    return geom


def _cabin_shell_mesh():
    # Rounded capsule body: a rounded-rectangle section extruded through the
    # cabin depth, with its long axis kept horizontal and its roof below the
    # leveling pivot.
    profile = rounded_rect_profile(7.4, 4.2, 1.25, corner_segments=12)
    shell = ExtrudeGeometry(profile, 4.25, center=True).rotate_x(math.pi / 2.0)
    return _mesh("capsule_rounded_shell", shell)


def _add_capsule_visuals(part, shell_mesh, materials) -> None:
    cabin_white, glass, dark_metal, rubber = materials

    part.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -3.25)),
        material=cabin_white,
        name="cabin_shell",
    )
    # Wide panoramic glazed bands on both sides, seated very slightly proud of
    # the shell so each capsule reads as enclosed rather than an open gondola.
    for side, y in (("front", 2.155), ("rear", -2.155)):
        part.visual(
            Box((5.95, 0.055, 2.25)),
            origin=Origin(xyz=(0.0, y, -3.13)),
            material=glass,
            name=f"{side}_glass",
        )
        part.visual(
            Box((6.35, 0.08, 0.16)),
            origin=Origin(xyz=(0.0, y * 1.003, -2.0)),
            material=dark_metal,
            name=f"{side}_top_mullion",
        )
        part.visual(
            Box((6.35, 0.08, 0.16)),
            origin=Origin(xyz=(0.0, y * 1.003, -4.25)),
            material=dark_metal,
            name=f"{side}_bottom_mullion",
        )
        for x in (-2.05, 0.0, 2.05):
            part.visual(
                Box((0.13, 0.085, 2.28)),
                origin=Origin(xyz=(x, y * 1.004, -3.13)),
                material=dark_metal,
                name=f"{side}_mullion_{x:+.0f}",
            )

    # Roof beam and central hanger tie the cabin body to the local leveling
    # pivot.  The saddle sits tangent to the wheel cross-shaft rather than
    # intersecting it, so the capsule reads as physically hung from the arm.
    part.visual(
        Box((7.1, 0.58, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, -1.13)),
        material=dark_metal,
        name="roof_beam",
    )
    part.visual(
        Cylinder(radius=0.32, length=2.25),
        origin=Origin(xyz=(0.0, 0.0, -0.75), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_saddle",
    )
    for strap_index, x in enumerate((-0.38, 0.38)):
        part.visual(
            Box((0.20, 0.46, 0.70)),
            origin=Origin(xyz=(x, 0.0, -0.98)),
            material=dark_metal,
            name=f"hanger_strap_{strap_index}",
        )
    part.visual(
        Cylinder(radius=0.18, length=3.9),
        origin=Origin(xyz=(0.0, 0.0, -1.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roof_service_pipe",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_observation_wheel")

    concrete = model.material("warm_concrete", rgba=(0.62, 0.61, 0.57, 1.0))
    white_steel = model.material("white_steel", rgba=(0.90, 0.92, 0.90, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.14, 0.16, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.42, 0.45, 0.47, 1.0))
    cabin_white = model.material("cabin_white", rgba=(0.94, 0.95, 0.92, 1.0))
    glass_blue = model.material("blue_tinted_glass", rgba=(0.32, 0.58, 0.78, 0.48))
    rubber = model.material("black_seal", rgba=(0.03, 0.035, 0.04, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        _mesh("support_frame_structure", _support_structure_geometry()),
        material=concrete,
        name="a_frame_structure",
    )
    support_frame.visual(
        Box((2.3, 2.3, math.hypot(30.0, WHEEL_CENTER_Z - 1.0))),
        origin=Origin(
            xyz=(-15.0, 5.8, (WHEEL_CENTER_Z + 1.0) / 2.0),
            rpy=(0.0, math.atan2(30.0, WHEEL_CENTER_Z - 1.0), 0.0),
        ),
        material=concrete,
        name="front_leg",
    )
    support_frame.visual(
        Box((2.3, 2.3, math.hypot(30.0, WHEEL_CENTER_Z - 1.0))),
        origin=Origin(
            xyz=(15.0, 5.8, (WHEEL_CENTER_Z + 1.0) / 2.0),
            rpy=(0.0, math.atan2(-30.0, WHEEL_CENTER_Z - 1.0), 0.0),
        ),
        material=concrete,
        name="front_leg_1",
    )
    for brace_z, width in ((18.0, 49.0), (36.0, 42.0)):
        support_frame.visual(
            Box((width, 1.25, 1.25)),
            origin=Origin(xyz=(0.0, 5.8, brace_z)),
            material=concrete,
            name=f"front_brace_{int(brace_z)}",
        )
        support_frame.visual(
            Box((width, 1.25, 1.25)),
            origin=Origin(xyz=(0.0, -5.8, brace_z)),
            material=concrete,
            name=f"rear_brace_{int(brace_z)}",
        )
    support_frame.visual(
        Box((78.0, 22.0, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=dark_steel,
        name="base_plaza",
    )
    for x in (-30.0, 30.0):
        for y in (-5.8, 5.8):
            support_frame.visual(
                Box((8.5, 4.8, 1.35)),
                origin=Origin(xyz=(x, y, 0.78)),
                material=concrete,
                name=f"leg_anchor_{'neg' if x < 0 else 'pos'}_{'rear' if y < 0 else 'front'}",
            )
    for y, name in ((4.5375, "front_bearing_collar"), (-4.5375, "rear_bearing_collar")):
        support_frame.visual(
            Cylinder(radius=3.0, length=1.175),
            origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hub_gray,
            name=name,
        )
    for y, name in ((6.0, "front_bearing_housing"), (-6.0, "rear_bearing_housing")):
        support_frame.visual(
            Cylinder(radius=4.8, length=1.75),
            origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=concrete,
            name=name,
        )
        support_frame.visual(
            Cylinder(radius=2.7, length=2.05),
            origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hub_gray,
            name=f"{name}_cap",
        )
    support_frame.inertial = Inertial.from_geometry(
        Box((80.0, 24.0, WHEEL_CENTER_Z + 6.0)),
        mass=2_000_000.0,
        origin=Origin(xyz=(0.0, 0.0, (WHEEL_CENTER_Z + 6.0) / 2.0)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        _mesh("rotating_wheel_rim_spokes", _wheel_geometry()),
        material=white_steel,
        name="rim_spoke_frame",
    )
    wheel.visual(
        Cylinder(radius=3.35, length=7.9),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="rotating_hub",
    )
    wheel.visual(
        Cylinder(radius=1.35, length=7.4),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="drive_axle",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_RADIUS, length=6.0),
        mass=850_000.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    wheel_joint = model.articulation(
        "wheel_rotation",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3_500_000.0, velocity=0.10),
    )

    shell_mesh = _cabin_shell_mesh()
    for idx in range(CABIN_COUNT):
        angle = math.pi / 2.0 - 2.0 * math.pi * idx / CABIN_COUNT
        c = math.cos(angle)
        s = math.sin(angle)
        capsule = model.part(f"capsule_{idx}")
        _add_capsule_visuals(capsule, shell_mesh, (cabin_white, glass_blue, dark_steel, rubber))
        capsule.inertial = Inertial.from_geometry(
            Box((7.8, 5.0, 5.4)),
            mass=9_500.0,
            origin=Origin(xyz=(0.0, 0.0, -3.0)),
        )
        model.articulation(
            f"capsule_{idx}_leveling",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=capsule,
            origin=Origin(xyz=(PIVOT_RADIUS * c, 0.0, PIVOT_RADIUS * s)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=45_000.0, velocity=0.45),
            mimic=Mimic(joint=wheel_joint.name, multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wheel = object_model.get_part("wheel")
    support = object_model.get_part("support_frame")
    top_capsule = object_model.get_part("capsule_0")
    bottom_capsule = object_model.get_part("capsule_6")
    wheel_rotation = object_model.get_articulation("wheel_rotation")
    capsule_leveling = object_model.get_articulation("capsule_0_leveling")

    ctx.check(
        "wheel rotates continuously on horizontal axis",
        str(wheel_rotation.articulation_type).lower().endswith("continuous")
        and tuple(round(v, 6) for v in wheel_rotation.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_rotation.articulation_type}, axis={wheel_rotation.axis}",
    )
    ctx.check(
        "capsules have separate leveling pivots",
        len([p for p in object_model.parts if p.name.startswith("capsule_")]) == CABIN_COUNT
        and str(capsule_leveling.articulation_type).lower().endswith("continuous")
        and capsule_leveling.mimic is not None,
        details=f"count={len([p for p in object_model.parts if p.name.startswith('capsule_')])}, "
        f"type={capsule_leveling.articulation_type}, mimic={capsule_leveling.mimic}",
    )
    ctx.expect_gap(
        bottom_capsule,
        support,
        axis="z",
        min_gap=0.20,
        positive_elem="cabin_shell",
        negative_elem="base_plaza",
        name="lowest capsule clears the plaza",
    )

    rest_pos = ctx.part_world_position(top_capsule)
    with ctx.pose({wheel_rotation: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(top_capsule)
        aabb = ctx.part_element_world_aabb(top_capsule, elem="cabin_shell")
        z_extent = None if aabb is None else aabb[1][2] - aabb[0][2]
        ctx.check(
            "top capsule travels around rim",
            rest_pos is not None
            and rotated_pos is not None
            and rotated_pos[0] > rest_pos[0] + 50.0
            and rotated_pos[2] < rest_pos[2] - 50.0,
            details=f"rest={rest_pos}, rotated={rotated_pos}",
        )
        ctx.check(
            "mimic leveling keeps capsule body upright",
            z_extent is not None and z_extent < 5.1,
            details=f"capsule shell z extent at quarter turn={z_extent}",
        )

    return ctx.report()


object_model = build_object_model()
