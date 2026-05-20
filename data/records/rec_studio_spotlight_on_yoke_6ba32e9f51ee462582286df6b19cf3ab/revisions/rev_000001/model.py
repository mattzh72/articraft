from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _yoke_plate_profile() -> list[tuple[float, float]]:
    """Rounded side-plate outline in the local X/Z profile plane."""
    width = 0.064
    half = width * 0.5
    bottom = -0.205
    top_center = 0.030
    profile: list[tuple[float, float]] = [(-half, bottom), (half, bottom), (half, top_center)]
    for index in range(1, 16):
        # Semicircular crown centered on the tilt axis, from right to left.
        angle = 0.0 + math.pi * index / 15.0
        profile.append((half * math.cos(angle), top_center + half * math.sin(angle)))
    profile.append((-half, bottom))
    return profile


def _make_yoke_plate_mesh():
    plate = ExtrudeWithHolesGeometry(
        _yoke_plate_profile(),
        [_circle_profile(0.0175, segments=36)],
        height=0.026,
        center=True,
    )
    # Local profile X stays object X, profile Y becomes object Z, extrusion becomes object Y.
    return mesh_from_geometry(plate.rotate_x(math.pi / 2.0), "rounded_yoke_plate")


def _make_head_shell_mesh():
    # A thin revolved shell: rear electronics cap, swelling barrel, front retaining lip.
    # Lathe axis is local Z; rotate later so the spotlight points along +X.
    shell_profile = [
        (0.052, -0.168),
        (0.069, -0.145),
        (0.086, -0.060),
        (0.096, 0.090),
        (0.106, 0.190),
        (0.101, 0.218),
        (0.081, 0.226),
        (0.074, 0.196),
        (0.070, 0.060),
        (0.052, -0.130),
        (0.034, -0.158),
        (0.052, -0.168),
    ]
    return mesh_from_geometry(
        LatheGeometry(shell_profile, segments=72).rotate_y(math.pi / 2.0),
        "tapered_spot_shell",
    )


def _make_inner_reflector_mesh():
    reflector_profile = [
        (0.018, 0.103),
        (0.045, 0.122),
        (0.066, 0.166),
        (0.071, 0.190),
        (0.063, 0.195),
        (0.041, 0.148),
        (0.016, 0.118),
        (0.018, 0.103),
    ]
    return mesh_from_geometry(
        LatheGeometry(reflector_profile, segments=64).rotate_y(math.pi / 2.0),
        "black_reflector_bowl",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_yoke_studio_spotlight")

    graphite = model.material("satin_graphite", rgba=(0.12, 0.125, 0.13, 1.0))
    warm_black = model.material("warm_black", rgba=(0.025, 0.026, 0.028, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.19, 0.20, 0.21, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    rubber = model.material("matte_elastomer", rgba=(0.035, 0.034, 0.033, 1.0))
    glass = model.material("smoked_glass", rgba=(0.42, 0.58, 0.68, 0.38))
    led = model.material("warm_led", rgba=(1.0, 0.82, 0.50, 1.0))

    yoke_plate_mesh = _make_yoke_plate_mesh()
    shell_mesh = _make_head_shell_mesh()
    reflector_mesh = _make_inner_reflector_mesh()
    friction_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.028, tube=0.0045, radial_segments=16, tubular_segments=48).rotate_x(
            math.pi / 2.0
        ),
        "friction_ring",
    )
    lock_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.043,
            0.023,
            body_style="lobed",
            base_diameter=0.033,
            top_diameter=0.039,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=12, depth=0.0012),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        ),
        "lobed_lock_knob",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.165, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite,
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=0.118, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=dark_metal,
        name="base_top_reveal",
    )
    for index, (x, y) in enumerate(((0.095, 0.095), (-0.095, 0.095), (-0.095, -0.095), (0.095, -0.095))):
        stand.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(x, y, 0.004)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )
    stand.visual(
        Cylinder(radius=0.026, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        material=graphite,
        name="upright_post",
    )
    stand.visual(
        Cylinder(radius=0.042, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.284)),
        material=dark_metal,
        name="post_collar",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.176), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="lower_yoke_bridge",
    )
    stand.visual(
        Box((0.058, 0.260, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=graphite,
        name="bridge_pad",
    )
    stand.visual(
        yoke_plate_mesh,
        origin=Origin(xyz=(0.0, -0.142, 0.400)),
        material=graphite,
        name="side_yoke_plate_0",
    )
    stand.visual(
        friction_ring_mesh,
        origin=Origin(xyz=(0.0, -0.151, 0.400)),
        material=brushed_steel,
        name="friction_ring_0",
    )
    stand.visual(
        yoke_plate_mesh,
        origin=Origin(xyz=(0.0, 0.142, 0.400)),
        material=graphite,
        name="side_yoke_plate_1",
    )
    stand.visual(
        friction_ring_mesh,
        origin=Origin(xyz=(0.0, 0.151, 0.400)),
        material=brushed_steel,
        name="friction_ring_1",
    )
    stand.visual(
        Box((0.074, 0.250, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        material=dark_metal,
        name="yoke_root_blend",
    )

    head = model.part("head")
    head.visual(shell_mesh, material=graphite, name="tapered_shell")
    head.visual(reflector_mesh, material=warm_black, name="reflector_bowl")
    head.visual(
        Cylinder(radius=0.079, length=0.005),
        origin=Origin(xyz=(0.197, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    head.visual(
        Cylinder(radius=0.094, length=0.012),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_black,
        name="front_snoot_lip",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.106, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=led,
        name="led_emitter",
    )
    head.visual(
        Cylinder(radius=0.050, length=0.014),
        origin=Origin(xyz=(-0.164, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_service_cap",
    )
    head.visual(
        Box((0.018, 0.050, 0.010)),
        origin=Origin(xyz=(-0.169, 0.0, 0.036)),
        material=rubber,
        name="rear_cable_grommet",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="trunnion_pin",
    )
    for index, y in enumerate((-0.106, 0.106)):
        head.visual(
            Cylinder(radius=0.031, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"side_bearing_{index}",
        )

    for index, side in enumerate((-1.0, 1.0)):
        knob = model.part(f"lock_knob_{index}")
        knob.visual(
            lock_knob_mesh,
            origin=Origin(
                xyz=(0.0, 0.014 * side, 0.0),
                rpy=(-side * math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.0075, length=0.036),
            origin=Origin(
                xyz=(0.0, -0.014 * side, 0.0),
                rpy=(-side * math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_steel,
            name="threaded_stem",
        )
        knob.visual(
            Cylinder(radius=0.013, length=0.004),
            origin=Origin(
                xyz=(0.0, -0.034 * side, 0.0),
                rpy=(-side * math.pi / 2.0, 0.0, 0.0),
            ),
            material=rubber,
            name="friction_pad",
        )
        model.articulation(
            f"stand_to_lock_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=stand,
            child=knob,
            origin=Origin(xyz=(0.0, 0.176 * side, 0.400)),
            axis=(0.0, side, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=5.0),
        )

    model.articulation(
        "stand_to_head",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=1.5, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    head = object_model.get_part("head")
    tilt = object_model.get_articulation("stand_to_head")

    ctx.allow_overlap(
        head,
        stand,
        elem_a="trunnion_pin",
        elem_b="side_yoke_plate_0",
        reason="The tilt pin is intentionally captured in the yoke cheek bore; the mesh cheek is a simplified solid pivot support.",
    )
    ctx.expect_overlap(
        head,
        stand,
        axes="y",
        elem_a="trunnion_pin",
        elem_b="side_yoke_plate_0",
        min_overlap=0.012,
        name="trunnion passes through one yoke cheek",
    )
    ctx.allow_overlap(
        head,
        stand,
        elem_a="trunnion_pin",
        elem_b="side_yoke_plate_1",
        reason="The opposite tilt pin end is intentionally captured in the yoke cheek bore.",
    )
    ctx.expect_overlap(
        head,
        stand,
        axes="y",
        elem_a="trunnion_pin",
        elem_b="side_yoke_plate_1",
        min_overlap=0.012,
        name="trunnion passes through opposite yoke cheek",
    )
    ctx.expect_within(
        head,
        stand,
        axes="xz",
        inner_elem="trunnion_pin",
        outer_elem="side_yoke_plate_0",
        margin=0.035,
        name="trunnion centerline is captured by yoke cheek profile",
    )

    for index, plate_name in ((0, "side_yoke_plate_0"), (1, "side_yoke_plate_1")):
        knob_name = f"lock_knob_{index}"
        ctx.allow_overlap(
            knob_name,
            stand,
            elem_a="threaded_stem",
            elem_b=plate_name,
            reason="The lock knob stem is intentionally represented passing through the yoke cheek bore.",
        )
        ctx.expect_overlap(
            knob_name,
            stand,
            axes="y",
            elem_a="threaded_stem",
            elem_b=plate_name,
            min_overlap=0.006,
            name=f"lock stem {index} passes through yoke cheek",
        )
        ctx.allow_overlap(
            head,
            knob_name,
            elem_a="trunnion_pin",
            elem_b="threaded_stem",
            reason="The coaxial friction lock screw seats into the simplified trunnion/pivot stack.",
        )
        ctx.expect_overlap(
            head,
            knob_name,
            axes="y",
            elem_a="trunnion_pin",
            elem_b="threaded_stem",
            min_overlap=0.010,
            name=f"lock stem {index} engages pivot stack",
        )

    rest_aabb = ctx.part_element_world_aabb(head, elem="front_glass")
    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_element_world_aabb(head, elem="front_glass")
    ctx.check(
        "tilt joint moves the spotlight aperture",
        rest_aabb is not None
        and tilted_aabb is not None
        and abs(((tilted_aabb[0][2] + tilted_aabb[1][2]) * 0.5) - ((rest_aabb[0][2] + rest_aabb[1][2]) * 0.5)) > 0.045,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
