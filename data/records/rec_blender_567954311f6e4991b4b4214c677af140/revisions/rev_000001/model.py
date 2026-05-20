from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _shell_mesh(
    outer: list[tuple[float, float]],
    inner: list[tuple[float, float]],
    name: str,
    *,
    segments: int = 72,
):
    """Thin revolved shell with ring caps at both ends."""

    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _blade_mesh(name: str) -> MeshGeometry:
    """A connected four-wing blender blade: two twisted, tapered strips."""

    geom = MeshGeometry()

    def add_plate(angle: float, lift_sign: float) -> None:
        length = 0.074
        root = 0.010
        tip = 0.006
        half = length / 2.0
        thick = 0.0018
        # Long, slightly swept quadrilateral, duplicated top/bottom for thickness.
        pts_2d = [
            (-half, -tip),
            (-root, -0.0065),
            (half, tip),
            (root, 0.0065),
        ]
        ca = math.cos(angle)
        sa = math.sin(angle)
        top: list[int] = []
        bottom: list[int] = []
        for x, y in pts_2d:
            # Opposite tips sit at different heights like a stamped blender blade.
            z_mid = lift_sign * 0.006 * (x / half)
            xr = x * ca - y * sa
            yr = x * sa + y * ca
            top.append(geom.add_vertex(xr, yr, z_mid + thick / 2.0))
            bottom.append(geom.add_vertex(xr, yr, z_mid - thick / 2.0))
        geom.add_face(top[0], top[1], top[2])
        geom.add_face(top[0], top[2], top[3])
        geom.add_face(bottom[2], bottom[1], bottom[0])
        geom.add_face(bottom[3], bottom[2], bottom[0])
        for i in range(4):
            j = (i + 1) % 4
            geom.add_face(top[i], bottom[i], bottom[j])
            geom.add_face(top[i], bottom[j], top[j])

    add_plate(math.radians(18.0), 1.0)
    add_plate(math.radians(108.0), -1.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="personal_single_serve_blender")

    white_plastic = Material("warm_white_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    charcoal = Material("charcoal_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    satin_black = Material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    translucent_cup = Material("clear_smoky_copolyester", rgba=(0.62, 0.84, 0.95, 0.34))
    teal = Material("teal_lid_plastic", rgba=(0.04, 0.44, 0.48, 1.0))
    rubber = Material("dark_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = Material("brushed_stainless_steel", rgba=(0.78, 0.78, 0.74, 1.0))

    base = model.part("motor_base")
    base.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.0, 0.000),
                    (0.064, 0.000),
                    (0.079, 0.012),
                    (0.087, 0.060),
                    (0.082, 0.096),
                    (0.062, 0.125),
                    (0.0, 0.125),
                ],
                segments=80,
            ),
            "tapered_motor_base",
        ),
        material=white_plastic,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.061, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=charcoal,
        name="top_shoulder",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=satin_black,
        name="drive_coupling_socket",
    )
    for angle in (0.0, math.pi / 2.0):
        base.visual(
            Box((0.054, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.153), rpy=(0.0, 0.0, angle)),
            material=charcoal,
            name=f"drive_spline_{int(angle * 100)}",
        )
    base.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        material=steel,
        name="metal_drive_boss",
    )
    base.visual(
        Box((0.052, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.084, 0.092)),
        material=charcoal,
        name="front_badge",
    )
    base.visual(
        Cylinder(radius=0.084, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="rubber_foot_ring",
    )

    cup = model.part("cup")
    cup.visual(
        _shell_mesh(
            [
                (0.041, 0.024),
                (0.043, 0.070),
                (0.044, 0.190),
                (0.047, 0.242),
            ],
            [
                (0.036, 0.024),
                (0.038, 0.070),
                (0.039, 0.190),
                (0.042, 0.242),
            ],
            "transparent_blending_cup",
        ),
        material=translucent_cup,
        name="clear_cup_wall",
    )
    cup.visual(
        _shell_mesh(
            [(0.054, -0.012), (0.054, 0.032)],
            [(0.041, -0.012), (0.041, 0.032)],
            "threaded_lower_collar",
        ),
        material=charcoal,
        name="threaded_collar",
    )
    for i, z in enumerate((-0.005, 0.006, 0.017)):
        cup.visual(
            mesh_from_geometry(TorusGeometry(radius=0.055, tube=0.0012), f"cup_thread_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=charcoal,
            name=f"thread_rib_{i}",
        )
    cup.visual(
        Cylinder(radius=0.046, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=teal,
        name="top_lid_deck",
    )
    cup.visual(
        _shell_mesh(
            [(0.049, 0.240), (0.049, 0.260)],
            [(0.039, 0.240), (0.039, 0.260)],
            "teal_screw_lid_ring",
        ),
        material=teal,
        name="top_lid_ring",
    )
    cup.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, -0.014, 0.254)),
        material=charcoal,
        name="spout_seal_rim",
    )
    cup.visual(
        Box((0.064, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.050, 0.251)),
        material=teal,
        name="hinge_bridge",
    )
    for x in (-0.026, 0.026):
        cup.visual(
            Box((0.007, 0.012, 0.016)),
            origin=Origin(xyz=(x, 0.056, 0.257)),
            material=teal,
            name=f"hinge_ear_{'a' if x < 0 else 'b'}",
        )

    blade = model.part("blade_assembly")
    blade.visual(
        _blade_mesh("crossed_blades"),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="cutting_blades",
    )
    blade.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="hub",
    )
    blade.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=charcoal,
        name="retaining_nut",
    )

    spout_lid = model.part("spout_lid")
    spout_lid.visual(
        Box((0.040, 0.050, 0.008)),
        origin=Origin(xyz=(0.0, -0.024, 0.004)),
        material=teal,
        name="flap_panel",
    )
    spout_lid.visual(
        Cylinder(radius=0.005, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=teal,
        name="hinge_barrel",
    )
    spout_lid.visual(
        Box((0.026, 0.008, 0.013)),
        origin=Origin(xyz=(0.0, -0.050, 0.014)),
        material=teal,
        name="thumb_tab",
    )

    button = model.part("power_button")
    button.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="button_cap",
    )
    button.visual(
        Box((0.022, 0.005, 0.011)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=satin_black,
        name="button_stem",
    )

    model.articulation(
        "base_to_cup",
        ArticulationType.FIXED,
        parent=base,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
    )
    model.articulation(
        "cup_to_blade",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=120.0, lower=0.0, upper=2.0 * math.pi),
    )
    model.articulation(
        "cup_to_spout_lid",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=spout_lid,
        origin=Origin(xyz=(0.0, 0.056, 0.257)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "base_to_power_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=button,
        origin=Origin(xyz=(0.0, -0.086, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.04, lower=0.0, upper=0.006),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("motor_base")
    cup = object_model.get_part("cup")
    blade = object_model.get_part("blade_assembly")
    spout_lid = object_model.get_part("spout_lid")
    button = object_model.get_part("power_button")
    blade_joint = object_model.get_articulation("cup_to_blade")
    lid_joint = object_model.get_articulation("cup_to_spout_lid")
    button_joint = object_model.get_articulation("base_to_power_button")

    ctx.allow_overlap(
        base,
        button,
        elem_a="base_shell",
        elem_b="button_cap",
        reason="The spring pushbutton cap is intentionally seated a few millimeters into the motor-base front housing.",
    )
    ctx.allow_overlap(
        base,
        button,
        elem_a="base_shell",
        elem_b="button_stem",
        reason="The pushbutton stem is intentionally captured inside the hidden switch pocket of the motor base.",
    )

    ctx.expect_gap(
        cup,
        base,
        axis="z",
        positive_elem="threaded_collar",
        negative_elem="top_shoulder",
        max_penetration=0.000001,
        max_gap=0.001,
        name="threaded cup collar sits on the base shoulder",
    )
    ctx.expect_within(
        blade,
        cup,
        axes="xy",
        inner_elem="hub",
        outer_elem="threaded_collar",
        margin=0.0,
        name="blade hub is centered inside the threaded cup mouth",
    )
    ctx.expect_gap(
        spout_lid,
        cup,
        axis="z",
        positive_elem="flap_panel",
        negative_elem="spout_seal_rim",
        min_gap=0.0,
        max_gap=0.002,
        name="closed flip spout rests just above the drinking seal",
    )
    ctx.expect_gap(
        base,
        button,
        axis="y",
        positive_elem="base_shell",
        negative_elem="button_cap",
        max_penetration=0.006,
        name="button cap has a shallow intentional seated insertion",
    )
    ctx.expect_gap(
        base,
        button,
        axis="y",
        positive_elem="base_shell",
        negative_elem="button_stem",
        max_penetration=0.007,
        name="button stem remains captured in the switch pocket",
    )

    closed_lid_aabb = ctx.part_world_aabb(spout_lid)
    with ctx.pose({lid_joint: 1.35}):
        open_lid_aabb = ctx.part_world_aabb(spout_lid)
    ctx.check(
        "flip spout opens upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.025,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_button_pos = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.006}):
        pressed_button_pos = ctx.part_world_position(button)
    ctx.check(
        "power button travels inward toward the base",
        closed_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > closed_button_pos[1] + 0.004,
        details=f"closed={closed_button_pos}, pressed={pressed_button_pos}",
    )

    blade_origin = ctx.part_world_position(blade)
    with ctx.pose({blade_joint: math.pi / 2.0}):
        spun_blade_origin = ctx.part_world_position(blade)
    ctx.check(
        "blade spins about the cup-mouth axis without translating",
        blade_origin is not None
        and spun_blade_origin is not None
        and abs(blade_origin[0] - spun_blade_origin[0]) < 1e-6
        and abs(blade_origin[1] - spun_blade_origin[1]) < 1e-6
        and abs(blade_origin[2] - spun_blade_origin[2]) < 1e-6,
        details=f"rest={blade_origin}, spun={spun_blade_origin}",
    )

    return ctx.report()


object_model = build_object_model()
