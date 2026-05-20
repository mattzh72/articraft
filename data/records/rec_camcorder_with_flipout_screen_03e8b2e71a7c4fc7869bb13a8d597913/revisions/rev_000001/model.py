from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_camcorder")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.058, 0.064, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    graphite = model.material("graphite_control", rgba=(0.17, 0.18, 0.19, 1.0))
    glass = model.material("blue_black_glass", rgba=(0.025, 0.055, 0.075, 0.92))
    lens_glass = model.material("coated_lens_glass", rgba=(0.015, 0.025, 0.038, 0.82))
    metal = model.material("hinge_dark_metal", rgba=(0.11, 0.115, 0.12, 1.0))

    body = model.part("body")

    # Slim consumer camcorder body: about 16 cm long, 7 cm wide, 9 cm tall.
    body_shell = superellipse_side_loft(
        (
            (-0.035, -0.040, 0.045, 0.160),
            (0.035, -0.040, 0.045, 0.160),
        ),
        exponents=3.4,
        segments=56,
        cap=True,
    )
    body.visual(
        mesh_from_geometry(body_shell, "rounded_body_shell"),
        material=satin_black,
        name="main_shell",
    )

    # Raised top shell and carry handle, with the mode dial placed behind it.
    body.visual(
        Box((0.130, 0.047, 0.011)),
        origin=Origin(xyz=(0.000, 0.000, 0.0505)),
        material=dark_plastic,
        name="top_shell",
    )
    body.visual(
        Box((0.012, 0.030, 0.034)),
        origin=Origin(xyz=(-0.010, 0.000, 0.070)),
        material=satin_black,
        name="handle_rear_post",
    )
    body.visual(
        Box((0.012, 0.030, 0.034)),
        origin=Origin(xyz=(0.048, 0.000, 0.070)),
        material=satin_black,
        name="handle_front_post",
    )
    body.visual(
        Box((0.086, 0.032, 0.014)),
        origin=Origin(xyz=(0.019, 0.000, 0.088)),
        material=dark_plastic,
        name="top_handle",
    )
    body.visual(
        Box((0.070, 0.036, 0.004)),
        origin=Origin(xyz=(0.019, 0.000, 0.097)),
        material=rubber,
        name="handle_grip_pad",
    )

    # Prominent forward lens assembly. Cylinders are rotated so their local Z axis
    # points along the camcorder's forward +X optical axis.
    body.visual(
        Cylinder(radius=0.039, length=0.014),
        origin=Origin(xyz=(0.083, 0.000, 0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="lens_mount_flange",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.093, 0.000, 0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="rear_lens_barrel",
    )
    # A side hinge socket that visibly carries the flip screen spine.
    body.visual(
        Box((0.014, 0.006, 0.070)),
        origin=Origin(xyz=(0.058, 0.038, 0.003)),
        material=metal,
        name="screen_hinge_socket",
    )
    body.visual(
        Box((0.078, 0.004, 0.050)),
        origin=Origin(xyz=(-0.006, 0.037, 0.004)),
        material=dark_plastic,
        name="side_screen_recess",
    )

    # Continuous focus ring: a hollow ridged sleeve captured between the fixed
    # rear and front barrel shoulders.
    outer_profile: list[tuple[float, float]] = []
    half_len = 0.011
    rib_count = 6
    for i in range(rib_count * 2 + 1):
        z = -half_len + (2.0 * half_len * i) / (rib_count * 2)
        radius = 0.0375 if i % 2 == 0 else 0.0358
        outer_profile.append((radius, z))
    focus_ring_mesh = LatheGeometry.from_shell_profiles(
        outer_profile,
        # Match the inner sleeve to the barrel radius so the rotating ring reads
        # as a supported bearing fit instead of a floating sleeve.
        [(0.0300, -half_len), (0.0300, half_len)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(focus_ring_mesh, "focus_ring_shell"),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="ring_shell",
    )
    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.119, 0.000, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    # The front optical nose is fixed to the camera barrel but authored as its
    # own part so the rotating focus ring can sit between real shoulders.
    front_lens = model.part("front_lens")
    front_lens.visual(
        Cylinder(radius=0.030, length=0.036),
        origin=Origin(xyz=(0.018, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="front_lens_barrel",
    )
    front_lens.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.037, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens_glass",
    )
    model.articulation(
        "body_to_front_lens",
        ArticulationType.FIXED,
        parent=body,
        child=front_lens,
        origin=Origin(xyz=(0.130, 0.000, 0.004)),
    )

    # Flip-out LCD panel on the left (+Y) side. Its frame is placed on the
    # vertical hinge spine; the closed panel extends rearward along local -X.
    screen = model.part("flip_screen")
    screen.visual(
        Cylinder(radius=0.004, length=0.067),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, 0.0, 0.0)),
        material=metal,
        name="hinge_spine",
    )
    screen.visual(
        Box((0.096, 0.007, 0.058)),
        origin=Origin(xyz=(-0.048, 0.004, 0.000)),
        material=dark_plastic,
        name="screen_panel",
    )
    screen.visual(
        Box((0.078, 0.002, 0.043)),
        origin=Origin(xyz=(-0.050, 0.0082, 0.001)),
        material=glass,
        name="display_glass",
    )
    screen.visual(
        Box((0.092, 0.002, 0.054)),
        origin=Origin(xyz=(-0.048, 0.0002, 0.000)),
        material=rubber,
        name="inner_bezel",
    )
    model.articulation(
        "body_to_flip_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(0.058, 0.045, 0.003)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.0, lower=0.0, upper=1.75),
    )

    # Low circular mode dial on the top shell, behind the handle.
    dial_mesh = KnobGeometry(
        0.032,
        0.010,
        body_style="cylindrical",
        edge_radius=0.0008,
        grip=KnobGrip(style="ribbed", count=20, depth=0.0008, width=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        mesh_from_geometry(dial_mesh, "mode_dial_cap"),
        material=graphite,
        name="dial_cap",
    )
    model.articulation(
        "top_to_mode_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.052, 0.000, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.18, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    screen = object_model.get_part("flip_screen")
    focus_ring = object_model.get_part("focus_ring")
    front_lens = object_model.get_part("front_lens")
    mode_dial = object_model.get_part("mode_dial")
    screen_hinge = object_model.get_articulation("body_to_flip_screen")
    focus_spin = object_model.get_articulation("barrel_to_focus_ring")
    dial_spin = object_model.get_articulation("top_to_mode_dial")

    ctx.check(
        "focus ring is continuous",
        focus_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="The lens focus ring should spin without stops.",
    )
    ctx.check(
        "mode dial is continuous",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="The top mode dial should be a distinct continuously rotating control.",
    )

    with ctx.pose({screen_hinge: 0.0}):
        ctx.expect_gap(
            screen,
            body,
            axis="y",
            max_gap=0.003,
            max_penetration=0.00005,
            elem_a="hinge_spine",
            elem_b="screen_hinge_socket",
            name="closed flip screen hinge seats on side socket",
        )
        ctx.expect_overlap(
            screen,
            body,
            axes="z",
            elem_a="screen_panel",
            elem_b="side_screen_recess",
            min_overlap=0.040,
            name="closed screen covers the side display recess",
        )
        closed_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")

    with ctx.pose({screen_hinge: 1.35}):
        opened_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")

    ctx.check(
        "flip screen rotates outward",
        closed_aabb is not None
        and opened_aabb is not None
        and ((opened_aabb[0][1] + opened_aabb[1][1]) * 0.5)
        > ((closed_aabb[0][1] + closed_aabb[1][1]) * 0.5) + 0.025,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="dial_cap",
        negative_elem="top_shell",
        name="mode dial sits on top shell",
    )
    ctx.expect_gap(
        focus_ring,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="ring_shell",
        negative_elem="rear_lens_barrel",
        name="focus ring seats against rear barrel shoulder",
    )
    ctx.expect_gap(
        front_lens,
        focus_ring,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="front_lens_barrel",
        negative_elem="ring_shell",
        name="front barrel captures focus ring",
    )

    return ctx.report()


object_model = build_object_model()
