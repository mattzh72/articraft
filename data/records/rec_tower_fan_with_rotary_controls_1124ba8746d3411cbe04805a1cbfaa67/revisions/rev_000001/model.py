from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _translated_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_tower_fan")

    warm_white = model.material("warm_white", rgba=(0.86, 0.85, 0.80, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.58, 0.60, 0.61, 1.0))
    dark_grille = model.material("dark_grille", rgba=(0.045, 0.047, 0.052, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    rubber_gray = model.material("rubber_gray", rgba=(0.10, 0.105, 0.11, 1.0))
    pointer_white = model.material("pointer_white", rgba=(0.96, 0.95, 0.88, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                superellipse_profile(0.38, 0.30, exponent=2.35, segments=72),
                0.045,
                center=True,
            ),
            "oval_base",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=warm_white,
        name="oval_base",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=satin_gray,
        name="pivot_collar",
    )
    base.visual(
        Cylinder(radius=0.158, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber_gray,
        name="rubber_foot_ring",
    )

    body = model.part("body")
    # A single U-shaped open extrusion: side walls and rounded rear casing leave
    # the front open for the nearly full-height grille and the blower clearance.
    u_shell_profile = [
        (-0.082, -0.065),
        (-0.061, -0.065),
        (-0.061, 0.052),
        (0.061, 0.052),
        (0.061, -0.065),
        (0.082, -0.065),
        (0.082, 0.076),
        (0.066, 0.083),
        (-0.066, 0.083),
        (-0.082, 0.076),
    ]
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(u_shell_profile, 0.780),
            "open_body_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=warm_white,
        name="open_body_shell",
    )
    body.visual(
        Cylinder(radius=0.049, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=satin_gray,
        name="rotating_neck",
    )
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                superellipse_profile(0.172, 0.142, exponent=2.6, segments=48),
                0.034,
                center=True,
            ),
            "lower_transition",
        ),
        origin=Origin(xyz=(0.0, 0.004, 0.084)),
        material=warm_white,
        name="lower_transition",
    )
    body.visual(
        Box((0.038, 0.036, 0.012)),
        origin=Origin(xyz=(0.0, 0.042, 0.116)),
        material=dark_grille,
        name="lower_bearing_bridge",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.002, 0.116)),
        material=dark_grille,
        name="lower_bearing",
    )
    body.visual(
        Box((0.038, 0.036, 0.012)),
        origin=Origin(xyz=(0.0, 0.042, 0.842)),
        material=dark_grille,
        name="upper_bearing_bridge",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.002, 0.842)),
        material=dark_grille,
        name="upper_bearing",
    )

    grille = model.part("grille")
    grille.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.135, 0.735),
                0.004,
                slot_size=(0.052, 0.0046),
                pitch=(0.0125, 0.064),
                frame=0.012,
                corner_radius=0.020,
                slot_angle_deg=88.0,
                stagger=True,
            ),
            "front_grille",
        ),
        # Local panel Z becomes world -Y, so the rear face seats on the open shell.
        origin=Origin(xyz=(0.0, -0.067, 0.472), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_grille,
        name="front_grille",
    )
    grille.visual(
        Box((0.006, 0.003, 0.690)),
        origin=Origin(xyz=(0.0, -0.0705, 0.472)),
        material=satin_gray,
        name="center_divider",
    )

    top_cap = model.part("top_cap")
    top_outer = superellipse_profile(0.188, 0.156, exponent=2.55, segments=72)
    handle_hole = _translated_profile(
        rounded_rect_profile(0.096, 0.046, 0.018, corner_segments=10),
        0.0,
        0.037,
    )
    top_cap.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(top_outer, [handle_hole], 0.040, center=True),
            "top_cap_handle",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=warm_white,
        name="top_cap_handle",
    )
    top_cap.visual(
        Box((0.120, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.061, 0.018)),
        material=satin_gray,
        name="rear_bridge_lip",
    )

    blower = model.part("blower")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.043,
                0.018,
                0.700,
                34,
                blade_thickness=0.0019,
                blade_sweep_deg=31.0,
                backplate=True,
                shroud=True,
            ),
            "vertical_blower_wheel",
        ),
        material=black_plastic,
        name="vertical_blower_wheel",
    )
    blower.visual(
        Cylinder(radius=0.006, length=0.740),
        origin=Origin(),
        material=satin_gray,
        name="axle",
    )
    blower.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(),
        material=black_plastic,
        name="center_hub",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.024,
            body_style="cylindrical",
            edge_radius=0.0012,
            grip=KnobGrip(style="knurled", count=40, depth=0.0010, helix_angle_deg=24.0),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "knurled_dial",
    )

    for index, x_pos in enumerate((-0.038, 0.038)):
        dial = model.part(f"dial_{index}")
        dial.visual(
            knob_mesh,
            origin=Origin(),
            material=satin_gray,
            name="knurled_dial",
        )
        dial.visual(
            Box((0.017, 0.003, 0.0012)),
            origin=Origin(xyz=(0.008, 0.0, 0.0246)),
            material=pointer_white,
            name="pointer_line",
        )
        model.articulation(
            f"dial_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=top_cap,
            child=dial,
            origin=Origin(xyz=(x_pos, -0.026, 0.040)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=6.0),
        )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "body_to_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
        origin=Origin(),
    )
    model.articulation(
        "body_to_top_cap",
        ArticulationType.FIXED,
        parent=body,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.855)),
    )
    model.articulation(
        "blower_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.002, 0.478)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=60.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    grille = object_model.get_part("grille")
    top_cap = object_model.get_part("top_cap")
    blower = object_model.get_part("blower")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")
    body_joint = object_model.get_articulation("base_to_body")
    blower_joint = object_model.get_articulation("blower_spin")

    ctx.allow_overlap(
        body,
        blower,
        elem_a="lower_bearing",
        elem_b="axle",
        reason="The spinning blower axle is intentionally captured inside the lower bearing sleeve.",
    )
    ctx.allow_overlap(
        body,
        blower,
        elem_a="upper_bearing",
        elem_b="axle",
        reason="The spinning blower axle is intentionally captured inside the upper bearing sleeve.",
    )

    ctx.check(
        "body_oscillation_is_limited_revolute",
        body_joint.articulation_type == ArticulationType.REVOLUTE
        and body_joint.motion_limits is not None
        and body_joint.motion_limits.lower < -0.5
        and body_joint.motion_limits.upper > 0.5,
        details=f"type={body_joint.articulation_type}, limits={body_joint.motion_limits}",
    )
    ctx.check(
        "blower_is_continuous",
        blower_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blower_joint.articulation_type}",
    )
    for name in ("dial_0_spin", "dial_1_spin"):
        joint = object_model.get_articulation(name)
        ctx.check(
            f"{name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )

    ctx.expect_gap(
        body,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rotating_neck",
        negative_elem="pivot_collar",
        name="rotating neck sits on base collar",
    )
    ctx.expect_gap(
        body,
        grille,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0002,
        positive_elem="open_body_shell",
        negative_elem="front_grille",
        name="front grille seats on shell lip without burying",
    )
    ctx.expect_gap(
        top_cap,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="top_cap_handle",
        negative_elem="open_body_shell",
        name="top cap closes the body shell",
    )
    ctx.expect_gap(
        dial_0,
        top_cap,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="knurled_dial",
        negative_elem="top_cap_handle",
        name="first dial sits on top cap",
    )
    ctx.expect_gap(
        dial_1,
        top_cap,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="knurled_dial",
        negative_elem="top_cap_handle",
        name="second dial sits on top cap",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        margin=0.0,
        inner_elem="vertical_blower_wheel",
        outer_elem="open_body_shell",
        name="blower wheel stays inside slim casing footprint",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        margin=0.0,
        inner_elem="axle",
        outer_elem="lower_bearing",
        name="axle is centered in lower bearing",
    )
    ctx.expect_overlap(
        blower,
        body,
        axes="z",
        min_overlap=0.010,
        elem_a="axle",
        elem_b="lower_bearing",
        name="axle remains captured by lower bearing",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        margin=0.0,
        inner_elem="axle",
        outer_elem="upper_bearing",
        name="axle is centered in upper bearing",
    )
    ctx.expect_overlap(
        blower,
        body,
        axes="z",
        min_overlap=0.010,
        elem_a="axle",
        elem_b="upper_bearing",
        name="axle remains captured by upper bearing",
    )
    ctx.expect_overlap(
        grille,
        body,
        axes="z",
        min_overlap=0.70,
        elem_a="front_grille",
        elem_b="open_body_shell",
        name="front grille runs nearly full body height",
    )

    rest_pos = ctx.part_world_position(dial_1)
    with ctx.pose({body_joint: 0.70}):
        turned_pos = ctx.part_world_position(dial_1)
    ctx.check(
        "oscillation carries top controls sideways",
        rest_pos is not None
        and turned_pos is not None
        and abs(turned_pos[0] - rest_pos[0]) > 0.005
        and abs(turned_pos[1] - rest_pos[1]) > 0.010,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
