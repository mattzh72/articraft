from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _rounded_front_plate(width: float, height: float, depth: float, radius: float):
    """Rounded rectangle in X/Z, with its thickness along local Y."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        depth,
        cap=True,
        center=True,
    )
    return geom.rotate_x(-math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domestic_rice_cooker")

    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_trim = model.material("charcoal_trim", rgba=(0.03, 0.035, 0.04, 1.0))
    warm_gray = model.material("warm_gray_plastic", rgba=(0.28, 0.27, 0.25, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    nonstick = model.material("dark_nonstick_bowl", rgba=(0.10, 0.105, 0.11, 1.0))
    red = model.material("cook_red", rgba=(0.78, 0.05, 0.035, 1.0))
    amber = model.material("warm_amber", rgba=(1.0, 0.62, 0.12, 1.0))

    body = model.part("body")

    body_shell = LatheGeometry(
        [
            (0.0, 0.000),
            (0.116, 0.000),
            (0.145, 0.012),
            (0.162, 0.060),
            (0.166, 0.120),
            (0.158, 0.172),
            (0.146, 0.205),
            (0.0, 0.205),
        ],
        segments=96,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(body_shell, "rounded_metal_body"),
        material=brushed_metal,
        name="body_shell",
    )

    body.visual(
        mesh_from_geometry(
            TorusGeometry(0.138, 0.0065, radial_segments=96, tubular_segments=14).translate(
                0.0, 0.0, 0.208
            ),
            "black_top_rim",
        ),
        material=dark_trim,
        name="top_rim",
    )

    inner_bowl = LatheGeometry.from_shell_profiles(
        [(0.066, 0.126), (0.112, 0.162), (0.130, 0.198), (0.134, 0.206)],
        [(0.047, 0.132), (0.094, 0.164), (0.118, 0.195), (0.122, 0.199)],
        segments=80,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    body.visual(
        mesh_from_geometry(inner_bowl, "nonstick_inner_bowl"),
        material=nonstick,
        name="inner_bowl",
    )

    control_panel = _rounded_front_plate(0.132, 0.106, 0.012, 0.012)
    body.visual(
        mesh_from_geometry(control_panel, "control_panel"),
        origin=Origin(xyz=(0.0, -0.169, 0.105)),
        material=dark_trim,
        name="control_panel",
    )

    body.visual(
        mesh_from_geometry(_rounded_front_plate(0.087, 0.034, 0.010, 0.007), "latch_seat"),
        origin=Origin(xyz=(0.0, -0.153, 0.190)),
        material=dark_trim,
        name="latch_seat",
    )

    body.visual(
        Box((0.205, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.149, 0.196)),
        material=dark_trim,
        name="rear_hinge_bridge",
    )

    # Side pivot bosses and a raised bail-style carry handle, fused visually into the body assembly.
    for x in (-0.166, 0.166):
        body.visual(
            Cylinder(radius=0.017, length=0.030),
            origin=Origin(xyz=(x, 0.0, 0.158), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_gray,
            name=f"handle_boss_{'neg' if x < 0 else 'pos'}",
        )

    carry_handle = tube_from_spline_points(
        [
            (-0.176, 0.0, 0.158),
            (-0.176, 0.0, 0.286),
            (-0.098, 0.0, 0.360),
            (0.0, 0.0, 0.382),
            (0.098, 0.0, 0.360),
            (0.176, 0.0, 0.286),
            (0.176, 0.0, 0.158),
        ],
        radius=0.006,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    body.visual(
        mesh_from_geometry(carry_handle, "arched_carry_handle"),
        material=warm_gray,
        name="carry_handle",
    )

    for x in (-0.078, 0.078):
        body.visual(
            Box((0.048, 0.017, 0.026)),
            origin=Origin(xyz=(x, 0.156, 0.211)),
            material=dark_trim,
            name=f"rear_hinge_mount_{'neg' if x < 0 else 'pos'}",
        )
        body.visual(
            Cylinder(radius=0.0075, length=0.045),
            origin=Origin(xyz=(x, 0.162, 0.223), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_trim,
            name=f"rear_hinge_barrel_{'neg' if x < 0 else 'pos'}",
        )
    body.visual(
        Cylinder(radius=0.0035, length=0.205),
        origin=Origin(xyz=(0.0, 0.162, 0.223), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="hinge_pin",
    )

    for x, y in ((-0.090, -0.085), (0.090, -0.085), (-0.080, 0.090), (0.080, 0.090)):
        body.visual(
            Cylinder(radius=0.020, length=0.014),
            origin=Origin(xyz=(x, y, -0.004)),
            material=black_rubber,
            name=f"rubber_foot_{x:.2f}_{y:.2f}",
        )

    lid = model.part("lid")
    lid_shell = LatheGeometry(
        [
            (0.0, -0.006),
            (0.126, -0.006),
            (0.146, -0.002),
            (0.148, 0.012),
            (0.124, 0.032),
            (0.062, 0.044),
            (0.0, 0.047),
        ],
        segments=96,
        closed=True,
    ).translate(0.0, -0.162, 0.0)
    lid.visual(
        mesh_from_geometry(lid_shell, "slightly_domed_lid"),
        material=brushed_metal,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_geometry(
            TorusGeometry(0.145, 0.0035, radial_segments=96, tubular_segments=10).translate(
                0.0, -0.162, -0.002
            ),
            "lid_lower_rim",
        ),
        material=dark_trim,
        name="lid_rim",
    )
    lid.visual(
        Box((0.082, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, -0.014, 0.003)),
        material=dark_trim,
        name="lid_hinge_lug",
    )
    lid.visual(
        Cylinder(radius=0.0075, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.036, -0.085, 0.043)),
        material=dark_trim,
        name="steam_vent_cap",
    )
    for x in (0.030, 0.042):
        lid.visual(
            Cylinder(radius=0.0042, length=0.0012),
            origin=Origin(xyz=(x, -0.085, 0.048)),
            material=black_rubber,
            name=f"steam_vent_hole_{'a' if x < 0.036 else 'b'}",
        )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_geometry(_rounded_front_plate(0.068, 0.022, 0.018, 0.006), "front_latch_cap"),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=warm_gray,
        name="latch_cap",
    )

    cook_button = model.part("cook_button")
    cook_button.visual(
        mesh_from_geometry(_rounded_front_plate(0.040, 0.026, 0.013, 0.006), "cook_button_cap"),
        origin=Origin(xyz=(0.0, -0.0065, 0.0)),
        material=red,
        name="cook_cap",
    )

    warm_button = model.part("warm_button")
    warm_button.visual(
        mesh_from_geometry(_rounded_front_plate(0.040, 0.026, 0.013, 0.006), "warm_button_cap"),
        origin=Origin(xyz=(0.0, -0.0065, 0.0)),
        material=amber,
        name="warm_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.162, 0.223)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_latch",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(0.0, -0.158, 0.190)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.10, lower=0.0, upper=0.012),
    )
    model.articulation(
        "body_to_cook",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cook_button,
        origin=Origin(xyz=(-0.033, -0.175, 0.107)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.007),
    )
    model.articulation(
        "body_to_warm",
        ArticulationType.PRISMATIC,
        parent=body,
        child=warm_button,
        origin=Origin(xyz=(0.033, -0.175, 0.107)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.007),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch_button")
    cook = object_model.get_part("cook_button")
    warm = object_model.get_part("warm_button")

    lid_joint = object_model.get_articulation("body_to_lid")
    latch_joint = object_model.get_articulation("body_to_latch")
    cook_joint = object_model.get_articulation("body_to_cook")
    warm_joint = object_model.get_articulation("body_to_warm")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_pin",
        elem_b="lid_hinge_barrel",
        reason="A continuous hinge pin is intentionally captured inside the rotating lid hinge barrel.",
    )
    ctx.expect_within(
        body,
        lid,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="lid_hinge_barrel",
        margin=0.001,
        name="hinge pin is centered inside lid barrel",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="x",
        elem_a="hinge_pin",
        elem_b="lid_hinge_barrel",
        min_overlap=0.055,
        name="hinge pin spans the rotating lid barrel",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_rim",
        negative_elem="top_rim",
        min_gap=0.001,
        max_gap=0.010,
        name="closed lid rim sits just above body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="top_rim",
        min_overlap=0.10,
        name="closed lid covers the cooker opening",
    )
    ctx.expect_gap(
        body,
        latch,
        axis="y",
        positive_elem="latch_seat",
        negative_elem="latch_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="latch button is seated on its front latch pocket",
    )
    for button, cap_name in ((cook, "cook_cap"), (warm, "warm_cap")):
        ctx.expect_gap(
            body,
            button,
            axis="y",
            positive_elem="control_panel",
            negative_elem=cap_name,
            max_gap=0.001,
            max_penetration=0.0002,
            name=f"{button.name} sits proud of the control panel",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid rotates upward about the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    for joint, button, travel, label in (
        (latch_joint, latch, 0.012, "front latch"),
        (cook_joint, cook, 0.007, "cook button"),
        (warm_joint, warm, 0.007, "warm button"),
    ):
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: travel}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"{label} depresses into the body",
            rest is not None and pressed is not None and pressed[1] > rest[1] + travel * 0.8,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
