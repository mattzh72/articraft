from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
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
    rounded_rect_profile,
    section_loft,
)


def _circle_profile(radius: float, *, center=(0.0, 0.0), segments: int = 64):
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _ellipse_profile(
    rx: float,
    ry: float,
    *,
    center=(0.0, 0.0),
    segments: int = 80,
):
    cx, cy = center
    return [
        (
            cx + rx * math.cos(2.0 * math.pi * i / segments),
            cy + ry * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_household_juicer")

    warm_white = model.material("warm_white", rgba=(0.92, 0.90, 0.84, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.035, 0.037, 0.040, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.74, 0.90, 1.0, 0.34))
    clear_blue = model.material("clear_blue", rgba=(0.66, 0.86, 1.0, 0.42))
    steel = model.material("brushed_steel", rgba=(0.78, 0.78, 0.72, 1.0))
    orange = model.material("orange_accent", rgba=(1.0, 0.42, 0.10, 1.0))

    body = model.part("body")

    # Rounded counter footprint and a tall tapered motor housing.
    base_geom = ExtrudeGeometry(
        rounded_rect_profile(0.40, 0.32, 0.055, corner_segments=10),
        0.070,
        center=False,
    )
    body.visual(
        mesh_from_geometry(base_geom, "base_foot"),
        material=warm_white,
        name="base_foot",
    )

    def body_section(width: float, depth: float, z: float, radius: float):
        return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)]

    shell_geom = section_loft(
        [
            body_section(0.28, 0.24, 0.060, 0.045),
            body_section(0.22, 0.18, 0.210, 0.040),
            body_section(0.27, 0.22, 0.340, 0.045),
        ]
    )
    body.visual(
        mesh_from_geometry(shell_geom, "motor_housing"),
        material=warm_white,
        name="motor_housing",
    )
    body.visual(
        Box((0.130, 0.073, 0.110)),
        origin=Origin(xyz=(0.0, -0.1265, 0.205)),
        material=satin_gray,
        name="front_control_panel",
    )

    shoulder_geom = ExtrudeGeometry(
        rounded_rect_profile(0.34, 0.16, 0.035, corner_segments=8),
        0.055,
        center=False,
    )
    body.visual(
        mesh_from_geometry(shoulder_geom, "upper_shoulders"),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=warm_white,
        name="upper_shoulders",
    )

    chamber_outer = [
        (0.104, 0.000),
        (0.122, 0.012),
        (0.128, 0.100),
        (0.120, 0.235),
        (0.128, 0.252),
    ]
    chamber_inner = [
        (0.088, 0.006),
        (0.109, 0.020),
        (0.113, 0.105),
        (0.106, 0.228),
        (0.112, 0.246),
    ]
    chamber_geom = LatheGeometry.from_shell_profiles(
        chamber_outer,
        chamber_inner,
        segments=72,
        lip_samples=6,
    )
    body.visual(
        mesh_from_geometry(chamber_geom, "clear_chamber"),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=clear_blue,
        name="clear_chamber",
    )

    body.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.331)),
        material=satin_gray,
        name="drive_socket",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.358)),
        material=satin_gray,
        name="drive_spindle",
    )

    # Side pivot bosses and rear hinge bracket are integral with the body.
    for sign, boss_name in [(-1.0, "left_pivot_boss"), (1.0, "right_pivot_boss")]:
        body.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(
                xyz=(sign * 0.172, 0.0, 0.372),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_gray,
            name=boss_name,
        )

    body.visual(
        Box((0.28, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, 0.132, 0.565)),
        material=warm_white,
        name="rear_hinge_tower",
    )
    for x, barrel_name in [(-0.112, "rear_hinge_barrel_0"), (0.112, "rear_hinge_barrel_1")]:
        body.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(x, 0.132, 0.602), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_gray,
            name=barrel_name,
        )

    basket = model.part("basket")
    basket.visual(
        Cylinder(radius=0.014, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=steel,
        name="basket_shaft",
    )
    basket.visual(
        Cylinder(radius=0.070, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=steel,
        name="grater_disc",
    )
    sieve_outer = [(0.052, 0.070), (0.088, 0.135), (0.098, 0.205), (0.102, 0.218)]
    sieve_inner = [(0.045, 0.074), (0.080, 0.136), (0.090, 0.204), (0.093, 0.214)]
    sieve_geom = LatheGeometry.from_shell_profiles(sieve_outer, sieve_inner, segments=72)
    basket.visual(
        mesh_from_geometry(sieve_geom, "sieve_cone"),
        material=steel,
        name="sieve_cone",
    )
    basket.visual(
        mesh_from_geometry(TorusGeometry(0.098, 0.004, radial_segments=18, tubular_segments=72), "basket_top_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.216)),
        material=steel,
        name="basket_top_rim",
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.376)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=120.0),
    )

    lid = model.part("lid")
    lid_plate_geom = ExtrudeWithHolesGeometry(
        _ellipse_profile(0.162, 0.125, center=(0.0, -0.125), segments=96),
        [_circle_profile(0.038, center=(0.0, -0.125), segments=64)],
        0.032,
        center=False,
    )
    lid.visual(
        mesh_from_geometry(lid_plate_geom, "clear_lid_plate"),
        material=clear_smoke,
        name="clear_lid_plate",
    )

    chute_geom = LatheGeometry.from_shell_profiles(
        [(0.039, 0.000), (0.048, 0.010), (0.050, 0.218), (0.047, 0.236)],
        [(0.032, 0.004), (0.034, 0.020), (0.034, 0.214), (0.032, 0.232)],
        segments=64,
        lip_samples=5,
    )
    lid.visual(
        mesh_from_geometry(chute_geom, "feed_chute"),
        origin=Origin(xyz=(0.0, -0.125, 0.032)),
        material=clear_smoke,
        name="feed_chute",
    )
    lid.visual(
        Box((0.130, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, -0.006, 0.012)),
        material=clear_smoke,
        name="hinge_lug",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.124),
        origin=Origin(xyz=(0.0, 0.000, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_smoke,
        name="lid_hinge_barrel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.132, 0.602)),
        # The closed lid extends toward local -Y from the rear hinge;
        # -X makes positive travel lift the front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.027, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=dark_plastic,
        name="pusher_plunger",
    )
    pusher.visual(
        Cylinder(radius=0.046, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=orange,
        name="pusher_cap",
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.125, 0.268)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.35, lower=0.0, upper=0.150),
    )

    for side_name, sign, x0 in [
        ("left_lock_arm", -1.0, -0.188),
        ("right_lock_arm", 1.0, 0.188),
    ]:
        arm = model.part(side_name)
        arm.visual(
            Cylinder(radius=0.019, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_gray,
            name="side_pivot",
        )
        arm.visual(
            Box((0.018, 0.020, 0.250)),
            origin=Origin(xyz=(0.0, 0.0, 0.126)),
            material=satin_gray,
            name="upright_strap",
        )
        arm.visual(
            Box((0.018, 0.052, 0.038)),
            origin=Origin(xyz=(0.0, -0.006, 0.260)),
            material=satin_gray,
            name="lid_clasp_pad",
        )
        arm.visual(
            Box((0.035, 0.020, 0.016)),
            origin=Origin(xyz=(-sign * 0.010, -0.006, 0.282)),
            material=satin_gray,
            name="top_hook",
        )
        model.articulation(
            f"body_to_{side_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=(x0, 0.0, 0.372)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.5,
                lower=math.radians(-45.0),
                upper=math.radians(25.0),
            ),
        )

    knob = model.part("selector_knob")
    knob_geom = KnobGeometry(
        0.060,
        0.032,
        body_style="skirted",
        top_diameter=0.046,
        edge_radius=0.002,
        grip=KnobGrip(style="fluted", count=18, depth=0.0015),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "selector_knob"),
        material=dark_plastic,
        name="selector_knob",
    )

    model.articulation(
        "body_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, -0.163, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=5.0,
            lower=math.radians(-130.0),
            upper=math.radians(130.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    knob = object_model.get_part("selector_knob")
    left_arm = object_model.get_part("left_lock_arm")
    right_arm = object_model.get_part("right_lock_arm")

    lid_joint = object_model.get_articulation("body_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    basket_joint = object_model.get_articulation("body_to_basket")
    left_joint = object_model.get_articulation("body_to_left_lock_arm")
    right_joint = object_model.get_articulation("body_to_right_lock_arm")

    ctx.check(
        "required user mechanisms exist",
        all(
            joint is not None
            for joint in [
                lid_joint,
                pusher_joint,
                knob_joint,
                basket_joint,
                left_joint,
                right_joint,
            ]
        ),
    )
    ctx.check(
        "basket spins on vertical axis",
        basket_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={basket_joint.axis}",
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_plunger",
        outer_elem="feed_chute",
        margin=0.006,
        name="pusher is centered inside the feed chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_plunger",
        elem_b="feed_chute",
        min_overlap=0.18,
        name="pusher remains inserted through the chute",
    )

    rest_pusher_z = ctx.part_world_position(pusher)[2]
    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="clear_lid_plate")
    with ctx.pose({pusher_joint: 0.150}):
        raised_pusher_z = ctx.part_world_position(pusher)[2]
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_plunger",
            elem_b="feed_chute",
            min_overlap=0.05,
            name="raised pusher is still retained by chute",
        )
    ctx.check(
        "pusher slides upward",
        raised_pusher_z > rest_pusher_z + 0.10,
        details=f"rest_z={rest_pusher_z}, raised_z={raised_pusher_z}",
    )

    with ctx.pose({lid_joint: math.radians(65.0)}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="clear_lid_plate")
    ctx.check(
        "lid hinge lifts the front edge",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.06,
        details=f"closed={rest_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({left_joint: math.radians(-35.0), right_joint: math.radians(-35.0)}):
        ctx.expect_origin_gap(left_arm, body, axis="z", min_gap=-0.38, max_gap=0.40, name="left latch stays near shoulder")
        ctx.expect_origin_gap(right_arm, body, axis="z", min_gap=-0.38, max_gap=0.40, name="right latch stays near shoulder")

    with ctx.pose({knob_joint: math.radians(90.0), basket_joint: math.pi}):
        ctx.expect_contact(
            knob,
            body,
            elem_a="selector_knob",
            contact_tol=0.012,
            name="selector knob remains mounted on the lower front body",
        )
        ctx.expect_within(
            basket,
            body,
            axes="xy",
            inner_elem="sieve_cone",
            outer_elem="clear_chamber",
            margin=0.002,
            name="rotating basket stays inside chamber footprint",
        )

    return ctx.report()


object_model = build_object_model()
