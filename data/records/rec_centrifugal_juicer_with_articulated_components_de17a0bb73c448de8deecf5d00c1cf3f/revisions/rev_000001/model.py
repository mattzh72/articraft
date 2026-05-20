from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _perforated_basket_geometry(
    *,
    radius: float = 0.145,
    height: float = 0.145,
    bottom_z: float = 0.020,
    radial_segments: int = 72,
    vertical_segments: int = 18,
) -> MeshGeometry:
    """A single connected strainer basket mesh with a perforated side wall."""

    mesh = MeshGeometry()
    rings: list[list[int]] = []
    for j in range(vertical_segments + 1):
        z = bottom_z + height * j / vertical_segments
        ring: list[int] = []
        for i in range(radial_segments):
            angle = math.tau * i / radial_segments
            ring.append(mesh.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z))
        rings.append(ring)

    for j in range(vertical_segments):
        for i in range(radial_segments):
            next_i = (i + 1) % radial_segments
            band_is_rim = j in (0, 1, vertical_segments - 2, vertical_segments - 1)
            radial_web = (i % 6) in (0, 1)
            vertical_web = (j % 4) == 0
            if band_is_rim or radial_web or vertical_web:
                a = rings[j][i]
                b = rings[j][next_i]
                c = rings[j + 1][next_i]
                d = rings[j + 1][i]
                mesh.add_face(a, b, c)
                mesh.add_face(a, c, d)

    # Bottom strainer plate: a connected disk with open radial slots between spokes.
    center = mesh.add_vertex(0.0, 0.0, bottom_z)
    inner_ring: list[int] = []
    inner_radius = radius * 0.34
    for i in range(radial_segments):
        angle = math.tau * i / radial_segments
        inner_ring.append(mesh.add_vertex(inner_radius * math.cos(angle), inner_radius * math.sin(angle), bottom_z))
    for i in range(radial_segments):
        next_i = (i + 1) % radial_segments
        mesh.add_face(center, inner_ring[i], inner_ring[next_i])
        if (i % 6) in (0, 1, 2):
            mesh.add_face(inner_ring[i], rings[0][i], rings[0][next_i])
            mesh.add_face(inner_ring[i], rings[0][next_i], inner_ring[next_i])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_mouth_centrifugal_juicer")

    white_plastic = model.material("warm_white_plastic", rgba=(0.88, 0.87, 0.82, 1.0))
    dark_plastic = model.material("charcoal_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    clear_poly = model.material("clear_smoked_polycarbonate", rgba=(0.62, 0.82, 0.96, 0.36))
    faint_clear = model.material("faint_clear_rim", rgba=(0.70, 0.88, 1.0, 0.22))
    satin_steel = model.material("satin_stainless_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    brushed_steel = model.material("brushed_basket_steel", rgba=(0.80, 0.82, 0.80, 1.0))
    safety_orange = model.material("safety_orange_detail", rgba=(0.95, 0.42, 0.10, 1.0))

    body = model.part("body")
    body.visual(Box((0.66, 0.46, 0.13)), origin=Origin(xyz=(0.0, 0.0, 0.065)), material=white_plastic, name="low_base")
    body.visual(Box((0.56, 0.38, 0.105)), origin=Origin(xyz=(0.0, 0.0, 0.182)), material=white_plastic, name="body_shoulder")
    body.visual(Box((0.48, 0.035, 0.045)), origin=Origin(xyz=(0.0, -0.238, 0.105)), material=dark_plastic, name="front_control_band")
    body.visual(Box((0.075, 0.012, 0.020)), origin=Origin(xyz=(0.0, -0.258, 0.108)), material=safety_orange, name="power_switch")
    body.visual(Box((0.18, 0.055, 0.020)), origin=Origin(xyz=(0.0, -0.255, 0.055)), material=black_rubber, name="rubber_foot_front")
    body.visual(Box((0.18, 0.055, 0.020)), origin=Origin(xyz=(0.0, 0.255, 0.055)), material=black_rubber, name="rubber_foot_rear")

    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.128, 0.000),
            (0.205, 0.035),
            (0.220, 0.120),
            (0.210, 0.190),
            (0.194, 0.202),
        ],
        [
            (0.080, 0.020),
            (0.152, 0.055),
            (0.164, 0.120),
            (0.166, 0.180),
            (0.154, 0.192),
        ],
        segments=80,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    body.visual(
        _save_mesh(bowl_shell, "static_clear_bowl"),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=faint_clear,
        name="static_clear_bowl",
    )
    body.visual(Cylinder(radius=0.050, length=0.022), origin=Origin(xyz=(0.0, 0.0, 0.232)), material=satin_steel, name="drive_plate")
    body.visual(Cylinder(radius=0.034, length=0.070), origin=Origin(xyz=(0.0, 0.0, 0.215)), material=satin_steel, name="drive_spindle")
    body.visual(Box((0.090, 0.095, 0.045)), origin=Origin(xyz=(0.0, -0.244, 0.304)), material=faint_clear, name="juice_spout")

    # Rear hinge support and side locking pivots are fixed features of the body.
    body.visual(Box((0.045, 0.085, 0.170)), origin=Origin(xyz=(-0.150, 0.225, 0.315)), material=white_plastic, name="rear_hinge_lug_0")
    body.visual(Box((0.045, 0.085, 0.170)), origin=Origin(xyz=(0.150, 0.225, 0.315)), material=white_plastic, name="rear_hinge_lug_1")
    body.visual(Cylinder(radius=0.018, length=0.090), origin=Origin(xyz=(-0.150, 0.245, 0.392), rpy=(0.0, math.pi / 2.0, 0.0)), material=satin_steel, name="rear_hinge_pin_0")
    body.visual(Cylinder(radius=0.018, length=0.090), origin=Origin(xyz=(0.150, 0.245, 0.392), rpy=(0.0, math.pi / 2.0, 0.0)), material=satin_steel, name="rear_hinge_pin_1")
    body.visual(Box((0.082, 0.088, 0.075)), origin=Origin(xyz=(-0.308, -0.015, 0.245)), material=white_plastic, name="left_pivot_plinth")
    body.visual(Box((0.082, 0.088, 0.075)), origin=Origin(xyz=(0.308, -0.015, 0.245)), material=white_plastic, name="right_pivot_plinth")
    body.visual(Cylinder(radius=0.030, length=0.035), origin=Origin(xyz=(-0.337, -0.015, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_plastic, name="left_pivot_boss")
    body.visual(Cylinder(radius=0.030, length=0.035), origin=Origin(xyz=(0.337, -0.015, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_plastic, name="right_pivot_boss")

    basket = model.part("basket")
    basket.visual(_save_mesh(_perforated_basket_geometry(), "perforated_basket"), material=brushed_steel, name="perforated_basket")
    basket.visual(Cylinder(radius=0.041, length=0.034), origin=Origin(xyz=(0.0, 0.0, 0.017)), material=satin_steel, name="basket_hub")
    for blade_index in range(8):
        angle = math.tau * blade_index / 8.0
        radius_mid = 0.082
        basket.visual(
            Box((0.095, 0.012, 0.008)),
            origin=Origin(
                xyz=(radius_mid * math.cos(angle), radius_mid * math.sin(angle), 0.024),
                rpy=(0.0, 0.0, angle + 0.18),
            ),
            material=satin_steel,
            name=f"cutting_tooth_{blade_index}",
        )

    lid = model.part("lid")
    lid_shell = LatheGeometry.from_shell_profiles(
        [
            (0.074, 0.050),
            (0.118, 0.078),
            (0.190, 0.086),
            (0.224, 0.055),
            (0.238, 0.023),
        ],
        [
            (0.074, 0.030),
            (0.118, 0.052),
            (0.186, 0.057),
            (0.213, 0.035),
            (0.225, 0.017),
        ],
        segments=88,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    lid.visual(_save_mesh(lid_shell, "clear_lid_shell"), origin=Origin(xyz=(0.0, -0.245, 0.0)), material=clear_poly, name="clear_lid_shell")
    chute_shell = LatheGeometry.from_shell_profiles(
        [
            (0.126, 0.075),
            (0.106, 0.092),
            (0.101, 0.395),
            (0.096, 0.415),
        ],
        [
            (0.078, 0.075),
            (0.077, 0.100),
            (0.077, 0.395),
            (0.074, 0.407),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=6,
    )
    lid.visual(_save_mesh(chute_shell, "wide_feed_chute"), origin=Origin(xyz=(0.0, -0.245, 0.0)), material=clear_poly, name="wide_feed_chute")
    lid.visual(Box((0.170, 0.030, 0.026)), origin=Origin(xyz=(0.0, -0.020, 0.020)), material=clear_poly, name="rear_hinge_leaf")
    lid.visual(Cylinder(radius=0.018, length=0.210), origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)), material=clear_poly, name="rear_hinge_barrel")

    pusher = model.part("feed_pusher")
    pusher.visual(Cylinder(radius=0.066, length=0.340), origin=Origin(xyz=(0.0, 0.0, -0.170)), material=dark_plastic, name="pusher_plunger")
    pusher.visual(Cylinder(radius=0.088, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=dark_plastic, name="pusher_cap")
    pusher.visual(Cylinder(radius=0.035, length=0.055), origin=Origin(xyz=(0.0, 0.0, 0.0675)), material=black_rubber, name="pusher_grip")

    left_arm = model.part("left_locking_arm")
    left_arm.visual(Cylinder(radius=0.028, length=0.030), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=satin_steel, name="pivot_hub")
    left_arm.visual(Box((0.026, 0.036, 0.290)), origin=Origin(xyz=(0.0, 0.0, 0.158)), material=satin_steel, name="upright_strap")
    left_arm.visual(Box((0.160, 0.036, 0.026)), origin=Origin(xyz=(0.080, 0.0, 0.315)), material=satin_steel, name="top_clamp")
    left_arm.visual(Box((0.034, 0.050, 0.052)), origin=Origin(xyz=(0.150, 0.0, 0.282)), material=black_rubber, name="clamp_pad")

    right_arm = model.part("right_locking_arm")
    right_arm.visual(Cylinder(radius=0.028, length=0.030), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=satin_steel, name="pivot_hub")
    right_arm.visual(Box((0.026, 0.036, 0.290)), origin=Origin(xyz=(0.0, 0.0, 0.158)), material=satin_steel, name="upright_strap")
    right_arm.visual(Box((0.160, 0.036, 0.026)), origin=Origin(xyz=(-0.080, 0.0, 0.315)), material=satin_steel, name="top_clamp")
    right_arm.visual(Box((0.034, 0.050, 0.052)), origin=Origin(xyz=(-0.150, 0.0, 0.282)), material=black_rubber, name="clamp_pad")

    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.245, 0.392)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.2, lower=0.0, upper=1.20),
    )
    model.articulation(
        "pusher_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.245, 0.415)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.30, lower=0.0, upper=0.120),
    )
    model.articulation(
        "left_arm_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_arm,
        origin=Origin(xyz=(-0.3695, -0.015, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.4, lower=-1.20, upper=0.0),
    )
    model.articulation(
        "right_arm_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_arm,
        origin=Origin(xyz=(0.3695, -0.015, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.4, lower=-1.20, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    left_arm = object_model.get_part("left_locking_arm")
    right_arm = object_model.get_part("right_locking_arm")

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_plunger",
        outer_elem="wide_feed_chute",
        margin=0.0,
        name="pusher centered within broad chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_plunger",
        elem_b="wide_feed_chute",
        min_overlap=0.28,
        name="pusher remains deeply inserted",
    )
    ctx.expect_within(
        basket,
        body,
        axes="xy",
        inner_elem="perforated_basket",
        outer_elem="static_clear_bowl",
        margin=0.0,
        name="basket fits inside clear bowl",
    )
    ctx.expect_origin_distance(left_arm, right_arm, axes="x", min_dist=0.72, max_dist=0.78, name="locking arms are matched across shoulders")

    lid_hinge = object_model.get_articulation("lid_hinge")
    pusher_slide = object_model.get_articulation("pusher_slide")
    left_pivot = object_model.get_articulation("left_arm_pivot")
    basket_spin = object_model.get_articulation("basket_spin")

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="wide_feed_chute")
    with ctx.pose({lid_hinge: 0.85}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="wide_feed_chute")
    ctx.check(
        "lid hinge lifts the clear cover",
        closed_lid_aabb is not None and opened_lid_aabb is not None and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.04,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.10}):
        pusher_raised = ctx.part_world_position(pusher)
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_plunger",
            elem_b="wide_feed_chute",
            min_overlap=0.18,
            name="raised pusher retains insertion",
        )
    ctx.check(
        "feed pusher slides upward along chute",
        pusher_rest is not None and pusher_raised is not None and pusher_raised[2] > pusher_rest[2] + 0.09,
        details=f"rest={pusher_rest}, raised={pusher_raised}",
    )

    arm_rest = ctx.part_element_world_aabb(left_arm, elem="top_clamp")
    with ctx.pose({left_pivot: -0.9}):
        arm_swung = ctx.part_element_world_aabb(left_arm, elem="top_clamp")
    ctx.check(
        "locking arm rotates down from clamp position",
        arm_rest is not None and arm_swung is not None and arm_swung[0][2] < arm_rest[0][2] - 0.08,
        details=f"rest={arm_rest}, swung={arm_swung}",
    )

    with ctx.pose({basket_spin: math.pi / 2.0}):
        ctx.expect_within(
            basket,
            body,
            axes="xy",
            inner_elem="perforated_basket",
            outer_elem="static_clear_bowl",
            margin=0.0,
            name="spinning basket stays on vertical drive axis",
        )

    return ctx.report()


object_model = build_object_model()
