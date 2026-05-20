from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_enclosure_body():
    body = cq.Workplane("XY").box(0.180, 0.060, 0.320)
    body = body.edges().fillet(0.008)
    antenna_clearance = cq.Workplane("XY").circle(0.0075).extrude(0.430, both=True)
    return body.cut(antenna_clearance)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    return mesh_from_cadquery(tube, name, tolerance=0.0007, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_rf_pendant_controller")

    body_mat = Material("charcoal_glass_filled_nylon", rgba=(0.055, 0.060, 0.058, 1.0))
    rubber_mat = Material("black_overmolded_rubber", rgba=(0.010, 0.011, 0.010, 1.0))
    panel_mat = Material("dark_recessed_front_panel", rgba=(0.020, 0.023, 0.025, 1.0))
    metal_mat = Material("brushed_stainless_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    red_mat = Material("emergency_stop_red", rgba=(0.90, 0.02, 0.01, 1.0))
    yellow_mat = Material("safety_yellow", rgba=(1.0, 0.78, 0.03, 1.0))
    green_mat = Material("green_momentary_button", rgba=(0.02, 0.55, 0.09, 1.0))
    white_mat = Material("white_momentary_button", rgba=(0.88, 0.88, 0.82, 1.0))
    led_mat = Material("lit_status_lens", rgba=(0.03, 0.85, 0.25, 1.0))
    guard_mat = Material("clear_smoked_polycarbonate", rgba=(0.58, 0.82, 0.95, 0.36))

    enclosure = model.part("enclosure")
    enclosure.visual(
        mesh_from_cadquery(_rounded_enclosure_body(), "rounded_enclosure", tolerance=0.001, angular_tolerance=0.08),
        material=body_mat,
        name="body_shell",
    )
    # Thick rubber overmold rails and end caps give the pendant its ruggedized silhouette.
    for x in (-0.092, 0.092):
        enclosure.visual(
            Box((0.020, 0.072, 0.292)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=rubber_mat,
            name=f"side_bumper_{0 if x < 0 else 1}",
        )
    enclosure.visual(
        Box((0.184, 0.073, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.158)),
        material=rubber_mat,
        name="end_bumper_0",
    )
    # The upper bumper is split around the antenna socket instead of blocking it.
    for x in (-0.058, 0.058):
        enclosure.visual(
            Box((0.062, 0.073, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.158)),
            material=rubber_mat,
            name=f"top_bumper_side_{0 if x < 0 else 1}",
        )
    for y in (-0.027, 0.027):
        enclosure.visual(
            Box((0.058, 0.019, 0.028)),
            origin=Origin(xyz=(0.0, y, 0.158)),
            material=rubber_mat,
            name=f"top_bumper_bar_{0 if y < 0 else 1}",
        )
    for x in (-0.104, 0.104):
        for z in (-0.085, -0.035, 0.035, 0.085):
            enclosure.visual(
                Box((0.010, 0.060, 0.010)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=rubber_mat,
                name=f"grip_rib_{len(enclosure.visuals)}",
            )

    # Recessed front face with fixed collars, status lens and an antenna socket.
    enclosure.visual(
        Box((0.125, 0.004, 0.232)),
        origin=Origin(xyz=(0.0, -0.0335, -0.010)),
        material=panel_mat,
        name="front_panel",
    )
    enclosure.visual(
        Box((0.075, 0.003, 0.022)),
        origin=Origin(xyz=(0.0, -0.0368, -0.112)),
        material=body_mat,
        name="rf_label_plate",
    )
    enclosure.visual(
        Sphere(0.006),
        origin=Origin(xyz=(0.050, -0.039, 0.102)),
        material=led_mat,
        name="status_led",
    )
    enclosure.visual(
        _tube_mesh(0.016, 0.0075, 0.066, "antenna_socket"),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=rubber_mat,
        name="antenna_socket",
    )
    enclosure.visual(
        _tube_mesh(0.034, 0.020, 0.004, "e_stop_collar"),
        origin=Origin(xyz=(0.0, -0.0353, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yellow_mat,
        name="e_stop_collar",
    )
    for idx, (x, z, mat) in enumerate(((-0.034, -0.070, green_mat), (0.034, -0.070, white_mat))):
        enclosure.visual(
            _tube_mesh(0.018, 0.0115, 0.0032, f"button_collar_{idx}"),
            origin=Origin(xyz=(x, -0.0352, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=body_mat,
            name=f"button_collar_{idx}",
        )

    # Safety-guard hinge hardware fixed to the enclosure.
    enclosure.visual(
        Box((0.126, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.0320, 0.095)),
        material=metal_mat,
        name="hinge_leaf",
    )
    for x in (-0.068, 0.068):
        enclosure.visual(
            Box((0.014, 0.018, 0.018)),
            origin=Origin(xyz=(x, -0.043, 0.095)),
            material=metal_mat,
            name=f"hinge_lug_{0 if x < 0 else 1}",
        )
    enclosure.visual(
        Cylinder(0.0032, 0.142),
        origin=Origin(xyz=(0.0, -0.043, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="guard_hinge_pin",
    )

    guard = model.part("guard")
    guard.visual(
        Cylinder(0.0065, 0.108),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_mat,
        name="guard_barrel",
    )
    guard.visual(
        Box((0.112, 0.004, 0.118)),
        origin=Origin(xyz=(0.0, -0.055, -0.060)),
        material=guard_mat,
        name="front_shield",
    )
    for x in (-0.060, 0.060):
        guard.visual(
            Box((0.008, 0.052, 0.118)),
            origin=Origin(xyz=(x, -0.029, -0.060)),
            material=guard_mat,
            name=f"side_cheek_{0 if x < 0 else 1}",
        )
    guard.visual(
        Box((0.120, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, -0.029, -0.122)),
        material=guard_mat,
        name="lower_lip",
    )
    guard.visual(
        Box((0.120, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, -0.029, -0.004)),
        material=guard_mat,
        name="upper_lip",
    )
    model.articulation(
        "guard_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=guard,
        origin=Origin(xyz=(0.0, -0.043, 0.095)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    e_stop = model.part("e_stop")
    e_stop.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.025,
                body_style="mushroom",
                base_diameter=0.036,
                top_diameter=0.052,
                crown_radius=0.004,
                edge_radius=0.001,
                center=False,
            ),
            "e_stop_mushroom",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_mat,
        name="mushroom",
    )
    model.articulation(
        "e_stop_push",
        ArticulationType.PRISMATIC,
        parent=enclosure,
        child=e_stop,
        origin=Origin(xyz=(0.0, -0.0393, 0.015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.010),
    )

    for idx, (x, z, mat) in enumerate(((-0.034, -0.070, green_mat), (0.034, -0.070, white_mat))):
        button = model.part(f"button_{idx}")
        button.visual(
            Cylinder(0.0125, 0.012),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name="cap",
        )
        model.articulation(
            f"button_{idx}_push",
            ArticulationType.PRISMATIC,
            parent=enclosure,
            child=button,
            origin=Origin(xyz=(x, -0.0384, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.006),
        )

    antenna = model.part("antenna")
    antenna.visual(
        Cylinder(0.0045, 0.270),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=metal_mat,
        name="lower_member",
    )
    antenna.visual(
        Cylinder(0.0032, 0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=metal_mat,
        name="middle_member",
    )
    antenna.visual(
        Cylinder(0.0018, 0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        material=metal_mat,
        name="tip_whip",
    )
    antenna.visual(
        Sphere(0.0042),
        origin=Origin(xyz=(0.0, 0.0, 0.372)),
        material=rubber_mat,
        name="tip_cap",
    )
    model.articulation(
        "antenna_slide",
        ArticulationType.PRISMATIC,
        parent=enclosure,
        child=antenna,
        origin=Origin(xyz=(0.0, 0.0, 0.222)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.35, lower=0.0, upper=0.140),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    enclosure = object_model.get_part("enclosure")
    guard = object_model.get_part("guard")
    e_stop = object_model.get_part("e_stop")
    antenna = object_model.get_part("antenna")
    guard_hinge = object_model.get_articulation("guard_hinge")
    antenna_slide = object_model.get_articulation("antenna_slide")

    ctx.allow_overlap(
        enclosure,
        guard,
        elem_a="guard_hinge_pin",
        elem_b="guard_barrel",
        reason="The visible steel hinge pin is intentionally captured inside the clear guard barrel.",
    )
    ctx.expect_within(
        enclosure,
        guard,
        axes="yz",
        inner_elem="guard_hinge_pin",
        outer_elem="guard_barrel",
        margin=0.001,
        name="hinge pin lies inside guard barrel bore envelope",
    )
    ctx.expect_overlap(
        enclosure,
        guard,
        axes="x",
        elem_a="guard_hinge_pin",
        elem_b="guard_barrel",
        min_overlap=0.100,
        name="hinge pin spans the guard barrel",
    )

    with ctx.pose({guard_hinge: 0.0}):
        ctx.expect_overlap(
            guard,
            e_stop,
            axes="xz",
            elem_a="front_shield",
            elem_b="mushroom",
            min_overlap=0.040,
            name="closed guard covers emergency stop face",
        )
        ctx.expect_gap(
            e_stop,
            guard,
            axis="y",
            positive_elem="mushroom",
            negative_elem="front_shield",
            min_gap=0.020,
            name="closed guard clears protruding e-stop cap",
        )
        closed_aabb = ctx.part_element_world_aabb(guard, elem="front_shield")

    with ctx.pose({guard_hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(guard, elem="front_shield")

    ctx.check(
        "safety guard pivots upward and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.045
        and open_aabb[0][1] < closed_aabb[0][1] - 0.010,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.expect_within(
        antenna,
        enclosure,
        axes="xy",
        inner_elem="lower_member",
        outer_elem="antenna_socket",
        margin=0.003,
        name="antenna lower member is centered in socket",
    )
    ctx.expect_overlap(
        antenna,
        enclosure,
        axes="z",
        elem_a="lower_member",
        elem_b="antenna_socket",
        min_overlap=0.055,
        name="retracted antenna remains deeply inserted",
    )
    retracted = ctx.part_world_aabb(antenna)
    with ctx.pose({antenna_slide: 0.140}):
        ctx.expect_overlap(
            antenna,
            enclosure,
            axes="z",
            elem_a="lower_member",
            elem_b="antenna_socket",
            min_overlap=0.025,
            name="extended antenna remains retained in socket",
        )
        extended = ctx.part_world_aabb(antenna)
    ctx.check(
        "antenna extends upward on prismatic slide",
        retracted is not None and extended is not None and extended[1][2] > retracted[1][2] + 0.120,
        details=f"retracted={retracted}, extended={extended}",
    )

    ctx.expect_gap(
        enclosure,
        e_stop,
        axis="y",
        positive_elem="e_stop_collar",
        negative_elem="mushroom",
        max_gap=0.001,
        max_penetration=0.0,
        name="e-stop mushroom seats against collar plane",
    )
    ctx.expect_overlap(
        e_stop,
        enclosure,
        axes="xz",
        elem_a="mushroom",
        elem_b="e_stop_collar",
        min_overlap=0.045,
        name="e-stop mushroom is centered over collar",
    )

    return ctx.report()


object_model = build_object_model()
