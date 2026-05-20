from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


LEG_SPLAY = math.radians(18.0)


def _leg_origin(theta: float) -> Origin:
    """Frame whose local -Z points down and outward along a deployed tripod leg."""
    return Origin(
        xyz=(0.21 * math.cos(theta), 0.21 * math.sin(theta), -0.045),
        rpy=(0.0, -LEG_SPLAY, theta),
    )


def _radial_cylinder_origin(radius: float, z: float, theta: float) -> Origin:
    """Cylinder local Z aligned with a horizontal radial spoke."""
    return Origin(
        xyz=(radius * math.cos(theta), radius * math.sin(theta), z),
        rpy=(0.0, math.pi / 2.0, theta),
    )


def _side_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Cylinder local Z aligned with world/part +Y."""
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_observation_tripod")

    aluminum = model.material("dark_anodized_aluminum", rgba=(0.07, 0.08, 0.085, 1.0))
    satin = model.material("satin_black_casting", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    clamp = model.material("brushed_lock_metal", rgba=(0.42, 0.43, 0.41, 1.0))
    latch_red = model.material("red_release_latch", rgba=(0.75, 0.08, 0.035, 1.0))
    glass = model.material("blue_coated_glass", rgba=(0.08, 0.22, 0.36, 0.72))
    label = model.material("muted_scale_markings", rgba=(0.75, 0.76, 0.68, 1.0))

    # Root hub: a real hollow collar for the sliding center column, with three
    # splayed spider arms carrying the fixed leg sockets.
    hub = model.part("tripod_hub")
    collar_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.065, -0.070), (0.065, 0.070)],
        inner_profile=[(0.040, -0.070), (0.040, 0.070)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    hub.visual(
        mesh_from_geometry(collar_shell, "hollow_center_collar"),
        material=satin,
        name="center_collar",
    )
    hub.visual(
        Cylinder(radius=0.012, length=0.064),
        origin=Origin(xyz=(-0.083, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clamp,
        name="column_lock_screw",
    )
    hub.visual(
        Box((0.030, 0.018, 0.026)),
        origin=Origin(xyz=(-0.118, 0.0, 0.025)),
        material=rubber,
        name="column_lock_knob",
    )
    guide_pad_names = ("column_guide_pad_0", "column_guide_pad_1", "column_guide_pad_2")
    for guide_name, theta in zip(guide_pad_names, (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        hub.visual(
            Box((0.014, 0.018, 0.120)),
            origin=Origin(
                xyz=(0.033 * math.cos(theta), 0.033 * math.sin(theta), 0.0),
                rpy=(0.0, 0.0, theta),
            ),
            material=clamp,
            name=guide_name,
        )

    leg_angles = [math.radians(90.0), math.radians(210.0), math.radians(330.0)]
    spider_arm_names = ("spider_arm_0", "spider_arm_1", "spider_arm_2")
    leg_socket_names = ("leg_socket_0", "leg_socket_1", "leg_socket_2")
    for i, theta in enumerate(leg_angles):
        hub.visual(
            Cylinder(radius=0.016, length=0.155),
            origin=_radial_cylinder_origin(0.132, -0.040, theta),
            material=satin,
            name=spider_arm_names[i],
        )
        hub.visual(
            Sphere(radius=0.030),
            origin=Origin(xyz=(0.205 * math.cos(theta), 0.205 * math.sin(theta), -0.045)),
            material=satin,
            name=leg_socket_names[i],
        )

    # Three deployed outer leg sleeves.  Each is a connected open rail pair so
    # the narrower sliding stage can visibly run between the side rails without
    # hiding in an overlapping solid proxy.
    for i, theta in enumerate(leg_angles):
        outer = model.part(f"outer_leg_{i}")
        for side, x in enumerate((-0.030, 0.030)):
            outer.visual(
                Box((0.014, 0.018, 0.780)),
                origin=Origin(xyz=(x, 0.0, -0.420)),
                material=aluminum,
                name=f"outer_rail_{side}",
            )
        outer.visual(
            Box((0.088, 0.032, 0.085)),
            origin=Origin(xyz=(0.0, 0.0, -0.120)),
            material=satin,
            name="top_socket_bridge",
        )
        outer.visual(
            Box((0.040, 0.028, 0.090)),
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
            material=satin,
            name="upper_mount_pin",
        )
        outer.visual(
            Box((0.086, 0.007, 0.060)),
            origin=Origin(xyz=(0.0, 0.012, -0.655)),
            material=satin,
            name="front_clamp_bridge",
        )
        outer.visual(
            Box((0.022, 0.024, 0.075)),
            origin=Origin(xyz=(-0.045, 0.002, -0.655)),
            material=satin,
            name="clamp_cheek_0",
        )
        outer.visual(
            Box((0.022, 0.024, 0.075)),
            origin=Origin(xyz=(0.045, 0.002, -0.655)),
            material=satin,
            name="clamp_cheek_1",
        )
        outer.visual(
            Box((0.008, 0.012, 0.135)),
            origin=Origin(xyz=(-0.020, 0.0, -0.600)),
            material=clamp,
            name="slide_guide_pad_0",
        )
        outer.visual(
            Box((0.008, 0.012, 0.135)),
            origin=Origin(xyz=(0.020, 0.0, -0.600)),
            material=clamp,
            name="slide_guide_pad_1",
        )
        outer.visual(
            Box((0.070, 0.004, 0.145)),
            origin=Origin(xyz=(0.0, -0.010, -0.360)),
            material=label,
            name="height_scale_strip",
        )
        model.articulation(
            f"hub_to_outer_leg_{i}",
            ArticulationType.FIXED,
            parent=hub,
            child=outer,
            origin=_leg_origin(theta),
        )

        inner = model.part(f"inner_leg_{i}")
        inner.visual(
            Box((0.032, 0.014, 0.860)),
            origin=Origin(xyz=(0.0, 0.0, -0.180)),
            material=aluminum,
            name="inner_tube",
        )
        inner.visual(
            Box((0.038, 0.012, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, 0.230)),
            material=clamp,
            name="retention_stop",
        )
        inner.visual(
            Box((0.082, 0.050, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.625)),
            material=rubber,
            name="rubber_foot",
        )
        model.articulation(
            f"outer_to_inner_leg_{i}",
            ArticulationType.PRISMATIC,
            parent=outer,
            child=inner,
            origin=Origin(xyz=(0.0, 0.0, -0.680)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=110.0, velocity=0.18, lower=0.0, upper=0.18),
        )

    center_column = model.part("center_column")
    center_column.visual(
        Cylinder(radius=0.026, length=0.960),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=aluminum,
        name="column_tube",
    )
    center_column.visual(
        Cylinder(radius=0.038, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        material=clamp,
        name="top_stop_collar",
    )
    model.articulation(
        "hub_to_center_column",
        ArticulationType.PRISMATIC,
        parent=hub,
        child=center_column,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.12, lower=0.0, upper=0.28),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.078, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=satin,
        name="pan_bearing",
    )
    pan_head.visual(
        Box((0.135, 0.150, 0.030)),
        origin=Origin(xyz=(0.000, 0.0, 0.050)),
        material=satin,
        name="tilt_yoke_base",
    )
    for side, y in enumerate((-0.072, 0.072)):
        pan_head.visual(
            Box((0.050, 0.018, 0.135)),
            origin=Origin(xyz=(0.0, y, 0.105)),
            material=satin,
            name=f"tilt_cheek_{side}",
        )
        pan_head.visual(
            Cylinder(radius=0.021, length=0.018),
            origin=_side_cylinder_origin(0.0, y, 0.130),
            material=clamp,
            name=f"hinge_boss_{side}",
        )
    pan_head.visual(
        Box((0.240, 0.016, 0.016)),
        origin=Origin(xyz=(-0.125, -0.125, 0.050), rpy=(0.0, 0.0, math.radians(45.0))),
        material=aluminum,
        name="pan_handle_stem",
    )
    pan_head.visual(
        Box((0.095, 0.030, 0.030)),
        origin=Origin(xyz=(-0.230, -0.230, 0.050), rpy=(0.0, 0.0, math.radians(45.0))),
        material=rubber,
        name="pan_handle_grip",
    )
    model.articulation(
        "column_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=center_column,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )

    tilt_plate = model.part("tilt_plate")
    tilt_plate.visual(
        Cylinder(radius=0.017, length=0.128),
        origin=_side_cylinder_origin(0.0, 0.0, 0.0),
        material=clamp,
        name="tilt_barrel",
    )
    tilt_plate.visual(
        Box((0.220, 0.100, 0.025)),
        origin=Origin(xyz=(0.035, 0.0, 0.025)),
        material=satin,
        name="quick_release_plate",
    )
    for side, y in enumerate((-0.032, 0.032)):
        tilt_plate.visual(
            Box((0.165, 0.016, 0.018)),
            origin=Origin(xyz=(0.045, y, 0.046)),
            material=clamp,
            name=f"dovetail_rail_{side}",
        )
    tilt_plate.visual(
        Box((0.028, 0.090, 0.032)),
        origin=Origin(xyz=(-0.083, 0.0, 0.044)),
        material=satin,
        name="fixed_jaw",
    )
    tilt_plate.visual(
        Box((0.044, 0.020, 0.028)),
        origin=Origin(xyz=(-0.055, -0.058, 0.050)),
        material=satin,
        name="latch_socket",
    )
    model.articulation(
        "pan_to_tilt_plate",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.55, upper=0.95),
    )

    latch = model.part("locking_latch")
    latch.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=clamp,
        name="latch_pivot_pin",
    )
    latch.visual(
        Box((0.098, 0.018, 0.014)),
        origin=Origin(xyz=(0.050, 0.0, 0.014)),
        material=latch_red,
        name="latch_lever",
    )
    latch.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.105, 0.0, 0.014)),
        material=rubber,
        name="latch_finger_pad",
    )
    model.articulation(
        "plate_to_locking_latch",
        ArticulationType.REVOLUTE,
        parent=tilt_plate,
        child=latch,
        origin=Origin(xyz=(-0.055, -0.063, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.18, upper=1.05),
    )

    scope = model.part("spotting_scope")
    scope.visual(
        Box((0.152, 0.050, 0.020)),
        origin=Origin(xyz=(0.030, 0.0, 0.0645)),
        material=clamp,
        name="quick_release_shoe",
    )
    scope.visual(
        Box((0.050, 0.034, 0.060)),
        origin=Origin(xyz=(0.020, 0.0, 0.104)),
        material=satin,
        name="scope_saddle",
    )
    scope.visual(
        Cylinder(radius=0.048, length=0.420),
        origin=Origin(xyz=(0.020, 0.0, 0.181), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="scope_body",
    )
    scope.visual(
        Cylinder(radius=0.061, length=0.090),
        origin=Origin(xyz=(0.275, 0.0, 0.181), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="objective_hood",
    )
    scope.visual(
        Cylinder(radius=0.050, length=0.008),
        origin=Origin(xyz=(0.324, 0.0, 0.181), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    scope.visual(
        Cylinder(radius=0.030, length=0.085),
        origin=Origin(xyz=(-0.220, 0.0, 0.181), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="eyepiece",
    )
    model.articulation(
        "plate_to_spotting_scope",
        ArticulationType.FIXED,
        parent=tilt_plate,
        child=scope,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    hub = object_model.get_part("tripod_hub")
    column = object_model.get_part("center_column")
    pan = object_model.get_articulation("column_to_pan_head")
    column_slide = object_model.get_articulation("hub_to_center_column")
    tilt = object_model.get_articulation("pan_to_tilt_plate")
    latch_joint = object_model.get_articulation("plate_to_locking_latch")

    ctx.expect_overlap(
        column,
        hub,
        axes="z",
        elem_a="column_tube",
        elem_b="center_collar",
        min_overlap=0.080,
        name="center column remains captured in hollow collar",
    )
    ctx.expect_contact(
        column,
        hub,
        elem_a="column_tube",
        elem_b="column_guide_pad_0",
        contact_tol=0.001,
        name="center column bears against a guide pad",
    )
    column_rest = ctx.part_world_position(column)
    with ctx.pose({column_slide: 0.28}):
        column_high = ctx.part_world_position(column)
        ctx.expect_overlap(
            column,
            hub,
            axes="z",
            elem_a="column_tube",
            elem_b="center_collar",
            min_overlap=0.050,
            name="raised center column still has retained insertion",
        )
    ctx.check(
        "center column slides upward",
        column_rest is not None and column_high is not None and column_high[2] > column_rest[2] + 0.20,
        details=f"rest={column_rest}, high={column_high}",
    )

    for i in range(3):
        outer = object_model.get_part(f"outer_leg_{i}")
        inner = object_model.get_part(f"inner_leg_{i}")
        leg_slide = object_model.get_articulation(f"outer_to_inner_leg_{i}")

        ctx.allow_overlap(
            hub,
            outer,
            elem_a=f"leg_socket_{i}",
            elem_b="upper_mount_pin",
            reason="The fixed leg top is seated inside the hub socket casting.",
        )
        ctx.allow_overlap(
            outer,
            hub,
            elem_a="upper_mount_pin",
            elem_b=f"spider_arm_{i}",
            reason="The leg mounting tenon passes into the spider arm casting at the socket.",
        )
        ctx.allow_overlap(
            inner,
            outer,
            elem_a="inner_tube",
            elem_b="slide_guide_pad_0",
            reason="The simplified sliding guide pad represents a light bearing preload on the telescoping leg tube.",
        )
        ctx.expect_overlap(
            outer,
            hub,
            axes="xyz",
            elem_a="upper_mount_pin",
            elem_b=f"leg_socket_{i}",
            min_overlap=0.010,
            name=f"outer leg {i} top is captured by the hub socket",
        )
        ctx.expect_overlap(
            outer,
            hub,
            axes="xyz",
            elem_a="upper_mount_pin",
            elem_b=f"spider_arm_{i}",
            min_overlap=0.006,
            name=f"outer leg {i} tenon is retained in the spider arm",
        )
        ctx.expect_contact(
            inner,
            outer,
            elem_a="inner_tube",
            elem_b="slide_guide_pad_0",
            contact_tol=0.002,
            name=f"inner leg {i} bears on its sliding guide",
        )
        inner_rest = ctx.part_world_position(inner)
        with ctx.pose({leg_slide: 0.18}):
            inner_extended = ctx.part_world_position(inner)
        ctx.check(
            f"inner leg {i} extends down and outward",
            inner_rest is not None
            and inner_extended is not None
            and inner_extended[2] < inner_rest[2] - 0.12,
            details=f"rest={inner_rest}, extended={inner_extended}",
        )

    ctx.allow_overlap(
        "locking_latch",
        "tilt_plate",
        elem_a="latch_pivot_pin",
        elem_b="latch_socket",
        reason="The latch pivot pin is captured in the quick-release plate socket.",
    )
    ctx.allow_overlap(
        "locking_latch",
        "tilt_plate",
        elem_a="latch_lever",
        elem_b="latch_socket",
        reason="The lever heel sits under the retaining lip of the quick-release latch pocket.",
    )
    ctx.expect_overlap(
        "locking_latch",
        "tilt_plate",
        axes="xyz",
        elem_a="latch_pivot_pin",
        elem_b="latch_socket",
        min_overlap=0.005,
        name="locking latch pivot is retained in its socket",
    )
    ctx.expect_overlap(
        "locking_latch",
        "tilt_plate",
        axes="xyz",
        elem_a="latch_lever",
        elem_b="latch_socket",
        min_overlap=0.004,
        name="locking latch heel engages the latch pocket",
    )

    objective_rest = aabb_center(
        ctx.part_element_world_aabb("spotting_scope", elem="objective_hood")
    )
    with ctx.pose({pan: 0.85}):
        objective_panned = aabb_center(
            ctx.part_element_world_aabb("spotting_scope", elem="objective_hood")
        )
    ctx.check(
        "pan head rotates the observation device",
        abs(objective_panned[1] - objective_rest[1]) > 0.12,
        details=f"rest={objective_rest}, panned={objective_panned}",
    )

    scope_rest = ctx.part_element_world_aabb("spotting_scope", elem="objective_hood")
    with ctx.pose({tilt: 0.75}):
        scope_raised = ctx.part_element_world_aabb("spotting_scope", elem="objective_hood")
    ctx.check(
        "tilt plate elevates the objective end",
        scope_rest is not None and scope_raised is not None and scope_raised[1][2] > scope_rest[1][2] + 0.05,
        details=f"rest={scope_rest}, raised={scope_raised}",
    )

    latch_rest = aabb_center(ctx.part_element_world_aabb("locking_latch", elem="latch_finger_pad"))
    with ctx.pose({latch_joint: 0.95}):
        latch_unlocked = aabb_center(ctx.part_element_world_aabb("locking_latch", elem="latch_finger_pad"))
    ctx.check(
        "locking latch swings on its short pivot",
        abs(latch_unlocked[1] - latch_rest[1]) > 0.040,
        details=f"rest={latch_rest}, unlocked={latch_unlocked}",
    )

    return ctx.report()


object_model = build_object_model()
