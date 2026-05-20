from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_trimmed_stand_mixer")

    enamel = model.material("warm_enamel", rgba=(0.92, 0.86, 0.74, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.82, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    dark = model.material("black_rubber", rgba=(0.02, 0.018, 0.015, 1.0))

    # Root base: a low rounded plinth with a rear pedestal, chrome bowl rails,
    # exposed hinge yoke, and side-mounted controls.
    base = model.part("base")
    base_body = superellipse_side_loft(
        [
            (-0.215, 0.000, 0.060, 0.12),
            (-0.165, 0.000, 0.105, 0.48),
            (0.000, 0.000, 0.130, 0.58),
            (0.165, 0.000, 0.105, 0.48),
            (0.215, 0.000, 0.060, 0.12),
        ],
        exponents=2.9,
        segments=64,
    )
    base.visual(mesh_from_geometry(base_body, "base_body"), material=enamel, name="rounded_plinth")

    pedestal = superellipse_side_loft(
        [
            (-0.145, 0.085, 0.445, 0.060),
            (-0.090, 0.075, 0.460, 0.135),
            (0.000, 0.070, 0.465, 0.165),
            (0.090, 0.075, 0.460, 0.135),
            (0.145, 0.085, 0.445, 0.060),
        ],
        exponents=2.7,
        segments=56,
    ).translate(-0.245, 0.0, 0.0)
    base.visual(mesh_from_geometry(pedestal, "rear_pedestal"), material=enamel, name="rear_pedestal")

    # Chrome front trim and raised slide rails on top of the base.
    base.visual(
        Box((0.022, 0.330, 0.020)),
        origin=Origin(xyz=(0.292, 0.0, 0.083)),
        material=chrome,
        name="front_chrome_band",
    )
    base.visual(
        Box((0.325, 0.032, 0.018)),
        origin=Origin(xyz=(0.115, -0.095, 0.124)),
        material=chrome,
        name="slide_rail_0",
    )
    base.visual(
        Box((0.325, 0.032, 0.018)),
        origin=Origin(xyz=(0.115, 0.095, 0.124)),
        material=chrome,
        name="slide_rail_1",
    )

    # Rear tilt yoke cheeks and chrome caps.
    for y in (-0.135, 0.135):
        base.visual(
            Box((0.072, 0.035, 0.112)),
            origin=Origin(xyz=(-0.180, y, 0.500)),
            material=enamel,
            name=f"hinge_cheek_{0 if y < 0 else 1}",
        )
        base.visual(
            Cylinder(radius=0.032, length=0.010),
            origin=Origin(
                xyz=(-0.180, y + (-0.021 if y < 0 else 0.021), 0.500),
                rpy=(pi / 2 if y < 0 else -pi / 2, 0.0, 0.0),
            ),
            material=chrome,
            name=f"hinge_cap_{0 if y < 0 else 1}",
        )

    # Control bezels are part of the base so the moving lever/button have
    # visible sockets rather than appearing to float beside the shell.
    base.visual(
        Box((0.125, 0.014, 0.070)),
        origin=Origin(xyz=(-0.155, -0.151, 0.315)),
        material=chrome,
        name="speed_slot_bezel",
    )
    base.visual(
        Box((0.085, 0.030, 0.050)),
        origin=Origin(xyz=(-0.205, 0.140, 0.350)),
        material=chrome,
        name="lock_button_bezel",
    )
    for y in (-0.118, 0.118):
        base.visual(
            Box((0.075, 0.030, 0.040)),
            origin=Origin(xyz=(-0.205, y, 0.435)),
            material=enamel,
            name=f"yoke_bridge_{0 if y < 0 else 1}",
        )

    # Sliding bowl carriage and a hollow polished bowl.
    bowl = model.part("bowl")
    bowl.visual(
        Box((0.245, 0.225, 0.018)),
        origin=Origin(xyz=(0.040, 0.0, 0.009)),
        material=chrome,
        name="slide_carriage",
    )
    bowl.visual(
        Cylinder(radius=0.062, length=0.016),
        origin=Origin(xyz=(0.040, 0.0, 0.026)),
        material=chrome,
        name="bowl_foot",
    )
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.050, 0.000),
            (0.078, 0.024),
            (0.130, 0.105),
            (0.166, 0.205),
            (0.174, 0.235),
        ],
        inner_profile=[
            (0.030, 0.020),
            (0.066, 0.046),
            (0.112, 0.113),
            (0.150, 0.197),
            (0.158, 0.222),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    ).translate(0.040, 0.0, 0.026)
    bowl.visual(mesh_from_geometry(bowl_shell, "hollow_bowl"), material=steel, name="polished_bowl")

    # Tilting motor head, with the part frame on the rear hinge line.
    head = model.part("head")
    head_body = superellipse_side_loft(
        [
            (-0.150, -0.040, 0.060, 0.150),
            (-0.095, -0.070, 0.092, 0.410),
            (0.000, -0.080, 0.110, 0.520),
            (0.095, -0.070, 0.092, 0.410),
            (0.150, -0.040, 0.060, 0.150),
        ],
        exponents=2.8,
        segments=64,
    ).translate(0.280, 0.0, 0.0)
    head.visual(mesh_from_geometry(head_body, "head_shell"), material=enamel, name="rounded_head")
    head.visual(
        Box((0.014, 0.215, 0.122)),
        origin=Origin(xyz=(0.525, 0.0, 0.010)),
        material=chrome,
        name="nose_chrome_band",
    )
    head.visual(
        Cylinder(radius=0.043, length=0.236),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.045),
        origin=Origin(xyz=(0.300, 0.0, -0.096)),
        material=chrome,
        name="drive_socket",
    )

    # Continuous vertical rotary wire whisk below the drive socket.
    whisk = model.part("whisk")
    whisk.visual(
        Cylinder(radius=0.007, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=chrome,
        name="drive_shaft",
    )
    whisk.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=chrome,
        name="wire_collar",
    )
    whisk.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.0, 0.0, -0.158)),
        material=chrome,
        name="wire_tip",
    )
    for i, angle in enumerate((0.0, pi / 4, pi / 2, 3 * pi / 4)):
        c = cos(angle)
        s = sin(angle)
        loop = tube_from_spline_points(
            [
                (0.018 * c, 0.018 * s, -0.060),
                (0.070 * c, 0.070 * s, -0.105),
                (0.082 * c, 0.082 * s, -0.132),
                (0.024 * c, 0.024 * s, -0.165),
                (-0.082 * c, -0.082 * s, -0.132),
                (-0.070 * c, -0.070 * s, -0.105),
                (-0.018 * c, -0.018 * s, -0.060),
            ],
            radius=0.0022,
            samples_per_segment=12,
            closed_spline=False,
            radial_segments=12,
            cap_ends=True,
        )
        whisk.visual(mesh_from_geometry(loop, f"whisk_wire_{i}"), material=chrome, name=f"wire_loop_{i}")

    # Side speed lever and head-lock push button.
    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.011, length=0.044),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=chrome,
        name="pivot_stem",
    )
    speed_lever.visual(
        Box((0.018, 0.018, 0.090)),
        origin=Origin(xyz=(0.035, -0.045, -0.030), rpy=(0.0, 0.0, -0.22)),
        material=dark,
        name="thumb_paddle",
    )
    speed_lever.visual(
        Box((0.040, 0.014, 0.018)),
        origin=Origin(xyz=(0.012, -0.044, -0.012)),
        material=dark,
        name="paddle_web",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Cylinder(radius=0.016, length=0.031),
        origin=Origin(xyz=(0.0, 0.0155, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=chrome,
        name="push_button",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.080, 0.0, 0.133)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.22, lower=0.0, upper=0.120),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.180, 0.0, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=0.82),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.300, 0.0, -0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=35.0),
    )
    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(-0.155, -0.158, 0.315)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.205, 0.155, 0.350)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.08, lower=-0.012, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_lever = object_model.get_part("speed_lever")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl")
    head_tilt = object_model.get_articulation("base_to_head")
    whisk_drive = object_model.get_articulation("head_to_whisk")
    speed_joint = object_model.get_articulation("base_to_speed_lever")
    lock_button = object_model.get_articulation("base_to_head_lock")

    canonical = {
        "base_to_bowl": (ArticulationType.PRISMATIC, "base", "bowl"),
        "base_to_head": (ArticulationType.REVOLUTE, "base", "head"),
        "head_to_whisk": (ArticulationType.CONTINUOUS, "head", "whisk"),
        "base_to_speed_lever": (ArticulationType.REVOLUTE, "base", "speed_lever"),
        "base_to_head_lock": (ArticulationType.PRISMATIC, "base", "head_lock"),
    }
    for name, (joint_type, parent, child) in canonical.items():
        joint = object_model.get_articulation(name)
        ctx.check(
            f"{name} canonical tree",
            joint.articulation_type == joint_type and joint.parent == parent and joint.child == child,
            details=f"type={joint.articulation_type}, parent={joint.parent}, child={joint.child}",
        )

    ctx.allow_overlap(
        head,
        whisk,
        elem_a="drive_socket",
        elem_b="drive_shaft",
        reason="The whisk drive shaft is intentionally captured inside the motor-head socket.",
    )
    ctx.expect_within(
        whisk,
        head,
        axes="xy",
        inner_elem="drive_shaft",
        outer_elem="drive_socket",
        margin=0.002,
        name="whisk shaft is centered in drive socket",
    )
    ctx.expect_overlap(
        head,
        whisk,
        axes="z",
        elem_a="drive_socket",
        elem_b="drive_shaft",
        min_overlap=0.008,
        name="whisk shaft remains inserted in socket",
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="slide_carriage",
        negative_elem="slide_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="bowl carriage sits on slide rail",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="x",
        elem_a="slide_carriage",
        elem_b="slide_rail_0",
        min_overlap=0.10,
        name="seated bowl carriage is retained on rail",
    )
    with ctx.pose({bowl_slide: 0.120}):
        ctx.expect_overlap(
            bowl,
            base,
            axes="x",
            elem_a="slide_carriage",
            elem_b="slide_rail_0",
            min_overlap=0.10,
            name="extended bowl carriage stays retained on rail",
        )

    rest_bowl = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: 0.120}):
        extended_bowl = ctx.part_world_position(bowl)
    ctx.check(
        "bowl slide moves toward the front",
        rest_bowl is not None and extended_bowl is not None and extended_bowl[0] > rest_bowl[0] + 0.10,
        details=f"rest={rest_bowl}, extended={extended_bowl}",
    )

    rest_socket = ctx.part_element_world_aabb(head, elem="drive_socket")
    with ctx.pose({head_tilt: 0.82}):
        tilted_socket = ctx.part_element_world_aabb(head, elem="drive_socket")
    ctx.check(
        "tilt head lifts the whisk socket",
        rest_socket is not None
        and tilted_socket is not None
        and tilted_socket[0][2] > rest_socket[0][2] + 0.08,
        details=f"rest={rest_socket}, tilted={tilted_socket}",
    )

    rest_lock = ctx.part_world_position(head_lock)
    with ctx.pose({lock_button: -0.012}):
        pressed_lock = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock button pushes inward",
        rest_lock is not None and pressed_lock is not None and pressed_lock[1] < rest_lock[1] - 0.010,
        details=f"rest={rest_lock}, pressed={pressed_lock}",
    )

    for joint, lower, upper in (
        (bowl_slide, 0.0, 0.120),
        (head_tilt, 0.0, 0.82),
        (speed_joint, -0.55, 0.55),
        (lock_button, -0.012, 0.0),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} realistic limits",
            limits is not None and limits.lower == lower and limits.upper == upper,
            details=f"limits={limits}",
        )
    ctx.check(
        "whisk has continuous vertical drive",
        whisk_drive.articulation_type == ArticulationType.CONTINUOUS and tuple(whisk_drive.axis) == (0.0, 0.0, 1.0),
        details=f"type={whisk_drive.articulation_type}, axis={whisk_drive.axis}",
    )

    return ctx.report()


object_model = build_object_model()
