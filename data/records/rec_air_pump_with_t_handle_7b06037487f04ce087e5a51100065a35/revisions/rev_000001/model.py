from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_station_pump")

    painted_steel = model.material("painted_steel", rgba=(0.12, 0.23, 0.38, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.02, 0.025, 1.0))
    gauge_face = model.material("gauge_face", rgba=(0.92, 0.90, 0.82, 1.0))
    needle_red = model.material("needle_red", rgba=(0.85, 0.05, 0.02, 1.0))
    button_red = model.material("button_red", rgba=(0.80, 0.03, 0.02, 1.0))

    body = model.part("body")

    # Low service-bay base with two foot pads and a socketed barrel mount.
    body.visual(
        Box((0.16, 0.46, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=painted_steel,
        name="base_crossbar",
    )
    for index, y_pos in enumerate((-0.225, 0.225)):
        body.visual(
            Box((0.38, 0.090, 0.042)),
            origin=Origin(xyz=(0.0, y_pos, 0.021)),
            material=dark_rubber,
            name=f"foot_pad_{index}",
        )
    body.visual(
        Cylinder(radius=0.066, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=painted_steel,
        name="base_socket",
    )

    barrel_shell = LatheGeometry.from_shell_profiles(
        [
            (0.047, 0.060),
            (0.052, 0.082),
            (0.052, 1.115),
            (0.046, 1.155),
        ],
        [
            (0.032, 0.064),
            (0.036, 0.086),
            (0.036, 1.120),
            (0.030, 1.160),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    body.visual(
        mesh_from_geometry(barrel_shell, "barrel_shell"),
        material=painted_steel,
        name="barrel_shell",
    )
    top_collar = LatheGeometry.from_shell_profiles(
        [(0.061, -0.021), (0.061, 0.021)],
        [(0.025, -0.021), (0.025, 0.021)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(top_collar, "top_collar"),
        origin=Origin(xyz=(0.0, 0.0, 1.132)),
        material=brushed_steel,
        name="top_collar",
    )
    body.visual(
        Box((0.012, 0.020, 0.030)),
        origin=Origin(xyz=(0.0195, 0.0, 1.132)),
        material=brushed_steel,
        name="guide_pad",
    )

    # Side gauge housing above the pads, with a clear bracket back to the barrel.
    body.visual(
        Box((0.100, 0.070, 0.070)),
        origin=Origin(xyz=(0.070, 0.0, 0.425)),
        material=painted_steel,
        name="gauge_neck",
    )
    body.visual(
        Cylinder(radius=0.102, length=0.060),
        origin=Origin(xyz=(0.115, 0.0, 0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="gauge_cup",
    )
    body.visual(
        Cylinder(radius=0.104, length=0.012),
        origin=Origin(xyz=(0.142, 0.0, 0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="gauge_bezel",
    )
    body.visual(
        Cylinder(radius=0.083, length=0.004),
        origin=Origin(xyz=(0.149, 0.0, 0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gauge_face,
        name="gauge_dial",
    )
    for index, angle in enumerate((-120, -60, 0, 60, 120)):
        radians = math.radians(angle)
        y_pos = 0.062 * math.sin(radians)
        z_pos = 0.425 + 0.062 * math.cos(radians)
        body.visual(
            Box((0.004, 0.004, 0.018)),
            origin=Origin(xyz=(0.152, y_pos, z_pos), rpy=(radians, 0.0, 0.0)),
            material=black_plastic,
            name=f"gauge_tick_{index}",
        )
    body.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.153, 0.0, 0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="needle_pivot_post",
    )

    # Top socket for the separate pressure-release pushbutton.
    body.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.115, -0.025, 0.523)),
        material=black_plastic,
        name="button_socket",
    )

    # Hose port and lower body clip for the stowed hose.
    hose_port = LatheGeometry.from_shell_profiles(
        [(0.020, -0.030), (0.020, 0.030)],
        [(0.014, -0.030), (0.014, 0.030)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(hose_port, "hose_port"),
        origin=Origin(xyz=(-0.058, 0.0, 0.280), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hose_port",
    )
    body.visual(
        Box((0.026, 0.120, 0.022)),
        origin=Origin(xyz=(-0.020, 0.060, 0.160)),
        material=painted_steel,
        name="clip_arm",
    )
    body.visual(
        Box((0.055, 0.026, 0.048)),
        origin=Origin(xyz=(-0.055, 0.120, 0.160)),
        material=black_plastic,
        name="lower_clip",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.42, 0.50, 1.20)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
    )

    handle_rod = model.part("handle_rod")
    handle_rod.visual(
        Cylinder(radius=0.014, length=0.880),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=brushed_steel,
        name="piston_rod",
    )
    handle_rod.visual(
        Cylinder(radius=0.023, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.400), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="t_handle_bar",
    )
    for index, y_pos in enumerate((-0.255, 0.255)):
        handle_rod.visual(
            Cylinder(radius=0.026, length=0.020),
            origin=Origin(xyz=(0.0, y_pos, 0.400), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_plastic,
            name=f"handle_end_{index}",
        )
    handle_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=0.90),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    needle = model.part("needle")
    needle.visual(
        Box((0.004, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        material=needle_red,
        name="needle_pointer",
    )
    needle.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="needle_hub",
    )
    needle.inertial = Inertial.from_geometry(
        Box((0.006, 0.080, 0.010)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=button_red,
        name="button_cap",
    )
    release_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.022),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    hose = model.part("hose")
    hose_tube = tube_from_spline_points(
        [
            (0.000, 0.000, 0.000),
            (-0.055, 0.020, 0.120),
            (-0.125, 0.120, 0.060),
            (-0.125, 0.190, -0.100),
            (-0.030, 0.170, -0.150),
            (0.030, 0.100, -0.120),
        ],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    hose.visual(
        mesh_from_geometry(hose_tube, "stowed_hose"),
        material=dark_rubber,
        name="hose_tube",
    )
    hose.visual(
        Cylinder(radius=0.014, length=0.075),
        origin=Origin(xyz=(0.030, 0.087, -0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hose_chuck",
    )
    hose.inertial = Inertial.from_geometry(
        Box((0.20, 0.24, 0.22)),
        mass=0.7,
        origin=Origin(xyz=(-0.050, 0.100, -0.040)),
    )

    model.articulation(
        "body_to_handle_rod",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle_rod,
        origin=Origin(xyz=(0.0, 0.0, 1.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.40, lower=0.0, upper=0.340),
    )
    model.articulation(
        "body_to_needle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=needle,
        origin=Origin(xyz=(0.158, 0.0, 0.425)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=12.0),
    )
    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(0.115, -0.025, 0.530)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.008),
    )
    model.articulation(
        "body_to_hose",
        ArticulationType.FIXED,
        parent=body,
        child=hose,
        origin=Origin(xyz=(-0.088, 0.0, 0.280)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle_rod = object_model.get_part("handle_rod")
    needle = object_model.get_part("needle")
    release_button = object_model.get_part("release_button")
    hose = object_model.get_part("hose")
    handle_slide = object_model.get_articulation("body_to_handle_rod")
    needle_spin = object_model.get_articulation("body_to_needle")
    button_slide = object_model.get_articulation("body_to_release_button")

    ctx.allow_overlap(
        body,
        hose,
        elem_a="hose_port",
        elem_b="hose_tube",
        reason="The parked rubber hose is intentionally seated inside the lower body port/barb.",
    )
    ctx.allow_overlap(
        body,
        hose,
        elem_a="lower_clip",
        elem_b="hose_tube",
        reason="The lower body clip intentionally grips the parked hose so it reads as stowed.",
    )
    ctx.allow_overlap(
        body,
        hose,
        elem_a="lower_clip",
        elem_b="hose_chuck",
        reason="The same lower clip captures the metal hose chuck when the hose is parked.",
    )

    ctx.expect_within(
        handle_rod,
        body,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="barrel_shell",
        margin=0.0,
        name="piston rod is centered inside the tubular barrel",
    )
    ctx.expect_overlap(
        handle_rod,
        body,
        axes="z",
        elem_a="piston_rod",
        elem_b="barrel_shell",
        min_overlap=0.40,
        name="lowered piston rod remains deeply inserted",
    )
    rest_handle = ctx.part_world_position(handle_rod)
    with ctx.pose({handle_slide: 0.340}):
        ctx.expect_within(
            handle_rod,
            body,
            axes="xy",
            inner_elem="piston_rod",
            outer_elem="barrel_shell",
            margin=0.0,
            name="raised piston rod stays on pump axis",
        )
        ctx.expect_overlap(
            handle_rod,
            body,
            axes="z",
            elem_a="piston_rod",
            elem_b="barrel_shell",
            min_overlap=0.15,
            name="raised piston rod retains insertion",
        )
        raised_handle = ctx.part_world_position(handle_rod)
    ctx.check(
        "handle translates upward along pump axis",
        rest_handle is not None
        and raised_handle is not None
        and raised_handle[2] > rest_handle[2] + 0.30,
        details=f"rest={rest_handle}, raised={raised_handle}",
    )

    ctx.expect_gap(
        release_button,
        body,
        axis="z",
        positive_elem="button_cap",
        negative_elem="button_socket",
        min_gap=0.0,
        max_gap=0.001,
        name="release button sits on its gauge-housing socket",
    )
    rest_button = ctx.part_world_position(release_button)
    with ctx.pose({button_slide: 0.008}):
        pressed_button = ctx.part_world_position(release_button)
    ctx.check(
        "release button pushes into gauge housing",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[2] < rest_button[2] - 0.006,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    rest_aabb = ctx.part_element_world_aabb(needle, elem="needle_pointer")
    with ctx.pose({needle_spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(needle, elem="needle_pointer")
    if rest_aabb is not None and turned_aabb is not None:
        rest_y = rest_aabb[1][1] - rest_aabb[0][1]
        rest_z = rest_aabb[1][2] - rest_aabb[0][2]
        turned_y = turned_aabb[1][1] - turned_aabb[0][1]
        turned_z = turned_aabb[1][2] - turned_aabb[0][2]
        needle_ok = rest_y > rest_z * 5.0 and turned_z > turned_y * 5.0
    else:
        needle_ok = False
    ctx.check(
        "gauge needle rotates on local pivot",
        needle_ok,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    ctx.expect_contact(
        hose,
        body,
        elem_a="hose_tube",
        elem_b="hose_port",
        contact_tol=0.004,
        name="stowed hose is connected to lower body port",
    )
    ctx.expect_overlap(
        hose,
        body,
        axes="x",
        elem_a="hose_tube",
        elem_b="hose_port",
        min_overlap=0.020,
        name="hose end remains inserted over the port barb",
    )
    ctx.expect_overlap(
        hose,
        body,
        axes="yz",
        elem_a="hose_chuck",
        elem_b="lower_clip",
        min_overlap=0.015,
        name="hose chuck is retained by the lower body clip",
    )
    ctx.expect_overlap(
        hose,
        body,
        axes="yz",
        elem_a="hose_tube",
        elem_b="lower_clip",
        min_overlap=0.020,
        name="rubber hose is captured by the lower body clip",
    )

    return ctx.report()


object_model = build_object_model()
