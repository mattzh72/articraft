from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_floor_pump")

    model.material("pump_blue", rgba=(0.05, 0.18, 0.62, 1.0))
    model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    model.material("dark_plastic", rgba=(0.02, 0.025, 0.03, 1.0))
    model.material("brushed_metal", rgba=(0.70, 0.70, 0.66, 1.0))
    model.material("gauge_face", rgba=(0.92, 0.90, 0.82, 1.0))
    model.material("needle_red", rgba=(0.85, 0.02, 0.01, 1.0))
    model.material("release_red", rgba=(0.95, 0.05, 0.02, 1.0))
    model.material("clip_gray", rgba=(0.18, 0.18, 0.17, 1.0))

    frame = model.part("pump_frame")

    # Wide garage floor-tool foot plate and raised standing pads.
    frame.visual(
        Box((0.42, 0.16, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="dark_plastic",
        name="wide_foot_base",
    )
    frame.visual(
        Box((0.17, 0.13, 0.010)),
        origin=Origin(xyz=(-0.115, 0.0, 0.042)),
        material="black_rubber",
        name="foot_pad_0",
    )
    frame.visual(
        Box((0.17, 0.13, 0.010)),
        origin=Origin(xyz=(0.115, 0.0, 0.042)),
        material="black_rubber",
        name="foot_pad_1",
    )
    for x in (-0.155, -0.115, -0.075, 0.075, 0.115, 0.155):
        frame.visual(
            Box((0.012, 0.122, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.0495)),
            material="clip_gray",
            name=f"pad_rib_{x:+.3f}",
        )

    # Barrel and fixed guide hardware.  The guide is a real ring so the
    # piston rod remains visible above it instead of being hidden in a block.
    frame.visual(
        Cylinder(radius=0.045, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material="dark_plastic",
        name="base_socket",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material="pump_blue",
        name="barrel",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.034, 0.006, radial_segments=32, tubular_segments=16), "top_guide_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.644)),
        material="brushed_metal",
        name="top_guide_ring",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.036, 0.007, radial_segments=32, tubular_segments=16), "bottom_barrel_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material="brushed_metal",
        name="bottom_barrel_ring",
    )

    # Gauge body mounted to the front lower barrel, with a separate dial face
    # and bezel.  The box neck visibly ties the gauge housing into the barrel.
    frame.visual(
        Box((0.048, 0.072, 0.044)),
        origin=Origin(xyz=(0.0, -0.050, 0.165)),
        material="dark_plastic",
        name="gauge_neck",
    )
    frame.visual(
        Cylinder(radius=0.067, length=0.038),
        origin=Origin(xyz=(0.0, -0.094, 0.190), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_plastic",
        name="gauge_body",
    )
    frame.visual(
        Cylinder(radius=0.052, length=0.003),
        origin=Origin(xyz=(0.0, -0.116, 0.190), rpy=(pi / 2.0, 0.0, 0.0)),
        material="gauge_face",
        name="dial_face",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.054, 0.004, radial_segments=40, tubular_segments=10), "gauge_bezel"),
        origin=Origin(xyz=(0.0, -0.120, 0.190), rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="gauge_bezel",
    )
    for angle, label in [(-0.85, "low_tick"), (-0.42, "mid_low_tick"), (0.0, "mid_tick"), (0.42, "mid_high_tick"), (0.85, "high_tick")]:
        x = 0.038 * sin(angle)
        z = 0.038 * cos(angle)
        frame.visual(
            Box((0.003, 0.002, 0.014)),
            origin=Origin(xyz=(x, -0.118, 0.190 + z), rpy=(0.0, angle, 0.0)),
            material="dark_plastic",
            name=label,
        )

    # Hose and side clip.  The clip bracket is fixed to the barrel side and
    # wraps around the black hose path.
    hose = tube_from_spline_points(
        [
            (0.041, -0.112, 0.165),
            (0.095, -0.060, 0.245),
            (0.102, 0.000, 0.365),
            (0.090, 0.010, 0.505),
            (0.036, 0.000, 0.605),
        ],
        radius=0.006,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    frame.visual(mesh_from_geometry(hose, "side_hose"), material="black_rubber", name="side_hose")
    frame.visual(
        Box((0.035, 0.034, 0.070)),
        origin=Origin(xyz=(0.044, 0.0, 0.365)),
        material="clip_gray",
        name="hose_clip_back",
    )
    frame.visual(
        Box((0.037, 0.014, 0.012)),
        origin=Origin(xyz=(0.075, -0.016, 0.390)),
        material="clip_gray",
        name="hose_clip_upper_jaw",
    )
    frame.visual(
        Box((0.037, 0.014, 0.012)),
        origin=Origin(xyz=(0.075, -0.016, 0.340)),
        material="clip_gray",
        name="hose_clip_lower_jaw",
    )
    frame.visual(
        Box((0.012, 0.014, 0.052)),
        origin=Origin(xyz=(0.092, -0.016, 0.365)),
        material="clip_gray",
        name="hose_clip_outer_lip",
    )

    slide = model.part("piston_slide")
    slide.visual(
        Cylinder(radius=0.038, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material="brushed_metal",
        name="sliding_guide_collar",
    )
    slide.visual(
        Cylinder(radius=0.0075, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material="brushed_metal",
        name="piston_rod",
    )
    slide.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.228)),
        material="dark_plastic",
        name="handle_stem",
    )
    slide.visual(
        Cylinder(radius=0.018, length=0.290),
        origin=Origin(xyz=(0.0, 0.0, 0.255), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_rubber",
        name="t_handle_grip",
    )
    slide.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(-0.145, 0.0, 0.255)),
        material="black_rubber",
        name="handle_end_0",
    )
    slide.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.145, 0.0, 0.255)),
        material="black_rubber",
        name="handle_end_1",
    )

    needle = model.part("gauge_needle")
    needle.visual(
        Box((0.052, 0.0025, 0.006)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material="needle_red",
        name="needle_pointer",
    )
    needle.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_plastic",
        name="needle_pivot_cap",
    )

    button = model.part("release_button")
    button.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="release_red",
        name="release_button_cap",
    )

    model.articulation(
        "frame_to_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slide,
        origin=Origin(xyz=(0.0, 0.0, 0.652)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55, lower=0.0, upper=0.230),
    )
    model.articulation(
        "frame_to_needle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=needle,
        origin=Origin(xyz=(0.0, -0.124, 0.190)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.06, velocity=3.0, lower=-0.95, upper=1.10),
    )
    model.articulation(
        "frame_to_button",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=button,
        origin=Origin(xyz=(-0.067, -0.094, 0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=0.15, lower=0.0, upper=0.008),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("pump_frame")
    slide = object_model.get_part("piston_slide")
    needle = object_model.get_part("gauge_needle")
    button = object_model.get_part("release_button")
    slide_joint = object_model.get_articulation("frame_to_slide")
    needle_joint = object_model.get_articulation("frame_to_needle")
    button_joint = object_model.get_articulation("frame_to_button")

    ctx.expect_gap(
        slide,
        frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.025,
        positive_elem="piston_rod",
        negative_elem="barrel",
        name="piston rod starts visibly above barrel",
    )
    ctx.expect_gap(
        frame,
        button,
        axis="x",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="gauge_body",
        negative_elem="release_button_cap",
        name="release button is separate on gauge housing",
    )
    ctx.expect_gap(
        frame,
        needle,
        axis="y",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="dial_face",
        negative_elem="needle_pointer",
        name="needle floats just above dial face",
    )

    rest_slide = ctx.part_world_position(slide)
    rest_button = ctx.part_world_position(button)
    with ctx.pose({slide_joint: 0.230, needle_joint: 0.85, button_joint: 0.008}):
        extended_slide = ctx.part_world_position(slide)
        pressed_button = ctx.part_world_position(button)
        needle_box = ctx.part_element_world_aabb(needle, elem="needle_pointer")

    ctx.check(
        "T-handle and rod slide upward along barrel axis",
        rest_slide is not None
        and extended_slide is not None
        and extended_slide[2] > rest_slide[2] + 0.20,
        details=f"rest={rest_slide}, extended={extended_slide}",
    )
    ctx.check(
        "pressure-release button pushes inward",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[0] > rest_button[0] + 0.006,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )
    ctx.check(
        "gauge needle rotates visibly about central pivot",
        needle_box is not None and (needle_box[1][2] - needle_box[0][2]) > 0.025,
        details=f"needle_aabb={needle_box}",
    )

    return ctx.report()


object_model = build_object_model()
