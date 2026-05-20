from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_box_x(length: float, width: float, height: float, radius: float) -> MeshGeometry:
    """Rounded rectangular solid whose long dimension is local X."""
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(height, width, radius, corner_segments=8),
        length,
    )
    geom.rotate_y(math.pi / 2.0)
    return geom


def _rounded_box_y(length: float, thickness: float, height: float, radius: float) -> MeshGeometry:
    """Rounded rectangular pad/panel whose thin dimension is local Y."""
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(length, height, radius, corner_segments=8),
        thickness,
    )
    geom.rotate_x(math.pi / 2.0)
    return geom


def _lens_ring_mesh(
    inner_radius: float,
    outer_radius: float,
    length: float,
    *,
    segments: int = 72,
    ridge_count: int = 24,
    ridge_depth: float = 0.001,
) -> MeshGeometry:
    """Short hollow knurled ring centered on local X for the rotating lens control."""
    geom = MeshGeometry()
    outer_back: list[int] = []
    outer_front: list[int] = []
    inner_back: list[int] = []
    inner_front: list[int] = []

    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        ridge = 0.5 + 0.5 * math.cos(ridge_count * angle)
        radius = outer_radius + ridge_depth * max(0.0, ridge) ** 3
        cy = math.cos(angle)
        sz = math.sin(angle)
        outer_back.append(geom.add_vertex(-length / 2.0, radius * cy, radius * sz))
        outer_front.append(geom.add_vertex(length / 2.0, radius * cy, radius * sz))
        inner_back.append(geom.add_vertex(-length / 2.0, inner_radius * cy, inner_radius * sz))
        inner_front.append(geom.add_vertex(length / 2.0, inner_radius * cy, inner_radius * sz))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer_back[i], outer_back[j], outer_front[j])
        geom.add_face(outer_back[i], outer_front[j], outer_front[i])
        # Inner bore.
        geom.add_face(inner_back[i], inner_front[i], inner_front[j])
        geom.add_face(inner_back[i], inner_front[j], inner_back[j])
        # Front annular face.
        geom.add_face(outer_front[i], outer_front[j], inner_front[j])
        geom.add_face(outer_front[i], inner_front[j], inner_front[i])
        # Rear annular face.
        geom.add_face(outer_back[i], inner_back[i], inner_back[j])
        geom.add_face(outer_back[i], inner_back[j], outer_back[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_hd_camcorder")

    graphite = model.material("graphite_plastic", rgba=(0.055, 0.058, 0.062, 1.0))
    black = model.material("satin_black", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.012, 0.012, 0.014, 1.0))
    dark_glass = model.material("blue_black_glass", rgba=(0.02, 0.06, 0.10, 0.92))
    clear = model.material("clear_hinge_polycarbonate", rgba=(0.74, 0.90, 1.0, 0.42))
    silver = model.material("brushed_silver", rgba=(0.62, 0.64, 0.66, 1.0))
    white = model.material("white_index_mark", rgba=(0.90, 0.92, 0.88, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_rounded_box_x(0.150, 0.066, 0.075, 0.012), "main_shell"),
        material=graphite,
        name="main_shell",
    )
    body.visual(
        mesh_from_geometry(_rounded_box_y(0.112, 0.020, 0.058, 0.020), "hand_grip"),
        origin=Origin(xyz=(0.000, -0.041, -0.002)),
        material=rubber,
        name="hand_grip",
    )
    for index, z_pos in enumerate((-0.018, -0.002, 0.014)):
        body.visual(
            Box((0.088, 0.004, 0.003)),
            origin=Origin(xyz=(0.000, -0.052, z_pos)),
            material=black,
            name=f"grip_rib_{index}",
        )

    # Short front lens barrel and fixed glass nested in the camcorder nose.
    body.visual(
        Cylinder(radius=0.027, length=0.007),
        origin=Origin(xyz=(0.0785, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lens_shoulder",
    )
    body.visual(
        Cylinder(radius=0.0185, length=0.038),
        origin=Origin(xyz=(0.096, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="front_barrel",
    )
    body.visual(
        Cylinder(radius=0.0135, length=0.0025),
        origin=Origin(xyz=(0.116, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_glass,
        name="front_glass",
    )

    # Side recess visible when the fold-out monitor is swung away.
    body.visual(
        Box((0.078, 0.0012, 0.039)),
        origin=Origin(xyz=(-0.014, 0.0331, 0.006)),
        material=black,
        name="screen_pocket",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.072, 0.036),
                (0.091, 0.052),
                0.003,
                opening_corner_radius=0.004,
                outer_corner_radius=0.008,
            ),
            "screen_bezel",
        ),
        origin=Origin(xyz=(-0.014, 0.0342, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="screen_bezel",
    )

    # Clear vertical spine on the side wall for the LCD hinge.
    body.visual(
        Cylinder(radius=0.0042, length=0.064),
        origin=Origin(xyz=(-0.064, 0.0368, 0.002)),
        material=clear,
        name="hinge_spine",
    )

    monitor = model.part("monitor")
    monitor.visual(
        mesh_from_geometry(_rounded_box_y(0.091, 0.006, 0.056, 0.006), "monitor_shell"),
        origin=Origin(xyz=(0.050, 0.008, 0.0)),
        material=black,
        name="monitor_shell",
    )
    monitor.visual(
        Box((0.071, 0.0010, 0.039)),
        origin=Origin(xyz=(0.053, 0.0115, 0.002)),
        material=dark_glass,
        name="lcd_glass",
    )
    monitor.visual(
        Box((0.006, 0.006, 0.055)),
        origin=Origin(xyz=(0.0025, 0.004, 0.0)),
        material=clear,
        name="hinge_leaf",
    )

    model.articulation(
        "body_to_monitor",
        ArticulationType.REVOLUTE,
        parent=body,
        child=monitor,
        origin=Origin(xyz=(-0.064, 0.0368, 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.4, lower=0.0, upper=1.75),
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        mesh_from_geometry(
            _lens_ring_mesh(0.01835, 0.0260, 0.012, ridge_depth=0.0009),
            "lens_ring_knurled",
        ),
        material=black,
        name="lens_ring_knurled",
    )
    lens_ring.visual(
        Box((0.006, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0278)),
        material=white,
        name="focus_mark",
    )
    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    button_x_positions = (-0.038, -0.014, 0.010)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.0046, length=0.0040),
            origin=Origin(xyz=(0.0, 0.0020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0032, length=0.0010),
            origin=Origin(xyz=(0.0, 0.0044, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="button_face",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0336, -0.024)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=0.05, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    monitor = object_model.get_part("monitor")
    lens_ring = object_model.get_part("lens_ring")
    monitor_hinge = object_model.get_articulation("body_to_monitor")
    lens_joint = object_model.get_articulation("body_to_lens_ring")

    ctx.allow_overlap(
        body,
        monitor,
        elem_a="hinge_spine",
        elem_b="hinge_leaf",
        reason="The transparent LCD hinge leaf is intentionally captured around the clear vertical hinge spine.",
    )
    ctx.allow_overlap(
        body,
        lens_ring,
        elem_a="front_barrel",
        elem_b="lens_ring_knurled",
        reason="The rotating knurled lens ring is modeled as a close bearing sleeve seated on the front barrel.",
    )

    ctx.expect_gap(
        monitor,
        body,
        axis="y",
        min_gap=0.001,
        max_gap=0.011,
        positive_elem="monitor_shell",
        negative_elem="main_shell",
        name="closed monitor sits proud of side wall",
    )
    ctx.expect_overlap(
        monitor,
        body,
        axes="z",
        min_overlap=0.045,
        name="monitor spans the side screen opening height",
    )
    ctx.expect_overlap(
        monitor,
        body,
        axes="z",
        elem_a="hinge_leaf",
        elem_b="hinge_spine",
        min_overlap=0.045,
        name="monitor hinge leaf is captured on clear spine",
    )
    closed_aabb = ctx.part_world_aabb(monitor)
    with ctx.pose({monitor_hinge: 1.35}):
        opened_aabb = ctx.part_world_aabb(monitor)
    ctx.check(
        "monitor rotates outward from side",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][1] > closed_aabb[1][1] + 0.035,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    ctx.check(
        "lens ring is continuous around barrel",
        lens_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={lens_joint.articulation_type}",
    )
    ctx.expect_within(
        lens_ring,
        body,
        axes="yz",
        inner_elem="lens_ring_knurled",
        outer_elem="front_barrel",
        margin=0.012,
        name="lens ring remains concentric around front barrel",
    )
    ctx.expect_overlap(
        lens_ring,
        body,
        axes="x",
        elem_a="lens_ring_knurled",
        elem_b="front_barrel",
        min_overlap=0.010,
        name="lens ring retains bearing length on front barrel",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"body_to_button_{index}")
        ctx.expect_contact(
            button,
            body,
            elem_a="button_cap",
            elem_b="main_shell",
            contact_tol=0.001,
            name=f"button_{index} cap is seated in body wall",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.004}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses inward independently",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
